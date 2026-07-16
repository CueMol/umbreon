// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt1 gather half: the per-pixel brute-force path-trace gather. pt1EvalVertex
// must stay bit-for-bit in sync with oneBounceRadiance (the cache's per-ray
// evaluator); pt1GatherPoint / gatherPt1Grid are the hottest pt1 functions.
// Split out of pt1_integrator.hpp for readability; everything stays
// header-inline (see the umbrella header for the unit-compatibility
// rationale).
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range2d.h>

#include "ao/ambient_occlusion.hpp"
#include "experimental/irradiance_cache/irradiance_cache.hpp"
#include "experimental/pt1/pt1_gbuffer.hpp"
#include "experimental/pt2/pt2_emissive.hpp"
#include "experimental/pt2/pt2_reservoir.hpp"
#include "experimental/pt2/pt2_sampler.hpp"
#include "render/progress_slice.hpp"
#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// pt2 extensions layered onto the pt1 gather core, resolved per gather pixel.
// A null/default ext keeps every pt1 code path bit-identical (pt1 is the
// frozen regression anchor); pt2 (giIntegrator == 2) turns these on.
struct Pt1GatherExt {
  bool emissive = false;       // add Material::emission at bounce vertices
  bool sobol = false;          // first-bounce 2D via Owen-scrambled Sobol
  Pt2PixelSampler sampler{};   // resolved per-pixel Sobol state
  // ReSTIR candidate sink (pt2 with pt2Rounds > 0): every gather sample is
  // streamed into this pixel's reservoir -- first-bounce hit point + the
  // FULL path radiance from it; misses anchor at kPt2EnvDistance so the
  // reconnection Jacobian needs no env special case. Null = no capture.
  Pt2Reservoir* reservoir = nullptr;
  // Adaptive-spp support (pt2). halfSum, when non-null, receives the SUM of
  // the first floor(spp/2) samples' path radiance -- the adaptive pass
  // compares mean(all) against mean(first half) as its convergence estimate
  // (Cycles' split-buffer scheme). sampleIndexOffset shifts the per-sample
  // index (tea2 counter AND Sobol index) so a refinement pass continues the
  // pixel's sample sequence instead of replaying it.
  Vec3* halfSum = nullptr;
  int sampleIndexOffset = 0;
  // Emissive-triangle NEE + MIS (pt2, stage 3): when non-null and non-empty,
  // each sample adds one NEE draw toward a power-sampled emissive triangle at
  // the gather ORIGIN, and a first-bounce hit on such a triangle gets the
  // matching balance-heuristic weight on its emitted term (deeper hits keep
  // full weight -- per-vertex partition, unbiased).
  const Pt2EmissiveLights* emissiveLights = nullptr;
};

// Grid-level pt2 configuration handed to gatherPt1Grid, which resolves the
// per-pixel Pt1GatherExt from it (blue-noise Morton indexing needs the pixel
// coordinates, which only the grid loop knows).
struct Pt2GatherCfg {
  int pattern = kPt2PatternBlueNoise;  // RenderOptions::pt2Pattern
  bool emissive = true;                // RenderOptions::pt2Emissive
  // ReSTIR candidate reservoirs, one per grid pixel (indexed pix = y*W + x);
  // null when pt2 runs without spatial resampling (pt2Rounds == 0).
  Pt2Reservoir* reservoirs = nullptr;
  // Adaptive spp: halfE (W*H*3, zero-initialized by the caller) receives the
  // per-pixel SUM of the first floor(spp/2) samples. sampleIndexOffset makes
  // a refinement pass continue each pixel's sequence at that index. The
  // blue-noise pixel sections must be sized for the WHOLE budget (base +
  // refinement), not this call's spp -- sppLayoutTotal carries that budget
  // (0 = just this call's spp).
  float* halfE = nullptr;
  int sampleIndexOffset = 0;
  uint32_t sppLayoutTotal = 0;
  // Emissive NEE light list (null/empty = off); see Pt1GatherExt.
  const Pt2EmissiveLights* emissiveLights = nullptr;
};

// One path vertex of the pt1 gather walk: the NEE radiance it reflects toward
// the previous vertex, plus what the walk needs to continue the path there.
struct Pt1Vertex {
  Vec3 radiance{0.0f, 0.0f, 0.0f};  // kd*C * shadow-tested direct irradiance
  // Emitted radiance (emission * pigment), SEPARATE from radiance so the
  // caller can MIS-weight it against an emissive-NEE strategy (pt2). Always
  // zero unless pt1EvalVertex ran with addEmission (pt2), so pt1 callers
  // adding radiance + emitted stay bit-identical (x + 0.0f == x).
  Vec3 emitted{0.0f, 0.0f, 0.0f};
  Vec3 albedo{0.0f, 0.0f, 0.0f};    // kd*C (continuation throughput factor)
  Vec3 P{0.0f, 0.0f, 0.0f};         // world position
  Vec3 N{0.0f, 0.0f, 0.0f};         // shading normal, faced toward the ray origin
  float tfar = 0.0f;                // hit distance (self-intersect eps scale)
};

// Evaluate a gather-ray hit as a path vertex. Mesh hits reproduce
// oneBounceRadiance bit-for-bit (same interpolation, faceforward, RNG seeding
// and shadow test -- keep the two in sync); REAL CSG primitives (atom balls /
// bonds, fromEdge == 0) use the identical formula with the analytic Ng and the
// primID side-table color/material, so an atom ball bounces light like the SES
// mesh around it instead of absorbing it. Returns false for outline decoration
// (a black occluder that ends the path; never a light interaction, like AO).
// The cache keeps calling oneBounceRadiance directly, so its mesh-only
// behavior (and byte-identical output) is untouched.
// softRng0/softRng1: when non-null (pt2), the NEE shadow ray of an AREA light
// (l.radius > 0) is drawn from the light's cone with this RNG stream -- one
// jittered sample per path vertex, averaged over the gather's spp paths, so
// indirect shadows soften at no extra ray cost. Null (pt1/cache) keeps the
// exact hard center ray and consumes no draws, preserving those integrators'
// sample streams bit-for-bit.
inline bool pt1EvalVertex(const IrradianceCacheParams& p, const RTCRayHit& rh,
                          const Vec3& O, const Vec3& wi, Pt1Vertex& v,
                          Pt1RayStatsLocal* stats = nullptr,
                          bool addEmission = false,
                          uint32_t* softRng0 = nullptr,
                          uint32_t* softRng1 = nullptr) {
  const GeomRecord& rec = p.built->records[rh.hit.geomID];
  Vec3 Ny, Cy, NgShadow;
  float kd;
  float em = 0.0f;
  if (rec.kind == GeomKind::Mesh) {
    float nbuf[3] = {0, 0, 0};
    float cbuf[4] = {0, 0, 0, 1};
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
    Ny = normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
    Cy = Vec3{cbuf[0], cbuf[1], cbuf[2]};
    const Material& mm = p.mesh->materialForTri(rh.hit.primID);
    kd = mm.diffuse;
    if (addEmission) em = mm.emission;
    NgShadow = Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
  } else {
    const BuiltScene& b = *p.built;
    const bool isSphere = (rec.kind == GeomKind::Sphere);
    const bool isCapped = (rec.kind == GeomKind::CylinderCapped);
    const bool fromEdge =
        isSphere ? (!b.sphereFromEdge.empty() &&
                    b.sphereFromEdge[rh.hit.primID])
        : isCapped ? (!b.cylCapFromEdge.empty() &&
                      b.cylCapFromEdge[rh.hit.primID])
                   : (!b.cylFromEdge.empty() && b.cylFromEdge[rh.hit.primID]);
    if (fromEdge) return false;
    const Vec4& fc = isSphere ? b.sphereColor[rh.hit.primID]
                     : isCapped ? b.cylCapColor[rh.hit.primID]
                                : b.cylColor[rh.hit.primID];
    const Material& pm = isSphere ? b.sphereMat[rh.hit.primID]
                         : isCapped ? b.cylCapMat[rh.hit.primID]
                                    : b.cylMat[rh.hit.primID];
    Ny = safeNormalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z});
    Cy = Vec3{fc.x, fc.y, fc.z};
    kd = pm.diffuse;
    if (addEmission) em = pm.emission;
    NgShadow = Ny;
  }
  // Face the normal toward the gather origin (same rule as oneBounceRadiance).
  if (dot(Ny, wi) > 0.0f) Ny = Vec3{-Ny.x, -Ny.y, -Ny.z};

  const Vec3 Py{O.x + wi.x * rh.ray.tfar, O.y + wi.y * rh.ray.tfar,
                O.z + wi.z * rh.ray.tfar};
  const float eps = selfIntersectEps(Py, wi, rh.ray.tfar);

  // Shadow-tested direct irradiance at the vertex (NEE; no emission -- the
  // direct pass owns source-to-receiver light), then the diffuse reflectance.
  Vec3 E{0.0f, 0.0f, 0.0f};
  uint32_t s0 = hashU32(rh.hit.primID),
           s1 = hashU32(rh.hit.geomID + 0x9E3779B9u);
  for (const Light& l : *p.lights) {
    const float ndl = dot(Ny, l.L);
    if (ndl <= 0.0f) continue;
    float sh;
    if (softRng0 && softRng1 && l.radius > 0.0f) {
      // pt2 area-light NEE: one cone-jittered shadow ray per path (the spp
      // average softens it). computeShadow's multi-sample branch needs
      // shadowSamples > 1, so draw the direction here and trace directly.
      tea2(*softRng0, *softRng1);
      const Vec3 dir =
          sampleLightDir(l, u32ToUnorm(*softRng0), u32ToUnorm(*softRng1));
      Vec3 ng = (dot(NgShadow, Ny) < 0.0f)
                    ? Vec3{-NgShadow.x, -NgShadow.y, -NgShadow.z}
                    : NgShadow;
      ng = safeNormalize(ng, Ny);
      const Vec3 So{Py.x + ng.x * eps, Py.y + ng.y * eps, Py.z + ng.z * eps};
      sh = occluded(p.scene, So, dir, eps,
                    std::numeric_limits<float>::infinity())
               ? 0.0f
               : 1.0f;
    } else {
      sh = computeShadow(p.scene, Py, NgShadow, Ny, eps, l, 1, s0, s1);
    }
    if (stats) {
      ++stats->neeRays;
      if (sh == 0.0f) ++stats->neeOccluded;
    }
    E.x += ndl * l.color.x * sh;
    E.y += ndl * l.color.y * sh;
    E.z += ndl * l.color.z * sh;
  }
  v.radiance = Vec3{kd * Cy.x * E.x, kd * Cy.y * E.y, kd * Cy.z * E.z};
  // pt2 emissive transport: light EMITTED by this vertex toward the gather
  // ray. The direct pass only adds emission as self-illumination on camera-
  // visible pixels (shading.hpp) and never transports it to other surfaces,
  // so adding it here is the missing emitter-to-receiver path, not a double
  // count. Kept SEPARATE from v.radiance so the b==1 caller can MIS-weight
  // it against the emissive-NEE strategy. pt1 keeps addEmission == false.
  if (addEmission && em > 0.0f) {
    v.emitted.x = em * Cy.x;
    v.emitted.y = em * Cy.y;
    v.emitted.z = em * Cy.z;
  }
  v.albedo = Vec3{kd * Cy.x, kd * Cy.y, kd * Cy.z};
  v.P = Py;
  v.N = Ny;
  v.tfar = rh.ray.tfar;
  return true;
}

// Indirect irradiance at shading point P with shading normal N and geometric
// normal Ng, by brute-force cosine-hemisphere path tracing (the estimator the
// irradiance cache approximates, evaluated per pixel). p.bounces sets the path
// length: 1 = the classic one-bounce gather, >1 continues each path with one
// cosine-sampled continuation ray per vertex (path tracing with NEE at every
// vertex; POV radiosity's recursion_limit analogue).
//
// Returns E_stored = (1/spp) * sum(L_i) = E_true/pi (the cache's convention;
// the composite multiplies by kd*pigment with no 1/pi). Per path:
//   vertex b -> L += throughput_b * pt1EvalVertex().radiance (shadow-tested
//           direct light reflected at the vertex, for mesh AND real CSG hits;
//           outline decoration absorbs the path; NO emission -- source-to-
//           receiver light is already counted by the direct pass), then
//           throughput_{b+1} = throughput_b * albedo_b
//   miss -> L += throughput * environmentRadiance (the sky lives ONLY in this
//           miss term, evaluated once where the path escapes)
// The cosine sampling at every vertex absorbs the cos/pi of the diffuse BRDF,
// so the no-1/pi convention holds at every bounce, and with bounces == 1 the
// sample stream and arithmetic are IDENTICAL to the original gather.
// From the second vertex on, Russian roulette (survival = max component of
// the throughput, clamped to [0.05, 0.95]) keeps long paths unbiased without
// tracing every one to the bounce cap.
//
// A first-bounce sample below the geometric horizon (dot(wi, Ng) <= 0,
// possible when the shading normal diverges from Ng) contributes 0 but still
// counts in the divisor, and counts as occluded for `outOcclusion` (the
// surface itself blocks that direction). `outOcclusion` stays the FIRST-bounce
// hit fraction regardless of p.bounces.
//
// epsT is a finite length scale for selfIntersectEps (e.g. the scene AABB
// diagonal); it must NOT be the gather tfar, which pt1 defaults to infinity.
// The RNG is tea2 seeded from (seed, sample index) only and re-mixed per draw
// -- deterministic and thread-count independent, no shared state.
//
// ld = true stratifies the FIRST-bounce direction with the Hammersley set
// (i/spp, radicalInverse2(i)) under a per-point Cranley-Patterson toroidal
// shift derived from `seed` (the AO sampler's scheme, see aoSample2d) --
// lower variance at the same spp, still fully deterministic. Continuation
// bounces and Russian roulette keep the tea2 stream (standard practice: the
// low-discrepancy set pays off in the first, dominant dimension).
// clampLum > 0 clamps each sample's path contribution to that luminance
// (Rec.709), scaling RGB uniformly -- a firefly suppressor for multi-bounce
// paths; 0 keeps the estimator unbiased.
inline Vec3 pt1GatherPoint(const IrradianceCacheParams& p, const Vec3& P,
                           const Vec3& N, const Vec3& Ng, int spp,
                           uint32_t seed, float epsT, float* outOcclusion,
                           bool ld = false, float clampLum = 0.0f,
                           Pt1RayStatsLocal* stats = nullptr,
                           const Pt1GatherExt* ext = nullptr) {
  const Frame f = frameFromNormal(N);
  const float eps = selfIntersectEps(P, N, epsT);
  const Vec3 O = P + N * eps;
  const int maxB = (p.bounces < 1) ? 1 : p.bounces;
  float cpx = 0.0f, cpy = 0.0f;
  if (ld) {
    uint32_t c0 = seed, c1 = 0x9E3779B9u;
    tea2(c0, c1);
    cpx = u32ToUnorm(c0);
    cpy = u32ToUnorm(c1);
  }
  Vec3 Esum{0.0f, 0.0f, 0.0f};
  int nOccluded = 0;
  // ReSTIR candidate streaming state (pt2). An independent tea2 chain drives
  // the reservoir selection so the existing per-sample streams stay intact.
  Pt2Reservoir* rsv = ext ? ext->reservoir : nullptr;
  uint32_t rs0 = 0, rs1 = 0;
  double rWSum = 0.0;
  Vec3 rSelPos{}, rSelN{}, rSelRad{};
  float rSelPHat = 0.0f;
  if (rsv) {
    rs0 = seed ^ 0x52455354u;
    rs1 = 0x47490001u;
  }
  const int sOff = ext ? ext->sampleIndexOffset : 0;
  for (int s = 0; s < spp; ++s) {
    uint32_t s0 = seed;
    uint32_t s1 = static_cast<uint32_t>(s + sOff);
    tea2(s0, s1);
    float u1, u2;
    if (ext && ext->sobol) {
      // pt2: shuffled Owen-scrambled Sobol replaces the ld/tea2 first-bounce
      // draw (true Owen scrambling instead of a toroidal shift; with the
      // blue-noise arrangement the pixel's section of the global sequence is
      // ext->sampler.indexBase + s, offset by any refinement continuation).
      pt2SobolBurley2D(ext->sampler.indexBase +
                           static_cast<uint32_t>(s + sOff),
                       ext->sampler.seed, &u1, &u2);
    } else if (ld) {
      u1 = static_cast<float>(s) / static_cast<float>(spp) + cpx;
      u2 = radicalInverse2(static_cast<uint32_t>(s)) + cpy;
      if (u1 >= 1.0f) u1 -= 1.0f;
      if (u2 >= 1.0f) u2 -= 1.0f;
    } else {
      u1 = u32ToUnorm(s0);
      u2 = u32ToUnorm(s1);
    }
    Vec3 wi = cosineSampleHemisphere(u1, u2, f);
    // A below-horizon draw (shading normal diverging from Ng) contributes no
    // path but is still one of the spp candidates: it must flow through the
    // shared per-sample tail below -- the reservoir M count, and the
    // vertex-0 emissive NEE, whose estimator divides by spp and would be
    // biased low if skipped samples also skipped their NEE draw.
    const bool horizonOk = dot(wi, Ng) > 0.0f;
    if (!horizonOk) ++nOccluded;
    // Candidate capture: the first segment's direction cosine (the source
    // pdf is cos/pi) and, once bounce 1 resolves, the sample point.
    const float cand0Cos = dot(wi, N);
    Vec3 candPos{0.0f, 0.0f, 0.0f}, candN{0.0f, 0.0f, 0.0f};
    bool candValid = false;
    Vec3 org = O;
    float tnear = eps;
    Vec3 throughput{1.0f, 1.0f, 1.0f};
    Vec3 L{0.0f, 0.0f, 0.0f};
    for (int b = 1; horizonOk && b <= maxB; ++b) {
      const RTCRayHit rh = intersectFull(p.scene, org, wi, tnear,
                                         p.maxDistance);
      if (stats) ++stats->gatherRays;
      if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
        const Vec3 env = environmentRadiance(p, wi);
        L.x += throughput.x * env.x;
        L.y += throughput.y * env.y;
        L.z += throughput.z * env.z;
        if (rsv && b == 1) {
          // Escaped first bounce: anchor the candidate at a huge distance so
          // reconnection Jacobians degenerate to ~1 (no env special case).
          candPos = Vec3{O.x + wi.x * kPt2EnvDistance,
                         O.y + wi.y * kPt2EnvDistance,
                         O.z + wi.z * kPt2EnvDistance};
          candN = Vec3{-wi.x, -wi.y, -wi.z};
          candValid = true;
        }
        break;
      }
      if (b == 1) ++nOccluded;
      if (stats) ++stats->gatherHits;
      Pt1Vertex v;
      // pt2 (ext non-null): hand the path's tea2 stream to the NEE so an
      // area light gets one jittered shadow sample per path. pt1 passes null
      // -- handing the stream over would change pt1's continuation RNG
      // whenever --light-radius is set.
      if (!pt1EvalVertex(p, rh, org, wi, v, stats,
                         ext && ext->emissive,
                         ext ? &s0 : nullptr, ext ? &s1 : nullptr)) {
        // Outline decoration absorbs the path; keep the (zero-radiance)
        // candidate anchored at the hit so it stays a counted M.
        if (rsv && b == 1) {
          candPos = Vec3{org.x + wi.x * rh.ray.tfar,
                         org.y + wi.y * rh.ray.tfar,
                         org.z + wi.z * rh.ray.tfar};
          candN = Vec3{-wi.x, -wi.y, -wi.z};
          candValid = true;
        }
        break;
      }
      if (rsv && b == 1) {
        // Snapshot the first-bounce sample point BEFORE the continuation
        // loop overwrites v with later vertices.
        candPos = v.P;
        candN = v.N;
        candValid = true;
      }
      // Emitted term: at b == 1 with emissive NEE active, this hit competes
      // with the NEE strategy at the gather origin -- weigh it by the balance
      // heuristic (pt2EmissiveHitWeight is 1 for non-emitter hits and for
      // CSG emitters, which NEE does not cover). Deeper vertices have no NEE
      // partner, so their emission keeps full weight.
      float emW = 1.0f;
      if (ext && ext->emissiveLights && b == 1 &&
          p.built->records[rh.hit.geomID].kind == GeomKind::Mesh)
        emW = pt2EmissiveHitWeight(*ext->emissiveLights, rh.hit.primID,
                                   rh.ray.tfar, cand0Cos, wi);
      L.x += throughput.x * (v.radiance.x + emW * v.emitted.x);
      L.y += throughput.y * (v.radiance.y + emW * v.emitted.y);
      L.z += throughput.z * (v.radiance.z + emW * v.emitted.z);
      if (b == maxB) break;
      throughput.x *= v.albedo.x;
      throughput.y *= v.albedo.y;
      throughput.z *= v.albedo.z;
      if (b >= 2) {
        // Russian roulette on the continuation to vertex b+1 (>= 3).
        tea2(s0, s1);
        float pc = std::fmax(throughput.x,
                             std::fmax(throughput.y, throughput.z));
        pc = std::fmin(0.95f, std::fmax(0.05f, pc));
        if (u32ToUnorm(s0) >= pc) break;
        const float inv = 1.0f / pc;
        throughput.x *= inv;
        throughput.y *= inv;
        throughput.z *= inv;
      }
      // Continue the path from the vertex with a fresh cosine sample around
      // its shading normal (the horizon guard is first-bounce only: Ng is not
      // tracked past the G-buffer, and cosine sampling keeps dot(wi, N) > 0).
      tea2(s0, s1);
      const Frame fv = frameFromNormal(v.N);
      wi = cosineSampleHemisphere(u32ToUnorm(s0), u32ToUnorm(s1), fv);
      const float epsV = selfIntersectEps(v.P, wi, v.tfar);
      org = Vec3{v.P.x + v.N.x * epsV, v.P.y + v.N.y * epsV,
                 v.P.z + v.N.z * epsV};
      tnear = epsV;
    }
    // Emissive NEE at the gather ORIGIN (vertex 0): one power-sampled draw
    // per sample, MIS-weighted against the cosine strategy above. Rides the
    // same per-sample slot so the spp mean and the luminance clamp treat it
    // like any other path contribution.
    if (ext && ext->emissiveLights) {
      const Vec3 nee = pt2EmissiveNee(*ext->emissiveLights, p.scene, P, N,
                                      Ng, epsT, s0, s1);
      if (stats) ++stats->neeRays;
      L.x += nee.x;
      L.y += nee.y;
      L.z += nee.z;
    }
    if (clampLum > 0.0f) {
      const float lum = 0.2126f * L.x + 0.7152f * L.y + 0.0722f * L.z;
      if (lum > clampLum) {
        const float k = clampLum / lum;
        L.x *= k;
        L.y *= k;
        L.z *= k;
      }
    }
    // Stream this sample into the pixel reservoir. RIS weight = p_hat / pdf
    // with p_hat = lum(L) * cos and pdf = cos / pi -- the cosines CANCEL
    // ANALYTICALLY to w = pi * lum(L), so grazing candidates get no spurious
    // amplification (computing the quotient numerically with a floored p_hat
    // was measured as dark speckle: near-zero-cosine self-hits won reservoirs
    // they contribute nothing to).
    if (rsv) {
      float w = 0.0f;
      float pHat = 0.0f;
      if (candValid && cand0Cos > 0.0f) {
        pHat = pt2Luminance(L) * cand0Cos;
        w = pt2Luminance(L) * 3.14159265358979323846f;
      }
      tea2(rs0, rs1);
      float dummyM = 0.0f;
      if (pt2ReservoirUpdate(rWSum, dummyM, w, 0.0f, u32ToUnorm(rs0))) {
        rSelPos = candPos;
        rSelN = candN;
        rSelRad = L;
        rSelPHat = pHat;
      }
    }
    Esum.x += L.x;
    Esum.y += L.y;
    Esum.z += L.z;
    if (ext && ext->halfSum && s < spp / 2) {
      ext->halfSum->x += L.x;
      ext->halfSum->y += L.y;
      ext->halfSum->z += L.z;
    }
  }
  if (rsv) {
    // Finalize: W = wSum / (M * p_hat_selected), M = the full spp candidate
    // count (skipped and absorbed draws included -- they were streamed with
    // zero weight above).
    rsv->samplePos = rSelPos;
    rsv->sampleNormal = rSelN;
    rsv->radiance = rSelRad;
    rsv->M = static_cast<float>(spp > 0 ? spp : 1);
    rsv->W = (rSelPHat > 0.0f && rWSum > 0.0)
                 ? static_cast<float>(rWSum /
                                      (static_cast<double>(rsv->M) *
                                       static_cast<double>(rSelPHat)))
                 : 0.0f;
  }
  const float invN = 1.0f / static_cast<float>(spp > 0 ? spp : 1);
  if (outOcclusion) *outOcclusion = static_cast<float>(nOccluded) * invN;
  return Vec3{Esum.x * invN, Esum.y * invN, Esum.z * invN};
}

// Gather E_stored for every gather-eligible pixel of a W x H grid into `E`
// (W*H*3) and the hit fraction into `occ` (W*H). position/normal are the
// first-hit AOVs; geomNormal may be null (full-res mode: the render G-buffer
// has no geometric normal, so the horizon guard degenerates to the shading
// normal, which cosine sampling already satisfies) or the pt1 G-buffer's
// geometric normal (half-res mode: the real guard). skip[pix] != 0 marks
// gather-eligible (mesh-hit) pixels; others stay zero.
//
// `depth` is the PRIMARY-ray hit distance per pixel and sets the gather-origin
// self-intersection epsilon scale: the first-hit position carries an absolute
// error ~ t * 2^-23 that grows with the camera distance, so the offset that
// lifts the gather origin off the surface must scale with that same t --
// exactly what the transparency walk does with its per-hit tfar. The previous
// scale (the pass scene's mesh AABB diagonal via `epsT`) only worked when the
// diagonal happened to be of the camera-distance order, and DEGENERATED to the
// 1.0 fallback for scenes with no mesh (CSG-only ball-and-stick), where the
// epsilon fell below the hit-position error and gather rays re-hit their own
// surface (false occlusion, darkened GI). `epsT` remains as the fallback for
// null/zero depth entries.
//
// TBB 16x16 tiles; each pixel is seeded from (pixel index, frameSeed) only, so
// the image is deterministic across thread counts. Tiles write disjoint pixel
// ranges (no locks). ld / clampLum forward to pt1GatherPoint (stratified
// first-bounce sampling / per-sample luminance clamp).
inline void gatherPt1Grid(const IrradianceCacheParams& p, int W, int H,
                          const float* position, const float* normal,
                          const float* geomNormal, const uint8_t* eligible,
                          const float* depth, int spp, uint32_t frameSeed,
                          float epsT, std::vector<float>& E,
                          std::vector<float>& occ, bool ld = false,
                          float clampLum = 0.0f,
                          Pt1RayStats* stats = nullptr,
                          const ProgressSlice* prog = nullptr,
                          const Pt2GatherCfg* pt2 = nullptr) {
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  E.assign(npix * 3, 0.0f);
  occ.assign(npix, 0.0f);
  const uint32_t seedMix = hashU32(frameSeed);
  // pt2 blue-noise: pixel sections of the ONE global sequence sized for the
  // whole sample budget (base + adaptive refinement), rounded up to a power
  // of two so sections never overlap.
  const uint32_t sppBudget =
      (pt2 && pt2->sppLayoutTotal > 0)
          ? pt2->sppLayoutTotal
          : static_cast<uint32_t>(spp > 0 ? spp : 1);
  const uint32_t sppPow2 = pt2NextPow2(sppBudget);
  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
        // Cooperative cancel: bail before this tile (any in-flight tile stops at
        // its next one). A plain atomic load, so the no-progress path is
        // untouched and the pixel values stay thread-count invariant.
        if (prog && prog->cancelled()) return;
        for (int py = r.rows().begin(); py != r.rows().end(); ++py) {
          for (int px = r.cols().begin(); px != r.cols().end(); ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            if (!eligible[pix]) continue;
            const Vec3 P{position[pix * 3 + 0], position[pix * 3 + 1],
                         position[pix * 3 + 2]};
            const Vec3 N{normal[pix * 3 + 0], normal[pix * 3 + 1],
                         normal[pix * 3 + 2]};
            const Vec3 Ng = geomNormal
                                ? Vec3{geomNormal[pix * 3 + 0],
                                       geomNormal[pix * 3 + 1],
                                       geomNormal[pix * 3 + 2]}
                                : N;
            const uint32_t seed =
                hashU32(static_cast<uint32_t>(pix) ^ seedMix);
            const float tEps =
                (depth != nullptr && depth[pix] > 0.0f) ? depth[pix] : epsT;
            Pt1RayStatsLocal local;
            float o = 0.0f;
            Pt1GatherExt ext;
            const Pt1GatherExt* extP = nullptr;
            Vec3 half{0.0f, 0.0f, 0.0f};
            if (pt2) {
              ext.emissive = pt2->emissive;
              ext.sobol = true;
              ext.sampler = pt2MakePixelSampler(
                  pt2->pattern, static_cast<uint32_t>(px),
                  static_cast<uint32_t>(py), seedMix, seed, sppPow2);
              if (pt2->reservoirs) ext.reservoir = &pt2->reservoirs[pix];
              if (pt2->halfE) ext.halfSum = &half;
              ext.sampleIndexOffset = pt2->sampleIndexOffset;
              if (pt2->emissiveLights && !pt2->emissiveLights->empty())
                ext.emissiveLights = pt2->emissiveLights;
              extP = &ext;
            }
            const Vec3 e = pt1GatherPoint(p, P, N, Ng, spp, seed, tEps, &o,
                                          ld, clampLum,
                                          stats ? &local : nullptr, extP);
            if (pt2 && pt2->halfE) {
              pt2->halfE[pix * 3 + 0] = half.x;
              pt2->halfE[pix * 3 + 1] = half.y;
              pt2->halfE[pix * 3 + 2] = half.z;
            }
            if (stats) stats->flush(local);
            E[pix * 3 + 0] = e.x;
            E[pix * 3 + 1] = e.y;
            E[pix * 3 + 2] = e.z;
            occ[pix] = o;
          }
        }
        // Report the tile's AREA: the range is 2D, so counting rows alone would
        // under-report by the column split.
        if (prog)
          prog->addWork(
              static_cast<std::uint64_t>(r.rows().end() - r.rows().begin()) *
                  static_cast<std::uint64_t>(r.cols().end() - r.cols().begin()),
              static_cast<std::uint64_t>(W) * static_cast<std::uint64_t>(H));
      });
}
}  // namespace detail
}  // namespace umbreon
