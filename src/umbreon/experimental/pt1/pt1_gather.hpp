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
#include "render/progress_slice.hpp"
#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// One path vertex of the pt1 gather walk: the NEE radiance it reflects toward
// the previous vertex, plus what the walk needs to continue the path there.
struct Pt1Vertex {
  Vec3 radiance{0.0f, 0.0f, 0.0f};  // kd*C * shadow-tested direct irradiance
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
inline bool pt1EvalVertex(const IrradianceCacheParams& p, const RTCRayHit& rh,
                          const Vec3& O, const Vec3& wi, Pt1Vertex& v,
                          Pt1RayStatsLocal* stats = nullptr) {
  const GeomRecord& rec = p.built->records[rh.hit.geomID];
  Vec3 Ny, Cy, NgShadow;
  float kd;
  if (rec.kind == GeomKind::Mesh) {
    float nbuf[3] = {0, 0, 0};
    float cbuf[4] = {0, 0, 0, 1};
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
    Ny = normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
    Cy = Vec3{cbuf[0], cbuf[1], cbuf[2]};
    kd = p.mesh->materialForTri(rh.hit.primID).diffuse;
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
    const float sh = computeShadow(p.scene, Py, NgShadow, Ny, eps, l, 1, s0, s1);
    if (stats) {
      ++stats->neeRays;
      if (sh == 0.0f) ++stats->neeOccluded;
    }
    E.x += ndl * l.color.x * sh;
    E.y += ndl * l.color.y * sh;
    E.z += ndl * l.color.z * sh;
  }
  v.radiance = Vec3{kd * Cy.x * E.x, kd * Cy.y * E.y, kd * Cy.z * E.z};
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
                           Pt1RayStatsLocal* stats = nullptr) {
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
  for (int s = 0; s < spp; ++s) {
    uint32_t s0 = seed;
    uint32_t s1 = static_cast<uint32_t>(s);
    tea2(s0, s1);
    float u1, u2;
    if (ld) {
      u1 = static_cast<float>(s) / static_cast<float>(spp) + cpx;
      u2 = radicalInverse2(static_cast<uint32_t>(s)) + cpy;
      if (u1 >= 1.0f) u1 -= 1.0f;
      if (u2 >= 1.0f) u2 -= 1.0f;
    } else {
      u1 = u32ToUnorm(s0);
      u2 = u32ToUnorm(s1);
    }
    Vec3 wi = cosineSampleHemisphere(u1, u2, f);
    if (dot(wi, Ng) <= 0.0f) {
      ++nOccluded;
      continue;
    }
    Vec3 org = O;
    float tnear = eps;
    Vec3 throughput{1.0f, 1.0f, 1.0f};
    Vec3 L{0.0f, 0.0f, 0.0f};
    for (int b = 1; b <= maxB; ++b) {
      const RTCRayHit rh = intersectFull(p.scene, org, wi, tnear,
                                         p.maxDistance);
      if (stats) ++stats->gatherRays;
      if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
        const Vec3 env = environmentRadiance(p, wi);
        L.x += throughput.x * env.x;
        L.y += throughput.y * env.y;
        L.z += throughput.z * env.z;
        break;
      }
      if (b == 1) ++nOccluded;
      if (stats) ++stats->gatherHits;
      Pt1Vertex v;
      if (!pt1EvalVertex(p, rh, org, wi, v, stats)) break;  // outline: absorbed
      L.x += throughput.x * v.radiance.x;
      L.y += throughput.y * v.radiance.y;
      L.z += throughput.z * v.radiance.z;
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
    if (clampLum > 0.0f) {
      const float lum = 0.2126f * L.x + 0.7152f * L.y + 0.0722f * L.z;
      if (lum > clampLum) {
        const float k = clampLum / lum;
        L.x *= k;
        L.y *= k;
        L.z *= k;
      }
    }
    Esum.x += L.x;
    Esum.y += L.y;
    Esum.z += L.z;
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
                          const ProgressSlice* prog = nullptr) {
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  E.assign(npix * 3, 0.0f);
  occ.assign(npix, 0.0f);
  const uint32_t seedMix = hashU32(frameSeed);
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
            const Vec3 e = pt1GatherPoint(p, P, N, Ng, spp, seed, tEps, &o,
                                          ld, clampLum,
                                          stats ? &local : nullptr);
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
