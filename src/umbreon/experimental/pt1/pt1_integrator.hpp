// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt1: experimental path-traced indirect diffuse integrator (one bounce,
// per-pixel brute-force cosine-hemisphere gather). An ALTERNATIVE to the
// irradiance cache post-pass: the main render loop keeps computing direct
// lighting unchanged, and pt1 replaces only the cache's placement/fill/
// interpolation stages with a per-pixel gather using the SAME radiance
// evaluators (oneBounceRadiance / environmentRadiance), so the two integrators
// are unit-compatible for A/B comparison by construction.
//
// Energy convention (matches the cache, see irradiance_cache.hpp): the E
// buffer stores E_stored = mean(L_i) over cosine-weighted samples = E_true/pi,
// and the composite multiplies by the receiver reflectance kd*pigment WITHOUT
// a 1/pi. The plan's estimator (pi/N)*sum(L_i) maps to E_stored=(1/N)*sum(L_i).
//
// Sky/emission division (double-counting guards):
//   - The sky enters ONLY through the gather's miss term (environmentRadiance);
//     the direct pass has no sky. Env-dome lights (--env-light) are DIRECT
//     distant lights, so combining them with any GI integrator double-counts
//     the sky energy -- the renderer warns on that combination.
//   - A gather-ray hit contributes its REFLECTED direct light only (no
//     emission): light travelling from a source straight to the receiver is
//     already counted by the direct pass, so adding emission at the bounce
//     point would double-count it. oneBounceRadiance implements exactly this.
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
#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// Camera basis + image-plane half extents, precomputed once by render() (the
// same math as the main pixel loop) so the pt1 half-res pass can shoot its own
// primary rays through an identical camera.
struct Pt1CameraBasis {
  Vec3 position{0.0f, 0.0f, 0.0f};
  Vec3 dir{0.0f, 0.0f, -1.0f};     // normalized view direction
  Vec3 right{1.0f, 0.0f, 0.0f};
  Vec3 trueUp{0.0f, 1.0f, 0.0f};
  bool orthographic = false;
  float halfW = 1.0f, halfH = 1.0f;          // orthographic half extents
  float persHalfW = 1.0f, persHalfH = 1.0f;  // perspective, at unit distance
};

// Per-pixel ray counters, accumulated as plain integers inside the gather
// loops and flushed ONCE per pixel into the atomic totals (per-ray atomics
// would cost ~0.1-0.3 s at the 50M+ rays of the high preset).
struct Pt1RayStatsLocal {
  uint64_t gatherRays = 0;   // intersectFull calls (all bounces)
  uint64_t gatherHits = 0;   // of which hit geometry
  uint64_t neeRays = 0;      // NEE shadow rays (rtcOccluded1)
  uint64_t neeOccluded = 0;  // of which occluded
};

struct Pt1RayStats {
  std::atomic<uint64_t> gatherRays{0};
  std::atomic<uint64_t> gatherHits{0};
  std::atomic<uint64_t> neeRays{0};
  std::atomic<uint64_t> neeOccluded{0};
  std::atomic<uint64_t> gbufferRays{0};  // half-res G-buffer primary rays
  void flush(const Pt1RayStatsLocal& l) {
    gatherRays.fetch_add(l.gatherRays, std::memory_order_relaxed);
    gatherHits.fetch_add(l.gatherHits, std::memory_order_relaxed);
    neeRays.fetch_add(l.neeRays, std::memory_order_relaxed);
    neeOccluded.fetch_add(l.neeOccluded, std::memory_order_relaxed);
  }
};

// First-hit G-buffer for an independently traced (typically half-res) grid.
// Normals are face-forwarded toward the viewer; misses and outline-decoration
// hits have hit == 0 with zeroed position/normal/albedo (a zero normal marks
// background for the OIDN guide). Mesh hits AND real CSG primitives (atom
// balls / bonds, fromEdge == 0) are gather-eligible.
struct Pt1GBuffer {
  int w = 0, h = 0;
  std::vector<float> position;    // w*h*3 world-space first hit
  std::vector<float> normal;      // w*h*3 shading normal (face-forward)
  std::vector<float> geomNormal;  // w*h*3 geometric normal (face-forward)
  std::vector<float> albedo;      // w*h*3 kd * pigment (OIDN guide)
  std::vector<float> depth;       // w*h   ray distance from camera
  std::vector<uint8_t> hit;       // w*h   1 = mesh hit (gather-eligible)
};

// Trace one primary ray per pixel center of a w x h grid through `basis` and
// fill the G-buffer. The grid's normalized [-1,1] plane coordinates match the
// main render grid's, so a (W+1)/2 x (H+1)/2 grid samples the centers of the
// full grid's 2x2 pixel blocks.
inline void tracePt1GBuffer(const IrradianceCacheParams& p,
                            const Pt1CameraBasis& cam, int w, int h,
                            Pt1GBuffer& g, Pt1RayStats* stats = nullptr) {
  const std::size_t npix = static_cast<std::size_t>(w) * h;
  g.w = w;
  g.h = h;
  g.position.assign(npix * 3, 0.0f);
  g.normal.assign(npix * 3, 0.0f);
  g.geomNormal.assign(npix * 3, 0.0f);
  g.albedo.assign(npix * 3, 0.0f);
  g.depth.assign(npix, 0.0f);
  g.hit.assign(npix, 0);

  if (stats)
    stats->gbufferRays.fetch_add(npix, std::memory_order_relaxed);

  tbb::parallel_for(tbb::blocked_range<int>(0, h),
                    [&](const tbb::blocked_range<int>& rows) {
    for (int py = rows.begin(); py != rows.end(); ++py) {
      const float v =
          1.0f - 2.0f * (static_cast<float>(py) + 0.5f) / static_cast<float>(h);
      for (int px = 0; px < w; ++px) {
        const float u =
            2.0f * (static_cast<float>(px) + 0.5f) / static_cast<float>(w) -
            1.0f;
        Vec3 org, rd;
        if (cam.orthographic) {
          org = cam.position + cam.right * (u * cam.halfW) +
                cam.trueUp * (v * cam.halfH);
          rd = cam.dir;
        } else {
          org = cam.position;
          rd = normalize(cam.dir + cam.right * (u * cam.persHalfW) +
                         cam.trueUp * (v * cam.persHalfH));
        }

        const RTCRayHit rh = intersectFull(
            p.scene, org, rd, 0.0f, std::numeric_limits<float>::infinity());
        if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) continue;
        const GeomRecord& rec = p.built->records[rh.hit.geomID];

        Vec3 N, Ng;
        float albedo[3];
        if (rec.kind == GeomKind::Mesh) {
          // Shading normal / pigment from the mesh vertex attributes (same
          // slots as oneBounceRadiance: 0 = normal, 1 = color).
          float nbuf[3] = {0, 0, 0};
          float cbuf[4] = {0, 0, 0, 1};
          rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                          RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
          rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                          RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
          N = safeNormalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
          Ng = safeNormalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z});
          const float kd = p.mesh->materialForTri(rh.hit.primID).diffuse;
          albedo[0] = kd * cbuf[0];
          albedo[1] = kd * cbuf[1];
          albedo[2] = kd * cbuf[2];
        } else {
          // Real CSG primitive (atom ball / bond): analytic normal from the
          // Embree Ng, color/material from the primID side-tables. Outline
          // decoration (fromEdge) is not a GI receiver -- leave hit == 0.
          const BuiltScene& b = *p.built;
          const bool isSphere = (rec.kind == GeomKind::Sphere);
          const bool isCapped = (rec.kind == GeomKind::CylinderCapped);
          const bool fromEdge =
              isSphere ? (!b.sphereFromEdge.empty() &&
                          b.sphereFromEdge[rh.hit.primID])
              : isCapped ? (!b.cylCapFromEdge.empty() &&
                            b.cylCapFromEdge[rh.hit.primID])
                         : (!b.cylFromEdge.empty() &&
                            b.cylFromEdge[rh.hit.primID]);
          if (fromEdge) continue;
          const Vec4& fc = isSphere ? b.sphereColor[rh.hit.primID]
                           : isCapped ? b.cylCapColor[rh.hit.primID]
                                      : b.cylColor[rh.hit.primID];
          const Material& pm = isSphere ? b.sphereMat[rh.hit.primID]
                               : isCapped ? b.cylCapMat[rh.hit.primID]
                                          : b.cylMat[rh.hit.primID];
          N = safeNormalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z});
          Ng = N;
          albedo[0] = pm.diffuse * fc.x;
          albedo[1] = pm.diffuse * fc.y;
          albedo[2] = pm.diffuse * fc.z;
        }
        // Face-forward toward the viewer (dot(N, -rd) < 0 => flip).
        if (dot(N, rd) > 0.0f) N = Vec3{-N.x, -N.y, -N.z};
        if (dot(Ng, rd) > 0.0f) Ng = Vec3{-Ng.x, -Ng.y, -Ng.z};

        const std::size_t pix = static_cast<std::size_t>(py) * w + px;
        const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                     org.z + rd.z * rh.ray.tfar};
        g.position[pix * 3 + 0] = P.x;
        g.position[pix * 3 + 1] = P.y;
        g.position[pix * 3 + 2] = P.z;
        g.normal[pix * 3 + 0] = N.x;
        g.normal[pix * 3 + 1] = N.y;
        g.normal[pix * 3 + 2] = N.z;
        g.geomNormal[pix * 3 + 0] = Ng.x;
        g.geomNormal[pix * 3 + 1] = Ng.y;
        g.geomNormal[pix * 3 + 2] = Ng.z;
        g.albedo[pix * 3 + 0] = albedo[0];
        g.albedo[pix * 3 + 1] = albedo[1];
        g.albedo[pix * 3 + 2] = albedo[2];
        g.depth[pix] = rh.ray.tfar;
        g.hit[pix] = 1;
      }
    }
  });
}

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
                          Pt1RayStats* stats = nullptr) {
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  E.assign(npix * 3, 0.0f);
  occ.assign(npix, 0.0f);
  const uint32_t seedMix = hashU32(frameSeed);
  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
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
      });
}

// Denoise the E buffer (w*h*3 indirect irradiance, pre-composite) in place:
// NaN/Inf scrub, then OIDN with albedo/normal guides (a-trous fallback when
// the build has no OIDN). albedo is the receiver reflectance kd*pigment in
// [0,1] (the giRefl side-channel at full res, the pt1 G-buffer at half res);
// normal marks background with zero vectors; position feeds the a-trous depth
// edge-stop. Any guide may be null. Defined in pt1_denoise.cpp (NOT inline:
// UMBREON_HAVE_OIDN is a target-private macro, see there).
void denoisePt1E(int w, int h, std::vector<float>& E, const float* albedo,
                 const float* normal, const float* position,
                 const RenderOptions& opt);

}  // namespace detail
}  // namespace umbreon
