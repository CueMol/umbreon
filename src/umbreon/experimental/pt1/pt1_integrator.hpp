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

#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range2d.h>

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

// First-hit G-buffer for an independently traced (typically half-res) grid.
// Normals are face-forwarded toward the viewer; non-mesh hits and misses have
// hit == 0 with zeroed position/normal/albedo (a zero normal marks background
// for the OIDN guide).
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
                            Pt1GBuffer& g) {
  const std::size_t npix = static_cast<std::size_t>(w) * h;
  g.w = w;
  g.h = h;
  g.position.assign(npix * 3, 0.0f);
  g.normal.assign(npix * 3, 0.0f);
  g.geomNormal.assign(npix * 3, 0.0f);
  g.albedo.assign(npix * 3, 0.0f);
  g.depth.assign(npix, 0.0f);
  g.hit.assign(npix, 0);

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
        if (rec.kind != GeomKind::Mesh) continue;  // gather is mesh-only

        // Shading normal / pigment from the mesh vertex attributes (same
        // slots as oneBounceRadiance: 0 = normal, 1 = color).
        float nbuf[3] = {0, 0, 0};
        float cbuf[4] = {0, 0, 0, 1};
        rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                        RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
        rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                        RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
        Vec3 N = safeNormalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
        Vec3 Ng = safeNormalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z});
        // Face-forward toward the viewer (dot(N, -rd) < 0 => flip).
        if (dot(N, rd) > 0.0f) N = Vec3{-N.x, -N.y, -N.z};
        if (dot(Ng, rd) > 0.0f) Ng = Vec3{-Ng.x, -Ng.y, -Ng.z};

        const Material& mat = p.mesh->materialForTri(rh.hit.primID);
        const float kd = mat.diffuse;

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
        g.albedo[pix * 3 + 0] = kd * cbuf[0];
        g.albedo[pix * 3 + 1] = kd * cbuf[1];
        g.albedo[pix * 3 + 2] = kd * cbuf[2];
        g.depth[pix] = rh.ray.tfar;
        g.hit[pix] = 1;
      }
    }
  });
}

// One-bounce indirect irradiance at shading point P with shading normal N and
// geometric normal Ng, by brute-force cosine-hemisphere gather (the estimator
// the irradiance cache approximates, evaluated per pixel).
//
// Returns E_stored = (1/spp) * sum(L_i) = E_true/pi (the cache's convention;
// the composite multiplies by kd*pigment with no 1/pi). Per sample:
//   hit  -> L_i = oneBounceRadiance (reflected shadow-tested direct light at
//           the bounce point; black for non-mesh hits; NO emission -- source-
//           to-receiver light is already counted by the direct pass)
//   miss -> L_i = environmentRadiance (the sky lives ONLY in this miss term)
// A sample below the geometric horizon (dot(wi, Ng) <= 0, possible when the
// shading normal diverges from Ng) contributes 0 but still counts in the
// divisor, and counts as occluded for `outOcclusion` (the surface itself
// blocks that direction).
//
// epsT is a finite length scale for selfIntersectEps (e.g. the scene AABB
// diagonal); it must NOT be the gather tfar, which pt1 defaults to infinity.
// The RNG is tea2 seeded from (seed, sample index) only -- deterministic and
// thread-count independent, no shared state.
inline Vec3 pt1GatherPoint(const IrradianceCacheParams& p, const Vec3& P,
                           const Vec3& N, const Vec3& Ng, int spp,
                           uint32_t seed, float epsT, float* outOcclusion) {
  const Frame f = frameFromNormal(N);
  const float eps = selfIntersectEps(P, N, epsT);
  const Vec3 O = P + N * eps;
  Vec3 E{0.0f, 0.0f, 0.0f};
  int nOccluded = 0;
  for (int s = 0; s < spp; ++s) {
    uint32_t s0 = seed;
    uint32_t s1 = static_cast<uint32_t>(s);
    tea2(s0, s1);
    const Vec3 wi = cosineSampleHemisphere(u32ToUnorm(s0), u32ToUnorm(s1), f);
    if (dot(wi, Ng) <= 0.0f) {
      ++nOccluded;
      continue;
    }
    const RTCRayHit rh = intersectFull(p.scene, O, wi, eps, p.maxDistance);
    if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      E = E + oneBounceRadiance(p, rh, O, wi, nullptr);
      ++nOccluded;
    } else {
      E = E + environmentRadiance(p, wi);
    }
  }
  const float invN = 1.0f / static_cast<float>(spp > 0 ? spp : 1);
  if (outOcclusion) *outOcclusion = static_cast<float>(nOccluded) * invN;
  return Vec3{E.x * invN, E.y * invN, E.z * invN};
}

// Gather E_stored for every gather-eligible pixel of a W x H grid into `E`
// (W*H*3) and the hit fraction into `occ` (W*H). position/normal are the
// first-hit AOVs; geomNormal may be null (full-res mode: the render G-buffer
// has no geometric normal, so the horizon guard degenerates to the shading
// normal, which cosine sampling already satisfies) or the pt1 G-buffer's
// geometric normal (half-res mode: the real guard). skip[pix] != 0 marks
// gather-eligible (mesh-hit) pixels; others stay zero.
//
// TBB 16x16 tiles; each pixel is seeded from (pixel index, frameSeed) only, so
// the image is deterministic across thread counts. Tiles write disjoint pixel
// ranges (no locks).
inline void gatherPt1Grid(const IrradianceCacheParams& p, int W, int H,
                          const float* position, const float* normal,
                          const float* geomNormal, const uint8_t* eligible,
                          int spp, uint32_t frameSeed, float epsT,
                          std::vector<float>& E, std::vector<float>& occ) {
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
            float o = 0.0f;
            const Vec3 e = pt1GatherPoint(p, P, N, Ng, spp, seed, epsT, &o);
            E[pix * 3 + 0] = e.x;
            E[pix * 3 + 1] = e.y;
            E[pix * 3 + 2] = e.z;
            occ[pix] = o;
          }
        }
      });
}

}  // namespace detail
}  // namespace umbreon
