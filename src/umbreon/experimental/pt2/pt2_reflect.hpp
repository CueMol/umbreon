// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt2 traced mirror reflection (full-PT track, stage 1): for every first-hit
// pixel whose material carries POV `reflection`, trace ONE mirror ray and
// composite  color += reflection * L(mirror)  where L is the reflected
// surface's radiance (shadow-tested NEE direct + its share of the gathered
// sky) or the gather sky on escape. Replaces shadeLocal's fake
// reflection * background term (shadeLocal skips it when the pass owns the
// pixel), so reflective surfaces mirror actual geometry instead of the
// background color.
//
// POV reflection is an ideal mirror (no cone), so a single deterministic ray
// per pixel is exact for the direct part -- no spp, no noise, bit-exact
// across thread counts by construction. The reflected surface's own indirect
// is approximated by the sky term only (one-bounce cut, consistent with the
// gather's energy budget; the alternative -- a full gather per reflective
// pixel -- costs a second GI pass for a second-order effect).
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "experimental/pt1/pt1_gather.hpp"
#include "render/progress_slice.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// Composite traced reflections into res.color at the RENDER grid. `reflAmt`
// is the per-pixel first-hit Material::reflection (0 = skip), position/normal
// the first-hit G-buffer, depth the primary hit distance (selfIntersectEps
// scale). `camPos`/`camDir`+ortho reconstruct the incident view ray per pixel
// exactly as the primary loop generated it.
inline void pt2ReflectPass(const IrradianceCacheParams& p, int W, int H,
                           const std::vector<float>& reflAmt,
                           const float* position, const float* normal,
                           const float* depth, bool orthographic,
                           const Vec3& camPos, const Vec3& camDir,
                           bool emissive, FrameResult& res,
                           const ProgressSlice* prog) {
  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
        if (prog && prog->cancelled()) return;
        for (int py = r.rows().begin(); py != r.rows().end(); ++py) {
          for (int px = r.cols().begin(); px != r.cols().end(); ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            const float k = reflAmt[pix];
            if (k <= 0.0f) continue;
            const Vec3 P{position[pix * 3 + 0], position[pix * 3 + 1],
                         position[pix * 3 + 2]};
            const Vec3 N{normal[pix * 3 + 0], normal[pix * 3 + 1],
                         normal[pix * 3 + 2]};
            // Incident direction: ortho rays share camDir; perspective rays
            // point from the camera to the hit.
            Vec3 wi;
            if (orthographic) {
              wi = camDir;
            } else {
              wi = safeNormalize(Vec3{P.x - camPos.x, P.y - camPos.y,
                                      P.z - camPos.z});
            }
            const float indot = dot(wi, N);
            const Vec3 rdir = safeNormalize(Vec3{wi.x - 2.0f * indot * N.x,
                                                 wi.y - 2.0f * indot * N.y,
                                                 wi.z - 2.0f * indot * N.z});
            const float t0 = (depth && depth[pix] > 0.0f) ? depth[pix] : 1.0f;
            const float eps = selfIntersectEps(P, rdir, t0);
            const Vec3 O{P.x + N.x * eps, P.y + N.y * eps, P.z + N.z * eps};

            const RTCRayHit rh =
                intersectFull(p.scene, O, rdir, eps,
                              std::numeric_limits<float>::infinity());
            Vec3 L{0.0f, 0.0f, 0.0f};
            if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
              L = environmentRadiance(p, rdir);
            } else {
              // Per-pixel deterministic stream for the area-light NEE jitter
              // at the reflected hit (mirrors the gather's per-path scheme).
              uint32_t rs0 = static_cast<uint32_t>(pix) ^ 0x52464c54u;
              uint32_t rs1 = 0x4e454531u;
              tea2(rs0, rs1);
              Pt1Vertex v;
              if (pt1EvalVertex(p, rh, O, rdir, v, nullptr, emissive, &rs0,
                                &rs1)) {
                // Direct (NEE, shadow-tested) + the surface's share of the
                // sky (albedo * E_sky ~ one-bounce ambient approximation).
                const Vec3 sky = environmentRadiance(p, rdir);
                L = Vec3{v.radiance.x + v.albedo.x * sky.x,
                         v.radiance.y + v.albedo.y * sky.y,
                         v.radiance.z + v.albedo.z * sky.z};
              }
            }
            res.color[pix * 4 + 0] += k * L.x;
            res.color[pix * 4 + 1] += k * L.y;
            res.color[pix * 4 + 2] += k * L.z;
          }
        }
        if (prog)
          prog->addWork(
              static_cast<std::uint64_t>(r.rows().end() - r.rows().begin()) *
                  static_cast<std::uint64_t>(r.cols().end() -
                                             r.cols().begin()),
              static_cast<std::uint64_t>(W) * static_cast<std::uint64_t>(H));
      });
}

}  // namespace detail
}  // namespace umbreon
