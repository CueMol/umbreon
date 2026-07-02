// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Joint bilateral upsampling of the pt1 half-res indirect irradiance to the
// render grid, guided by the full-res normal/depth AOVs (Kopf 2007 style).
// Per full-res pixel the 2x2 half-res neighborhood is blended with
//   w = w_bilinear * max(0, dot(Nf, Nh))^normalPow
//                  * exp(-|zf - zh| / (depthScale * zf + 1e-6))
// so the smooth interior interpolates while normal/depth discontinuities
// (object silhouettes, creases) keep the irradiance from leaking across.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "experimental/pt1/pt1_integrator.hpp"

namespace umbreon {
namespace detail {

// Upsample halfE (and the matching hit-fraction halfOcc) from the half G-buffer
// grid to the full W x H grid. fullNormal/fullDepth are the render AOVs;
// fullEligible marks the gather-eligible (mesh-hit) full pixels -- others stay
// zero. Half pixels with hit == 0 get zero weight. When every neighbor weight
// dies (< 1e-6 total; disagreeing edge pixels), fall back to the nearest valid
// half pixel; with no valid neighbor at all the output stays zero (a thin
// feature the half grid missed entirely -- accepted, documented limitation).
inline void upsampleJointBilateral(int W, int H, const float* fullNormal,
                                   const float* fullDepth,
                                   const uint8_t* fullEligible,
                                   const Pt1GBuffer& half,
                                   const std::vector<float>& halfE,
                                   const std::vector<float>& halfOcc,
                                   float normalPow, float depthScale,
                                   std::vector<float>& outE,
                                   std::vector<float>& outOcc) {
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  outE.assign(npix * 3, 0.0f);
  outOcc.assign(npix, 0.0f);
  const int hw = half.w, hh = half.h;

  tbb::parallel_for(tbb::blocked_range<int>(0, H),
                    [&](const tbb::blocked_range<int>& rows) {
    for (int py = rows.begin(); py != rows.end(); ++py) {
      // Continuous half-grid coordinate of this full pixel's center (the two
      // grids share the same normalized [0,1] image plane).
      const float sy = (static_cast<float>(py) + 0.5f) * hh / H - 0.5f;
      const int by = static_cast<int>(std::floor(sy));
      const float fy = sy - static_cast<float>(by);
      for (int px = 0; px < W; ++px) {
        const std::size_t pix = static_cast<std::size_t>(py) * W + px;
        if (!fullEligible[pix]) continue;
        const float sx = (static_cast<float>(px) + 0.5f) * hw / W - 0.5f;
        const int bx = static_cast<int>(std::floor(sx));
        const float fx = sx - static_cast<float>(bx);

        const Vec3 Nf{fullNormal[pix * 3 + 0], fullNormal[pix * 3 + 1],
                      fullNormal[pix * 3 + 2]};
        const float zf = fullDepth[pix];

        float wSum = 0.0f;
        Vec3 eSum{0.0f, 0.0f, 0.0f};
        float occSum = 0.0f;
        float bestBilin = -1.0f;
        std::size_t bestPix = static_cast<std::size_t>(-1);
        for (int dy = 0; dy < 2; ++dy) {
          for (int dx = 0; dx < 2; ++dx) {
            int hx = bx + dx, hy = by + dy;
            if (hx < 0) hx = 0;
            if (hy < 0) hy = 0;
            if (hx >= hw) hx = hw - 1;
            if (hy >= hh) hy = hh - 1;
            const std::size_t hpix = static_cast<std::size_t>(hy) * hw + hx;
            if (!half.hit[hpix]) continue;
            const float wb = (dx ? fx : 1.0f - fx) * (dy ? fy : 1.0f - fy);
            if (wb > bestBilin) {
              bestBilin = wb;
              bestPix = hpix;
            }
            const Vec3 Nh{half.normal[hpix * 3 + 0], half.normal[hpix * 3 + 1],
                          half.normal[hpix * 3 + 2]};
            const float wn =
                std::pow(std::fmax(0.0f, dot(Nf, Nh)), normalPow);
            const float zh = half.depth[hpix];
            const float wz =
                std::exp(-std::fabs(zf - zh) / (depthScale * zf + 1e-6f));
            const float w = wb * wn * wz;
            wSum += w;
            eSum.x += w * halfE[hpix * 3 + 0];
            eSum.y += w * halfE[hpix * 3 + 1];
            eSum.z += w * halfE[hpix * 3 + 2];
            occSum += w * halfOcc[hpix];
          }
        }

        if (wSum >= 1e-6f) {
          const float inv = 1.0f / wSum;
          outE[pix * 3 + 0] = eSum.x * inv;
          outE[pix * 3 + 1] = eSum.y * inv;
          outE[pix * 3 + 2] = eSum.z * inv;
          outOcc[pix] = occSum * inv;
        } else if (bestPix != static_cast<std::size_t>(-1)) {
          // Edge fallback: every guide weight died -- take the nearest valid
          // half pixel verbatim rather than leaving a hole.
          outE[pix * 3 + 0] = halfE[bestPix * 3 + 0];
          outE[pix * 3 + 1] = halfE[bestPix * 3 + 1];
          outE[pix * 3 + 2] = halfE[bestPix * 3 + 2];
          outOcc[pix] = halfOcc[bestPix];
        }
      }
    }
  });
}

}  // namespace detail
}  // namespace umbreon
