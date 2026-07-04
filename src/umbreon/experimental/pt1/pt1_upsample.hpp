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

#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "experimental/pt1/pt1_integrator.hpp"

namespace umbreon {
namespace detail {

// Upsample halfE (and the matching hit-fraction halfOcc) from the low G-buffer
// grid to the full W x H grid (any integer or non-integer scale: both grids
// sample pixel centers of the same normalized plane, so the continuous
// projection below is exact for every ratio). fullNormal/fullDepth are the
// render AOVs; fullEligible marks the gather-eligible (mesh-hit) full pixels
// -- others stay zero. Low pixels with hit == 0 get zero weight. When every
// neighbor weight dies (< 1e-6 total; disagreeing edge pixels), fall back to
// the nearest valid low pixel of the 2x2 quad. When the WHOLE quad is invalid
// (a thin feature the low grid missed entirely) and fallbackRadius > 0, search
// the (2r+1)^2 window for the nearest valid low pixel -- a rare path that only
// coarse grids (divisor >= 3) enable; fallbackRadius == 0 keeps the historical
// quad-only behavior byte-identical. Pixels that STILL find nothing stay zero
// and are counted into *outHoles (diagnostic).
inline void upsampleJointBilateral(int W, int H, const float* fullNormal,
                                   const float* fullDepth,
                                   const uint8_t* fullEligible,
                                   const Pt1GBuffer& half,
                                   const std::vector<float>& halfE,
                                   const std::vector<float>& halfOcc,
                                   float normalPow, float depthScale,
                                   std::vector<float>& outE,
                                   std::vector<float>& outOcc,
                                   int fallbackRadius = 0,
                                   std::atomic<uint64_t>* outHoles = nullptr,
                                   std::vector<uint8_t>* outNeedsPatch =
                                       nullptr,
                                   float patchWeightThresh = 0.0f) {
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  outE.assign(npix * 3, 0.0f);
  outOcc.assign(npix, 0.0f);
  // Optional per-pixel "no compatible low-res sample" mask: set wherever the
  // guide weights died (silhouette rims whose surface the low grid never
  // sampled at this locale) so the caller can re-gather exactly those pixels
  // at full resolution instead of accepting a copied/zero E.
  if (outNeedsPatch) outNeedsPatch->assign(npix, 0);
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
          // Mark LOW-CONFIDENCE pixels for the full-res patch too: the guide
          // weights survive but carry little support (grazing silhouettes
          // whose compatible neighbors sit slightly off-surface), which is
          // where the upsampled rim drifts from the converged reference.
          if (outNeedsPatch && wSum < patchWeightThresh)
            (*outNeedsPatch)[pix] = 1;
          const float inv = 1.0f / wSum;
          outE[pix * 3 + 0] = eSum.x * inv;
          outE[pix * 3 + 1] = eSum.y * inv;
          outE[pix * 3 + 2] = eSum.z * inv;
          outOcc[pix] = occSum * inv;
        } else if (bestPix != static_cast<std::size_t>(-1)) {
          if (outNeedsPatch) (*outNeedsPatch)[pix] = 1;
          // Edge fallback: every guide weight died -- take the nearest valid
          // half pixel verbatim rather than leaving a hole.
          outE[pix * 3 + 0] = halfE[bestPix * 3 + 0];
          outE[pix * 3 + 1] = halfE[bestPix * 3 + 1];
          outE[pix * 3 + 2] = halfE[bestPix * 3 + 2];
          outOcc[pix] = halfOcc[bestPix];
        } else {
          if (outNeedsPatch) (*outNeedsPatch)[pix] = 1;
          // The whole 2x2 quad is invalid: the low grid missed this feature.
          // Widened fallback (coarse grids only): nearest valid low pixel by
          // grid distance within the (2r+1)^2 window around the quad.
          std::size_t foundPix = static_cast<std::size_t>(-1);
          if (fallbackRadius > 0) {
            float bestD2 = 1e30f;
            for (int wy = by - fallbackRadius + 1;
                 wy <= by + fallbackRadius; ++wy) {
              if (wy < 0 || wy >= hh) continue;
              for (int wx = bx - fallbackRadius + 1;
                   wx <= bx + fallbackRadius; ++wx) {
                if (wx < 0 || wx >= hw) continue;
                const std::size_t hpix =
                    static_cast<std::size_t>(wy) * hw + wx;
                if (!half.hit[hpix]) continue;
                const float ddx = static_cast<float>(wx) - sx;
                const float ddy = static_cast<float>(wy) - sy;
                const float d2 = ddx * ddx + ddy * ddy;
                if (d2 < bestD2) {
                  bestD2 = d2;
                  foundPix = hpix;
                }
              }
            }
          }
          if (foundPix != static_cast<std::size_t>(-1)) {
            outE[pix * 3 + 0] = halfE[foundPix * 3 + 0];
            outE[pix * 3 + 1] = halfE[foundPix * 3 + 1];
            outE[pix * 3 + 2] = halfE[foundPix * 3 + 2];
            outOcc[pix] = halfOcc[foundPix];
          } else if (outHoles) {
            outHoles->fetch_add(1, std::memory_order_relaxed);
          }
        }
      }
    }
  });
}

// Guided local smoothing of the PATCHED pixels only: each masked pixel's E
// (and occ) is re-estimated as the guide-weighted mean of its (2r+1)^2
// neighborhood in the FULL-res E field -- the same normal/depth edge-stops as
// the upsample, plus the center's own value at full weight. Patched pixels
// carry raw (un-denoised) Monte-Carlo variance; their compatible neighbors
// are mostly denoised upsampled values, so this pulls the rim toward the
// smooth field where the guides agree and keeps the raw value where they do
// not (no cross-silhouette leakage). Reads from a snapshot so the result is
// order-independent and deterministic. Cost is proportional to the patched
// pixel count (0.1-0.2% of the frame).
inline void smoothPatchedPixels(int W, int H, const std::vector<uint8_t>& mask,
                                const float* fullNormal, const float* fullDepth,
                                const uint8_t* fullEligible, float normalPow,
                                float depthScale, int radius,
                                std::vector<float>& E, std::vector<float>& occ) {
  const std::vector<float> Esrc = E;      // snapshot (order independence)
  const std::vector<float> occSrc = occ;
  tbb::parallel_for(tbb::blocked_range<int>(0, H),
                    [&](const tbb::blocked_range<int>& rows) {
    for (int py = rows.begin(); py != rows.end(); ++py) {
      for (int px = 0; px < W; ++px) {
        const std::size_t pix = static_cast<std::size_t>(py) * W + px;
        if (!mask[pix]) continue;
        const Vec3 Nf{fullNormal[pix * 3 + 0], fullNormal[pix * 3 + 1],
                      fullNormal[pix * 3 + 2]};
        const float zf = fullDepth[pix];
        float wSum = 1.0f;  // self at full weight
        Vec3 eSum{Esrc[pix * 3 + 0], Esrc[pix * 3 + 1], Esrc[pix * 3 + 2]};
        float occSum = occSrc[pix];
        for (int dy = -radius; dy <= radius; ++dy) {
          const int ny = py + dy;
          if (ny < 0 || ny >= H) continue;
          for (int dx = -radius; dx <= radius; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int nx = px + dx;
            if (nx < 0 || nx >= W) continue;
            const std::size_t npix2 = static_cast<std::size_t>(ny) * W + nx;
            if (!fullEligible[npix2]) continue;
            const Vec3 Nn{fullNormal[npix2 * 3 + 0], fullNormal[npix2 * 3 + 1],
                          fullNormal[npix2 * 3 + 2]};
            const float wn =
                std::pow(std::fmax(0.0f, dot(Nf, Nn)), normalPow);
            const float zn = fullDepth[npix2];
            const float wz =
                std::exp(-std::fabs(zf - zn) / (depthScale * zf + 1e-6f));
            const float w = wn * wz;
            if (w < 1e-4f) continue;
            wSum += w;
            eSum.x += w * Esrc[npix2 * 3 + 0];
            eSum.y += w * Esrc[npix2 * 3 + 1];
            eSum.z += w * Esrc[npix2 * 3 + 2];
            occSum += w * occSrc[npix2];
          }
        }
        const float inv = 1.0f / wSum;
        E[pix * 3 + 0] = eSum.x * inv;
        E[pix * 3 + 1] = eSum.y * inv;
        E[pix * 3 + 2] = eSum.z * inv;
        occ[pix] = occSum * inv;
      }
    }
  });
}

}  // namespace detail
}  // namespace umbreon
