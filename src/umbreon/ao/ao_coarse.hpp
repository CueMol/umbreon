// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Coarse-grid AO (--ao-res out): evaluate the AO gather ONCE per coarse cell
// (output resolution, i.e. the hi-res render grid divided by the supersample
// factor) on a low-res first-hit G-buffer, and let every shading hit look the
// result up with a normal/depth-guided bilateral filter over its 2x2
// surrounding cells. Hits whose guides reject the interpolation (silhouette
// rims, transparency layers behind the front surface, sub-cell features the
// coarse grid stepped over) fall back to the exact inline gather at the call
// site, and the supersample box-average denoises those pixels exactly like the
// full-grid render. Mirrors the pt1 gather-grid decoupling (reduced-grid
// gather + joint-bilateral upsample), but the "upsample" is lazy and per-hit:
// each hit is its own guide (it knows its exact N and camera distance), so no
// full-res guide buffers or ordering pre-pass are needed.
//
// AO ray budget drops from W*H*aoSamples (hi-res) to (W/div)*(H/div)*aoSamples
// plus the inline-fallback pixels -- ~1/div^2 for the smooth interior.
// Deterministic: the coarse gather seeds from the coarse lattice only
// (base = cx + cy*w), the fallback keeps the unchanged hi-res seeds, and the
// lookup is a pure function of the hit -- thread-count invariant like the rest
// of the renderer.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "ao/ao_shade.hpp"
#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// Bilateral lookup constants, shared shape/values with the pt1 joint-bilateral
// upsample (pt1_upsample.hpp): normal weight pow(dot, 32), depth weight
// exp(-|dz|/(0.02*z)), and the pt1EdgePatchThresh default 0.3 as the minimum
// accepted weight sum (well-supported interior pixels sum to ~1).
constexpr float kAoUpsampleNormalPow = 32.0f;
constexpr float kAoUpsampleDepthScale = 0.02f;
constexpr float kAoPatchThresh = 0.3f;

// The coarse AO grid: gathered AO channels plus the lookup guides, all at the
// coarse (output) resolution. Both grids sample pixel CENTERS of the same
// normalized [-1,1] camera plane (the pt1 reduced-grid contract), so the
// continuous mapping between lattices is exact for any size ratio.
// bent/contact/shape/avgHitDist are allocated only when the quality estimator
// runs (aoEnhanced() or aoWriteAov) -- the legacy binary path needs only
// openness, and the empty vectors keep the memory footprint down.
struct CoarseAoGrid {
  int w = 0, h = 0;
  std::vector<float> openness;    // w*h
  std::vector<float> bent;        // w*h*3 (quality gather only)
  std::vector<float> contact;     // w*h   (quality gather only)
  std::vector<float> shape;       // w*h   (quality gather only)
  std::vector<float> avgHitDist;  // w*h   (quality gather only)
  std::vector<float> normal;      // w*h*3 lookup guide (shading normal)
  std::vector<float> depth;       // w*h   lookup guide (primary tfar)
  std::vector<uint8_t> hit;       // w*h   1 = AO-eligible first hit
};

// Gather AO at every eligible cell of a low-res first-hit G-buffer (the
// position/normal/geomNormal/depth/hit planes of a Pt1GBuffer, passed as raw
// pointers so ao/ stays free of experimental/ includes). Runs the SAME
// estimator selection as the inline path (aoGather: legacy vs quality, incl.
// the aoWriteAov extra pass), with nSamples = opt.aoSamples as-is and the RNG
// seeded from the COARSE lattice (px=cx, py=cy, wSeed=w) -- a pure function of
// the cell, thread-count invariant.
inline void buildCoarseAoGrid(RTCScene rscene, const RenderOptions& opt, int w,
                              int h, const float* position, const float* normal,
                              const float* geomNormal, const float* depth,
                              const uint8_t* hit, CoarseAoGrid& out) {
  const std::size_t npix = static_cast<std::size_t>(w) * h;
  out.w = w;
  out.h = h;
  out.openness.assign(npix, 1.0f);
  const bool quality = opt.aoEnhanced() || opt.aoWriteAov;
  if (quality) {
    out.bent.assign(npix * 3, 0.0f);
    out.contact.assign(npix, 1.0f);
    out.shape.assign(npix, 1.0f);
    out.avgHitDist.assign(npix, 0.0f);
  }
  out.normal.assign(normal, normal + npix * 3);
  out.depth.assign(depth, depth + npix);
  out.hit.assign(hit, hit + npix);

  tbb::parallel_for(tbb::blocked_range<int>(0, h),
                    [&](const tbb::blocked_range<int>& rows) {
    for (int cy = rows.begin(); cy != rows.end(); ++cy) {
      for (int cx = 0; cx < w; ++cx) {
        const std::size_t pix = static_cast<std::size_t>(cy) * w + cx;
        if (!hit[pix]) continue;
        const Vec3 P{position[pix * 3 + 0], position[pix * 3 + 1],
                     position[pix * 3 + 2]};
        const Vec3 N{normal[pix * 3 + 0], normal[pix * 3 + 1],
                     normal[pix * 3 + 2]};
        const Vec3 Ng{geomNormal[pix * 3 + 0], geomNormal[pix * 3 + 1],
                      geomNormal[pix * 3 + 2]};
        // Scale-adaptive self-intersection epsilon from the primary-ray hit
        // distance (the same rule the inline path derives from rh.ray.tfar;
        // N stands in for the ray direction -- only component magnitudes
        // matter and this is a new opt-in path with no bitwise contract).
        const float secEps = selfIntersectEps(P, N, depth[pix]);
        AOResult aov;
        const float op =
            aoGather(rscene, opt, P, Ng, N, secEps, static_cast<uint32_t>(cx),
                     static_cast<uint32_t>(cy), /*sampleMul=*/1, /*wSeed=*/w,
                     aov);
        out.openness[pix] = op;
        if (quality) {
          out.bent[pix * 3 + 0] = aov.bent.x;
          out.bent[pix * 3 + 1] = aov.bent.y;
          out.bent[pix * 3 + 2] = aov.bent.z;
          out.contact[pix] = aov.contact;
          out.shape[pix] = aov.shape;
          out.avgHitDist[pix] = aov.avgHitDist;
        }
      }
    }
  });
}

// Per-hit bilateral lookup. (px, py) is the hit's pixel on the latticeW x
// latticeH seed lattice (the hi-res grid, or W*k for --aa-depth fine cells);
// N / z are the hit's OWN shading normal and primary-ray camera distance --
// the hit is its own upsample guide. Interpolates over the 2x2 surrounding
// coarse cells with bilinear x normal^pow x depth weights, skipping miss
// cells. Returns false when the accepted weight sum is below kAoPatchThresh
// (silhouette rim, occluded layer, sub-cell feature): the caller then runs
// the exact inline gather instead -- strictly better than any copied value,
// so there is no nearest-cell fallback here. On success fills `openness` and
// the quality channels of `aov` (bent renormalized around N; neutral defaults
// when the grid ran the legacy estimator).
inline bool sampleCoarseAo(const CoarseAoGrid& g, int latticeW, int latticeH,
                           uint32_t px, uint32_t py, const Vec3& N, float z,
                           float& openness, AOResult& aov) {
  if (g.w <= 0 || g.h <= 0) return false;
  const float sx = (static_cast<float>(px) + 0.5f) * static_cast<float>(g.w) /
                       static_cast<float>(latticeW) -
                   0.5f;
  const float sy = (static_cast<float>(py) + 0.5f) * static_cast<float>(g.h) /
                       static_cast<float>(latticeH) -
                   0.5f;
  const int bx = static_cast<int>(std::floor(sx));
  const int by = static_cast<int>(std::floor(sy));
  const float fx = sx - static_cast<float>(bx);
  const float fy = sy - static_cast<float>(by);

  const bool quality = !g.bent.empty();
  float wSum = 0.0f, opSum = 0.0f;
  Vec3 bentSum{0.0f, 0.0f, 0.0f};
  float contactSum = 0.0f, shapeSum = 0.0f, ahdSum = 0.0f;
  for (int dy = 0; dy < 2; ++dy) {
    const int cy = by + dy;
    if (cy < 0 || cy >= g.h) continue;
    for (int dx = 0; dx < 2; ++dx) {
      const int cx = bx + dx;
      if (cx < 0 || cx >= g.w) continue;
      const std::size_t cpix = static_cast<std::size_t>(cy) * g.w + cx;
      if (!g.hit[cpix]) continue;  // background / outline cell: no AO sample
      const float wb = (dx ? fx : 1.0f - fx) * (dy ? fy : 1.0f - fy);
      const Vec3 Nc{g.normal[cpix * 3 + 0], g.normal[cpix * 3 + 1],
                    g.normal[cpix * 3 + 2]};
      const float nd = dot(N, Nc);
      if (nd <= 0.0f) continue;
      const float wn = std::pow(nd, kAoUpsampleNormalPow);
      const float zc = g.depth[cpix];
      const float wz =
          std::exp(-std::fabs(z - zc) / (kAoUpsampleDepthScale * z + 1e-6f));
      const float wgt = wb * wn * wz;
      if (wgt <= 0.0f) continue;
      wSum += wgt;
      opSum += wgt * g.openness[cpix];
      if (quality) {
        bentSum.x += wgt * g.bent[cpix * 3 + 0];
        bentSum.y += wgt * g.bent[cpix * 3 + 1];
        bentSum.z += wgt * g.bent[cpix * 3 + 2];
        contactSum += wgt * g.contact[cpix];
        shapeSum += wgt * g.shape[cpix];
        ahdSum += wgt * g.avgHitDist[cpix];
      }
    }
  }
  if (wSum < kAoPatchThresh) return false;
  const float inv = 1.0f / wSum;
  openness = opSum * inv;
  aov = AOResult{};
  aov.openness = openness;
  if (quality) {
    aov.bent = safeNormalize(
        Vec3{bentSum.x * inv, bentSum.y * inv, bentSum.z * inv}, N);
    aov.contact = contactSum * inv;
    aov.shape = shapeSum * inv;
    aov.avgHitDist = ahdSum * inv;
  }
  return true;
}

}  // namespace detail
}  // namespace umbreon
