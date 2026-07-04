// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Adaptive-AA refinement mask: given the Phase-1 center shading of every OUTPUT
// pixel, decide which output pixels need full subpixel refinement (Phase 3) and
// which can replicate their center result. Pure functions of the center buffer
// (plus, in edges mode, the Phase-0 hi-res G-buffer), so the mask -- and hence
// the render -- is deterministic and thread-count invariant. Free of Embree
// calls so tests can drive it with hand-built PixelResult arrays.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstddef>
#include <vector>

#include "shading/transparency.hpp"  // detail::PixelResult

namespace umbreon {
namespace detail {

// Mask predicate thresholds. Defaults: color contrast 0.1 (POV-like Antialias_
// Threshold, but per-channel linear and geometry-aware predicates on top),
// normal delta 0.3 (screen_vector_edges strongNdelta, the genuine-occlusion
// discriminator), depth crack = 3x the local slope or 12 output-pixel worlds.
struct AaMaskParams {
  float threshold = 0.1f;    // per-channel linear color/alpha contrast
  float normalDelta = 0.3f;  // 1 - dot(n_a, n_b) that flags a pair
  float depthK = 3.0f;       // crack = step > depthK * min(neighbor slopes)
  float depthAbsPx = 12.0f;  // ... and > depthAbsPx * pixelWorldSize (absolute)
  bool perspective = false;  // scale the absolute depth term by the local |z|
};

// 3x3 binary dilation (one ring), border-clamped. Separate output buffer so the
// pass is order-independent (deterministic).
inline std::vector<uint8_t> dilateMask3x3(const std::vector<uint8_t>& in, int W,
                                          int H) {
  std::vector<uint8_t> out(in.size(), 0);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      uint8_t v = 0;
      for (int dy = -1; dy <= 1 && !v; ++dy) {
        const int yy = y + dy;
        if (yy < 0 || yy >= H) continue;
        for (int dx = -1; dx <= 1; ++dx) {
          const int xx = x + dx;
          if (xx < 0 || xx >= W) continue;
          if (in[static_cast<std::size_t>(yy) * W + xx]) {
            v = 1;
            break;
          }
        }
      }
      out[static_cast<std::size_t>(y) * W + x] = v;
    }
  }
  return out;
}

// One pair test: true if output pixels a and b (4-neighbors along one axis)
// show a discontinuity that needs subpixel refinement. za0/zb1 are the viewZ of
// the OUTER neighbors (prev-of-a / next-of-b along the same axis; at the image
// border, pass the linear extrapolation 2*a.viewZ - b.viewZ / 2*b.viewZ -
// a.viewZ so the missing outer slope mirrors the pair's own step -- a constant
// slope then stays slope-compensated at the border instead of firing).
inline bool aaPairNeedsRefine(const PixelResult& a, const PixelResult& b,
                              float za0, float zb1, float pixelWorldSize,
                              const AaMaskParams& p) {
  // Different first-hit geometry / CueMol section (includes hit-vs-background
  // via the sentinels): always a silhouette-class boundary.
  if (a.firstGeomID != b.firstGeomID || a.firstGroup != b.firstGroup)
    return true;
  // Per-channel linear color + coverage contrast (HDR, unclamped: a specular
  // crest next to a normal pixel must refine even when both are > 1).
  if (std::fabs(a.r - b.r) > p.threshold || std::fabs(a.g - b.g) > p.threshold ||
      std::fabs(a.b - b.b) > p.threshold || std::fabs(a.a - b.a) > p.threshold)
    return true;
  const bool aHit = a.firstGeomID != 0xFFFFFFFFu;
  const bool bHit = b.firstGeomID != 0xFFFFFFFFu;
  if (!aHit || !bHit) return false;  // both background: nothing else to test
  // Shading-normal delta (facet-noise tolerant, fires at genuine occlusion
  // folds); skipped when a path leaves the normal at the zero sentinel.
  const float la = a.worldNormal.x * a.worldNormal.x +
                   a.worldNormal.y * a.worldNormal.y +
                   a.worldNormal.z * a.worldNormal.z;
  const float lb = b.worldNormal.x * b.worldNormal.x +
                   b.worldNormal.y * b.worldNormal.y +
                   b.worldNormal.z * b.worldNormal.z;
  if (la > 0.0f && lb > 0.0f) {
    const float d = a.worldNormal.x * b.worldNormal.x +
                    a.worldNormal.y * b.worldNormal.y +
                    a.worldNormal.z * b.worldNormal.z;
    if (1.0f - d > p.normalDelta) return true;
  }
  // Same-id depth crack (self-occlusion fold): a step that exceeds BOTH the
  // slope-compensated local gradient (a smooth slanted surface has
  // |zb - za| ~= its neighbor slopes; a fold does not) and an absolute
  // world-size floor so noisy flat regions cannot fire.
  const float step = std::fabs(b.viewZ - a.viewZ);
  const float sa = std::fabs(a.viewZ - za0);
  const float sb = std::fabs(zb1 - b.viewZ);
  const float slope = sa < sb ? sa : sb;
  float absTerm = p.depthAbsPx * pixelWorldSize;
  if (p.perspective) {
    const float zmin = std::fabs(a.viewZ) < std::fabs(b.viewZ)
                           ? std::fabs(a.viewZ)
                           : std::fabs(b.viewZ);
    absTerm *= zmin;
  }
  const float thresh = p.depthK * slope > absTerm ? p.depthK * slope : absTerm;
  return step > thresh;
}

// Build the (dilated) refinement mask over the Wf x Hf OUTPUT grid from the
// Phase-1 center results. pixelWorldSize is the world-space size of one OUTPUT
// pixel (ortho: cam.height / Hf; perspective: per-unit-z, see
// AaMaskParams::perspective). Optional edges-mode inputs: the Phase-0 hi-res
// (W x H, supersample ss) objectId/viewZ planes -- any ss x ss block that is
// itself non-uniform flags its output pixel (the thin-feature detector: a
// subpixel-wide sliver no center ray hits still refines its block). Pass
// nullptr when edges are off.
inline std::vector<uint8_t> buildAaMask(int Wf, int Hf,
                                        const PixelResult* centers,
                                        float pixelWorldSize,
                                        const AaMaskParams& p,
                                        const uint32_t* objIdHi = nullptr,
                                        const float* viewZHi = nullptr, int W = 0,
                                        int ss = 1) {
  std::vector<uint8_t> raw(static_cast<std::size_t>(Wf) * Hf, 0);
  auto at = [&](int x, int y) -> const PixelResult& {
    return centers[static_cast<std::size_t>(y) * Wf + x];
  };
  // Horizontal pairs (a = x, b = x+1); outer slopes from x-1 / x+2, clamped.
  for (int y = 0; y < Hf; ++y) {
    for (int x = 0; x + 1 < Wf; ++x) {
      const PixelResult& a = at(x, y);
      const PixelResult& b = at(x + 1, y);
      const float za0 =
          (x > 0) ? at(x - 1, y).viewZ : 2.0f * a.viewZ - b.viewZ;
      const float zb1 =
          (x + 2 < Wf) ? at(x + 2, y).viewZ : 2.0f * b.viewZ - a.viewZ;
      if (aaPairNeedsRefine(a, b, za0, zb1, pixelWorldSize, p)) {
        raw[static_cast<std::size_t>(y) * Wf + x] = 1;
        raw[static_cast<std::size_t>(y) * Wf + x + 1] = 1;
      }
    }
  }
  // Vertical pairs (a = y, b = y+1).
  for (int y = 0; y + 1 < Hf; ++y) {
    for (int x = 0; x < Wf; ++x) {
      const PixelResult& a = at(x, y);
      const PixelResult& b = at(x, y + 1);
      const float za0 =
          (y > 0) ? at(x, y - 1).viewZ : 2.0f * a.viewZ - b.viewZ;
      const float zb1 =
          (y + 2 < Hf) ? at(x, y + 2).viewZ : 2.0f * b.viewZ - a.viewZ;
      if (aaPairNeedsRefine(a, b, za0, zb1, pixelWorldSize, p)) {
        raw[static_cast<std::size_t>(y) * Wf + x] = 1;
        raw[static_cast<std::size_t>(y + 1) * Wf + x] = 1;
      }
    }
  }
  // Edges mode: intra-block discontinuity from the Phase-0 hi-res G-buffer.
  if (objIdHi != nullptr && viewZHi != nullptr && ss > 1) {
    for (int Y = 0; Y < Hf; ++Y) {
      for (int X = 0; X < Wf; ++X) {
        std::size_t base =
            (static_cast<std::size_t>(Y) * ss) * W + static_cast<std::size_t>(X) * ss;
        const uint32_t id0 = objIdHi[base];
        float zmin = viewZHi[base], zmax = zmin;
        bool fire = false;
        for (int j = 0; j < ss && !fire; ++j) {
          const std::size_t row = base + static_cast<std::size_t>(j) * W;
          for (int i = 0; i < ss; ++i) {
            if (objIdHi[row + i] != id0) {
              fire = true;
              break;
            }
            const float z = viewZHi[row + i];
            if (z < zmin) zmin = z;
            if (z > zmax) zmax = z;
          }
        }
        float absTerm = p.depthAbsPx * pixelWorldSize;
        if (p.perspective) absTerm *= std::fabs(zmin);
        if (fire || (zmax - zmin) > absTerm)
          raw[static_cast<std::size_t>(Y) * Wf + X] = 1;
      }
    }
  }
  return dilateMask3x3(raw, Wf, Hf);
}

}  // namespace detail
}  // namespace umbreon
