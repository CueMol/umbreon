// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Procedural Tonal Art Map ink math for the tone-hatching pass (--hatch):
// the lattice / nesting-level logic (hatchFirstLevel), the per-layer runtime
// normalization (min-feature clamp) and the analytic line-coverage evaluator.
// Everything here is a pure function of coordinates and layer constants --
// no thread state, no accumulation order -- so the TBB row tiling of the
// consumer (npr/hatch_shade.cpp) is bit-exact at any thread count.
//
// Design record: docs/plans/npr-tone-hatching.md section 6 (including the
// deviations from the brief's pseudo-code: the exact box-filter coverage
// replacing the smoothstep profile, and the K=0 / INT_MIN guards).
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "render/hatch_types.hpp"

namespace umbreon {
namespace detail {

inline float hatchClamp01(float v) {
  return std::min(1.0f, std::max(0.0f, v));
}

// Nesting level of lattice index j under K subdivision levels: j on the
// level-0 (coarsest) lattice returns 0, odd j returns K (finest). Appearance
// thresholds decrease with the level, so darkening the tone only ever
// INSERTS marks between existing ones -- the TAM nesting guarantee reduced
// to a count-trailing-zeros. The unsigned negation avoids the -j UB at
// INT_MIN (never reached with real windows, but free to make impossible).
inline int hatchFirstLevel(int j, int K) {
  if (j == 0 || K <= 0) return 0;
  unsigned u = static_cast<unsigned>(j);
  if (j < 0) u = 0u - u;
  int ntz = 0;
  while (!(u & 1u)) {
    u >>= 1u;
    ++ntz;
  }
  return ntz >= K ? 0 : K - ntz;
}

// One layer, normalized for evaluation at FINAL-resolution pixel coordinates.
struct HatchLayerRt {
  LayerKind kind = LayerKind::Line;
  float cosA = 1.0f, sinA = 0.0f;  // angleDeg rotation
  float step = 10.0f;              // finest lattice pitch spacing/2^K (>= 2px)
  int K = 0;                       // effective subdiv after the clamp below
  float halfWidth = 0.55f;         // half line width
  float halfAA = 0.5f;             // AA filter half-width
  float toneHi = 0.95f;
  float toneLo = 0.55f;
  float fadeInv = 16.0f;
  float opacity = 1.0f;
};

// Clamp a layer into its evaluable range. The min-feature clamp caps the
// subdivision so the finest pitch stays >= 2 px: the ink is laid at FINAL
// resolution, so supersampling cannot rescue a sub-pixel lattice (unlike the
// tone, which IS box-downsampled), and a finer pitch would alias against the
// pixel grid. Equivalent inputs normalize identically (spacing 10 / K 5 and
// spacing 10 / K 2 produce byte-identical ink).
inline HatchLayerRt hatchNormalizeLayer(const HatchLayer& L) {
  HatchLayerRt r;
  r.kind = L.kind;
  const float a = L.angleDeg * 0.017453292519943295f;
  r.cosA = std::cos(a);
  r.sinA = std::sin(a);
  const float spacing = std::max(4.0f, L.spacingPx);  // degenerate-lattice guard
  int K = std::max(0, L.subdiv);
  const int kMax = std::max(
      0, static_cast<int>(std::floor(std::log2(spacing / 2.0f))));
  if (K > kMax) K = kMax;
  r.K = K;
  r.step = spacing / static_cast<float>(1 << K);
  r.halfAA = std::min(2.0f, std::max(0.5f, L.mark.edgeSoftness));
  r.halfWidth =
      0.5f * std::min(std::max(0.0f, L.widthPx), 2.0f * r.step);
  r.toneHi = hatchClamp01(L.toneHi);
  r.toneLo = std::min(r.toneHi, hatchClamp01(L.toneLo));
  r.fadeInv = std::max(1.0e-3f, L.fadeInv);
  r.opacity = hatchClamp01(L.opacity);
  return r;
}

// Appearance threshold of nesting level lv (toneHi at level 0, toneLo at
// level K; the K == 0 guard avoids the 0/0 of the interpolation).
inline float hatchLevelThreshold(const HatchLayerRt& L, int lv) {
  if (L.K <= 0) return L.toneHi;
  return L.toneHi +
         (L.toneLo - L.toneHi) * (static_cast<float>(lv) /
                                  static_cast<float>(L.K));
}

// Ink coverage of one Line layer at pixel center (x, y), tone in [0,1]
// display-encoded (1 = paper). Analytic 1D evaluation on the axis u
// perpendicular to the lines: for every lattice line within reach, gate on
// its nesting threshold, grow its half-width from zero by the fade (Webb
// 2002 style), and take the exact overlap of the line band with the box
// filter footprint [d - halfAA, d + halfAA]. The exact overlap -- unlike
// the smoothstep profile of the design brief -- is correct at ALL widths,
// so a fading-in line grows from zero coverage instead of popping in at
// half coverage on its center pixel, and per-pixel coverage is a monotone
// non-decreasing, continuous function of darkening tone by construction.
inline float hatchLineInk(const HatchLayerRt& L, float x, float y,
                          float tone) {
  const float u = -L.sinA * x + L.cosA * y;
  const float h = L.halfAA;
  const float pad = L.halfWidth + h;
  const int j0 = static_cast<int>(std::floor((u - pad) / L.step));
  const int j1 = static_cast<int>(std::floor((u + pad) / L.step));
  float cov = 0.0f;
  for (int j = j0; j <= j1; ++j) {
    const int lv = hatchFirstLevel(j, L.K);
    const float t = hatchLevelThreshold(L, lv);
    if (tone >= t) continue;  // not yet appeared at this tone
    const float fade = std::min(1.0f, (t - tone) * L.fadeInv);
    const float w = L.halfWidth * fade;
    if (w <= 0.0f) continue;
    const float d = std::fabs(u - static_cast<float>(j) * L.step);
    float c = (std::min(d + h, w) - std::max(d - h, -w)) / (2.0f * h);
    c = hatchClamp01(c);
    if (c > cov) cov = c;
  }
  return cov;
}

// Ink coverage of one layer (kind dispatch). Dot layers are Phase 2 of the
// plan and currently contribute no ink.
inline float hatchLayerInk(const HatchLayerRt& L, float x, float y,
                           float tone) {
  if (L.kind == LayerKind::Line) return hatchLineInk(L, x, y, tone);
  return 0.0f;
}

}  // namespace detail
}  // namespace umbreon
