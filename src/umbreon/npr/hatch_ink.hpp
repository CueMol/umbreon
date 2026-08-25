// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Procedural Tonal Art Map ink math for the tone-hatching pass (--hatch):
// the lattice / nesting-level logic (hatchFirstLevel), the per-layer runtime
// normalization (min-feature and perturbation clamps, Lp area constants),
// the analytic line / halftone-dot / stochastic-stipple coverage evaluators
// and the deterministic hash / value-noise primitives behind the hand-drawn
// perturbations.
// Everything here is a pure function of coordinates and layer constants --
// no thread state, no accumulation order -- so the TBB row tiling of the
// consumer (npr/hatch_shade.cpp) is bit-exact at any thread count. The
// perturbations are functions of the lattice-index hash and the along-mark
// coordinate ONLY, never of the tone: marks never move or pop when the
// shading changes (the TAM nesting guarantee).
//
// Design record: docs/plans/npr-tone-hatching.md section 6 (including the
// deviations from the brief's pseudo-code: the exact box-filter line
// coverage, the sub-filter energy clamp on dots and the K=0 / INT_MIN
// guards) and docs/plans/npr-hatch-mark-geometry.md (dot gain, the
// coverage -> radius table that replaced the dual-lattice inversion, the
// stochastic stipple and the auto fade).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "render/hatch_types.hpp"

namespace umbreon {
namespace detail {

inline float hatchClamp01(float v) {
  return std::min(1.0f, std::max(0.0f, v));
}

inline float hatchSmoothstep(float e0, float e1, float x) {
  float t = (x - e0) / (e1 - e0);
  t = hatchClamp01(t);
  return t * t * (3.0f - 2.0f * t);
}

// ---- deterministic hash / value noise ------------------------------------
// Wellons' lowbias32: a fast, well-mixed 32-bit integer hash. All hatch
// randomness (jitter, wobble phases, stroke offsets, paper tooth) derives
// from it, keyed by (user seed, layer index, lattice index, stream id), so
// the pattern is reproducible across runs and thread counts by construction.

inline std::uint32_t hatchLowbias32(std::uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

inline std::uint32_t hatchHashCombine(std::uint32_t h, std::uint32_t v) {
  return hatchLowbias32(h ^ (v + 0x9e3779b9U + (h << 6) + (h >> 2)));
}

// Independent hash streams per perturbation, so e.g. the X and Y jitter of
// one dot are decorrelated without eating lattice-index bits.
enum : std::uint32_t {
  kHatchStreamJitX = 1,
  kHatchStreamJitY = 2,
  kHatchStreamWobble = 3,
  kHatchStreamWidth = 4,
  kHatchStreamStroke = 5,
  kHatchStreamTooth = 6,
  kHatchStreamHoleX = 7,      // (retired: dual-lattice holes)
  kHatchStreamHoleY = 8,      // (retired)
  kHatchStreamDashLen = 9,    // per-stroke length scatter
  kHatchStreamDashPress = 10, // per-stroke pressure (width + darkness)
  kHatchStreamDashAngle = 11, // per-stroke angle scatter
  kHatchStreamField = 12,     // coherent direction-drift field
  kHatchStreamDashTaper = 13, // per-stroke asymmetric entry/tail lengths
  kHatchStreamBelly = 14,     // along-stroke width swell
  kHatchStreamThreshold = 15, // per-cell appearance threshold (Stipple)
};

// Per-mark seed: pure function of (user seed, layer, lattice index, stream).
// The per-line hash is folded into the SEED (not added to the noise
// coordinate as in the design brief) to avoid float precision loss at large
// coordinate offsets; the intent -- every line gets its own phase -- is the
// same.
inline std::uint32_t hatchMarkSeed(std::uint32_t seed, std::uint32_t layer,
                                   std::int32_t i, std::int32_t j,
                                   std::uint32_t stream) {
  std::uint32_t h = hatchLowbias32(seed ^ 0x9e3779b9U);
  h = hatchHashCombine(h, layer);
  h = hatchHashCombine(h, static_cast<std::uint32_t>(i));
  h = hatchHashCombine(h, static_cast<std::uint32_t>(j));
  return hatchHashCombine(h, stream);
}

inline float hatchU01(std::uint32_t h) {
  return static_cast<float>(h >> 8) * (1.0f / 16777216.0f);  // [0,1)
}

inline float hatchSFloat(std::uint32_t h) {
  return hatchU01(h) * 2.0f - 1.0f;  // [-1,1)
}

// 1D value noise: cubic-smoothed lerp of hashed integer-lattice values,
// C1-continuous, in [-1,1). Pure function of (seed, x).
inline float hatchValueNoise1(std::uint32_t seed, float x) {
  const float xf = std::floor(x);
  const std::int32_t i = static_cast<std::int32_t>(xf);
  const float f = x - xf;
  const float g0 = hatchSFloat(
      hatchLowbias32(hatchHashCombine(seed, static_cast<std::uint32_t>(i))));
  const float g1 = hatchSFloat(hatchLowbias32(
      hatchHashCombine(seed, static_cast<std::uint32_t>(i + 1))));
  const float s = f * f * (3.0f - 2.0f * f);
  return g0 + (g1 - g0) * s;
}

// 2D value noise (paper tooth): bilinear of 4 hashed corners with the same
// cubic smoothing on both axes, in [-1,1).
inline float hatchValueNoise2(std::uint32_t seed, float x, float y) {
  const float xf = std::floor(x), yf = std::floor(y);
  const std::int32_t xi = static_cast<std::int32_t>(xf);
  const std::int32_t yi = static_cast<std::int32_t>(yf);
  const float fx = x - xf, fy = y - yf;
  auto corner = [seed](std::int32_t cx, std::int32_t cy) {
    std::uint32_t h = hatchHashCombine(seed, static_cast<std::uint32_t>(cx));
    h = hatchHashCombine(h, static_cast<std::uint32_t>(cy));
    return hatchSFloat(hatchLowbias32(h));
  };
  const float sx = fx * fx * (3.0f - 2.0f * fx);
  const float sy = fy * fy * (3.0f - 2.0f * fy);
  const float a = corner(xi, yi), b = corner(xi + 1, yi);
  const float c = corner(xi, yi + 1), d = corner(xi + 1, yi + 1);
  const float ab = a + (b - a) * sx;
  const float cd = c + (d - c) * sx;
  return ab + (cd - ab) * sy;
}

// ---- lattice / nesting ----------------------------------------------------

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

// Resolution of the Dot coverage -> radius table (HatchLayerRt::rTab).
constexpr int kHatchRadN = 64;

// One layer, normalized for evaluation at FINAL-resolution pixel coordinates.
struct HatchLayerRt {
  LayerKind kind = LayerKind::Line;
  float cosA = 1.0f, sinA = 0.0f;  // angleDeg rotation (lattice frame)
  float step = 10.0f;              // finest lattice pitch spacing/2^K (>= 2px)
  int K = 0;                       // effective subdiv after the clamp below
  float halfWidth = 0.55f;         // half line width
  float dotScale = 1.0f;           // Dot gain / Stipple radius scale, clamped
  float rFixed = 0.0f;             // Stipple: fixed dot radius (px)
  float halfAA = 0.5f;             // AA filter half-width
  float toneHi = 0.95f;
  float toneLo = 0.55f;
  float fadeInv = 16.0f;
  float opacity = 1.0f;
  float inkScale = 1.0f;           // per-layer pencil darkness
  std::uint32_t seed = 0;          // MarkStyle::seed
  std::uint32_t layerId = 0;       // index in the layer list (hash stream)
  // Line perturbations (0 = off).
  float wobbleAmp = 0.0f;
  float wobbleWave = 40.0f;
  float widthJitter = 0.0f;
  float strokeLen = 0.0f;
  float strokeGap = 0.0f;
  float strokeTaper = 0.3f;
  float angleJitterTan = 0.0f;   // tan of the per-stroke angle scatter
  float strokeLenJitter = 0.0f;  // relative per-stroke length scatter
  // Paper tooth (both kinds; 0 = off).
  float toothAmp = 0.0f;
  float toothScale = 3.0f;
  // Dot marks.
  float pExp = 2.0f;    // Lp exponent, clamped; >= 16 evaluates as L-inf
  float aP = 3.14159265f;  // Lp unit-ball area constant A_p
  float rhoP = 0.5641896f; // 1/sqrt(A_p): area-normalized radius scale
  float sx = 1.0f, sy = 1.0f;  // aspect stretch (sx*sy == 1, area-neutral)
  float dotCos = 1.0f, dotSin = 0.0f;  // mark-shape rotation
  float jitterAmp = 0.0f;              // px (jitter * step)
  // Dot layers: coverage -> radius table (hatchDotRadius). rTab[k] is the
  // radius whose mean cell coverage is k / kHatchRadN, sampled with this
  // layer's own mark function; rMax is the last entry -- past the covering
  // radius (solid black) when invertAbove50 lets the dots merge, else the
  // area-normalized full-cell radius.
  float rMax = 0.0f;
  float rTab[kHatchRadN + 1] = {};
  // Precomputed search windows (px).
  float padLine = 0.0f;
  float padDot = 0.0f;   // Dot and Stipple marks
};

// Lp unit-ball area A_p = (2*Gamma(1+1/p))^2 / Gamma(1+2/p); p >= 16 is
// treated as L-inf (A = 4). Scaling every dot radius by 1/sqrt(A_p) makes
// the mean coverage at a given tone independent of the mark shape.
inline float hatchLpArea(float p) {
  if (p >= 16.0f) return 4.0f;
  const float g1 = std::tgamma(1.0f + 1.0f / p);
  const float g2 = std::tgamma(1.0f + 2.0f / p);
  return (2.0f * g1) * (2.0f * g1) / g2;
}

// ---- mark shape (Dot / Stipple) --------------------------------------------

// Lp distance of (dx, dy) from a mark center, in the mark frame (rotated by
// dotAngleDeg, aspect stretch undone so the iso-contours are the stretched
// shape while the enclosed area stays A_p * r^2).
inline float hatchLpDist(const HatchLayerRt& L, float dx, float dy) {
  const float rx = (L.dotCos * dx + L.dotSin * dy) / L.sx;
  const float ry = (-L.dotSin * dx + L.dotCos * dy) / L.sy;
  const float ax = std::fabs(rx), ay = std::fabs(ry);
  if (L.pExp >= 16.0f) return std::max(ax, ay);
  return std::pow(std::pow(ax, L.pExp) + std::pow(ay, L.pExp),
                  1.0f / L.pExp);
}

// Coverage of one dot (or hole) mark of radius r at Lp distance d: the
// smoothstep profile with a sub-filter ENERGY CLAMP -- a mark smaller than
// the filter footprint contributes at most its area fraction, so a growing
// dot rises from zero coverage instead of popping in at half coverage
// (the dot-mark analogue of the exact line box filter).
inline float hatchDotMark(const HatchLayerRt& L, float r, float d) {
  if (r <= 0.0f) return 0.0f;
  const float h = L.halfAA;
  const float c = 1.0f - hatchSmoothstep(r - h, r + h, d);
  const float att = std::min(1.0f, L.aP * r * r / (4.0f * h * h));
  return att * c;
}

// Clamp a layer into its evaluable range and precompute the derived
// constants. The min-feature clamp caps the subdivision so the finest pitch
// stays >= 2 px: the ink is laid at FINAL resolution, so supersampling
// cannot rescue a sub-pixel lattice (unlike the tone, which IS
// box-downsampled), and a finer pitch would alias against the pixel grid.
// Equivalent inputs normalize identically (spacing 10 / K 5 and spacing 10 /
// K 2 produce byte-identical ink). Perturbation amplitudes are clamped so
// the search windows stay small (a few cells per axis; a line width up to
// the level-0 pitch is the one knob that can widen them further).
inline HatchLayerRt hatchNormalizeLayer(const HatchLayer& L,
                                        std::uint32_t layerId) {
  HatchLayerRt r;
  r.kind = L.kind;
  r.layerId = layerId;
  r.seed = L.mark.seed;
  const float a = L.angleDeg * 0.017453292519943295f;
  r.cosA = std::cos(a);
  r.sinA = std::sin(a);
  // Min-feature guard: the ink is laid at FINAL resolution, so the finest
  // lattice pitch must stay >= 2 px (below that the marks alias against the
  // pixel grid and supersampling cannot help -- unlike the tone, which IS
  // box-downsampled). Clamp the base pitch to that floor, then cap the
  // subdivision so spacing / 2^K still clears it.
  const float spacing = std::max(2.0f, L.spacingPx);
  // Stipple layers are one flat lattice: the tone is carried by the dot
  // count, not by nesting levels.
  int K = (L.kind == LayerKind::Stipple) ? 0 : std::max(0, L.subdiv);
  const int kMax = std::max(
      0, static_cast<int>(std::floor(std::log2(spacing / 2.0f))));
  if (K > kMax) K = kMax;
  r.K = K;
  r.step = spacing / static_cast<float>(1 << K);
  r.halfAA = std::min(2.0f, std::max(0.5f, L.mark.edgeSoftness));
  // Width cap: the level-0 pitch (there the level-0 lines alone are solid)
  // or twice the finest step, whichever is larger -- never below the
  // former 2 * step cap, which for a K = 0 layer is the wider of the two
  // (a wider band still matters while the fade is partial). Below the cap
  // the width is the caller's (over-darkening is a legitimate choice).
  r.halfWidth = 0.5f * std::min(std::max(0.0f, L.widthPx),
                                std::max(spacing, 2.0f * r.step));
  r.toneHi = hatchClamp01(L.toneHi);
  r.toneLo = std::min(r.toneHi, hatchClamp01(L.toneLo));
  if (L.fadeInv > 0.0f) {
    r.fadeInv = L.fadeInv;
  } else if (L.kind == LayerKind::Stipple) {
    r.fadeInv = 32.0f;
  } else {
    // Auto fade: a mark grows from zero at its own threshold to full size
    // at the next level's threshold (toneLo when there is no nesting), so
    // the layer's coverage is continuous in the tone. The band is the
    // uniform threshold spacing of hatchLevelThreshold.
    const float band =
        (K > 0) ? (r.toneHi - r.toneLo) / static_cast<float>(K)
                : (r.toneHi - r.toneLo);
    r.fadeInv = 1.0f / std::max(band, 1.0e-3f);
  }
  r.dotScale = std::min(4.0f, std::max(0.0f, L.dotScale));
  r.opacity = hatchClamp01(L.opacity);
  r.inkScale = hatchClamp01(L.inkScale);

  // Line perturbations, clamped (the pad formulas below rely on these).
  r.wobbleAmp = std::min(std::max(0.0f, L.mark.wobbleAmpPx), r.step);
  r.wobbleWave = std::max(2.0f, L.mark.wobbleWavePx);
  r.widthJitter = hatchClamp01(L.mark.widthJitter);
  r.strokeLen = std::max(0.0f, L.mark.strokeLenPx);
  if (r.strokeLen > 0.0f) r.strokeLen = std::max(2.0f, r.strokeLen);
  r.strokeGap = std::max(0.0f, L.mark.strokeGapPx);
  r.strokeTaper = std::min(0.9f, std::max(0.0f, L.mark.strokeTaper));
  const float angleJit =
      std::min(15.0f, std::max(0.0f, L.mark.angleJitterDeg));
  r.angleJitterTan = std::tan(angleJit * 0.017453292519943295f);
  r.strokeLenJitter = std::min(0.9f, std::max(0.0f, L.mark.strokeLenJitter));
  r.toothAmp = hatchClamp01(L.mark.toothAmp);
  r.toothScale = std::max(0.5f, L.mark.toothScalePx);

  // Dot mark shape: Lp exponent, area constant, area-neutral aspect stretch
  // and the mark rotation.
  r.pExp = std::min(64.0f, std::max(1.0f, L.mark.shapeExponent));
  r.aP = hatchLpArea(r.pExp);
  r.rhoP = 1.0f / std::sqrt(r.aP);
  const float aspect = std::min(2.0f, std::max(0.5f, L.mark.dotAspect));
  r.sx = std::sqrt(aspect);
  r.sy = 1.0f / r.sx;
  const float da = L.mark.dotAngleDeg * 0.017453292519943295f;
  r.dotCos = std::cos(da);
  r.dotSin = std::sin(da);
  const float jitter = std::min(0.5f, std::max(0.0f, L.mark.jitter));
  r.jitterAmp = jitter * r.step;

  // Dot radius range. With invertAbove50 the dots may merge past touching
  // up to the covering radius (the Lp distance from any point to its
  // nearest jittered lattice center is <= 2^(1/p) * (0.5 + jitter) * step,
  // inflated by the aspect stretch) plus the AA half-width, where the
  // lattice is solid; otherwise they stop at the area-normalized full-cell
  // radius (coverage ~0.9 at tone 0).
  const float aspectInflate = std::max(r.sx, r.sy);
  const float rCover =
      std::pow(2.0f, 1.0f / r.pExp) * (0.5f + jitter) * aspectInflate * r.step;
  const bool merge = L.kind == LayerKind::Dot && L.mark.invertAbove50;
  r.rMax = merge ? rCover + r.halfAA : r.step * r.rhoP;

  // Search windows: widen by every perturbation so displaced marks are not
  // clipped (the brief's fixed window would tear wobbled lines). Lines add
  // the per-line position scatter and the worst-case pivot reach of the
  // per-stroke angle scatter (half the longest stroke).
  const float dashReach = r.angleJitterTan * 0.5f * r.strokeLen *
                          (1.0f + 0.5f * r.strokeLenJitter);
  // Width can exceed halfWidth by the per-line modulation AND the belly
  // swell (both bounded by widthJitter fractions).
  const float wMax = r.halfWidth * (1.0f + r.widthJitter) *
                     (1.0f + 0.6f * r.widthJitter);
  r.padLine = wMax + r.wobbleAmp + r.jitterAmp + dashReach + r.halfAA;
  const float stretch =
      std::pow(2.0f, std::max(0.0f, 0.5f - 1.0f / r.pExp)) * aspectInflate;
  // Dot radii never exceed rMax; Stipple dots are fixed.
  r.rFixed = r.dotScale * r.step * r.rhoP;
  const float rReach = (L.kind == LayerKind::Stipple) ? r.rFixed : r.rMax;
  r.padDot = r.jitterAmp + stretch * rReach + r.halfAA;

  // Coverage -> radius table (Dot layers): sample the mean coverage of one
  // lattice cell for radii up to rMax with the layer's own mark function
  // (Lp shape, aspect, AA; jitter ignored), then invert it. Exact where the
  // dots stand apart (the area-normalized closed form) and continuous
  // through the overlap regime up to solid black, so a K = 0 screen tracks
  // 1 - tone over the whole range. (The former dual-lattice inversion --
  // frozen dots plus shrinking holes under a max composite -- held the
  // coverage at 50% until its holes had shrunk well below the cell, a flat
  // band from about tone 0.5 down to 0.3.)
  if (L.kind == LayerKind::Dot) {
    constexpr int kR = 64;  // radius samples
    constexpr int kS = 16;  // sample points per cell axis
    float cov[kR + 1];
    for (int q = 0; q <= kR; ++q) {
      const float rad = r.rMax * static_cast<float>(q) / static_cast<float>(kR);
      double acc = 0.0;
      for (int sy = 0; sy < kS; ++sy) {
        for (int sx = 0; sx < kS; ++sx) {
          const float px = (static_cast<float>(sx) + 0.5f) / kS * r.step;
          const float py = (static_cast<float>(sy) + 0.5f) / kS * r.step;
          float c = 0.0f;
          for (int j = -1; j <= 2; ++j) {
            for (int i = -1; i <= 2; ++i) {
              const float m = hatchDotMark(
                  r, rad,
                  hatchLpDist(r, px - static_cast<float>(i) * r.step,
                              py - static_cast<float>(j) * r.step));
              if (m > c) c = m;
            }
          }
          acc += c;
        }
      }
      cov[q] = static_cast<float>(acc / (kS * kS));
    }
    // Invert the (non-decreasing) samples: rTab[k] = radius at coverage
    // k / kHatchRadN; targets above the reachable coverage saturate at rMax.
    int q = 0;
    for (int k = 0; k <= kHatchRadN; ++k) {
      const float target =
          static_cast<float>(k) / static_cast<float>(kHatchRadN);
      while (q < kR && cov[q] < target) ++q;
      if (q == 0) {
        r.rTab[k] = 0.0f;
      } else if (cov[q] < target) {
        r.rTab[k] = r.rMax;
      } else {
        const float c0 = cov[q - 1], c1 = cov[q];
        const float f = (c1 > c0) ? (target - c0) / (c1 - c0) : 1.0f;
        r.rTab[k] = r.rMax * (static_cast<float>(q - 1) + f) /
                    static_cast<float>(kR);
      }
    }
  }
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

// Paper-tooth multiplier at (x, y): 1 with toothAmp 0. Tone-independent, so
// it cannot break the nesting monotonicity (a fixed per-pixel scale).
inline float hatchTooth(const HatchLayerRt& L, float x, float y) {
  if (L.toothAmp <= 0.0f) return 1.0f;
  const std::uint32_t s =
      hatchMarkSeed(L.seed, L.layerId, 0, 0, kHatchStreamTooth);
  const float n =
      hatchValueNoise2(s, x / L.toothScale, y / L.toothScale);
  return 1.0f - L.toothAmp * (0.5f + 0.5f * n);
}

// ---- Line layers ----------------------------------------------------------

// Ink coverage of one Line layer at pixel center (x, y), tone in [0,1]
// display-encoded (1 = paper). Analytic evaluation on the axis u
// perpendicular to the lines (v runs along them): for every lattice line
// within reach, gate on its nesting threshold, grow its half-width from
// zero by the fade (Webb 2002 style), apply the hand-drawn perturbations
// (pure functions of hash(j) and v -- never tone), and take the exact
// overlap of the line band with the box filter footprint [d - halfAA,
// d + halfAA]. The exact overlap -- unlike the smoothstep profile of the
// design brief -- is correct at ALL widths, so a fading-in line grows from
// zero coverage instead of popping in at half coverage on its center pixel,
// and per-pixel coverage is a monotone non-decreasing, continuous function
// of darkening tone by construction.
inline float hatchLineInk(const HatchLayerRt& L, float x, float y,
                          float tone) {
  const float u = -L.sinA * x + L.cosA * y;
  const float v = L.cosA * x + L.sinA * y;
  const float h = L.halfAA;
  const int j0 = static_cast<int>(std::floor((u - L.padLine) / L.step));
  const int j1 = static_cast<int>(std::floor((u + L.padLine) / L.step));
  float cov = 0.0f;
  for (int j = j0; j <= j1; ++j) {
    const int lv = hatchFirstLevel(j, L.K);
    const float t = hatchLevelThreshold(L, lv);
    if (tone >= t) continue;  // not yet appeared at this tone
    const float fade = std::min(1.0f, (t - tone) * L.fadeInv);
    float w = L.halfWidth * fade;
    float markScale = 1.0f;  // per-stroke darkness (pressure)
    float c0 = static_cast<float>(j) * L.step;
    if (L.jitterAmp > 0.0f)
      c0 += (hatchU01(hatchMarkSeed(L.seed, L.layerId, 0, j,
                                    kHatchStreamJitX)) -
             0.5f) *
            2.0f * L.jitterAmp;
    if (L.wobbleAmp > 0.0f)
      c0 += L.wobbleAmp *
            hatchValueNoise1(
                hatchMarkSeed(L.seed, L.layerId, 0, j, kHatchStreamWobble),
                v / L.wobbleWave);
    if (L.widthJitter > 0.0f)
      w *= 1.0f + L.widthJitter *
                      hatchValueNoise1(hatchMarkSeed(L.seed, L.layerId, 0, j,
                                                     kHatchStreamWidth),
                                       v / L.wobbleWave);
    if (L.strokeLen > 0.0f) {
      // Finite strokes: a per-line random phase shifts the duty cycle so
      // gaps do not align across lines, and every stroke (dash index k)
      // gets its own length, pressure (width + darkness) and angle from
      // the (j, k) hash -- individual pen movements, not a dashed ruler
      // line. All of it is tone-independent, so the nesting stays exact.
      const float period = L.strokeLen + L.strokeGap;
      const float vp =
          v + period * hatchU01(hatchMarkSeed(L.seed, L.layerId, 0, j,
                                              kHatchStreamStroke));
      const float slot = std::floor(vp / period);
      const int k = static_cast<int>(slot);
      float ph = vp - slot * period;  // [0, period)
      float len = L.strokeLen;
      if (L.strokeLenJitter > 0.0f) {
        len *= 1.0f + L.strokeLenJitter *
                          (hatchU01(hatchMarkSeed(L.seed, L.layerId, j, k,
                                                  kHatchStreamDashLen)) -
                           0.5f);
        len = std::min(std::max(2.0f, len), period);
      }
      if (ph >= len) continue;  // in the gap
      if (L.widthJitter > 0.0f) {
        // Pressure: lighter strokes are both thinner and paler.
        const float press =
            1.0f - L.widthJitter *
                       hatchU01(hatchMarkSeed(L.seed, L.layerId, j, k,
                                              kHatchStreamDashPress));
        w *= press;
        markScale = 0.6f + 0.4f * press;
      }
      if (L.angleJitterTan > 0.0f) {
        // Each stroke pivots around its own center. The angle is a
        // COHERENT direction-drift field sampled at the stroke center
        // (nearby strokes lean together, distant patches drift apart --
        // the arm repositioning of hand hatching) plus a small
        // independent per-stroke scatter. Both are tone-free.
        const float vC = v + ((slot + 0.5f) * period - vp);
        const float fieldScale = std::max(60.0f, 2.5f * L.strokeLen);
        const float field = hatchValueNoise2(
            hatchMarkSeed(L.seed, L.layerId, 0, 0, kHatchStreamField),
            static_cast<float>(j) * L.step / fieldScale, vC / fieldScale);
        float amt = 1.2f * field +
                    0.45f * hatchSFloat(hatchMarkSeed(
                                L.seed, L.layerId, j, k,
                                kHatchStreamDashAngle));
        amt = std::min(1.0f, std::max(-1.0f, amt));
        c0 += L.angleJitterTan * amt * (ph - 0.5f * len);
      }
      // Praun-style stroke body (TAM, Fig. 2): an ASYMMETRIC envelope -- a
      // short rounded entry and a long release tail, flipped at random per
      // stroke -- times a mid-wavelength belly swell, so each stroke
      // thickens and thins along its length instead of being a constant
      // bar with symmetric cone ends. strokeTaper scales both end lengths.
      const float s01 = ph / len;
      const std::uint32_t hTaper = hatchMarkSeed(L.seed, L.layerId, j, k,
                                                 kHatchStreamDashTaper);
      float a0 = L.strokeTaper * (0.35f + 0.35f * hatchU01(hTaper));
      float a1 = L.strokeTaper *
                 (0.9f + 1.1f * hatchU01(hatchLowbias32(hTaper)));
      a0 = std::min(0.45f, std::max(0.02f, a0));
      a1 = std::min(0.60f, std::max(0.05f, a1));
      if (hTaper & 1u) {  // which end is the fat one flips per stroke
        const float tmp = a0;
        a0 = a1;
        a1 = tmp;
      }
      const float head = std::min(1.0f, s01 / a0);
      const float tail = std::min(1.0f, (1.0f - s01) / a1);
      w *= head * (2.0f - head) * tail * (2.0f - tail);  // rounded ends
      if (L.widthJitter > 0.0f) {
        const float bellyWave = std::max(8.0f, len * 0.4f);
        w *= 1.0f + 0.6f * L.widthJitter *
                        hatchValueNoise1(
                            hatchMarkSeed(L.seed, L.layerId, j, k,
                                          kHatchStreamBelly),
                            ph / bellyWave);
      }
    }
    if (w <= 0.0f) continue;
    const float d = std::fabs(u - c0);
    float c = (std::min(d + h, w) - std::max(d - h, -w)) / (2.0f * h);
    c = hatchClamp01(c) * markScale;
    if (c > cov) cov = c;
  }
  return cov * hatchTooth(L, x, y);
}

// ---- Dot layers -----------------------------------------------------------


// Radius that gives a Dot layer the target mean coverage (table lookup).
inline float hatchDotRadius(const HatchLayerRt& L, float coverage) {
  const float x = hatchClamp01(coverage) * static_cast<float>(kHatchRadN);
  const int k = static_cast<int>(x);
  if (k >= kHatchRadN) return L.rTab[kHatchRadN];
  const float f = x - static_cast<float>(k);
  return L.rTab[k] + (L.rTab[k + 1] - L.rTab[k]) * f;
}

// Ink coverage of one Dot layer at pixel center (x, y). 2D lattice with
// quadtree nesting (level = max of the two 1D levels); every present dot
// takes the radius that the coverage -> radius table assigns to the target
// coverage 1 - t_eff, times the fade of its nesting level. With K = 0 this
// is the classic AM halftone screen (coverage == 1 - tone, exact where the
// dots stand apart and continuous through the merge to solid black);
// optional per-dot jitter. Dot gain (HatchLayer::dotScale): the target
// coverage sees an EFFECTIVE tone darkened by dotScale^2
// (t_eff = 1 - dotScale^2 (1 - t), clamped) while the nesting thresholds
// and the fade keep the true tone, so the gain resizes the marks without
// changing which marks exist. Per-pixel monotone: the table radius is
// non-increasing in t_eff, the fade non-increasing in t, and positions
// are hash-only.
inline float hatchDotInk(const HatchLayerRt& L, float x, float y,
                         float tone) {
  const float ux = L.cosA * x + L.sinA * y;
  const float uy = -L.sinA * x + L.cosA * y;
  const float tEff =
      (L.dotScale == 1.0f)
          ? tone
          : hatchClamp01(1.0f - L.dotScale * L.dotScale * (1.0f - tone));
  const float rBase = hatchDotRadius(L, 1.0f - tEff);
  const bool unionMode = L.jitterAmp > 0.0f;  // jittered dots overlap
  float cov = 0.0f;
  const int i0 = static_cast<int>(std::floor((ux - L.padDot) / L.step));
  const int i1 = static_cast<int>(std::floor((ux + L.padDot) / L.step));
  const int j0 = static_cast<int>(std::floor((uy - L.padDot) / L.step));
  const int j1 = static_cast<int>(std::floor((uy + L.padDot) / L.step));
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      const int lv =
          std::max(hatchFirstLevel(i, L.K), hatchFirstLevel(j, L.K));
      const float t = hatchLevelThreshold(L, lv);
      if (tone >= t) continue;
      const float fade = std::min(1.0f, (t - tone) * L.fadeInv);
      const float r = rBase * fade;
      float cx = static_cast<float>(i) * L.step;
      float cy = static_cast<float>(j) * L.step;
      if (L.jitterAmp > 0.0f) {
        cx += (hatchU01(hatchMarkSeed(L.seed, L.layerId, i, j,
                                      kHatchStreamJitX)) -
               0.5f) *
              2.0f * L.jitterAmp;
        cy += (hatchU01(hatchMarkSeed(L.seed, L.layerId, i, j,
                                      kHatchStreamJitY)) -
               0.5f) *
              2.0f * L.jitterAmp;
      }
      const float c = hatchDotMark(L, r, hatchLpDist(L, ux - cx, uy - cy));
      if (unionMode)
        cov = 1.0f - (1.0f - cov) * (1.0f - c);
      else if (c > cov)
        cov = c;
    }
  }
  return cov * hatchTooth(L, x, y);
}

// ---- Stipple layers -------------------------------------------------------

// Ink coverage of one Stipple layer at pixel center (x, y): a jittered 2D
// lattice whose cells each own a hashed appearance threshold
// uT = toneLo + (toneHi - toneLo) * u, u ~ U[0,1). The cell's dot exists
// where tone < uT, fades in over 1/fadeInv below it and otherwise keeps the
// FIXED radius rFixed -- the size is tone-free, like a line's width, and the
// tone is carried by the dot COUNT: expected coverage is
// dotScale^2 * (toneHi - tone) / (toneHi - toneLo) before overlap (union
// composite, since jittered dots overlap). Presence and radius are both
// non-increasing in the tone and the positions are hash-only, so the layer
// is per-pixel monotone and its marks never move. At tone 1 no cell is
// below its threshold (u < 1), so paper stays ink-free.
inline float hatchStippleInk(const HatchLayerRt& L, float x, float y,
                             float tone) {
  const float ux = L.cosA * x + L.sinA * y;
  const float uy = -L.sinA * x + L.cosA * y;
  const float range = L.toneHi - L.toneLo;
  float cov = 0.0f;
  const int i0 = static_cast<int>(std::floor((ux - L.padDot) / L.step));
  const int i1 = static_cast<int>(std::floor((ux + L.padDot) / L.step));
  const int j0 = static_cast<int>(std::floor((uy - L.padDot) / L.step));
  const int j1 = static_cast<int>(std::floor((uy + L.padDot) / L.step));
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      const float u = hatchU01(
          hatchMarkSeed(L.seed, L.layerId, i, j, kHatchStreamThreshold));
      const float uT = L.toneLo + range * u;
      if (tone >= uT) continue;
      const float r = L.rFixed * std::min(1.0f, (uT - tone) * L.fadeInv);
      float cx = static_cast<float>(i) * L.step;
      float cy = static_cast<float>(j) * L.step;
      if (L.jitterAmp > 0.0f) {
        cx += (hatchU01(hatchMarkSeed(L.seed, L.layerId, i, j,
                                      kHatchStreamJitX)) -
               0.5f) *
              2.0f * L.jitterAmp;
        cy += (hatchU01(hatchMarkSeed(L.seed, L.layerId, i, j,
                                      kHatchStreamJitY)) -
               0.5f) *
              2.0f * L.jitterAmp;
      }
      const float c = hatchDotMark(L, r, hatchLpDist(L, ux - cx, uy - cy));
      cov = 1.0f - (1.0f - cov) * (1.0f - c);
    }
  }
  return cov * hatchTooth(L, x, y);
}

// Ink coverage of one layer (kind dispatch).
inline float hatchLayerInk(const HatchLayerRt& L, float x, float y,
                           float tone) {
  if (L.kind == LayerKind::Line) return hatchLineInk(L, x, y, tone);
  if (L.kind == LayerKind::Stipple) return hatchStippleInk(L, x, y, tone);
  return hatchDotInk(L, x, y, tone);
}

}  // namespace detail
}  // namespace umbreon
