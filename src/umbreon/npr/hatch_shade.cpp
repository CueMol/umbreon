#include "npr/hatch_shade.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "npr/hatch_ink.hpp"
#include "postprocess/image_ops.hpp"

namespace umbreon {

namespace {

// Rec.709 display-luminance of a display-encoded triple (the contrast
// guarantee compares and moves DISPLAY values; see hatch_types.hpp).
inline float displayLuma(const float c[3]) {
  return 0.2126f * c[0] + 0.7152f * c[1] + 0.0722f * c[2];
}

// Contrast guarantee (ink side only -- moving the base would break the
// molecular color coding): ensure the ink sits at least inkMinContrast of
// display luminance BELOW the base. On a base too dark to allow that, lift
// the ink to base + contrast instead (bright hatching on dark ground,
// woodcut-like; a fixed rule, not an option). Hue-preserving scale, each
// channel clamped to [0,1].
inline void ensureInkContrast(float I[3], const float B[3],
                              float minContrast) {
  if (minContrast <= 0.0f) return;
  const float lb = displayLuma(B);
  const float li = displayLuma(I);
  if (lb - li >= minContrast) return;
  if (lb >= minContrast) {
    const float target = std::max(0.0f, lb - minContrast);
    const float s = (li > 1.0e-4f) ? target / li : 0.0f;
    for (int k = 0; k < 3; ++k)
      I[k] = std::min(1.0f, std::max(0.0f, I[k] * s));
  } else {
    const float target = std::min(1.0f, lb + minContrast);
    if (li > 1.0e-4f) {
      const float s = target / li;
      for (int k = 0; k < 3; ++k)
        I[k] = std::min(1.0f, std::max(0.0f, I[k] * s));
    } else {
      I[0] = I[1] = I[2] = target;
    }
  }
}

}  // namespace

bool applyHatchPreset(HatchOptions& opt, const std::string& name) {
  if (name == "pen-cross") {
    // 3 hard crosshatch layers with staggered thresholds: the 45-deg layer
    // carries the light tones, -45 deg joins in the midtones (crosshatch),
    // the horizontal layer only in the deep shadows. No perturbation, so
    // the look is seed-independent. Values: plan section 6.5.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Line;
    l.spacingPx = 10.0f;
    l.subdiv = 2;
    l.widthPx = 1.1f;
    l.fadeInv = 16.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.angleDeg = 45.0f;
    l.toneHi = 0.95f;
    l.toneLo = 0.55f;
    opt.layers.push_back(l);
    l.angleDeg = -45.0f;
    l.toneHi = 0.62f;
    l.toneLo = 0.30f;
    opt.layers.push_back(l);
    l.angleDeg = 0.0f;
    l.toneHi = 0.32f;
    l.toneLo = 0.10f;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "pencil") {
    // Two soft graphite layers built from INDIVIDUAL strokes: per-line
    // position scatter off the lattice, per-stroke length / pressure /
    // angle scatter, wobble, tapered ends and paper tooth. Seed-stable but
    // hand-drawn looking (a dashed-ruler look is exactly what the
    // per-stroke scatter exists to avoid).
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Line;
    l.spacingPx = 9.0f;
    l.subdiv = 2;
    l.widthPx = 2.4f;
    l.fadeInv = 10.0f;
    l.opacity = 0.85f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 1.2f;
    l.mark.toothAmp = 0.22f;
    l.mark.toothScalePx = 3.0f;
    l.mark.jitter = 0.22f;         // per-line position scatter
    l.mark.wobbleAmpPx = 1.2f;
    l.mark.wobbleWavePx = 60.0f;
    l.mark.widthJitter = 0.45f;    // + per-stroke pressure scatter
    l.mark.strokeLenPx = 44.0f;
    l.mark.strokeGapPx = 5.0f;
    l.mark.strokeTaper = 0.25f;
    l.mark.angleJitterDeg = 5.0f;  // coherent drift + per-stroke scatter
    l.mark.strokeLenJitter = 0.5f;
    l.angleDeg = 55.0f;
    l.toneHi = 0.92f;
    l.toneLo = 0.50f;
    opt.layers.push_back(l);
    l.angleDeg = -35.0f;
    l.toneHi = 0.55f;
    l.toneLo = 0.22f;
    l.mark.seed = 1;               // decorrelate the two layers' strokes
    opt.layers.push_back(l);
    return true;
  }
  if (name == "engraving") {
    // One direction, deep subdivision: tone is carried by line insertion
    // and width modulation alone (copperplate look).
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Line;
    l.angleDeg = 0.0f;
    l.spacingPx = 16.0f;
    l.subdiv = 3;
    l.widthPx = 2.4f;
    l.toneHi = 0.97f;
    l.toneLo = 0.12f;
    l.fadeInv = 8.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.mark.widthJitter = 0.5f;
    l.mark.wobbleAmpPx = 0.5f;
    l.mark.wobbleWavePx = 64.0f;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "stipple") {
    // Jittered nested dot lattice: scientific-illustration stippling. The
    // lattice keeps the TAM nesting exact under the jitter.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Dot;
    l.angleDeg = 0.0f;
    l.spacingPx = 10.0f;
    l.subdiv = 2;
    l.toneHi = 0.96f;
    l.toneLo = 0.35f;
    l.fadeInv = 12.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.6f;
    l.mark.shapeExponent = 2.0f;
    l.mark.jitter = 0.4f;
    l.mark.invertAbove50 = true;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "screentone-60") {
    // Classic AM halftone screen at 45 deg (K = 0: every dot present, the
    // radius alone carries the tone; ~60 lpi at a 300 dpi print figure).
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Dot;
    l.angleDeg = 45.0f;
    l.spacingPx = 5.0f;
    l.subdiv = 0;
    l.toneHi = 1.0f;
    l.toneLo = 1.0f;
    l.fadeInv = 32.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.mark.shapeExponent = 2.0f;
    l.mark.jitter = 0.0f;
    l.mark.invertAbove50 = true;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "manga-square") {
    // Square-element screen (L-inf marks) at 45 deg, coarse enough to read
    // at full zoom.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Dot;
    l.angleDeg = 45.0f;
    l.spacingPx = 6.0f;
    l.subdiv = 0;
    l.toneHi = 1.0f;
    l.toneLo = 1.0f;
    l.fadeInv = 32.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.mark.shapeExponent = 16.0f;
    l.mark.jitter = 0.0f;
    l.mark.invertAbove50 = true;
    opt.layers.push_back(l);
    return true;
  }
  return false;
}

void applyHatch(int w, int h, float* rgba, const float* tone,
                const float* mask, const float* albedo,
                const HatchOptions& opt) {
  if (!opt.enable || w <= 0 || h <= 0 || rgba == nullptr ||
      tone == nullptr || mask == nullptr)
    return;

  // Normalize the layers once (min-feature and perturbation clamps, Lp
  // area constants, search windows). The layer index keys the hash streams,
  // so two otherwise-identical layers still decorrelate.
  std::vector<detail::HatchLayerRt> layers;
  layers.reserve(opt.layers.size());
  for (std::size_t li = 0; li < opt.layers.size(); ++li) {
    const HatchLayer& L = opt.layers[li];
    if (L.opacity <= 0.0f) continue;
    layers.push_back(
        detail::hatchNormalizeLayer(L, static_cast<std::uint32_t>(li)));
  }

  const ToneRecipe& tr = opt.tone;
  const float wpRange = std::max(1.0e-4f, tr.whitePoint - tr.blackPoint);
  const bool useAlbedoBase =
      opt.base == HatchBase::Albedo && albedo != nullptr;
  const bool useAlbedoInk =
      opt.ink == HatchInk::FromAlbedo && albedo != nullptr;

  // Row-parallel: every pixel is a pure function of (x, y) and the input
  // buffers, so the tiling is bit-exact at any thread count.
  tbb::parallel_for(
      tbb::blocked_range<int>(0, h),
      [&](const tbb::blocked_range<int>& rows) {
        for (int y = rows.begin(); y < rows.end(); ++y) {
          for (int x = 0; x < w; ++x) {
            const std::size_t p =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + x;
            const float m = mask[p];
            if (m <= 0.0f) continue;  // background: never painted

            // Tone shaping: linear black/white remap -> artistic gamma
            // (linear domain) -> display encode -> optional quantization.
            // Every stage maps 1 -> 1, so fully lit stays exactly ink-free.
            float t = detail::hatchClamp01((tone[p] - tr.blackPoint) / wpRange);
            if (tr.gamma != 1.0f) t = std::pow(t, tr.gamma);
            t = srgbEncodeF(t);
            if (opt.toneLevels > 1) {
              const float n = static_cast<float>(opt.toneLevels - 1);
              t = std::round(t * n) / n;
            }

            float* px4 = rgba + p * 4;

            // Contrast-guarantee reference base: what this pixel shows
            // between the marks. In Ink mode the pipeline already painted
            // the flat base (paper / albedo) into rgba BEFORE the stroke
            // edge pass, so the composite below only multiplies ink in and
            // the contour ink survives; B here is recomputed from the
            // options purely as the contrast reference (the actual pixel
            // may carry edge ink or fog).
            float B[3];
            if (opt.mode == HatchMode::Over) {
              // The shaded frame is the base. Under a transparent
              // background rgba is premultiplied; the multiply composite
              // commutes with premultiplication, so using it directly is
              // exact (see the write-back below).
              B[0] = px4[0];
              B[1] = px4[1];
              B[2] = px4[2];
            } else if (useAlbedoBase) {
              for (int k = 0; k < 3; ++k) {
                float b = srgbEncodeF(albedo[p * 3 + k]);
                if (opt.albedoQuantize > 1) {
                  const float n = static_cast<float>(opt.albedoQuantize);
                  b = std::round(b * n) / n;
                }
                B[k] = b;
              }
            } else {
              for (int k = 0; k < 3; ++k)
                B[k] = detail::hatchClamp01(opt.paperColor[k]);
            }
            float I[3];
            if (useAlbedoInk) {
              for (int k = 0; k < 3; ++k)
                I[k] = srgbEncodeF(albedo[p * 3 + k]);
            } else {
              for (int k = 0; k < 3; ++k)
                I[k] = detail::hatchClamp01(opt.inkColor[k]);
            }
            ensureInkContrast(I, B, opt.inkMinContrast);

            // Multiply-composite the layers: f *= 1 - c*(1 - I). Black ink
            // reduces to "over"; colored ink darkens where layers cross
            // (I^2), like real colored pencil / color tone stacking.
            float f[3] = {1.0f, 1.0f, 1.0f};
            const float xc = static_cast<float>(x) + 0.5f;
            const float yc = static_cast<float>(y) + 0.5f;
            for (const detail::HatchLayerRt& L : layers) {
              const float c = detail::hatchLayerInk(L, xc, yc, t) * L.opacity;
              if (c <= 0.0f) continue;
              for (int k = 0; k < 3; ++k) f[k] *= 1.0f - c * (1.0f - I[k]);
            }

            // Write-back with silhouette-coverage AA (mask): multiply-only
            // in BOTH modes, so the ink can only darken -- stroke-edge ink
            // already in the frame is preserved, and premultiplied pixels
            // stay premultiplied (the multiply commutes with the alpha).
            for (int k = 0; k < 3; ++k)
              px4[k] *= (1.0f - m) + m * f[k];
          }
        }
      });
}

}  // namespace umbreon
