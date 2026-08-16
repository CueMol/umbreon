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
    // Three PENCILS (layers) of the same hue, each darker than the last,
    // built from INDIVIDUAL strokes: per-line position scatter off the
    // lattice, per-stroke length / pressure / angle scatter, wobble,
    // tapered ends and paper tooth. The light stick lays the midtone wash,
    // the darker sticks cross in as the tone deepens -- hand shading
    // reaches its darks by SWITCHING pencils, not only by pressing harder.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Line;
    l.spacingPx = 9.0f;
    l.subdiv = 2;
    l.widthPx = 3.6f;              // peak width; the stroke envelope thins it
    l.fadeInv = 10.0f;
    l.opacity = 0.85f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.8f;
    l.mark.toothAmp = 0.15f;
    l.mark.toothScalePx = 3.0f;
    l.mark.jitter = 0.22f;         // per-line position scatter
    l.mark.wobbleAmpPx = 1.2f;
    l.mark.wobbleWavePx = 60.0f;
    l.mark.widthJitter = 0.45f;    // pressure scatter + belly swell
    l.mark.strokeLenPx = 50.0f;
    l.mark.strokeGapPx = 5.0f;
    l.mark.strokeTaper = 0.35f;    // asymmetric entry/tail scale
    l.mark.angleJitterDeg = 5.0f;  // coherent drift + per-stroke scatter
    l.mark.strokeLenJitter = 0.5f;
    l.angleDeg = 55.0f;            // light stick: midtone wash
    l.toneHi = 0.92f;
    l.toneLo = 0.55f;
    l.inkScale = 1.0f;
    opt.layers.push_back(l);
    l.angleDeg = -35.0f;           // darker stick crosses in
    l.toneHi = 0.62f;
    l.toneLo = 0.30f;
    l.inkScale = 0.62f;
    l.mark.seed = 1;               // decorrelate the layers' strokes
    opt.layers.push_back(l);
    l.angleDeg = 80.0f;            // darkest stick: shadow cores
    l.toneHi = 0.34f;
    l.toneLo = 0.12f;
    l.inkScale = 0.38f;
    l.mark.seed = 2;
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

bool applyHatchLook(HatchOptions& opt, const std::string& name) {
  if (name == "richardson") {
    // Jane-Richardson-style colored-pencil ribbon drawing: NO flat fill --
    // the paper carries the highlights, the midtones are stroke density
    // and the darks are denser strokes of a darker pencil of the same hue
    // (the object's own color). Tuned against the published TIM drawings.
    applyHatchPreset(opt, "pencil");
    opt.mode = HatchMode::Ink;
    opt.base = HatchBase::Paper;     // no flat fill: bare paper between marks
    opt.ink = HatchInk::FromAlbedo;  // each section draws in its own color
    opt.paperColor[0] = 0.941f;      // warm drawing paper
    opt.paperColor[1] = 0.925f;
    opt.paperColor[2] = 0.867f;
    opt.inkMinContrast = 0.15f;
    opt.inkShadeDark = 0.28f;  // pencil pressure: shadows darken the stick
    opt.toneFog = true;        // far side fades into the paper
    // Lighting-only tone: a low ambient floor keeps the shadows readable,
    // the compressed white point opens the lit side up to bare paper, and
    // the specular cut punches the highlight through as pure paper.
    opt.tone.diffuseWeight = 0.85f;
    opt.tone.ambient = 0.05f;
    // A drawing does not reproduce a hard terminator, and a hand-drawn
    // figure is lit flatly from the front: wrap softens the terminator and
    // keeps a frontal key from clipping the tone flat, while the rim term
    // supplies the shading that follows the FORM (darkening toward each
    // silhouette) rather than one light direction.
    opt.tone.wrap = 0.5f;
    opt.tone.rimDarken = 1.0f;      // strong contour shading
    opt.tone.rimPower = 1.4f;       // reaching in from the silhouette
    opt.tone.rimLightBias = 0.35f;  // biased to each form's shaded side
    opt.tone.contactAoPow = 1.0f;
    opt.tone.shapeAoPow = 0.6f;
    // Do NOT clip the lit end: whitePoint below 1 turns every gently lit
    // face into bare paper, so the highlights spread into large white
    // holes. Opening it past 1 keeps the light side as sparse strokes and
    // leaves the paper for the true highlights only; the higher gamma
    // restores the dark end that the wider range would otherwise lift.
    opt.tone.whitePoint = 1.2f;
    opt.tone.gamma = 2.4f;
    // The specular blow-out is off by default for the same reason: on a
    // broad-lobe finish it paints a large white patch rather than a
    // highlight. Raise it per scene if a crisp glint is wanted.
    opt.tone.specularCut = 0.0f;
    // Highlights: a narrow band at the top of the range goes to EXACT
    // paper white. The knee sets where that band starts, so the white is
    // clean without the highlight spreading (which is what lowering
    // whitePoint would do).
    opt.tone.highlightAt = 0.86f;
    opt.tone.highlightSoft = 0.05f;
    // Fine strokes: with the default supersampled ink these are OUTPUT
    // pixels, so ss decides how far below one pixel they actually land.
    for (HatchLayer& l : opt.layers) {
      l.spacingPx = 0.5f;   // dense drawing grain (OUTPUT px; see hatch-res)
      l.widthPx = 0.45f;
      l.mark.edgeSoftness = 0.55f;
      l.opacity = 1.0f;
    }
    return true;
  }
  if (name == "ink-cross") {
    // The plain pen-and-ink look: white paper, black crosshatch.
    applyHatchPreset(opt, "pen-cross");
    opt.mode = HatchMode::Ink;
    opt.base = HatchBase::Paper;
    opt.ink = HatchInk::Fixed;
    for (int k = 0; k < 3; ++k) {
      opt.inkColor[k] = 0.0f;
      opt.paperColor[k] = 1.0f;
    }
    opt.inkMinContrast = 0.25f;
    opt.inkShadeDark = 1.0f;
    opt.tone = ToneRecipe{};
    return true;
  }
  if (name == "manga") {
    // Flat section fill under a halftone screen, hard black ink.
    applyHatchPreset(opt, "screentone-60");
    opt.mode = HatchMode::Ink;
    opt.base = HatchBase::Albedo;  // flat fill, unshaded
    opt.ink = HatchInk::Fixed;
    for (int k = 0; k < 3; ++k) {
      opt.inkColor[k] = 0.0f;
      opt.paperColor[k] = 1.0f;
    }
    opt.albedoQuantize = 4;
    opt.inkMinContrast = 0.35f;
    opt.inkShadeDark = 1.0f;
    opt.tone = ToneRecipe{};
    opt.tone.ambient = 0.10f;
    return true;
  }
  return false;
}

void applyHatch(int w, int h, float* rgba, const float* tone,
                const float* mask, const float* albedo,
                const HatchOptions& opt, const std::uint16_t* groups,
                int groupSs, const GroupHatchStyle* styles,
                std::size_t styleCount) {
  if (!opt.enable || w <= 0 || h <= 0 || rgba == nullptr ||
      tone == nullptr || mask == nullptr)
    return;
  const bool perSection =
      groups != nullptr && styles != nullptr && styleCount > 0;
  const int gss = groupSs < 1 ? 1 : groupSs;
  const std::size_t gW = static_cast<std::size_t>(w) * gss;

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
  // Sections that rescale the mark density / width need their own
  // normalized layer set (the pitch feeds the min-feature clamp and every
  // search window, so it cannot be applied at evaluation time). Sections at
  // the neutral scales share the global set.
  std::vector<std::vector<detail::HatchLayerRt>> sectionLayers;
  if (perSection) {
    sectionLayers.resize(styleCount);
    for (std::size_t si = 0; si < styleCount; ++si) {
      const GroupHatchStyle& g = styles[si];
      const float dens = std::max(0.1f, std::min(8.0f, g.density));
      const float wsc = std::max(0.1f, std::min(8.0f, g.widthScale));
      if (dens == 1.0f && wsc == 1.0f) continue;  // reuse the global set
      sectionLayers[si].reserve(opt.layers.size());
      for (std::size_t li = 0; li < opt.layers.size(); ++li) {
        HatchLayer L = opt.layers[li];
        if (L.opacity <= 0.0f) continue;
        L.spacingPx /= dens;
        L.widthPx *= wsc;
        // Perturbation amplitudes are pitch-relative in spirit; scale the
        // absolute-pixel ones with the pitch so a denser section keeps the
        // same look instead of turning into noise.
        L.mark.wobbleAmpPx /= dens;
        L.mark.strokeLenPx /= dens;
        L.mark.strokeGapPx /= dens;
        sectionLayers[si].push_back(
            detail::hatchNormalizeLayer(L, static_cast<std::uint32_t>(li)));
      }
    }
  }

  const ToneRecipe& tr = opt.tone;
  const float wpRange = std::max(1.0e-4f, tr.whitePoint - tr.blackPoint);

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

            // Per-section style: sample the hi-res id buffer at the cell
            // center. A disabled section keeps its frame color untouched.
            const GroupHatchStyle* st = nullptr;
            const std::vector<detail::HatchLayerRt>* lay = &layers;
            if (perSection) {
              const std::size_t gp =
                  (static_cast<std::size_t>(y) * gss + gss / 2) * gW +
                  static_cast<std::size_t>(x) * gss + gss / 2;
              const std::uint16_t g = groups[gp];
              if (g != 0xFFFFu && g < styleCount) {
                st = &styles[g];
                if (!st->enable) continue;
                if (!sectionLayers[g].empty()) lay = &sectionLayers[g];
              }
            }

            // Tone shaping: per-section scale -> linear black/white remap ->
            // artistic gamma (linear domain) -> display encode -> optional
            // quantization. Every stage maps 1 -> 1 (toneScale >= 1 lifts
            // toward paper and is clamped), so fully lit stays ink-free.
            float tLin = tone[p];
            if (st != nullptr) tLin *= st->toneScale;
            float t = detail::hatchClamp01((tLin - tr.blackPoint) / wpRange);
            if (tr.gamma != 1.0f) t = std::pow(t, tr.gamma);
            t = srgbEncodeF(t);
            // Highlight knee: lift only the top of the range to exact
            // paper white, leaving the mid tones (and so the highlight's
            // AREA) where the curve above put them.
            if (tr.highlightAt < 1.0f) {
              const float lo =
                  std::max(0.0f, tr.highlightAt - std::max(1.0e-4f,
                                                           tr.highlightSoft));
              const float k = detail::hatchSmoothstep(lo, tr.highlightAt, t);
              t = t + (1.0f - t) * k;
            }
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
            const HatchBase baseSel = st != nullptr ? st->base : opt.base;
            const HatchInk inkSel = st != nullptr ? st->ink : opt.ink;
            const float* inkFixed =
                st != nullptr ? st->inkColor : opt.inkColor;
            // Contrast reference = what this pixel actually shows between
            // the marks. In Over mode that is the shaded frame; in Ink mode
            // the pipeline already painted the flat base into the frame, so
            // the frame is the base there too -- reading it (instead of
            // re-deriving the base from the options) keeps the reference
            // exact under fog, edge ink and any display transfer. Under a
            // transparent background rgba is premultiplied; the multiply
            // composite commutes with it (see the write-back below).
            float B[3] = {px4[0], px4[1], px4[2]};
            if (opt.mode == HatchMode::Ink && baseSel == HatchBase::Paper &&
                albedo == nullptr) {
              // Standalone callers may hand us an unpainted canvas; fall
              // back to the configured paper.
              for (int k = 0; k < 3; ++k)
                B[k] = detail::hatchClamp01(opt.paperColor[k]);
            }
            float I[3];
            if (inkSel == HatchInk::FromAlbedo && albedo != nullptr) {
              // Bring the LINEAR albedo into the frame's display space with
              // the pipeline's own transfer, so ink and base agree.
              for (int k = 0; k < 3; ++k) {
                const float a = std::max(0.0f, albedo[p * 3 + k]);
                I[k] = detail::hatchClamp01(
                    opt.displayGamma != 1.0f ? std::pow(a, opt.displayGamma)
                                             : a);
              }
            } else {
              for (int k = 0; k < 3; ++k)
                I[k] = detail::hatchClamp01(inkFixed[k]);
            }
            // Colored-pencil pressure: darken the ink with the tone (the
            // mark geometry stays tone-free; see HatchOptions::inkShadeDark).
            if (opt.inkShadeDark < 1.0f) {
              const float sh =
                  opt.inkShadeDark + (1.0f - opt.inkShadeDark) * t;
              for (int k = 0; k < 3; ++k) I[k] *= sh;
            }
            ensureInkContrast(I, B, opt.inkMinContrast);

            // Multiply-composite the layers: f *= 1 - c*(1 - I). Black ink
            // reduces to "over"; colored ink darkens where layers cross
            // (I^2), like real colored pencil / color tone stacking.
            float f[3] = {1.0f, 1.0f, 1.0f};
            const float xc = static_cast<float>(x) + 0.5f;
            const float yc = static_cast<float>(y) + 0.5f;
            for (const detail::HatchLayerRt& L : *lay) {
              if (st != nullptr && L.layerId < 31u &&
                  ((st->layerMask >> L.layerId) & 1) == 0)
                continue;  // layer disabled for this section
              const float c = detail::hatchLayerInk(L, xc, yc, t) * L.opacity;
              if (c <= 0.0f) continue;
              // Each layer is a pencil: later (shadow) layers may draw with
              // a darker stick of the same hue (HatchLayer::inkScale).
              for (int k = 0; k < 3; ++k) {
                const float ik = I[k] * L.inkScale;
                f[k] *= 1.0f - c * (1.0f - ik);
              }
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
