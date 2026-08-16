// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// NPR tone-hatching option types (--hatch): procedural Tonal Art Map layers
// (lines / halftone dots), the shading-tone recipe and the paper/ink model.
// Re-exported through render/render_types.hpp. Pure C++17 data, no
// rendering-library dependency.
//
// Color convention: HatchOptions::inkColor / paperColor (and
// GroupHatchStyle::inkColor) are DISPLAY-ENCODED values -- the hatch
// composite runs AFTER applyAssumedGamma, on the display-encoded frame,
// like the group-alpha blendpng-equivalent blend. This differs from
// EdgeClassStyle::color, which is linear (composited before the gamma).
// Pure black {0,0,0} and white {1,1,1} are identical in both conventions.
#pragma once

#include <cstdint>
#include <vector>

namespace umbreon {

// What the hatch pass does with the shaded frame color.
// Ink: pure ink drawing -- renderFrame paints the UNSHADED flat base
//      (paper / first-hit albedo) over every surface pixel before fog and
//      the stroke edge pass, so contour ink lands on the flat base and the
//      final hatch composite only multiplies its ink in (GI/denoiser are
//      normalized off; tone is carried by hatch density alone).
// Over: the hatch is composited over the shaded (display-encoded) frame,
//      which stays visible between the marks (toon-like styles).
enum class HatchMode : std::uint8_t { Ink = 0, Over = 1 };

// Base (background of the drawing) selection for HatchMode::Ink.
enum class HatchBase : std::uint8_t { Paper = 0, Albedo = 1 };

// Ink color source: a fixed color, or the first-hit albedo (darkened by the
// contrast guarantee so the marks stay visible against their own base).
enum class HatchInk : std::uint8_t { Fixed = 0, FromAlbedo = 1 };

// Mark lattice kind of one hatch layer. Line = 1D lattice of parallel
// strokes; Dot = 2D lattice of halftone dots / stipple points.
enum class LayerKind : std::uint8_t { Line = 0, Dot = 1 };

// Mark appearance and hand-drawn perturbation parameters, orthogonal to the
// lattice/nesting logic. HARD CONSTRAINT: every perturbation is a pure
// function of the lattice index hash and the along-mark coordinate -- never
// of the tone -- so marks never move or pop when the shading changes.
// (Dot marks and every perturbation except edgeSoftness are consumed from
// Phase 2 on; the fields are declared now so the option struct is stable.)
struct MarkStyle {
  // --- shared ---
  float edgeSoftness = 0.5f;  // AA half-width, FINAL px (clamped to [0.5, 2])
  float toothAmp = 0.0f;      // paper-tooth noise amplitude on the coverage
  float toothScalePx = 3.0f;  // paper-tooth spatial scale
  unsigned seed = 0;          // explicit seed for the deterministic hashes
  // --- Dot / Line shared ---
  // Lattice-position scatter in units of the lattice pitch (max 0.5).
  // Dots: 2D center jitter (0 = halftone screen, ~0.4 = stipple).
  // Lines: per-line offset from the exact lattice (breaks the metronomic
  // spacing of hand hatching).
  float jitter = 0.0f;
  // --- Dot ---
  float shapeExponent = 2.0f;  // Lp exponent: 1=diamond, 2=circle, >=16=square
  float dotAspect = 1.0f;      // ellipse stretch
  float dotAngleDeg = 0.0f;    // rotation of non-circular marks
  bool invertAbove50 = true;   // grow white holes past ~50% coverage
  // --- Line ---
  float wobbleAmpPx = 0.0f;   // along-line 1D-noise displacement amplitude
  float wobbleWavePx = 40.0f; // its wavelength
  // Relative width modulation (max 1). With finite strokes it also drives
  // the per-stroke pressure scatter (each stroke's width and darkness).
  float widthJitter = 0.0f;
  float strokeLenPx = 0.0f;   // finite stroke length; 0 = continuous line
  float strokeGapPx = 0.0f;   // gap between strokes
  float strokeTaper = 0.3f;   // taper fraction at stroke ends
  // Per-stroke individuality (finite strokes only): angle scatter in
  // degrees (each stroke pivots around its own center; max 15) and
  // relative stroke-length variation (max 0.9).
  float angleJitterDeg = 0.0f;
  float strokeLenJitter = 0.0f;
};

// One procedural Tonal Art Map layer. Marks sit on a lattice of pitch
// spacingPx / 2^subdiv; each lattice index j appears at nesting level
// firstLevel(j, subdiv) with tone threshold
//   t(lv) = toneHi + (toneLo - toneHi) * lv / subdiv     (toneHi at lv 0),
// so darkening the tone only ever INSERTS marks between existing ones --
// marks never disappear or move (the TAM nesting guarantee).
// All lengths are FINAL-resolution pixels; tone thresholds are compared in
// the display-encoded domain (after ToneRecipe and srgbEncodeF).
struct HatchLayer {
  LayerKind kind = LayerKind::Line;
  float angleDeg = 45.0f;   // line direction / dot lattice rotation
  float spacingPx = 10.0f;  // base lattice pitch S (level-0 marks)
  int subdiv = 2;           // K: number of nesting subdivision levels
  float widthPx = 1.1f;     // full line width (Line layers)
  float toneHi = 0.95f;     // level-0 appearance threshold
  float toneLo = 0.55f;     // level-K appearance threshold (<= toneHi)
  float fadeInv = 16.0f;    // mark grow-in speed below its threshold
  float opacity = 1.0f;     // layer ink opacity multiplier
  // Ink darkness of THIS layer relative to the resolved ink color: layers
  // are pencils, and a hand drawing reaches its dark tones by switching to
  // a DARKER pencil for the later (shadow) layers, not only by pressing
  // harder. 1 = the shared ink; ~0.5 = a distinctly darker pencil.
  float inkScale = 1.0f;
  MarkStyle mark;
};

// How the scalar shading tone is built inside the hit shader. The tone is
// intentionally NOT the color luminance: chain/SS coloring must not leak
// into the shading (a dark-blue helix would hatch black, a yellow one
// white). Lighting only: per-light saturate(N.L)^brilliance x shadow x
// light luminance, an ambient floor, and the AO split.
struct ToneRecipe {
  float diffuseWeight = 1.0f;  // weight of the summed per-light diffuse
  float ambient = 0.12f;       // floor so shadows do not crush to black
  float contactAoPow = 1.0f;   // contact AO exponent (crevices / contacts)
  float shapeAoPow = 0.6f;     // shape AO exponent (domain-scale relief)
  float blackPoint = 0.0f;     // applied at consumption, linear domain
  float whitePoint = 1.0f;
  float gamma = 1.0f;          // artistic curve, linear domain
  float specularCut = 0.0f;    // >0: blow out to paper where spec exceeds it
};

// Per CueMol section (transparency group) hatch styling override, indexed by
// group id like Scene::groupEdgeStyle. Consumed from Phase 3 on; declared now
// so the Scene layout is stable.
struct GroupHatchStyle {
  bool enable = true;
  HatchBase base = HatchBase::Paper;
  HatchInk ink = HatchInk::Fixed;
  float inkColor[3] = {0.0f, 0.0f, 0.0f};  // display-encoded
  int layerMask = 0x7;    // which HatchOptions::layers apply to this section
  float toneScale = 1.0f; // per-section tone lift/drop
  // Mark density of this section relative to the global layers: every
  // layer's lattice pitch is DIVIDED by it, so 2 means twice as many lines
  // / dots. Small sections (ligands, sticks) need a finer grain than a
  // ribbon to carry any tone at all -- with the global pitch a thin stick
  // may catch only one or two marks and read as flat.
  float density = 1.0f;
  float widthScale = 1.0f;  // and their mark width (thinner marks when < 1)
};

// Master options for the tone-hatching pass (--hatch).
// The `enable` flag gates ALL new work: buffers, shader tone taps and the
// final applyHatch composite. A default-constructed HatchOptions keeps the
// render byte-identical to a build without the feature.
struct HatchOptions {
  bool enable = false;  // MASTER gate; false => zero new work, byte-identical
  HatchMode mode = HatchMode::Ink;
  HatchBase base = HatchBase::Paper;  // Ink-mode base (Over uses frame color)
  HatchInk ink = HatchInk::Fixed;
  float inkColor[3] = {0.0f, 0.0f, 0.0f};    // display-encoded
  float paperColor[3] = {1.0f, 1.0f, 1.0f};  // display-encoded
  // Minimum display-luminance separation between base and ink. If the ink is
  // not darker than the base by this much it is darkened (hue-preserving);
  // on a base darker than the contrast itself the ink is instead LIFTED to
  // base + contrast (bright hatching on dark ground, woodcut-like).
  float inkMinContrast = 0.25f;
  // Tone-driven ink darkening (colored-pencil pressure): the ink color is
  // scaled by mix(inkShadeDark, 1, displayTone), so strokes in lit areas
  // keep the light tint while shadow strokes darken toward
  // ink * inkShadeDark -- like pressing harder with the same pencil. 1
  // (default) = constant ink. Mark GEOMETRY never depends on tone, so the
  // TAM nesting guarantee is untouched, and the per-pixel ink stays
  // monotone (darker tone => more coverage AND darker ink).
  float inkShadeDark = 1.0f;
  // Fade the TONE toward paper with the scene fog factor (renderFrame,
  // before the downsample): distant strokes thin out and -- via
  // inkShadeDark -- lighten, matching the fogged silhouette ink, the way a
  // drawn figure fades its far side. Only active when the scene has fog.
  bool toneFog = true;
  // Lay the ink at the SUPERSAMPLED resolution instead of the output
  // resolution (--hatch-res hi). The box downsample then averages the
  // strokes: their pitch effectively drops below the 2-output-px floor
  // (2/ss px), trading stroke crispness for a finer, softer grain --
  // individual lines partially merge into an accurate coverage tone, like
  // viewing a large drawing from farther away. The default (false) keeps
  // pixel-exact output-resolution strokes; prefer rendering LARGER with the
  // default when the fine strokes themselves should stay resolvable.
  bool inkHiRes = false;
  int toneLevels = 0;      // >1: quantize the encoded tone to N levels
  int albedoQuantize = 0;  // >1: posterize the Albedo base to N steps
  // Copied from RenderOptions::transparentBackground by renderFrame so the
  // standalone applyHatch() call sees the premultiplied-background rule
  // (background pixels are never painted; partially covered pixels scale the
  // hatch by the pixel's coverage alpha).
  bool transparentBackground = false;
  // Display transfer the frame was encoded with (Scene::assumedGamma; set by
  // renderFrame). applyHatch needs it to bring the LINEAR albedo AOV into
  // the same display space as the frame it composites into -- the encode
  // must be the pipeline's own, not a fixed sRGB curve, or albedo-derived
  // ink lands at a different brightness than the base it draws on.
  // 1.0 (the default) means "albedo is already display-encoded".
  float displayGamma = 1.0f;
  ToneRecipe tone;
  // TAM layers, multiply-composited. Empty (the default) is normalized by
  // renderFrame to the "pen-cross" preset (applyHatchPreset).
  std::vector<HatchLayer> layers;
  // Set by renderFrame's normalization when any per-section style
  // (Scene::groupHatchStyle) derives its base or ink from the albedo, so
  // the albedo AOV is captured even though the GLOBAL base/ink do not need
  // it. Library callers can leave it false.
  bool sectionNeedsAlbedo = false;

  // True when the pass reads the albedo AOV (base or ink derives from it).
  bool needsAlbedo() const {
    return base == HatchBase::Albedo || ink == HatchInk::FromAlbedo ||
           sectionNeedsAlbedo;
  }
};

}  // namespace umbreon
