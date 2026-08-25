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
// strokes; Dot = 2D lattice of halftone dots whose RADIUS carries the tone
// (AM screen); Stipple = 2D lattice of fixed-radius dots whose PRESENCE
// carries the tone: every cell owns a hashed threshold and its dot exists
// only where the tone is darker than that threshold (stochastic
// stippling; subdiv nesting and the 50% inversion do not apply).
enum class LayerKind : std::uint8_t { Line = 0, Dot = 1, Stipple = 2 };

// Where the coordinate the marks are laid out in comes from.
//   Screen   the pixel raster (default): strokes keep a fixed screen angle.
//   Analytic a surface parameterization built from the analytic tangent
//            frame of CSG primitives -- the SAME frame the principled
//            anisotropy uses (sphere = meridian off the world +Y pole,
//            cylinder = the axis projected into the tangent plane), so a
//            stick is hatched along its own axis instead of across the
//            screen. Mesh hits have no analytic tangent and fall back to
//            Screen per pixel.
//   Host     a caller-supplied per-pixel UV (FrameResult::hatchUv filled
//            externally, or the `uv` argument of applyHatch). Pixels whose
//            UV is absent fall back to Screen.
// Any source other than Screen fills the hatchUv AOV.
enum class HatchUvSource : std::uint8_t { Screen = 0, Analytic = 1, Host = 2 };

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
  // Let the dots merge past touching up to solid black (the classic AM
  // screen); off stops them at the area-normalized full-cell radius, so
  // the darkest tone keeps white gaps (coverage ~0.9).
  bool invertAbove50 = true;
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
  // Relative mark radius of Dot / Stipple layers (dimensionless; NOT scaled
  // by the supersample factor). 1 = the area-normalized radius that makes a
  // full lattice cover exactly (1 - tone). Dot: a dot GAIN -- the radius
  // grows as if the tone were darker by dotScale^2, so > 1 over-darkens up
  // to solid black through the inversion holes and < 1 never reaches
  // black. Stipple: the fixed dot radius, dotScale * pitch / sqrt(A_p).
  float dotScale = 1.0f;
  float toneHi = 0.95f;     // level-0 appearance threshold
  float toneLo = 0.55f;     // level-K appearance threshold (<= toneHi)
  // Mark grow-in speed below its appearance threshold: a mark reaches full
  // size 1/fadeInv below the tone it appears at. <= 0 = auto: the mark
  // grows linearly from zero at its own threshold to full size at the NEXT
  // nesting level's threshold (toneLo for K = 0), so the layer's coverage
  // is a continuous function of the tone instead of a staircase. Stipple
  // layers treat auto as 32.
  float fadeInv = 16.0f;
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
  // Wrap lighting for the tone only: saturate((N.L + wrap) / (1 + wrap)).
  // A hand drawing does not reproduce a hard terminator -- it wraps the
  // light around the form -- and this also keeps a strong frontal light
  // from clipping the tone flat, since the gradient is redistributed
  // instead of saturating. 0 = plain N.L (the shading model's own).
  float wrap = 0.0f;
  // Contour darkening for the tone only: surfaces turning away from the
  // viewer darken toward the silhouette, which is the shading a draftsman
  // applies to a rounded form and the reason the look survives a flat
  // frontal key light (N.L alone leaves nothing to hatch there).
  //
  //   edge   = (1 - saturate(N.V))^rimPower           how contour-facing
  //   bias   = mix(1, saturate(N.L-ish tone), rimLightBias)
  //   tone  *= 1 - rimDarken * edge * bias
  //
  // rimLightBias is what keeps this from degenerating into a uniform
  // outline: at 0 every silhouette darkens equally (every sphere gets a
  // black ring, and the light direction disappears); at 1 the contour only
  // darkens where the surface is ALSO turning away from the light, so a
  // form reads as lit from somewhere while still being shaded by its form.
  float rimDarken = 0.0f;
  float rimPower = 1.0f;
  float rimLightBias = 0.6f;
  float contactAoPow = 1.0f;   // contact AO exponent (crevices / contacts)
  float shapeAoPow = 0.6f;     // shape AO exponent (domain-scale relief)
  float blackPoint = 0.0f;     // applied at consumption, linear domain
  float whitePoint = 1.0f;
  // Highlight knee: tones at or above `highlightAt` are pushed to exactly
  // 1 (bare paper), with `highlightSoft` as the width of the ramp below it:
  //
  //   t >= at            -> 1
  //   at-soft < t < at   -> smoothstep up to 1
  //
  // This separates the two things whitePoint conflates. whitePoint scales
  // the WHOLE range, so lowering it to force a clean white also lifts every
  // mid tone and the highlight spreads; the knee lifts only the top of the
  // range, so the paper-white area stays exactly as large as `at` says
  // while still being a true 1.0 hole rather than a sparse-stroke area.
  // at >= 1 disables it (the default).
  float highlightAt = 1.0f;
  float highlightSoft = 0.06f;
  float gamma = 1.0f;          // artistic curve, linear domain
  float specularCut = 0.0f;    // >0: blow out to paper where spec exceeds it
  // Tone -> ink-amount mapping, applied in COVERAGE space after the display
  // encode and the highlight knee (the last shaping step before the
  // optional level quantization):
  //   c = clamp01(strength * (1 - t)^curve);  t' = 1 - c
  // strength is a linear ink gain (2 = twice the coverage a display tone
  // asks for, saturating at solid black); curve bends the response with the
  // end points pinned (paper stays paper, black stays black): > 1 keeps the
  // mid tones light (ink pushed toward the shadows), < 1 fills them in.
  // Defaults (1, 1) = coverage linear in the display tone, which is what a
  // classic halftone screen reproduces.
  float strength = 1.0f;
  float curve = 1.0f;
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
  // ... and their mark size (thinner marks when < 1): multiplies widthPx of
  // Line layers and dotScale of Dot / Stipple layers.
  float widthScale = 1.0f;
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
  // Lay the ink at the SUPERSAMPLED resolution (--hatch-res hi, the
  // default) instead of the output resolution. The pixel-unit layer
  // parameters keep their FINAL-resolution meaning -- renderFrame converts
  // them to the hi-res grid -- so the look is invariant under the
  // supersample factor, and the minimum stroke pitch drops from 2 output
  // px to 2/ss output px (ss=4 allows 0.5 px): the box downsample averages
  // sub-pixel strokes into a fine drawing-like grain instead of flooring
  // them at 2 px. false = pixel-exact output-resolution strokes (crisper,
  // but the 2-output-px pitch floor applies).
  bool inkHiRes = true;
  // Coordinate space the marks are laid out in (see HatchUvSource). Screen
  // (the default) keeps the pass byte-identical to a build without UV
  // support.
  HatchUvSource uvSource = HatchUvSource::Screen;
  // Scale from UV units to the pixel units the layer parameters are
  // expressed in: a spacing of `spacingPx` covers `spacingPx / uvScale` of
  // UV. With the Analytic source the UV is in WORLD units, so this is
  // "pixels per world unit" -- roughly the on-screen size of a world unit
  // if the strokes should read at their nominal pixel pitch. renderFrame
  // folds the supersample factor in, exactly as it does for the pixel-unit
  // layer parameters.
  float uvScale = 1.0f;
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
