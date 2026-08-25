// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// Tone-hatching ink composite (--hatch): the renderer-agnostic image op that
// turns the shading-tone / coverage AOVs into hatched ink over paper, and
// the named mark-style presets. Deliberately a plain-pointer API (unlike the
// internal applyStrokeEdges, which takes a FrameResult): a host can feed a
// hand-painted or retouched tone, and the nesting-monotonicity contract is
// unit-testable with a synthetic gradient alone.
// Design record: docs/plans/npr-tone-hatching.md.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "render/hatch_types.hpp"

namespace umbreon {

// Replace opt.layers (and the per-layer mark styles) with a named preset.
// Returns false and leaves opt untouched for an unknown name. Presets
// (concrete values: docs/plans/npr-tone-hatching.md section 6.5):
//   pen-cross      3 hard crosshatch Line layers (45/-45/0 deg), no
//                  perturbation -- the default for a bare --hatch on.
//   pencil         2 soft graphite Line layers: wobble, width modulation,
//                  finite tapered strokes, paper tooth.
//   engraving      1 deeply subdivided Line direction (copperplate).
//   stipple        jittered Stipple lattice: fixed-size dots whose count
//                  carries the tone (scientific stippling).
//   screentone-60  classic AM halftone dot screen at 45 deg (K = 0).
//   manga-square   square-element (L-inf) screen at 45 deg.
bool applyHatchPreset(HatchOptions& opt, const std::string& name);

// Tone recipe that goes with a mark preset: how the shading tone is built
// (wrap / rim / gamma / white point ...) so the preset's marks read a
// molecular figure at their intended density out of the box. Writes `out`
// and returns true for a known preset name, false (out untouched) otherwise.
// applyHatchPreset itself never touches the tone (API compatibility);
// applyHatchStyle combines the two.
bool hatchPresetTone(const std::string& presetName, ToneRecipe& out);

// Resolve a style NAME the way a host's single style selector should: a
// look (applyHatchLook) wins; otherwise a mark preset (applyHatchPreset)
// plus its hatchPresetTone recipe. The preset path leaves the paper / ink
// model alone (the caller decides those). Returns false, opt untouched,
// for an unknown name.
bool applyHatchStyle(HatchOptions& opt, const std::string& name);

// Replace the WHOLE hatch configuration with a named look: paper / ink
// model, tone recipe, pencil pressure and the mark layers together. Mark
// presets (applyHatchPreset) only choose the layers; a look additionally
// fixes how the tone is built and how the ink is colored, which is what a
// recognizable drawing style actually is. Returns false for an unknown
// name, leaving opt untouched. Looks:
//   richardson   Jane-Richardson-style colored-pencil ribbon drawing:
//                paper base + per-object-color ink (no flat fill), three
//                pencils of one hue, highlights left as bare paper,
//                lighting-only tone with a compressed white point.
//   ink-cross    the plain pen-and-ink default: white paper, black
//                crosshatch, neutral tone recipe.
//   manga        flat section fill + screentone dots and a hard ink line.
bool applyHatchLook(HatchOptions& opt, const std::string& name);

// ---- key=value access and the spec text ------------------------------------
// One textual form of a hatch configuration, shared by the CLI
// (--hatch-layer / --hatch-tone / --hatch-spec) and embedding hosts that
// let a user edit a style: line-oriented,
//   layer: kind=line,angle=45,spacing=10,width=1.1,...   (one line per layer)
//   tone:  strength=1,curve=1,wrap=0.5,rim=1,...
//   ink:   base=paper,ink=fixed,inkcolor=#000000,mincontrast=0.25,...
// Lines are separated by '\n' (or ';'), entries by ','; blank lines and
// '#' comments are ignored. Keys are lower-case; numbers use the C locale;
// booleans are on/off (1/0, true/false accepted); colors are #rrggbb.
//   layer keys: kind(line|dot|stipple) angle spacing subdiv width dotscale
//               tonehi tonelo fade opacity inkscale soft seed shape aspect
//               dotangle jitter invert wobble wobwave wjitter slen sgap
//               taper anglejitter lenjitter tooth toothscale
//   tone keys:  diffuse ambient wrap rim rimpow rimbias contact shape black
//               white hl hlsoft gamma speccut strength curve levels
//   ink keys:   mode(ink|over) base(paper|albedo) ink(fixed|albedo)
//               inkcolor papercolor mincontrast inkshade tonefog albedoquant
// Render-setup fields (inkHiRes, uvSource, uvScale, displayGamma,
// transparentBackground) are not part of the text.

// Apply one key=value pair. Returns false (target untouched) for an unknown
// key or an unparsable value.
bool applyHatchLayerKv(HatchLayer& layer, const std::string& key,
                       const std::string& value);
bool applyHatchToneKv(ToneRecipe& tone, const std::string& key,
                      const std::string& value);
// The ink / paper model keys, plus `levels` (HatchOptions::toneLevels) and
// `albedoquant`.
bool applyHatchInkKv(HatchOptions& opt, const std::string& key,
                     const std::string& value);

// Which sections of a spec text to read / write.
enum HatchSpecSection : unsigned {
  kHatchSpecLayers = 1u,
  kHatchSpecTone = 2u,
  kHatchSpecInk = 4u,
  kHatchSpecAll = 7u,
};

// Apply a spec text. `layer:` lines REPLACE opt.layers (in line order) when
// at least one is present; `tone:` / `ink:` lines override the named keys
// only. Sections outside `sections` are skipped. On any error opt is left
// untouched, `error` (optional) receives "line N: ..." and false is
// returned.
bool applyHatchSpec(HatchOptions& opt, const std::string& text,
                    unsigned sections = kHatchSpecAll,
                    std::string* error = nullptr);

// Serialize a configuration to the spec text: one `layer:` line per layer
// (keys filtered by the layer kind), then `tone:`, then `ink:`, each
// selected by `sections`; '\n'-separated with a trailing newline. Feeding
// the result to applyHatchSpec reproduces the configuration, and
// re-serializing gives the same text.
std::string hatchStyleToSpec(const HatchOptions& opt,
                             unsigned sections = kHatchSpecAll);

// Composite procedural hatching over rgba (w*h*4, DISPLAY-ENCODED, in
// place; run AFTER the gamma encode). rgba is the BASE CANVAS the ink
// multiplies into -- the composite only ever darkens, so contour ink
// already present survives. In the renderer's Ink mode the pipeline paints
// the flat paper/albedo base into the frame BEFORE fog and the stroke edge
// pass (a standalone caller supplies its own base image); Over mode
// multiplies into the shaded frame as-is. tone/mask are the w*h hatch AOVs
// at the same (final) resolution: tone is the linear shading tone
// (1 = lit), mask the surface coverage in [0,1] (0 = background pixel,
// left untouched; fractional silhouette pixels blend the ink by the
// coverage). albedo (w*h*3, linear; nullable) feeds the FromAlbedo ink and
// the Albedo contrast reference. No-op when opt.enable is false.
// Deterministic: every output value is a pure function of its coordinates
// and the inputs (TBB tiling does not change results).
//
// Per-section styling (all optional; pass nullptr/0 for the global path):
// groups is the HI-RES section-id buffer (w*groupSs x h*groupSs, 0xFFFF =
// background; FrameResult::hatchGroup) sampled at each output pixel's cell
// center, and styles/styleCount the per-section table
// (Scene::groupHatchStyle). A section with enable == false keeps its frame
// color untouched; otherwise its base/ink/layerMask/toneScale override the
// global options.
// `uv` (w*h*2, nullable) is the surface parameterization the marks are laid
// out in -- FrameResult::hatchUv, or any coordinate field the caller
// produces itself. Where it is null, or a pixel's pair is exactly (0, 0),
// that pixel falls back to screen coordinates. HatchOptions::uvScale maps
// UV units to the pixel units the layer parameters use.
void applyHatch(int w, int h, float* rgba, const float* tone,
                const float* mask, const float* albedo,
                const HatchOptions& opt,
                const std::uint16_t* groups = nullptr, int groupSs = 1,
                const GroupHatchStyle* styles = nullptr,
                std::size_t styleCount = 0, const float* uv = nullptr);

}  // namespace umbreon
