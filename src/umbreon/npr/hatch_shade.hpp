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

#include <string>

#include "render/hatch_types.hpp"

namespace umbreon {

// Replace opt.layers (and the per-layer mark styles) with a named preset.
// Returns false and leaves opt untouched for an unknown name. Presets:
//   pen-cross      3 hard crosshatch Line layers (45/-45/0 deg), no
//                  perturbation -- the default for a bare --hatch on.
//   pencil, engraving, stipple, screentone-60, manga-square
//                  land with Phase 2 of the plan (Dot marks / perturbations)
//                  and return false until then.
bool applyHatchPreset(HatchOptions& opt, const std::string& name);

// Composite procedural hatching over rgba (w*h*4, DISPLAY-ENCODED, in
// place; run AFTER the gamma encode). tone/mask are the w*h hatch AOVs at
// the same (final) resolution: tone is the linear shading tone (1 = lit),
// mask the surface coverage in [0,1] (0 = background pixel, left untouched;
// fractional silhouette pixels blend the ink by the coverage). albedo
// (w*h*3, linear; nullable) feeds HatchBase::Albedo / HatchInk::FromAlbedo.
// No-op when opt.enable is false. Deterministic: every output value is a
// pure function of its coordinates and the inputs (TBB tiling does not
// change results).
void applyHatch(int w, int h, float* rgba, const float* tone,
                const float* mask, const float* albedo,
                const HatchOptions& opt);

}  // namespace umbreon
