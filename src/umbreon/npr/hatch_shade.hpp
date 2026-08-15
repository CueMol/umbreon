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
//   stipple        jittered nested Dot lattice (scientific stippling).
//   screentone-60  classic AM halftone dot screen at 45 deg (K = 0).
//   manga-square   square-element (L-inf) screen at 45 deg.
bool applyHatchPreset(HatchOptions& opt, const std::string& name);

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
void applyHatch(int w, int h, float* rgba, const float* tone,
                const float* mask, const float* albedo,
                const HatchOptions& opt,
                const std::uint16_t* groups = nullptr, int groupSs = 1,
                const GroupHatchStyle* styles = nullptr,
                std::size_t styleCount = 0);

}  // namespace umbreon
