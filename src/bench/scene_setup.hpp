// CLI Options -> Scene / RenderOptions translation, split out of main.cpp.
// The functions run in this order (matching the historical main() blocks):
//   1. buildSceneFromPov     .pov viewing setup + geometry + group alpha
//   2. applyEdgeOptions      Freestyle stroke edges + baked-edge removal
//   3. applyShadingOptions   transparency/AO/GI/denoise/supersample/AA wiring
//   4. applyObjectSpaceEdges analytic object-space edges (--obj-edges)
#pragma once

#include <string>
#include <vector>

#include "cli.hpp"
#include "umbreon.hpp"

namespace umbreon {

// Reproduce the CueMol POV-Ray viewing setup: read the .pov + its geometry
// include, fill `scene` (mesh/primitives/camera/lights/ambient/fog/gamma,
// group alpha) and the output size in `ropt`, and report to stdout.
// `groupNames` receives the section names for the later --edge resolution.
// Returns false when --list-groups printed its report (caller exits 0).
// Throws on input errors. May adjust opt.declares (GI energy rebalance).
bool buildSceneFromPov(Options& opt, Scene& scene, RenderOptions& ropt,
                       std::vector<std::string>& groupNames);

// Freestyle STROKE edges (--edges): wire the --stroke-* knobs, seed the
// per-section styling and remove the baked POV edge primitives that the
// generated strokes replace. No-op (byte-identical default) with edges off.
void applyEdgeOptions(const Options& opt, Scene& scene, RenderOptions& ropt,
                      const std::vector<std::string>& groupNames);

// Transparency / AO / GI / denoiser / supersample / adaptive-AA wiring from
// the parsed CLI options, with the configuration summary prints.
void applyShadingOptions(const Options& opt, const Scene& scene,
                         RenderOptions& ropt);

// Analytic OBJECT-SPACE edges (--obj-edges): baked-edge removal + options,
// including the --obj-edge-only verification path (which mutates the scene).
void applyObjectSpaceEdges(const Options& opt, Scene& scene,
                           RenderOptions& ropt);

}  // namespace umbreon
