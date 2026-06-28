// The umbreon frame pipeline: orchestrates a single render -- supersample ->
// Embree primary-ray direct shading -> POV ground fog -> Freestyle stroke edges
// -> linear box-downsample -> assumed_gamma -> final linear HDR FrameResult.
// Split out of umbreon.cpp so the public umbreon.cpp stays a thin entry point.
// Internal API (not installed); the public surface is umbreon::render().
#pragma once

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// Run the full frame pipeline at opt.supersample and return the final LINEAR HDR
// framebuffer (top-left pixel origin). opt.width/height are the FINAL output size.
FrameResult renderFrame(const Scene& scene, const RenderOptions& opt);

}  // namespace umbreon
