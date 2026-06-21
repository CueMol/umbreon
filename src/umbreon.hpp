// umbreon: a CueMol-embeddable offline renderer backend. Build a Scene
// (geometry, camera, lights, material, fog), call render(), and get a linear
// HDR framebuffer. Depends only on Intel Embree 4 + TBB -- no POV-Ray SDL and
// no OSPRay. This is the public surface intended for static linking into
// libcuemol2; the bench harness drives the same API behind its .pov/.inc parser.
#pragma once

#include <cstdint>
#include <vector>

#include "render/render_types.hpp"
#include "scene.hpp"

// Library version (kept in sync with project() in CMakeLists.txt).
#define UMBREON_VERSION_MAJOR 0
#define UMBREON_VERSION_MINOR 1
#define UMBREON_VERSION_PATCH 0

namespace umbreon {

// High-level render. Runs: supersample (opt.supersample) -> Embree primary-ray
// direct shading -> POV ground fog (scene.fog) -> linear box-downsample ->
// assumed_gamma (opt.assumedGamma). opt.width/height are the FINAL output size.
// Returns the final LINEAR HDR framebuffer (top-left pixel origin).
FrameResult render(const Scene& scene, const RenderOptions& opt);

// --- Post-process utilities (also reused by the bench harness) -------

// Box-average a linear-space image from w x h down to (w/ss) x (h/ss).
std::vector<float> boxDownsample(const std::vector<float>& src, int w, int h,
                                 int channels, int ss);

// Raise the RGB channels to the power g in place (POV assumed_gamma; g == 1 is a
// no-op). Alpha is left unchanged.
void applyAssumedGamma(FrameResult& frame, float g);

// Encode the linear RGBA framebuffer to interleaved 8-bit sRGB bytes for display
// or hand-off to CueMol. channels is 3 (RGB) or 4 (RGBA; alpha stored linear).
std::vector<std::uint8_t> srgbEncode8(const FrameResult& frame, int channels);

}  // namespace umbreon
