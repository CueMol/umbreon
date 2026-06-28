// umbreon: a CueMol-embeddable offline renderer backend. Build a Scene
// (geometry, camera, lights, material, fog), call render(), and get a linear
// HDR framebuffer. Depends only on Intel Embree 4 + TBB -- no POV-Ray SDL and
// no OSPRay. This is the public surface intended for static linking into
// libcuemol2; the bench harness drives the same API behind its .pov/.inc parser.
#pragma once

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
// Returns the final LINEAR HDR framebuffer (top-left pixel origin). The pipeline
// itself lives in render/pipeline.hpp; the image post-process helpers
// (boxDownsample / applyAssumedGamma / srgbEncode8) in postprocess/image_ops.hpp.
FrameResult render(const Scene& scene, const RenderOptions& opt);

}  // namespace umbreon
