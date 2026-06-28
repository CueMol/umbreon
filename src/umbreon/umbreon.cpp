#include "umbreon.hpp"

#include "render/pipeline.hpp"

namespace umbreon {

// Thin public entry point: the full frame pipeline lives in render/pipeline.cpp
// (renderFrame), and the image post-process helpers in postprocess/image_ops.cpp.
FrameResult render(const Scene& scene, const RenderOptions& opt) {
  return renderFrame(scene, opt);
}

}  // namespace umbreon
