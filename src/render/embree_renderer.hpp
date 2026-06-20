// Embree 4 rendering wrapper: turns a Scene into a rendered framebuffer using
// primary camera rays plus direct POV-Ray-style local shading, with optional
// ambient occlusion (secondary occlusion rays; off by default). No shadows yet
// and no global illumination.
//
// Uses the shared RenderOptions / FrameResult structs (render_types.hpp) so the
// two renderers are interchangeable from the bench harness.
#pragma once

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// Renders a Scene with Intel Embree 4. Owns no persistent device state between
// frames (the device is created and released inside render()).
class EmbreeRenderer {
 public:
  EmbreeRenderer() = default;
  ~EmbreeRenderer() = default;
  EmbreeRenderer(const EmbreeRenderer&) = delete;
  EmbreeRenderer& operator=(const EmbreeRenderer&) = delete;

  FrameResult render(const Scene& scene, const RenderOptions& opt);
};

}  // namespace umbreon
