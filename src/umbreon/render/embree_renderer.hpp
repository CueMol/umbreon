// Embree 4 rendering wrapper: turns a Scene into a rendered framebuffer using
// primary camera rays plus direct POV-Ray-style local shading, with optional
// ambient occlusion and shadows (secondary rays; both off by default). No
// global illumination.
//
// Uses the shared RenderOptions / FrameResult structs (render_types.hpp) so the
// two renderers are interchangeable from the bench harness.
#pragma once

#include <embree4/rtcore.h>

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// Renders a Scene with Intel Embree 4.
//
// LIFETIME: render() builds the Embree device + committed scene (BVH) and, by
// default, KEEPS THEM ALIVE in the renderer until the renderer is destroyed (not
// released at the end of render()). This lets the Freestyle stroke edge pass
// (render/stroke_edges.cpp) cast ray-cast-visibility occlusion queries against
// the live BVH after primary rendering, via occluded(). The default (edges off)
// render is byte-identical: the same image is produced; only the device/scene
// teardown is deferred to the destructor.
class EmbreeRenderer {
 public:
  EmbreeRenderer() = default;
  ~EmbreeRenderer();
  EmbreeRenderer(const EmbreeRenderer&) = delete;
  EmbreeRenderer& operator=(const EmbreeRenderer&) = delete;

  FrameResult render(const Scene& scene, const RenderOptions& opt);

  // Any-hit occlusion test against the live committed scene along the segment
  // P->Q, trimmed by `eps` (relative + absolute floor) at both ends to skip
  // self-hits. True if any geometry occludes the segment in the OPEN interval
  // (matching Freestyle's (0, raylength)). The mesh triangles in
  // `excludeFaces`/`nExclude` (the edge's OWN incident faces) are NOT counted as
  // occluders -- Freestyle self/adjacent-face exclusion -- implemented with an
  // Embree argument intersection filter that rejects hits on the excluded
  // (meshGeomID, primID). Returns false if no scene is currently built.
  bool occluded(const Vec3& p, const Vec3& q, const int* excludeFaces,
                int nExclude, float eps = 1.0e-4f) const;

 private:
  // Release the currently held device/scene (if any). Idempotent.
  void releaseEmbree();

  RTCDevice device_ = nullptr;
  RTCScene scene_ = nullptr;  // committed BVH; owned, released in releaseEmbree()
  // Mesh geometry identity captured at render() time, so occluded() can map an
  // excluded mesh-triangle id to an Embree (geomID, primID) hit. meshGeomID_ is
  // RTC_INVALID_GEOMETRY_ID when the scene has no mesh; meshBaseTriCount_ is the
  // de-indexed base triangle count (primID == triId for the first instance copy;
  // primID % meshBaseTriCount_ recovers the base id when instances are baked).
  unsigned int meshGeomID_ = static_cast<unsigned int>(-1);  // RTC_INVALID_GEOMETRY_ID
  unsigned int meshBaseTriCount_ = 0;
};

}  // namespace umbreon
