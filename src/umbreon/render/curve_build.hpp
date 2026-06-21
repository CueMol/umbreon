// Cylinder geometry construction for the umbreon renderer. POV emits two kinds
// of cylinders that need different Embree linear-curve types and side tables:
// `open` silhouette edges (chained ROUND_LINEAR_CURVE) and capped bonds
// (CONE_LINEAR_CURVE). The open-edge path also runs a polyline-stitching pass to
// drop the internal swept-sphere caps at shared joints. This is the largest and
// most intricate slice of the cold scene build, so it lives in its own unit.
#pragma once

#include <vector>

#include <embree4/rtcore.h>

#include "render/scene_build.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// Build the cylinder geometries for `scene` (partitioned into `open` chained
// edges and capped bonds), attach them to `rscene`, and append their geometry
// records plus primID side tables (cyl* / cylCap*) to `out`. No-op when the
// scene has no cylinders.
void buildCylinderGeometry(RTCDevice device, RTCScene rscene, const Scene& scene,
                           const std::vector<Vec3>& bakeOffsets, BuiltScene& out);

}  // namespace detail
}  // namespace umbreon
