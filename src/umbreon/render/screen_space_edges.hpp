// Screen-space NPR edge detection + styling (Warabi-style Stage B/C).
//
// This is the SCREEN-SPACE (image-post-process) edge method, selected by
// --edges. Its counterpart is the OBJECT-SPACE (3D geometry) method in
// render/object_space_edges.hpp (ObjectSpaceEdgeOptions, --obj-edges). The two
// are independent; never enable both at once (they would double-draw).
//
// Runs as a post-process on the hi-res (supersampled) FrameResult, AFTER fog
// and BEFORE the box downsample, so the existing box-average produces the
// resolution-dependent antialiasing of the composited edge lines. The pass
// reads the edge G-buffer AOVs captured by Stage A (objectId / viewZ / normal /
// materialId) and composites per-class edge lines over frame.color in LINEAR
// space.
//
// The whole pass is gated by the caller on RenderOptions::edges.enable; with
// edges off it is never invoked, so the default render path is byte-identical.
#pragma once

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// Detect and composite screen-space edges over `frame.color` in place, at the
// frame's current (hi-res) resolution. `scene` supplies the camera/projection
// used by the depth-gap classes (later steps); `opt.edges` supplies the master
// gate, detection thresholds and the per-class style. Requires the edge AOVs to
// be populated (i.e. opt.edges.enable was set for the render); a no-op if the
// objectId AOV is empty.
void applyScreenSpaceEdges(FrameResult& frame, const Scene& scene,
                const RenderOptions& opt);

}  // namespace umbreon
