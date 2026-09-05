// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Shared stroke DRAW stage: style resolution + parametric-stroke stylization +
// ribbon rasterization, factored out of the Freestyle stroke path
// (edges/stroke_edges.cpp:applyStrokeEdges) so that any edge-chain SOURCE can
// feed it. The stroke edge pass (--edges on) is driven by the SCREEN source:
// AOV crack tracing -> 2D polylines (screen_vector_edges.cpp), visibility exact
// from the z-buffer. (The retired mesh-topology source -- feature edges ->
// chaining -> QI visibility -> projected polylines -- fed this same stage; the
// draw half stays source-agnostic.)
//
// A source produces StrokeChainInput chains (2D pixel-space backbone points at
// the HI-RES frame resolution + per-chain style key); renderStrokeChains then
// runs the UNCHANGED back half -- resolveStrokeStyle -> buildStroke ->
// resampleStroke -> stroke shaders -> buildStrokeReps -> depth/precedence
// stable sort -> TBB row-tiled rasterization over frame.color in LINEAR space.
// The mesh path routed through this stage is byte-identical to the pre-refactor
// code (locked by tests/test_edge_regression.cpp).
#pragma once

#include <cstdint>
#include <vector>

#include "render/render_progress.hpp"
#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// One backbone vertex of a source-produced chain: 2D pixel position at the
// HI-RES frame resolution (pixel (x,y) center == coordinate (x,y)), linear
// view-z (depth sort key / future depth cues), the surface alpha multiplier
// (first-hit fragment opacity of the owner surface; the ribbon opacity is the
// resolved style opacity times this, lerped per vertex so a vertex-color alpha
// gradient fades the edge with it) and a visibility flag (the mesh source
// marks QI-hidden vertices; the screen source is always visible).
struct StrokePoint {
  float x = 0.0f, y = 0.0f;
  float vz = 0.0f;
  float alpha = 1.0f;
  bool visible = true;
  // Contact weight, 0..1 (ScreenChainVert::contact): a vertex >= 0.5 lies
  // on a depth-continuous contact contour, and its offset band is exempt
  // from the depth permission / outer-room clamp (the surface beside it is
  // the other section's own surface at this depth, which rises and falls
  // along the contour; culling on it would notch the band).
  float contact = 0.0f;
};

// Optional END CLIP for the outside alignment (set by the screen source on
// junction stem ends): ink of THIS chain lying within `radius` of the anchor
// (px, py) AND on the positive side of the unit normal (nx, ny) is culled at
// rasterization time. The stem keeps its offset band and terminates flush
// against the far edge of the met line's ink instead of poking past it or
// re-centering (a taper visibly necks shallow-angle junctions).
// A chain end that MEETS another line: a junction stem on a woven bar, or a
// free end the crack-field probe connected to a line. The draw stage extends
// the raster backbone by the pad past the vector endpoint (so a smoothing
// deviation of the met line cannot open a pinhole) and draws no round cap
// there. What that overshoot may paint is decided per pixel by the depth
// permission of the offset band (it never paints over a surface nearer than
// its own contour; see DepthPermit in stroke_render.cpp), not by any clip
// geometry: the earlier clip planes, discs, zones and parallel guards were
// local approximations of the met line that failed whenever the stem's own
// body, a curved bar or a mis-fitted line re-entered them.
struct StrokeEndClip {
  bool enabled = false;
};

// One chain handed to the shared draw stage. styleSlot indexes EdgeStyle::cls[]
// (the EdgeClass slot; the mesh source passes natureStyleSlot(nature)).
// precedence is the overlap paint order (higher paints later == on top; the
// mesh source passes naturePrecedence(nature)). group keys the per-section
// style table (Scene::groupEdgeStyle).
struct StrokeChainInput {
  std::vector<StrokePoint> pts;
  int styleSlot = 0;
  int precedence = 0;
  std::uint16_t group = 0;
  // Outside stroke alignment (StrokeAlign::Outside; the screen source sets
  // it on occlusion-contour runs): 0 = centered ribbon (legacy), +1 = the
  // contour's OUTER (occluded / background) side is on the LEFT (+normal)
  // of the backbone direction, -1 = on the RIGHT. A nonzero value shifts
  // the resolved width to that side (a thin inner pad remains; the full
  // footprint stays the resolved width).
  std::int8_t outsideSide = 0;
  // Junction taper for a nonzero outsideSide: blend the offset back to the
  // symmetric ribbon over the last stroke-width of arc length at this end,
  // so the ribbon arrives centered where it meets other lines (an offset
  // butt end otherwise sticks its full width out sideways past the meeting
  // line). The screen source sets these where the band does not continue on
  // the same side (run boundaries with a side change, deep folds, junction
  // ends with no identified met line). A tapered OR clipped end draws no
  // round cap.
  bool taperStart = false;
  bool taperEnd = false;
  // End clips (preferred over the taper at junction stem ends whose met
  // line is known; see StrokeEndClip).
  StrokeEndClip clipStart, clipEnd;
  // A closed loop: pts.front() == pts.back() (the seam vertex duplicated)
  // and the loop is continuous across it. The draw stage joins the ribbon
  // at the seam like any interior corner and draws no end caps there -- an
  // open-polyline seam left a wedge gap (butt) or, under outside alignment,
  // cap fans whose outer -> pad radius lerp bulged into the object.
  bool closed = false;
};

// Resolve the ribbon style for one chain by its style slot + section group:
// the per-section EdgeStyle when Scene::groupEdgeStyle is populated (falling
// back to strokeEdges.defaultStyle for an out-of-range group), else the single
// global stroke style. Returns false when the section explicitly disables the
// slot's class (the chain is skipped). outHalf is the half band width in
// HI-RES px (>= 0.5); ssScale is the supersample factor scaling the FINAL-px
// style widths. Master per-nature gates (silhouette/crease/border toggles) are
// NOT applied here -- they stay at the source, which knows its natures.
bool resolveStrokeStyle(const Scene& scene, const StrokeEdgeOptions& se,
                        float ssScale, int styleSlot, std::uint16_t group,
                        float& outHalf, float outColor[3], float& outOpacity);

// Stylize and composite source-produced chains over frame.color in place at
// the frame's (hi-res) resolution: per chain resolve the style, wrap the
// backbone as a parametric Stroke, arc-length resample, run the stroke shaders
// (smooth/taper per opt.strokeEdges), split at hidden runs into miter-joined
// ribbon strips, then stable-sort all strips (farther view-z first, precedence
// tie-break) and rasterize row-tiled with TBB (deterministic). Chains that
// resolve to a disabled style are skipped. `progress`, when non-null, is
// polled for cancellation per chain (before rasterizing anything lands on the
// frame) and per raster row chunk; a cancelled call may leave the strokes
// partially composited.
void renderStrokeChains(FrameResult& frame, const Scene& scene,
                        const RenderOptions& opt,
                        const std::vector<StrokeChainInput>& chains,
                        const RenderProgress* progress = nullptr);

}  // namespace umbreon
