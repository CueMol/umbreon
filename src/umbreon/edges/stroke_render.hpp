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

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// One backbone vertex of a source-produced chain: 2D pixel position at the
// HI-RES frame resolution (pixel (x,y) center == coordinate (x,y)), linear
// view-z (depth sort key / future depth cues) and a visibility flag (the mesh
// source marks QI-hidden vertices; the screen source is always visible).
struct StrokePoint {
  float x = 0.0f, y = 0.0f;
  float vz = 0.0f;
  bool visible = true;
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
// resolve to a disabled style are skipped.
void renderStrokeChains(FrameResult& frame, const Scene& scene,
                        const RenderOptions& opt,
                        const std::vector<StrokeChainInput>& chains);

}  // namespace umbreon
