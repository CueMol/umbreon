// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Freestyle-style stroke edge rendering (--edges).
//
// The active --edges implementation dispatches to the SCREEN source
// (edges/screen_vector_edges.hpp): vectorizes per-pixel edge AOVs by crack
// tracing (tessellation-independent, z-buffer visibility, no QI rays). The
// stylization + rasterization back half (edges/stroke_render.hpp) is shared.
//
// Runs as a post-process on the hi-res (supersampled) FrameResult, AFTER fog and
// BEFORE the box downsample. Gated by the caller on RenderOptions::strokeEdges
// .enable; with edges off it is never invoked, so the default render path is
// byte-identical.
#pragma once

#include <functional>

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// Ray-cast occlusion predicate: true iff some geometry lies strictly between the
// world points `p` and `q` (the camera), EXCLUDING the mesh triangles listed in
// `excludeFaces` (`nExclude` ids; the edge's own incident faces). Both ends are
// epsilon-trimmed by the implementation so a surface a ray leaves / the target
// itself are not counted, and a hit on an excluded incident face is not counted
// either -- this is Freestyle's self/adjacent-face exclusion
// (ViewMapBuilder.cpp:2152-2195), which decides visibility WITHOUT the old
// along-view origin nudge. The stroke pass binds this to EmbreeRenderer::occluded
// against the live BVH (the binding knows the mesh geomID); tests bind a
// synthetic occluder. A null/empty query => everything reads as visible.
using OcclusionQuery =
    std::function<bool(const Vec3& p, const Vec3& q, const int* excludeFaces,
                       int nExclude)>;

// Detect, chain, stylize and composite Freestyle-style stroke edges over
// `frame.color` in place, at the frame's current (hi-res) resolution. Dispatches
// to the screen-space vector edge extractor (edges/screen_vector_edges.hpp).
// `occluded` and `occludedRaw` are accepted for API compatibility but ignored
// (the screen source derives visibility from the z-buffer, not ray casting).
void applyStrokeEdges(FrameResult& frame, const Scene& scene,
                      const RenderOptions& opt, const OcclusionQuery& occluded,
                      const OcclusionQuery& occludedRaw = OcclusionQuery{});

}  // namespace umbreon
