// Freestyle-style stroke edge rendering (--edges).
//
// This is the active --edges implementation: a Freestyle-faithful stroke
// renderer that REPLACES the retired per-pixel screen-space pass. It
// extracts topology-tagged feature edges (silhouette / crease / border) from the
// scene mesh, chains them into continuous polylines, computes ray-cast
// visibility (Quantitative Invisibility) against the live Embree BVH, then
// resamples and rasterizes variable-width ribbons composited over frame.color in
// LINEAR space at hi-res (the box downsample antialiases).
//
// Its counterpart is the OBJECT-SPACE (3D cylinder) method in
// render/object_space_edges.hpp (--obj-edges); the two share the feature-edge
// extractor but are independent passes.
//
// Runs as a post-process on the hi-res (supersampled) FrameResult, AFTER fog and
// BEFORE the box downsample. Gated by the caller on RenderOptions::strokeEdges
// .enable; with edges off it is never invoked, so the default render path is
// byte-identical.
#pragma once

#include <array>
#include <functional>
#include <vector>

#include "render/mesh_feature_edges.hpp"
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

// One chained polyline: an ORDERED run of FeatureSeg indices (`segs`, in walk
// order) with the matching world-space backbone vertices (`pts`, one more than
// `segs` for an open chain; equal count with pts.front()==pts.back() for a
// closed loop). `closed` marks a loop that returned to its seed node. Produced
// by chainFeatureSegs; consumed by the stroke rasterizer (drawn as a connected
// polyline) and later steps (resample / visibility / ribbon).
struct EdgeChain {
  std::vector<int> segs;   // FeatureSeg indices in walk order
  std::vector<Vec3> pts;   // backbone world points in walk order
  bool closed = false;     // true => pts.front() and pts.back() coincide
  // Per-backbone-vertex incident mesh faces (the union of the incident faces of
  // the chain segments meeting at that vertex; -1 padding), used to exclude the
  // vertex's own surface from the visibility ray-cast (Freestyle self-face
  // exclusion). Sized to pts; empty when no incident faces were carried.
  static constexpr int kMaxIncidentFaces = 4;
  std::vector<std::array<int, kMaxIncidentFaces>> incidentFaces;
};

// Bidirectional chaining ported from Freestyle Operators::bidirectionalChain +
// ChainSilhouetteIterator::traverse, adapted to FeatureSeg[]. Per NATURE, builds
// a node->incident-segment adjacency over the welded/synthetic chain node ids
// (v0/v1, in [0,nodeCount)) and walks each unconsumed seed forward then backward,
// continuing only through a node with EXACTLY ONE other unvisited same-nature
// segment (stopping at branches >1 and dead-ends 0). Each segment is emitted in
// exactly one chain. Segments with no valid node ids (v0<0 or v1<0) are skipped.
std::vector<EdgeChain> chainFeatureSegs(const std::vector<FeatureSeg>& segs,
                                        int nodeCount);

// Per-vertex ray-cast Quantitative Invisibility for one chain. For each backbone
// point P (in `chain.pts`) cast a ray from P toward the camera (`sp.pos`); QI==0
// (no occluder) => visible. The ray origin is P itself: P's OWN incident mesh
// faces (chain.incidentFaces[i]) are passed to `occluded` and excluded there, so
// the feature surface the edge sits on is not counted as a self-occluder. This
// is Freestyle's self/adjacent-face exclusion (ViewMapBuilder.cpp:2152-2195) --
// faithful and robust on grazing / self-occluding geometry, replacing the prior
// uniform along-view nudge. A point that fails to project (perspective at/behind
// the eye) is marked hidden. Returns one flag per chain.pts entry; a null
// `occluded` (no live BVH) => all visible.
std::vector<char> computeChainVisibility(const EdgeChain& chain,
                                         const ScreenProj& sp,
                                         const OcclusionQuery& occluded);

// Morphological CLOSE of a per-vertex visible(1)/hidden(0) mask along a chain:
// any maximal HIDDEN run no longer than `maxBridge`, bracketed on BOTH sides by
// a visible vertex, is reclassified VISIBLE (in place). This bridges the
// isolated 1-2 vertex spurious-hidden flickers a grazing silhouette produces
// (the self-occlusion ray re-hits the source's near-tangent surface) so a
// continuous contour is not fragmented into dash-strips, while a genuine (long)
// occlusion still splits the stroke. Runs touching an OPEN polyline end are not
// bridged (no visible bracket); for a `closed` loop index 0 and n-1 wrap so a
// run straddling the seam is bracketed. maxBridge<=0 is a no-op. Exposed for
// unit testing; applyStrokeEdges applies it before the run splitter.
void closeVisibilityMask(std::vector<char>& vis, int maxBridge, bool closed);

// 2D screen-space point (hi-res pixel coordinates) used by the ribbon API.
struct Vec2f {
  float x = 0.0f, y = 0.0f;
};

// Build variable-width RIBBON strips for one already-projected backbone polyline.
// `backbone2d` is the 2D (pixel) polyline; `visible` is one flag per point (empty
// => all visible). The polyline is arc-length resampled every `resampleStepPx`
// px, split into maximal runs of consecutive VISIBLE vertices, and each run is
// turned into a miter-joined offset strip of half-width `halfThick` (ported from
// Freestyle Strip::createStrip: per-vertex normal offset, miter join by line-line
// intersection, spike-clamp to the averaged normal on overrun). Returns one strip
// per visible run; each strip is a flat list of 2*M border vertices, consumed as
// a triangle strip ([2k]=left/+normal, [2k+1]=right/-normal per backbone vertex).
// Exposed for unit testing the ribbon geometry; the frame pass rasterizes strips
// built by the same internal path.
std::vector<std::vector<Vec2f>> buildRibbonStrips(
    const std::vector<Vec2f>& backbone2d, const std::vector<char>& visible,
    float halfThick, float resampleStepPx);

// Detect, chain, stylize and composite Freestyle-style stroke edges over
// `frame.color` in place, at the frame's current (hi-res) resolution. `scene`
// supplies the camera/projection and mesh topology; `opt.strokeEdges` supplies
// the master gate, extraction params and per-section style. `occluded` is the
// ray-cast visibility query against the live Embree scene kept alive through
// this pass (see EmbreeRenderer); an empty query draws every vertex as visible.
// Polylines are split at hidden runs so only visible portions are drawn.
void applyStrokeEdges(FrameResult& frame, const Scene& scene,
                      const RenderOptions& opt, const OcclusionQuery& occluded);

}  // namespace umbreon
