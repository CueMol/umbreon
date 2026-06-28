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
// edges/object_space_edges.hpp (--obj-edges); the two share the feature-edge
// extractor but are independent passes.
//
// Runs as a post-process on the hi-res (supersampled) FrameResult, AFTER fog and
// BEFORE the box downsample. Gated by the caller on RenderOptions::strokeEdges
// .enable; with edges off it is never invoked, so the default render path is
// byte-identical.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include "edges/mesh_feature_edges.hpp"
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
  // Per-SEGMENT incident mesh-triangle ids (one entry per `segs`, oriented to the
  // backbone): the two faces of that feature segment (FeatureSeg::face0/face1),
  // used as the EXACT, per-segment exclude set for the QI ray cast at the
  // segment center (Freestyle self/adjacent-face skip). -1 = unused slot. This is
  // narrower (and correct) versus the per-vertex hub union `incidentFaces`, which
  // over-excludes by unioning every face meeting at a hub. Sized to segs; empty
  // when no faces were carried.
  std::vector<std::array<int, 2>> segFaces;
  // Per-SEGMENT EXPANDED exclude set for the QI ray cast (one entry per `segs`,
  // oriented to the backbone): the edge's 1-RING of mesh faces
  // (FeatureSeg::excludeFaces -- {face0,face1} plus every triangle sharing a
  // vertex with them). This is the Freestyle-faithful occluder skip
  // (ViewMapBuilder.cpp:2152-2196) that prevents a silhouette grazing its OWN
  // nearby surface near a T-junction from being wrongly self-hidden. Preferred by
  // computeChainVisibility over the narrow per-segment `segFaces`; empty when not
  // populated (then `segFaces` / `incidentFaces` are the fallback, keeping the
  // unit tests' synthetic chains working).
  std::vector<std::vector<int>> segExclude;
  // Per-SEGMENT nature, oriented to the backbone (one per `segs`). Carried so the
  // global crossing pass can apply Freestyle's silhouette_binary_rule (at least
  // one SILHOUETTE/BORDER edge per occlusion T-vertex) without re-fetching the
  // FeatureSeg array.
  std::vector<EdgeNature> segNature;
  // Per-SEGMENT smooth-contour surface normal (one per `segs`; FeatureSeg::nrm).
  // Nonzero only on the smooth n.v==0 silhouette contour; zero for hard-edge
  // straddle silhouettes, creases and borders. Drives cusp detection
  // (computeChainCusps, Freestyle computeCusps): a sign flip of
  // (edgeDir x nrm).viewdir along the contour is a fold-back, splitting the
  // ViewEdge so the per-ViewEdge QI majority does not average the visible
  // front-branch with the hidden back-branch. Sized to segs; empty when unset.
  std::vector<Vec3> segNrm;
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

// Per-backbone-vertex Quantitative-Invisibility (QI) visibility for one chain
// (LEGACY per-vertex variant; retained for unit tests -- the live applyStrokeEdges
// path uses the sub-span split + per-sub-span QI in stroke_edges.cpp). For each
// SEGMENT a QI ray is cast from the segment's 3D center (and interior samples for
// long segments -- Freestyle FEdge::center3d) toward the eye, EXCLUDING that
// segment's own two incident mesh faces (FeatureSeg::face0/face1) so a grazing
// silhouette does not self-occlude (Freestyle ViewMapBuilder self/adjacent-face
// skip, enabled by the now-live Embree argument filter). A segment is QI-visible
// iff every sample is un-occluded; a backbone vertex is QI-visible iff BOTH its
// adjacent segments are. `target` is the eye for perspective and a far
// camera-ward point for ortho. No primary z-buffer / G-buffer is read -- this is
// a pure BVH-occlusion count, keeping method B distinct from --obj-edges (A).
// A null `occluded` (no live BVH) => all visible. Returns one flag per
// chain.pts entry.
std::vector<char> computeChainVisibility(const EdgeChain& chain,
                                         const ScreenProj& sp,
                                         const OcclusionQuery& occluded);

// One backbone segment hidden by a 2D image-space crossing (stage B): the
// chain/segment that is OCCLUDED at a screen crossing, plus the crossing
// parameter `t` in [0,1] along that segment (Freestyle CreateTVertex). The
// stroke builder turns these into per-segment hidden intervals so the visible
// run ends exactly at the T-vertex.
struct EdgeCrossing {
  int chainIdx = 0;  // chain whose segment is hidden
  int segIdx = 0;    // backbone-segment index within that chain (0..pts.size()-2)
  float t = 0.0f;    // crossing parameter along the segment [0,1]
};

// FREESTYLE-FAITHFUL 2D image-space crossing pass (stage B). Projects every
// backbone segment of every chain to screen (worldToScreen -> 2D endpoints +
// linear view-z), finds pairwise 2D crossings (intersect2dLine2dLine) and, at
// each crossing, hides the FARTHER segment (larger interpolated view-z) by
// emitting an EdgeCrossing on it (Freestyle CreateTVertex depth order). This is
// the silhouette-vs-silhouette / self-occlusion-at-folds robustness QI cannot
// resolve (two grazing rays give unstable counts; the view-z compare at the
// crossing pixel is exact). Filtered like silhouette_binary_rule: a crossing is
// recorded only if at least one segment is SILHOUETTE or BORDER (crease-vs-crease
// skipped); chain-adjacent / shared-node pairs are skipped (a chain never
// self-hides at its own joints); a near-equal view-z crossing (|z1-z2| <= zTol)
// is a true junction and hides NEITHER. No 3D geometry and no primary z-buffer is
// used. Returns one EdgeCrossing per hidden side per qualifying crossing.
std::vector<EdgeCrossing> computeEdgeCrossings(
    const std::vector<EdgeChain>& chains, const ScreenProj& sp, float zTol);

// Morphological CLOSE of a per-vertex visible(1)/hidden(0) mask along a chain:
// any maximal HIDDEN run no longer than `maxBridge`, bracketed on BOTH sides by
// a visible vertex, is reclassified VISIBLE (in place). This bridges the
// isolated 1-2 vertex spurious-hidden flickers a grazing silhouette produces
// (the self-occlusion ray re-hits the source's near-tangent surface) so a
// continuous contour is not fragmented into dash-strips, while a genuine (long)
// occlusion still splits the stroke. Runs touching an OPEN polyline end are not
// bridged (no visible bracket); for a `closed` loop index 0 and n-1 wrap so a
// run straddling the seam is bracketed. maxBridge<=0 is a no-op. Exposed for
// unit testing only; the live applyStrokeEdges path no longer calls it.
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
