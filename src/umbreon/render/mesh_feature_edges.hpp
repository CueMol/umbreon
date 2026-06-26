// Shared feature-edge extraction + screen projection for the edge passes.
//
// This module factors the topology-bearing feature-edge detection out of the
// object-space pass (render/object_space_edges.cpp:emitMeshEdges) so BOTH edge
// methods can share it:
//   * the OBJECT-SPACE method (--obj-edges) wraps each FeatureSeg into a 3D
//     cylinder (render/object_space_edges.cpp);
//   * the FREESTYLE STROKE method (--edges) chains the FeatureSegs into
//     continuous polylines, projects them to screen and rasterizes ribbons
//     (render/stroke_edges.cpp).
//
// The detector (extractMeshFeatureEdges) reproduces emitMeshEdges EXACTLY -- the
// same quantized-position weld, edge->2-face adjacency (emap), per-corner
// smoothing-cluster normals, and the silhouette (smooth n.v==0 contour + hard-
// edge straddle) / crease (dihedral + smooth-facet/convex/degree gates) / border
// (single-face + coplanar veto) predicates -- but emits TOPOLOGY-TAGGED segments
// (FeatureSeg) instead of disconnected RawSegs, so the stroke pass can chain
// them. To keep --obj-edges byte-identical, FeatureSeg endpoints carry the same
// world positions emitMeshEdges produced (including the silhouette lift/cam bias
// driven by ExtractParams), so wrapping each FeatureSeg in a cylinder reproduces
// method A's output exactly.
//
// worldToScreen is the algebraic inverse of the renderer's pixel->ray map
// (embree_renderer.cpp). It maps
// a world point to a top-left-origin pixel coordinate plus a linear view-z in the
// SAME units as the AOV firstViewZ, so a later visibility compare is
// apples-to-apples.
//
// Pure C++17, no rendering-library dependency.
#pragma once

#include <cstdint>
#include <vector>

#include "scene.hpp"

namespace umbreon {

// Feature-edge nature, the chaining/styling key. A stroke is chained only with
// same-nature neighbours, and each nature maps to a styling slot.
enum class EdgeNature : std::uint8_t { Silhouette, Border, Crease };

// One topology-tagged feature segment. v0/v1 are node ids into a shared id space
// used ONLY for chaining (NOT indices into FeatureMesh::vpos): welded vertex ids
// 0..nV-1 for hard-edge/crease/border endpoints and for smooth-contour crossings
// that land exactly on a vertex; SYNTHETIC ids >= nV for smooth-contour crossings
// interior to a face, keyed on the welded mesh edge the crossing lies on so the
// two faces sharing that edge agree on the same id and the contour chains across
// faces. p0/p1 are the world-space endpoints (already lifted/biased per
// ExtractParams, matching emitMeshEdges).
//
// face0/face1 are the segment's INCIDENT mesh-triangle ids (indices into the
// de-indexed triangle list, == Embree primID for the mesh geometry). They drive
// Freestyle-faithful self-face exclusion in the ray-cast visibility test
// (ViewMapBuilder.cpp:2152-2195): a self-occlusion hit on the edge's own
// incident face is NOT counted as an occluder. For a hard-edge / crease segment
// both incident faces are set; for a border (single-face) edge only face0; for a
// smooth n.v==0 contour crossing interior to a face, face0 is that face (face1
// the other face across the welded edge the crossing endpoint lies on, when the
// crossing is on an interior edge). -1 means "no face" (unused slot).
struct FeatureSeg {
  int v0 = -1, v1 = -1;
  Vec3 p0, p1;
  EdgeNature nature = EdgeNature::Silhouette;
  std::uint16_t group = 0;  // CueMol section (mesh.groupForTri source)
  int face0 = -1, face1 = -1;  // incident mesh-triangle ids (== Embree primID)
  // EXPANDED self/adjacent-face exclude set for the QI ray cast: the 1-RING of
  // mesh triangles around this edge -- {face0, face1} UNION every triangle that
  // shares a VERTEX with face0 or face1 (deduplicated; face1 < 0 contributes
  // nothing). This is Freestyle's ViewMapBuilder occluder skip
  // (ViewMapBuilder.cpp:2152-2196): an occluder that shares ANY vertex with the
  // edge's face is not counted, so a silhouette that grazes its OWN nearby
  // surface near a T-junction is not wrongly self-hidden. The stroke pass
  // (render/stroke_edges.cpp) carries this to computeChainVisibility; --obj-edges
  // ignores it (so that path stays byte-identical).
  std::vector<int> excludeFaces;
};

// Extracted feature edges of a mesh. `vpos` is the welded vertex table (for
// reference / future use); the chain id space spans [0, nodeCount).
struct FeatureMesh {
  std::vector<Vec3> vpos;       // welded vertex positions, indexed by welded id
  std::vector<FeatureSeg> segs;
  int nodeCount = 0;            // total chain node ids (welded + synthetic)
  // Mean triangle edge length (elsum / (3*nTri)) of the welded mesh. Surfaced so
  // the stroke pass can reproduce the obj-edges silhouette camBias
  // (max(0.5*w, 0.15*meanEdge), mesh_feature_edges.cpp) for its screen-space
  // depth test -- the same camera-ward bias that lifts obj-edges silhouettes off
  // the grazing tangent shell. 0 for an empty mesh.
  float meanEdge = 0.0f;
};

// Extraction parameters. Mirrors the mesh-edge knobs ObjectSpaceEdgeOptions
// feeds emitMeshEdges, so --obj-edges can drive the shared extractor with its
// existing values and stay byte-identical. The stroke pass passes its own values
// (raise/width 0: ray-cast visibility needs no 3D lift).
struct ExtractParams {
  float raise = 0.0f;   // outward contour offset, world units
  float width = 0.0f;   // edge radius: drives the silhouette lift + cam bias (A)

  bool silhouette = true;  // smooth n.v==0 contour + hard-edge straddle
  bool crease = true;      // interior fold edges
  bool border = true;      // open boundary edges

  float meshHardEdgeDeg = 40.0f;
  float creaseAngleDeg = 30.0f;
  float meshCreaseSmoothVetoDeg = 0.0f;
  bool meshCreaseConvexOnly = false;
  float meshBorderCoplanarVetoDeg = 0.0f;
  int meshCreaseMaxDegree = 0;
};

// Detect topology-tagged feature edges of `mesh` for camera `cam`. Reproduces
// emitMeshEdges exactly (see header note); empty mesh => empty result.
FeatureMesh extractMeshFeatureEdges(const Mesh& mesh, const Camera& cam,
                                    const ExtractParams& params);

// Camera projection basis built once per frame. Mirrors the renderer's camera
// basis (embree_renderer.cpp:81-89). halfW/halfH are the ORTHO image-plane
// half-extents (world units); persHalfW/persHalfH are the perspective half-
// extents at unit forward distance. W/H are the (hi-res) pixel dims.
struct ScreenProj {
  Vec3 pos;       // camera position
  Vec3 dir;       // normalized forward (view) axis
  Vec3 right;     // normalized image-plane right axis
  Vec3 up;        // normalized image-plane up axis
  bool ortho = false;
  float halfW = 1.0f, halfH = 1.0f;          // ortho half-extents (world units)
  float persHalfW = 1.0f, persHalfH = 1.0f;  // persp half-extents at unit depth
  int W = 0, H = 0;                          // pixel dimensions
};

// Build the projection basis for `cam` at pixel resolution w x h.
ScreenProj makeScreenProj(const Camera& cam, int w, int h);

// Project world point P to a top-left-origin pixel coordinate (x,y) and a linear
// view-z (vz). Algebraic inverse of the renderer's pixel->ray map. Returns false
// only for a perspective point at/behind the eye (zc <= ~0).
bool worldToScreen(const ScreenProj& sp, const Vec3& P, float& x, float& y,
                   float& vz);

}  // namespace umbreon
