// Analytic silhouette (n.v == 0 contour) extraction for the ANALYTIC primitives
// (Sphere, Cylinder), shared by BOTH edge methods -- mirroring how
// mesh_feature_edges.{hpp,cpp} factors the triangle-mesh feature extraction out
// of the two passes:
//   * the OBJECT-SPACE method (--obj-edges, render/object_space_edges.cpp) flattens
//     each loop into a RawSeg chord polygon, clips it against the union of solids
//     and wraps the survivors into 3D edge cylinders;
//   * the FREESTYLE STROKE method (--edges, render/stroke_edges.cpp) converts each
//     loop into Silhouette FeatureSegs and merges them into the same chain /
//     ray-cast-visibility / ribbon pipeline as the mesh edges, so ball-and-stick
//     geometry is outlined too (with true cross-primitive QI hidden-line).
//
// The core emitters (sphere horizon ring, cylinder side generators + end-on cap
// rim) are MOVED here verbatim from object_space_edges.cpp so the two methods can
// never disagree on WHERE a silhouette is. The only behavioural fork is the
// `circumscribe` flag (see emitAnalyticSilhouettes): off => byte-identical to the
// former inline obj-edges emitters.
//
// Pure C++17, no rendering-library dependency.
#pragma once

#include <cstdint>
#include <vector>

#include "edges/mesh_feature_edges.hpp"  // FeatureSeg + scene.hpp (Scene/Camera/Vec3)

namespace umbreon {

// One ordered analytic silhouette loop on a source primitive's surface. `pts` are
// the ordered contour vertices (sphere horizon ring / cylinder cap rim / the two
// cylinder side generators). `closed` => the last vertex connects back to pts[0]
// (a ring); open => a polyline (a single cylinder side generator is a 2-point
// open loop). `group` is the source primitive's transparency section; srcSphere /
// srcCyl carry the source index (the obj-edges union clip skips a loop's own
// primitive). -1 = none.
struct AnalyticLoop {
  std::vector<Vec3> pts;
  bool closed = false;
  std::uint16_t group = 0;
  int srcSphere = -1;
  int srcCyl = -1;
};

// Append the analytic silhouette loops of every (non-fromEdgeMacro) sphere then
// cylinder in `scene` for camera `cam`, in primitive-index order (spheres first,
// then cylinders -- the order the former inline obj-edges code produced). `N` is
// the ring tessellation (>= 3); `raise` offsets the contour outward (sphere along
// the surface normal, cylinder/cap radially). When `circumscribe` is true the
// sphere ring and cap rim are scaled by 1/cos(pi/N) so the chord polygon stays
// ON/OUTSIDE the source surface (the stroke QI then never re-enters the source and
// self-hides the contour into dashes); cylinder side generators are straight
// tangent lines and are unaffected. `circumscribe == false` reproduces the former
// inline emitters EXACTLY (byte-identical --obj-edges).
void emitAnalyticSilhouettes(const Scene& scene, const Camera& cam, int N,
                             float raise, bool circumscribe,
                             std::vector<AnalyticLoop>& out);

// Append the analytic sphere/cylinder silhouettes as Silhouette FeatureSegs into
// `segs`, allocating fresh chain node ids starting at `nodeBase` (so they never
// collide with the mesh's [0, nodeBase) ids). Each loop gets its OWN contiguous id
// block (a ring closes into a loop; a side generator stays a standalone 1-seg
// chain), so two different primitives never accidentally chain. Always
// circumscribes (the stroke QI path). Returns the new node count (nodeBase + total
// loop vertices). face0/face1 are -1 and excludeFaces empty: an analytic
// silhouette point self-rejects in the QI ray by the geometry-agnostic grazing /
// coplanar filter, not by a mesh face id. nrm is left zero (a convex primitive
// silhouette has no image-space fold-back cusp).
int appendAnalyticFeatureSegs(const Scene& scene, const Camera& cam, int N,
                              float raise, int nodeBase,
                              std::vector<FeatureSeg>& segs);

}  // namespace umbreon
