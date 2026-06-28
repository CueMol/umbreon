// Dedicated edge-mesh BVH for object-space edge visibility / clipping queries.
//
// The object-space edge pass (edges/object_space_edges.cpp) runs BEFORE the
// render BVH is built (it appends edge cylinders to the scene, which the renderer
// then traces). So it cannot reuse EmbreeRenderer's scene for the hidden-line
// visibility test CueMol did with a CGAL AABB tree (RendIntData_AABBTree.cpp).
// This builds a SEPARATE, throwaway Embree BVH over the mesh triangles, used only
// to answer "is this edge point visible toward the camera?" and (Phase C) "where
// along this edge does the surface cross it?".
//
// The BVH is a single de-indexed/indexed TRIANGLE geometry, so its primID equals
// the de-indexed triangle id -- exactly FeatureSeg::face0/face1/excludeFaces
// (mesh_feature_edges.hpp). The visibility query therefore excludes the edge's
// own incident faces (and 1-ring) by primID, reproducing CueMol's contains_id
// self-face skip with the same self/adjacent-face exclusion umbreon's stroke
// pass already uses (OcclusionQuery, stroke_edges.hpp).
//
// Pure Embree; owns its own RTCDevice (independent of the renderer's).
#pragma once

#include <embree4/rtcore.h>

#include "scene.hpp"

namespace umbreon {
namespace detail {

// A throwaway triangle BVH over a mesh, RAII over its Embree device + scene.
// Move-only; valid() is false for an empty/degenerate mesh (queries then report
// everything visible, matching "no occluder").
struct EdgeBVH {
  RTCDevice device = nullptr;
  RTCScene scene = nullptr;
  unsigned int triCount = 0;
  float boundsDiag = 0.0f;  // mesh bounds diagonal (ortho far-point distance)

  EdgeBVH() = default;
  EdgeBVH(const EdgeBVH&) = delete;
  EdgeBVH& operator=(const EdgeBVH&) = delete;
  EdgeBVH(EdgeBVH&& o) noexcept;
  EdgeBVH& operator=(EdgeBVH&& o) noexcept;
  ~EdgeBVH();

  bool valid() const { return scene != nullptr; }
};

// Build the triangle BVH for `mesh` (primID == triangle id). Returns an invalid
// EdgeBVH (valid()==false) for a mesh with no triangles.
EdgeBVH buildEdgeMeshBVH(const Mesh& mesh);

// True iff the straight path from `P` to `Q` is clear of mesh triangles, EXCLUDING
// the `nExclude` triangle ids in `excludeFaces` (the edge's own incident / 1-ring
// faces). Both ends are epsilon-trimmed so the surface P sits on and the target Q
// are not counted. This is umbreon's OcclusionQuery semantics (stroke_edges.hpp):
// "geometry strictly between p and q, minus the self faces". An invalid BVH (no
// triangles) reports visible. Returns true == visible (unoccluded).
bool isSegmentClear(const EdgeBVH& bvh, const Vec3& P, const Vec3& Q,
                    const int* excludeFaces, int nExclude);

// True iff `P` is visible toward the camera: casts P -> camera (perspective) or
// P -> a far camera-ward point (orthographic, derived from the mesh bounds) and
// runs isSegmentClear with the edge's self-face excludes. Mirrors CueMol
// isVertSilVisible (RendIntData_AABBTree.cpp:167) with umbreon face exclusion.
bool isPointVisibleToViewer(const EdgeBVH& bvh, const Vec3& P, const Camera& cam,
                            const int* excludeFaces, int nExclude);

}  // namespace detail
}  // namespace umbreon
