// Unit tests for the object-space edge visibility BVH
// (src/umbreon/edges/edge_mesh_bvh.{hpp,cpp}).
//
// Guards the hidden-line contract the object-space edge pass relies on:
//   * an invalid BVH (no triangles) reports everything visible (no occluder);
//   * a point behind an occluder triangle (toward the camera) is hidden;
//   * excluding that triangle by id (the edge's own incident face, CueMol
//     contains_id self-skip) makes the point visible again;
//   * a point whose camera ray misses the occluder is visible;
//   * a point in FRONT of the occluder (occluder farther from camera) is visible;
//   * orthographic and perspective cameras both resolve occlusion;
//   * isSegmentClear honors the exclude set and the strictly-between semantics.
#include <vector>

#include "edges/edge_mesh_bvh.hpp"
#include "scene.hpp"
#include "test_util.hpp"

namespace {

using umbreon::Camera;
using umbreon::Mesh;
using umbreon::Vec3;
using umbreon::detail::buildEdgeMeshBVH;
using umbreon::detail::EdgeBVH;
using umbreon::detail::isPointVisibleToViewer;
using umbreon::detail::isSegmentClear;

// A single large occluder triangle in the z=5 plane, comfortably containing the
// z-axis so a ray from the origin toward a camera at +z crosses it. A single
// triangle (not a quad) avoids a diagonal-seam ambiguity at (0,0).
Mesh wallMesh() {
  Mesh m;
  m.positions = {{-4.0f, -4.0f, 5.0f}, {4.0f, -4.0f, 5.0f}, {0.0f, 6.0f, 5.0f}};
  // de-indexed: triangleCount==1, primID 0; normals/colors unused by the BVH.
  return m;
}

}  // namespace

int main() {
  umbreon::test::Suite s("edge_visibility");

  // ---- (0) invalid BVH (empty mesh) => everything visible -----------------
  {
    EdgeBVH bvh = buildEdgeMeshBVH(Mesh{});
    s.check("empty mesh => invalid BVH", !bvh.valid());
    Camera cam;
    cam.position = {0, 0, 10};
    cam.direction = {0, 0, -1};
    s.check("invalid BVH => point visible",
            isPointVisibleToViewer(bvh, Vec3{0, 0, 0}, cam, nullptr, 0));
  }

  EdgeBVH bvh = buildEdgeMeshBVH(wallMesh());

  // ---- (1) BVH built with one triangle ------------------------------------
  s.check("wall => valid BVH", bvh.valid());
  s.check("wall => triCount 1", bvh.triCount == 1u);

  // Perspective camera at +z looking toward the wall and the origin behind it.
  Camera persp;
  persp.position = {0, 0, 10};
  persp.direction = {0, 0, -1};
  persp.orthographic = false;

  // ---- (2) point behind the occluder => hidden ----------------------------
  s.check("origin behind wall => hidden (persp)",
          !isPointVisibleToViewer(bvh, Vec3{0, 0, 0}, persp, nullptr, 0));

  // ---- (3) exclude the occluder face => visible (self-face skip) -----------
  {
    const int excl[1] = {0};
    s.check("origin, exclude face 0 => visible",
            isPointVisibleToViewer(bvh, Vec3{0, 0, 0}, persp, excl, 1));
  }

  // ---- (4) camera ray misses the occluder => visible ----------------------
  // From (10,0,0) toward (0,0,10): at z=5 the crossing is x=5, outside the
  // triangle (|x|>~2.4 at y=0).
  s.check("ray misses wall => visible",
          isPointVisibleToViewer(bvh, Vec3{10, 0, 0}, persp, nullptr, 0));

  // ---- (5) point in front of the occluder => visible ----------------------
  // At z=8 (between wall z=5 and camera z=10) the ray runs +z, away from the wall.
  s.check("point in front of wall => visible",
          isPointVisibleToViewer(bvh, Vec3{0, 0, 8}, persp, nullptr, 0));

  // ---- (6) orthographic camera resolves occlusion the same way ------------
  {
    Camera ortho;
    ortho.orthographic = true;
    ortho.direction = {0, 0, -1};  // toViewer = +z
    s.check("origin behind wall => hidden (ortho)",
            !isPointVisibleToViewer(bvh, Vec3{0, 0, 0}, ortho, nullptr, 0));
    const int excl[1] = {0};
    s.check("origin, exclude face 0 => visible (ortho)",
            isPointVisibleToViewer(bvh, Vec3{0, 0, 0}, ortho, excl, 1));
  }

  // ---- (7) isSegmentClear: strictly-between + exclude ----------------------
  {
    // P=(0,0,0) -> Q=(0,0,10) crosses the wall at z=5 => blocked.
    s.check("segment through wall => blocked",
            !isSegmentClear(bvh, Vec3{0, 0, 0}, Vec3{0, 0, 10}, nullptr, 0));
    const int excl[1] = {0};
    s.check("segment through wall, excluded => clear",
            isSegmentClear(bvh, Vec3{0, 0, 0}, Vec3{0, 0, 10}, excl, 1));
    // A segment that ends BEFORE the wall (z=0 -> z=4) never reaches it => clear.
    s.check("segment short of wall => clear",
            isSegmentClear(bvh, Vec3{0, 0, 0}, Vec3{0, 0, 4}, nullptr, 0));
  }

  // ---- (8) clipSegmentToVisibleSpans: visible-interval split --------------
  using umbreon::detail::clipSegmentToVisibleSpans;
  {
    // A segment in z=0 from x=-10 to x=10 toward the camera at (0,0,10). Each
    // point P=(x,0,0) is hidden when its z=5 crossing (x*0.5, 0) lands in the
    // wall triangle (|x*0.5| <~ 2.4 at y=0, i.e. |x| <~ 4.8). So the middle is
    // hidden and the two ends are visible => exactly 2 spans.
    auto spans = clipSegmentToVisibleSpans(bvh, persp, Vec3{-10, 0, 0},
                                           Vec3{10, 0, 0}, nullptr, 0, 0.25f);
    s.check("partially occluded segment => 2 spans", spans.size() == 2);
    if (spans.size() == 2) {
      // First span starts at the left end; last span ends at the right end.
      s.check("span 0 starts at left end", spans[0].first.x <= -9.0f);
      s.check("span 1 ends at right end", spans[1].second.x >= 9.0f);
      // The hidden gap brackets the origin (x in ~[-4.8, 4.8]).
      s.check("span 0 ends before the wall shadow", spans[0].second.x < 0.0f);
      s.check("span 1 starts after the wall shadow", spans[1].first.x > 0.0f);
    }

    // A fully visible segment (in front of the wall, z=8 plane) => 1 span.
    auto vis = clipSegmentToVisibleSpans(bvh, persp, Vec3{-3, 0, 8},
                                         Vec3{3, 0, 8}, nullptr, 0, 0.25f);
    s.check("fully visible segment => 1 span", vis.size() == 1);

    // A fully hidden segment (short, centered behind the wall) => 0 spans.
    auto hid = clipSegmentToVisibleSpans(bvh, persp, Vec3{-1, 0, 0},
                                         Vec3{1, 0, 0}, nullptr, 0, 0.25f);
    s.check("fully hidden segment => 0 spans", hid.empty());

    // Invalid BVH => the whole segment is returned as one span (verbatim).
    EdgeBVH empty;
    auto whole = clipSegmentToVisibleSpans(empty, persp, Vec3{-1, 0, 0},
                                           Vec3{1, 0, 0}, nullptr, 0, 0.25f);
    s.check("invalid BVH => whole segment (1 span)", whole.size() == 1);
  }

  return s.report();
}
