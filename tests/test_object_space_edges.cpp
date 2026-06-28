// Unit / integration tests for analytic OBJECT-SPACE silhouette edges
// (src/umbreon/render/object_space_edges.{hpp,cpp}).
//
// Guards the geometric contract of the n.v==0 silhouette generator so a future
// edit that breaks the math (wrong ring radius/center, side-line direction, the
// snapshot, the no-op default, or the emitted cylinder's render attributes) is
// caught:
//   * disabled => no edges appended (byte-identical default);
//   * each emitted edge is open / flatOutline / fromEdgeMacro==false / carries
//     the source group / has the requested radius+color (so it renders exactly
//     like the baked POV edge_line outlines);
//   * the snapshot: appended edges are not themselves re-silhouetted;
//   * SPHERE ortho: ring is a great circle (every vertex r from center, normal
//     perpendicular to the view dir);
//   * SPHERE persp: every ring vertex lies ON the sphere surface (|P-O|==r) and
//     its surface normal is perpendicular to the view ray from the camera (the
//     horizon/tangent circle);
//   * CYLINDER side view: the two generators are offset perpendicular to BOTH
//     the axis and the view, at distance radius, on opposite sides;
//   * CYLINDER raise pushes the side lines out by exactly `raise`;
//   * CYLINDER end-on: side lines collapse, a cap-rim circle is emitted instead.
#include <cmath>
#include <cstddef>

#include "edges/object_space_edges.hpp"
#include "scene.hpp"
#include "test_util.hpp"

namespace {

using umbreon::Camera;
using umbreon::Cylinder;
using umbreon::Scene;
using umbreon::ObjectSpaceEdgeOptions;
using umbreon::Sphere;
using umbreon::Vec3;
using umbreon::Vec4;

bool nearf(float a, float b, float eps = 1.0e-3f) {
  return std::fabs(a - b) <= eps;
}

// Common attribute contract every emitted edge must satisfy to render like a
// baked POV outline: open, flat outline material, NOT tagged fromEdgeMacro,
// requested radius, requested color (rgb, opacity 1), source group.
bool edgeAttrsOk(const Cylinder& c, float width, const float col[3],
                 uint16_t group) {
  const umbreon::Material& m = c.material;
  const bool mat = nearf(m.ambient, 1.0f) && nearf(m.diffuse, 0.0f) &&
                   nearf(m.specular, 0.0f);
  return c.open && !c.fromEdgeMacro && mat && nearf(c.radius, width) &&
         c.group == group && nearf(c.color.x, col[0]) &&
         nearf(c.color.y, col[1]) && nearf(c.color.z, col[2]) &&
         nearf(c.color.w, 1.0f);
}

}  // namespace

int main() {
  umbreon::test::Suite s("object_space_edges");

  // ---- (0) disabled => no-op (byte-identical default) --------------------
  {
    Scene sc;
    Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.0f;
    sc.spheres.push_back(sp);
    sc.camera.orthographic = true;
    sc.camera.direction = {0, 0, -1};
    ObjectSpaceEdgeOptions opt;  // enable defaults to false
    umbreon::generateObjectSpaceEdges(sc, opt);
    s.check("disabled: no cylinders appended", sc.cylinders.empty());
  }

  // ---- (1) SPHERE ortho great circle + emitted attributes ----------------
  {
    Scene sc;
    sc.camera.orthographic = true;
    sc.camera.direction = {0, 0, -1};  // view along -z
    Sphere sp;
    sp.center = {1, 2, 3};
    sp.radius = 2.0f;
    sp.group = 7;
    sc.spheres.push_back(sp);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.width = 0.05f;
    opt.raise = 0.0f;
    opt.segments = 32;
    opt.color[0] = 0.1f; opt.color[1] = 0.2f; opt.color[2] = 0.3f;
    umbreon::generateObjectSpaceEdges(sc, opt);

    s.check_eq("ortho sphere: ring has `segments` cylinders",
               sc.cylinders.size(), static_cast<std::size_t>(32));

    bool allRadiusOk = true, allPlaneOk = true, allAttrsOk = true,
         chained = true;
    const Vec3 O = sp.center;
    for (std::size_t i = 0; i < sc.cylinders.size(); ++i) {
      const Cylinder& c = sc.cylinders[i];
      if (!edgeAttrsOk(c, opt.width, opt.color, 7)) allAttrsOk = false;
      // Each ring vertex is exactly r from the center (great circle).
      if (!nearf(umbreon::length(c.p0 - O), sp.radius)) allRadiusOk = false;
      // Great circle lies in the plane through O perpendicular to the view dir:
      // (P - O) . viewDir == 0, i.e. constant z == O.z here.
      if (!nearf(c.p0.z, O.z)) allPlaneOk = false;
      // Closed chain: this segment's end is the next segment's start.
      const Cylinder& n = sc.cylinders[(i + 1) % sc.cylinders.size()];
      if (!nearf(umbreon::length(c.p1 - n.p0), 0.0f)) chained = false;
    }
    s.check("ortho sphere: every vertex is radius r from center", allRadiusOk);
    s.check("ortho sphere: ring lies in plane perpendicular to view", allPlaneOk);
    s.check("ortho sphere: edges open/flat/group/radius/color correct",
            allAttrsOk);
    s.check("ortho sphere: ring is a closed chain", chained);
  }

  // ---- (2) SPHERE persp horizon (tangent) circle -------------------------
  {
    Scene sc;
    sc.camera.orthographic = false;
    sc.camera.position = {0, 0, 10};
    Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 2.0f;
    sc.spheres.push_back(sp);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.segments = 24;
    umbreon::generateObjectSpaceEdges(sc, opt);
    s.check_eq("persp sphere: ring has `segments` cylinders",
               sc.cylinders.size(), static_cast<std::size_t>(24));

    bool onSurface = true, normalPerpView = true;
    for (const Cylinder& c : sc.cylinders) {
      // Persp horizon circle: every point is exactly ON the sphere surface.
      if (!nearf(umbreon::length(c.p0 - sp.center), sp.radius)) onSurface = false;
      // Surface normal there is perpendicular to the view ray from the camera
      // (definition of the silhouette: n . v == 0).
      const Vec3 nrm = umbreon::normalize(c.p0 - sp.center);
      const Vec3 view = umbreon::normalize(c.p0 - sc.camera.position);
      if (!nearf(umbreon::dot(nrm, view), 0.0f, 5.0e-3f)) normalPerpView = false;
    }
    s.check("persp sphere: every ring vertex is on the surface", onSurface);
    s.check("persp sphere: surface normal perpendicular to view ray",
            normalPerpView);
  }

  // ---- (2b) SPHERE persp: camera inside => skipped -----------------------
  {
    Scene sc;
    sc.camera.orthographic = false;
    sc.camera.position = {0, 0, 0};  // at the center
    Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 2.0f;
    sc.spheres.push_back(sp);
    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    umbreon::generateObjectSpaceEdges(sc, opt);
    s.check("persp sphere: camera inside => no edges", sc.cylinders.empty());
  }

  // ---- (3) CYLINDER side view: two generators perpendicular to axis&view --
  {
    Scene sc;
    sc.camera.orthographic = true;
    sc.camera.direction = {0, 0, -1};  // view along -z
    Cylinder cyl;
    cyl.p0 = {-3, 0, 0};
    cyl.p1 = {3, 0, 0};  // axis along +x, seen broadside from +z
    cyl.radius = 0.5f;
    cyl.group = 4;
    sc.cylinders.push_back(cyl);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.width = 0.02f;
    opt.raise = 0.0f;
    opt.color[0] = 0.0f; opt.color[1] = 0.0f; opt.color[2] = 0.0f;
    umbreon::generateObjectSpaceEdges(sc, opt);

    // Original + 2 side generators.
    s.check_eq("side cylinder: 2 edges appended", sc.cylinders.size(),
               static_cast<std::size_t>(3));
    const Cylinder& l1 = sc.cylinders[1];
    const Cylinder& l2 = sc.cylinders[2];
    s.check("side cylinder: edges open/flat/group/radius/color correct",
            edgeAttrsOk(l1, opt.width, opt.color, 4) &&
                edgeAttrsOk(l2, opt.width, opt.color, 4));

    // The silhouette offset is perpendicular to the axis (+x) AND the view (z):
    // for this geometry s == +/- y, so the side lines sit at y == +/- radius,
    // z == 0, running parallel to the axis.
    const Vec3 u = umbreon::normalize(cyl.p1 - cyl.p0);
    auto offset = [&](const Cylinder& c) { return c.p0 - Vec3{c.p0.x, 0, 0}; };
    const Vec3 o1 = offset(l1);
    const Vec3 o2 = offset(l2);
    s.check("side cylinder: offsets at +/- radius", nearf(umbreon::length(o1),
            cyl.radius) && nearf(umbreon::length(o2), cyl.radius));
    s.check("side cylinder: offsets perpendicular to axis",
            nearf(umbreon::dot(o1, u), 0.0f) && nearf(umbreon::dot(o2, u), 0.0f));
    s.check("side cylinder: two lines on opposite sides",
            nearf(umbreon::length(o1 + o2), 0.0f));
    // Both generators are parallel to the axis (a side line of a cylinder).
    s.check("side cylinder: generators parallel to axis",
            nearf(umbreon::length(umbreon::cross(
                      umbreon::normalize(l1.p1 - l1.p0), u)), 0.0f) &&
            nearf(umbreon::length(umbreon::cross(
                      umbreon::normalize(l2.p1 - l2.p0), u)), 0.0f));
  }

  // ---- (3b) CYLINDER raise pushes the side lines outward by `raise` -------
  {
    Scene sc;
    sc.camera.orthographic = true;
    sc.camera.direction = {0, 0, -1};
    Cylinder cyl;
    cyl.p0 = {-3, 0, 0};
    cyl.p1 = {3, 0, 0};
    cyl.radius = 0.5f;
    sc.cylinders.push_back(cyl);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.raise = 0.1f;
    umbreon::generateObjectSpaceEdges(sc, opt);
    const Cylinder& l1 = sc.cylinders[1];
    const Vec3 o1 = l1.p0 - Vec3{l1.p0.x, 0, 0};
    s.check("raise cylinder: offset == radius + raise",
            nearf(umbreon::length(o1), cyl.radius + opt.raise));
  }

  // ---- (4) CYLINDER end-on => cap-rim circle, not side lines -------------
  {
    Scene sc;
    sc.camera.orthographic = true;
    sc.camera.direction = {0, 0, -1};  // view along -z
    Cylinder cyl;
    cyl.p0 = {0, 0, -2};
    cyl.p1 = {0, 0, 2};  // axis along the view direction (end-on)
    cyl.radius = 0.7f;
    cyl.group = 9;
    sc.cylinders.push_back(cyl);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.segments = 20;
    opt.width = 0.03f;
    umbreon::generateObjectSpaceEdges(sc, opt);
    // A cap circle of `segments` cylinders replaces the (collapsed) side lines.
    s.check_eq("end-on cylinder: cap-rim ring emitted", sc.cylinders.size(),
               static_cast<std::size_t>(1 + 20));
    bool ringRadiusOk = true, attrsOk = true;
    // The nearer cap (to a +z ortho viewer) is at z == +2; rim radius == r.
    for (std::size_t i = 1; i < sc.cylinders.size(); ++i) {
      const Cylinder& c = sc.cylinders[i];
      const Vec3 radial = c.p0 - Vec3{0, 0, c.p0.z};
      if (!nearf(umbreon::length(radial), cyl.radius)) ringRadiusOk = false;
      if (!edgeAttrsOk(c, opt.width, opt.color, 9)) attrsOk = false;
    }
    s.check("end-on cylinder: rim vertices at radius r from axis", ringRadiusOk);
    s.check("end-on cylinder: rim edges open/flat/group/radius/color correct",
            attrsOk);
  }

  // ---- (5) snapshot: appended edges are not themselves silhouetted -------
  {
    Scene sc;
    sc.camera.orthographic = true;
    sc.camera.direction = {0, 0, -1};
    Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.0f;
    sc.spheres.push_back(sp);
    Cylinder cyl;
    cyl.p0 = {-2, 0, 0};
    cyl.p1 = {2, 0, 0};
    cyl.radius = 0.3f;
    sc.cylinders.push_back(cyl);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.segments = 16;
    opt.clip = false;  // raw-geometry count (the clip is exercised in test 6)
    umbreon::generateObjectSpaceEdges(sc, opt);
    // Exactly: 1 original cylinder + 16 sphere-ring + 2 cylinder side lines.
    // (If the snapshot were broken, the appended sphere-ring/side cylinders
    // would be silhouetted again and the count would explode.)
    s.check_eq("snapshot: count is original + sphere-ring + 2 (no recursion)",
               sc.cylinders.size(), static_cast<std::size_t>(1 + 16 + 2));
  }

  // ---- (6) union-boundary clip: a bond through an atom is trimmed --------
  {
    // A cylinder (radius 0.3) running through a sphere (radius 1): its side
    // lines pass deep inside the sphere. The clip must drop those inside parts
    // (the connecting atom's surface takes over there) so connecting primitives
    // join along the intersection instead of crossing at coincident depth.
    Scene base;
    base.camera.orthographic = true;
    base.camera.direction = {0, 0, -1};
    Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.0f;
    base.spheres.push_back(sp);
    Cylinder cyl;
    cyl.p0 = {-2, 0, 0};
    cyl.p1 = {2, 0, 0};  // axis through the sphere center
    cyl.radius = 0.3f;
    base.cylinders.push_back(cyl);

    auto deepInsideAtom = [&](const Scene& s2) {
      // Any APPENDED edge (index >= 1, after the original cylinder) whose
      // midpoint is well inside the sphere (beyond the ~0.02 ring-chord sag).
      for (std::size_t i = 1; i < s2.cylinders.size(); ++i) {
        const Vec3 mid = (s2.cylinders[i].p0 + s2.cylinders[i].p1) * 0.5f;
        if (umbreon::length(mid - sp.center) < sp.radius - 0.1f) return true;
      }
      return false;
    };

    Scene off = base;
    ObjectSpaceEdgeOptions oOff;
    oOff.enable = true;
    oOff.segments = 16;
    oOff.clip = false;
    umbreon::generateObjectSpaceEdges(off, oOff);
    s.check("clip off: a side line runs deep inside the atom", deepInsideAtom(off));

    Scene on = base;
    ObjectSpaceEdgeOptions oOn;
    oOn.enable = true;
    oOn.segments = 16;
    oOn.clip = true;
    umbreon::generateObjectSpaceEdges(on, oOn);
    s.check("clip on: no edge runs deep inside the atom (union boundary)",
            !deepInsideAtom(on));
    // The clip splits/drops segments, so it must still leave a non-empty outline.
    s.check("clip on: outline is not emptied", on.cylinders.size() > 1);
  }

  // ---- (7) ball-and-stick: a protruding atom keeps its FULL circle ----------
  {
    // When a sphere protrudes beyond a bond (rs > rc, ball-and-stick) the bond is
    // buried inside the atom's silhouette, so the atom's circle must stay
    // continuous -- the thin bond must NOT clip the ring. Only an equal/larger
    // bond (rs <= rc, licorice) takes over and clips it (no "ball bump" on the
    // smooth tube). Edges carry their source primitive's group, so the ring edges
    // (sphere group) are countable apart from the bond's side lines.
    auto ringEdges = [](const Scene& sc, uint16_t grp, std::size_t origCyl) {
      std::size_t n = 0;
      for (std::size_t i = origCyl; i < sc.cylinders.size(); ++i)
        if (sc.cylinders[i].group == grp) ++n;
      return n;
    };
    ObjectSpaceEdgeOptions o;
    o.enable = true;
    o.segments = 24;
    o.clip = true;
    Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.0f;
    sp.group = 1;
    Camera cam;
    cam.orthographic = true;
    cam.direction = {0, 0, -1};

    // Sphere alone: the ring is a full closed loop of `segments` edges.
    Scene a;
    a.camera = cam;
    a.spheres.push_back(sp);
    umbreon::generateObjectSpaceEdges(a, o);
    const std::size_t fullRing = ringEdges(a, 1, 0);
    s.check("sphere-alone ring is a full closed loop", fullRing == 24);

    // Sphere + THIN coaxial bond (ball-and-stick): ring stays complete.
    Scene b;
    b.camera = cam;
    b.spheres.push_back(sp);
    Cylinder thin;
    thin.p0 = {0, 0, 0};
    thin.p1 = {3, 0, 0};
    thin.radius = 0.3f;
    thin.group = 2;
    b.cylinders.push_back(thin);
    umbreon::generateObjectSpaceEdges(b, o);
    s.check("ball-stick: a thinner bond does NOT clip the atom's circle",
            ringEdges(b, 1, 1) == fullRing);

    // Sphere + EQUAL-radius bond (licorice): ring is clipped where the bond runs.
    Scene c;
    c.camera = cam;
    c.spheres.push_back(sp);
    Cylinder thick;
    thick.p0 = {0, 0, 0};
    thick.p1 = {3, 0, 0};
    thick.radius = 1.0f;
    thick.group = 2;
    c.cylinders.push_back(thick);
    umbreon::generateObjectSpaceEdges(c, o);
    s.check("licorice: an equal-radius bond clips the ring (no ball bump)",
            ringEdges(c, 1, 1) < fullRing);
  }

  // ---- (8) triangle-mesh edges: border / smooth silhouette / crease ---------
  {
    Camera cam;
    cam.orthographic = true;
    cam.direction = {0, 0, -1};  // viewer toward +z

    // BORDER: a lone triangle has three single-face boundary edges.
    {
      Scene sc;
      sc.camera = cam;
      sc.mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
      sc.mesh.normals = {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = false;
      o.meshCrease = false;
      o.meshBorder = true;
      umbreon::generateObjectSpaceEdges(sc, o);
      s.check("mesh border: a lone triangle emits 3 boundary edges",
              sc.cylinders.size() == 3);
    }

    // SMOOTH SILHOUETTE: vertex normals (+,+,-) make n.v change sign across the
    // face, so exactly one zero-crossing segment is emitted (Freestyle smooth).
    {
      Scene sc;
      sc.camera = cam;
      sc.mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
      sc.mesh.normals = {{0, 0, 1}, {0, 0, 1}, {0, 0, -1}};
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = true;
      o.meshCrease = false;
      o.meshBorder = false;
      umbreon::generateObjectSpaceEdges(sc, o);
      s.check("mesh silhouette: one n.v sign-change face -> one smooth edge",
              sc.cylinders.size() == 1);
    }

    // CREASE: two triangles folded 90 degrees share one edge whose dihedral
    // fires a crease below the threshold but not above it.
    auto creaseCount = [&](float deg) {
      Scene sc;
      sc.camera = cam;
      // shared edge = the x-axis (0,0,0)-(1,0,0); tri A in z=0, tri B in y=0.
      sc.mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                           {1, 0, 0}, {0, 0, 0}, {0, 0, 1}};
      sc.mesh.normals.assign(6, Vec3{0, 0, 1});
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = false;
      o.meshCrease = true;
      o.meshBorder = false;
      o.creaseAngleDeg = deg;
      umbreon::generateObjectSpaceEdges(sc, o);
      return sc.cylinders.size();
    };
    s.check("mesh crease: a 90deg fold IS a crease at 45deg threshold",
            creaseCount(45.0f) == 1);
    s.check("mesh crease: a 90deg fold is NOT a crease at 120deg threshold",
            creaseCount(120.0f) == 0);
  }

  // ---- (9) mesh crease/border GEOMETRIC gates (no color) --------------------
  {
    Camera cam;
    cam.orthographic = true;
    cam.direction = {0, 0, -1};
    auto creaseN = [&](const std::vector<Vec3>& pos, const std::vector<Vec3>& nrm,
                       float creaseDeg, bool convexOnly, float smoothDeg) {
      Scene sc;
      sc.camera = cam;
      sc.mesh.positions = pos;
      if (!nrm.empty()) sc.mesh.normals = nrm;
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = false;
      o.meshCrease = true;
      o.meshBorder = false;
      o.creaseAngleDeg = creaseDeg;
      o.meshCreaseConvexOnly = convexOnly;
      o.meshCreaseSmoothVetoDeg = smoothDeg;
      umbreon::generateObjectSpaceEdges(sc, o);
      return sc.cylinders.size();
    };

    // CONVEX-ONLY: a convex ridge survives; a concave valley of the same dihedral
    // is dropped. Shared ridge edge along x; apexes below (convex) vs above.
    const std::vector<Vec3> convex = {{0, 0, 0}, {2, 0, 0}, {1, 1, -0.5f},
                                      {2, 0, 0}, {0, 0, 0}, {1, -1, -0.5f}};
    const std::vector<Vec3> concave = {{0, 0, 0}, {2, 0, 0}, {1, 1, 0.5f},
                                       {2, 0, 0}, {0, 0, 0}, {1, -1, 0.5f}};
    s.check("mesh convex-only OFF: a ridge fold is a crease",
            creaseN(convex, {}, 30.0f, false, 0.0f) == 1);
    s.check("mesh convex-only: a convex ridge is kept",
            creaseN(convex, {}, 30.0f, true, 0.0f) == 1);
    s.check("mesh convex-only: a concave valley is dropped",
            creaseN(concave, {}, 30.0f, true, 0.0f) == 0);

    // SMOOTH-FACET VETO: a ~20deg fold whose vertex normals AGREE (smooth-shaded,
    // here the +z bisector) is tessellation, not a crease. creaseDeg=10 makes the
    // fold a candidate.
    const std::vector<Vec3> facet = {{0, 0, 0},    {1, 0, 0}, {0.5f, 1, 0.176f},
                                     {1, 0, 0},    {0, 0, 0}, {0.5f, -1, 0.176f}};
    const std::vector<Vec3> smoothN(6, Vec3{0, 0, 1});
    s.check("mesh smooth-veto OFF: a 20deg fold is a crease",
            creaseN(facet, smoothN, 10.0f, false, 0.0f) == 1);
    s.check("mesh smooth-veto: a smooth-shaded facet seam is dropped",
            creaseN(facet, smoothN, 10.0f, false, 25.0f) == 0);

    // BORDER COPLANAR VETO: a straight top border chain p0-p1-p2-p3 over a fan apex
    // q. The middle edge p1-p2 continues collinearly at BOTH ends (internal seam)
    // and is dropped; the end and side borders remain.
    auto borderN = [&](float vetoDeg) {
      Scene sc;
      sc.camera = cam;
      sc.mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1.5f, -1, 0},
                           {1, 0, 0}, {2, 0, 0}, {1.5f, -1, 0},
                           {2, 0, 0}, {3, 0, 0}, {1.5f, -1, 0}};
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = false;
      o.meshCrease = false;
      o.meshBorder = true;
      o.meshBorderCoplanarVetoDeg = vetoDeg;
      umbreon::generateObjectSpaceEdges(sc, o);
      return sc.cylinders.size();
    };
    const std::size_t b0 = borderN(0.0f);
    s.check("mesh border-veto OFF: all border edges kept", b0 == 5);
    s.check("mesh border-veto: the collinear internal seam edge is dropped",
            borderN(35.0f) == b0 - 1);
  }

  // ---- (10) Freestyle on-contour vertex + crease-cluster degree filter -------
  {
    const float kPiT = 3.14159265358979f;
    Camera cam;
    cam.orthographic = true;
    cam.direction = {0, 0, -1};  // viewer toward +z

    // FREESTYLE on-contour vertex: a vertex whose normal is exactly perpendicular
    // to the view (DotP==0) lies ON the silhouette. The face must still emit one
    // segment (vertex -> opposite-edge crossing) instead of being skipped, which
    // the plain two-sign-change extraction would do (nicking the outline).
    {
      Scene sc;
      sc.camera = cam;
      sc.mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
      sc.mesh.normals = {{1, 0, 0}, {0, 0, 1}, {0, 0, -1}};  // DotP = 0, +1, -1
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = true;
      o.meshCrease = false;
      o.meshBorder = false;
      umbreon::generateObjectSpaceEdges(sc, o);
      s.check("mesh silhouette: an on-contour vertex (DotP==0) still emits a segment",
              sc.cylinders.size() == 1);
    }

    // CREASE-CLUSTER DEGREE FILTER: a hexagonal cone's six lateral creases all meet
    // at the apex (crease degree 6). A high limit keeps them; a low limit drops the
    // whole cluster -- the geometric stand-in for a tube/terminus cap blob.
    auto coneCreases = [&](int maxDeg) {
      Scene sc;
      sc.camera = cam;
      Vec3 r[6];
      for (int i = 0; i < 6; ++i) {
        const float a = static_cast<float>(i) * (kPiT / 3.0f);
        r[i] = Vec3{std::cos(a), std::sin(a), 0.0f};
      }
      const Vec3 c{0, 0, 1};
      for (int i = 0; i < 6; ++i) {
        sc.mesh.positions.push_back(c);
        sc.mesh.positions.push_back(r[i]);
        sc.mesh.positions.push_back(r[(i + 1) % 6]);
      }
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = false;
      o.meshCrease = true;
      o.meshBorder = false;
      o.creaseAngleDeg = 20.0f;
      o.meshCreaseConvexOnly = false;
      o.meshCreaseSmoothVetoDeg = 0.0f;
      o.meshCreaseMaxDegree = maxDeg;
      umbreon::generateObjectSpaceEdges(sc, o);
      return sc.cylinders.size();
    };
    s.check("cone apex: the six lateral creases are present (filter off)",
            coneCreases(0) == 6);
    s.check("cone apex: the crease cluster is dropped at a low degree limit",
            coneCreases(4) == 0);
  }

  return s.report();
}
