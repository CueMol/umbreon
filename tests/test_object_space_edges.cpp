// Unit / integration tests for OBJECT-SPACE edges
// (src/umbreon/edges/object/object_space_edges.{hpp,cpp}).
//
// Guards the geometric and rendering contract a future edit could break:
//   * disabled => nothing appended (byte-identical default);
//   * sphere horizon rings / end-on cap rims are exact circles drawn as
//     Bezier arcs (Scene::edgeArcs), raised by `raise`; cylinder side lines are
//     open flat cylinders offset by radius + raise;
//   * the union clip: same-section solids clip, thinner bonds included; other
//     sections and line-drawing cylinders (Cylinder::noEdge) do not;
//   * mesh contours lie on mesh edges (face-normal straddle); a zero-area
//     sliver adds no line; crease / border gates;
//   * per-section styles (Scene::groupObjectEdgeStyle);
//   * render(): silhouette (Outline) mode drops a line only over its own
//     section, and the near clip plane keeps or drops a line by its center.
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "edges/object/object_space_edges.hpp"
#include "scene.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"  // render(): drives the pass via RenderOptions::objectSpaceEdges

namespace {

using umbreon::Camera;
using umbreon::Cylinder;
using umbreon::EdgeArc;
using umbreon::ObjectEdgeStyle;
using umbreon::ObjectSpaceEdgeOptions;
using umbreon::Scene;
using umbreon::Sphere;
using umbreon::Vec3;
using umbreon::Vec4;

constexpr float kPi = 3.14159265358979f;

bool nearf(float a, float b, float eps = 1.0e-3f) {
  return std::fabs(a - b) <= eps;
}

Camera orthoZ() {
  Camera c;
  c.orthographic = true;
  c.position = {0, 0, 10};
  c.direction = {0, 0, -1};  // view along -z
  c.up = {0, 1, 0};
  c.height = 4.0f;
  return c;
}

// An emitted straight edge renders like a flat outline: open, flat material,
// not a baked POV macro line, requested radius / color / group, an object edge.
bool edgeAttrsOk(const Cylinder& c, float width, const float col[3],
                 uint16_t group) {
  const umbreon::Material& m = c.material;
  const bool mat = nearf(m.ambient, 1.0f) && nearf(m.diffuse, 0.0f) &&
                   nearf(m.specular, 0.0f);
  return c.open && !c.fromEdgeMacro && mat && nearf(c.radius, width) &&
         c.group == group && c.objEdge == umbreon::kObjEdgeFull &&
         nearf(c.color.x, col[0]) && nearf(c.color.y, col[1]) &&
         nearf(c.color.z, col[2]) && nearf(c.color.w, 1.0f);
}

// Total angle (radians) the arcs of `group` cover around `center`.
float arcAngle(const Scene& sc, const Vec3& center, uint16_t group) {
  float sum = 0.0f;
  for (const EdgeArc& a : sc.edgeArcs) {
    if (a.group != group) continue;
    const Vec3 u = umbreon::normalize(a.cp[0] - center);
    const Vec3 v = umbreon::normalize(a.cp[3] - center);
    sum += std::acos(std::fmax(-1.0f, std::fmin(1.0f, umbreon::dot(u, v))));
  }
  return sum;
}

// Flat white sphere / mesh: renders as plain white so a black line shows.
umbreon::Material flatWhite() { return umbreon::Material::flatOutline(); }

}  // namespace

int main() {
  umbreon::test::Suite s("object_space_edges");

  // ---- (0) disabled => no-op (byte-identical default) --------------------
  {
    Scene sc;
    Sphere sp;
    sp.radius = 1.0f;
    sc.spheres.push_back(sp);
    sc.camera = orthoZ();
    ObjectSpaceEdgeOptions opt;  // enable defaults to false
    umbreon::generateObjectSpaceEdges(sc, opt);
    s.check("disabled: nothing appended",
            sc.cylinders.empty() && sc.edgeArcs.empty());
  }

  // ---- (1) SPHERE ortho ring: an exact circle of Bezier arcs, raised -------
  {
    Scene sc;
    sc.camera = orthoZ();
    Sphere sp;
    sp.center = {1, 2, 3};
    sp.radius = 2.0f;
    sp.group = 7;
    sc.spheres.push_back(sp);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.width = 0.05f;
    opt.raise = 0.1f;
    opt.color[0] = 0.1f; opt.color[1] = 0.2f; opt.color[2] = 0.3f;
    umbreon::generateObjectSpaceEdges(sc, opt);

    s.check_eq("ortho sphere: no straight edges", sc.cylinders.size(),
               static_cast<std::size_t>(0));
    // At most 45 degrees per arc: a full circle is 8 arcs.
    s.check_eq("ortho sphere: full ring is 8 arcs", sc.edgeArcs.size(),
               static_cast<std::size_t>(8));
    bool onCircle = true, inPlane = true, attrs = true, chained = true,
         midOnCircle = true;
    const Vec3 O = sp.center;
    const float R = sp.radius + opt.raise;  // raised along the normal
    for (std::size_t i = 0; i < sc.edgeArcs.size(); ++i) {
      const EdgeArc& a = sc.edgeArcs[i];
      if (!nearf(umbreon::length(a.cp[0] - O), R)) onCircle = false;
      if (!nearf(a.cp[0].z, O.z) || !nearf(a.cp[3].z, O.z)) inPlane = false;
      if (!nearf(a.radius, opt.width) || a.group != 7 ||
          a.kind != umbreon::kObjEdgeFull || !nearf(a.color.x, 0.1f) ||
          !nearf(a.color.z, 0.3f))
        attrs = false;
      // The cubic's midpoint stays on the circle (not a chord).
      const Vec3 mid = (a.cp[0] + a.cp[1] * 3.0f + a.cp[2] * 3.0f + a.cp[3]) *
                       (1.0f / 8.0f);
      if (!nearf(umbreon::length(mid - O), R, 2.0e-3f)) midOnCircle = false;
      const EdgeArc& n = sc.edgeArcs[(i + 1) % sc.edgeArcs.size()];
      if (!nearf(umbreon::length(a.cp[3] - n.cp[0]), 0.0f)) chained = false;
    }
    s.check("ortho sphere: arc ends on the raised circle", onCircle);
    s.check("ortho sphere: arc midpoints on the circle", midOnCircle);
    s.check("ortho sphere: ring in the plane perpendicular to view", inPlane);
    s.check("ortho sphere: arc radius/group/kind/color", attrs);
    s.check("ortho sphere: arcs form a closed chain", chained);
  }

  // ---- (2) SPHERE persp horizon (tangent) circle --------------------------
  {
    Scene sc;
    sc.camera.orthographic = false;
    sc.camera.position = {0, 0, 10};
    Sphere sp;
    sp.radius = 2.0f;
    sc.spheres.push_back(sp);
    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    umbreon::generateObjectSpaceEdges(sc, opt);
    bool onSurface = !sc.edgeArcs.empty(), perpView = true;
    for (const EdgeArc& a : sc.edgeArcs) {
      if (!nearf(umbreon::length(a.cp[0] - sp.center), sp.radius))
        onSurface = false;
      const Vec3 nrm = umbreon::normalize(a.cp[0] - sp.center);
      const Vec3 view = umbreon::normalize(a.cp[0] - sc.camera.position);
      if (!nearf(umbreon::dot(nrm, view), 0.0f, 5.0e-3f)) perpView = false;
    }
    s.check("persp sphere: ring points on the surface", onSurface);
    s.check("persp sphere: normal perpendicular to the view ray", perpView);

    Scene in = sc;
    in.edgeArcs.clear();
    in.camera.position = {0, 0, 0};  // inside the sphere
    umbreon::generateObjectSpaceEdges(in, opt);
    s.check("persp sphere: camera inside => no ring", in.edgeArcs.empty());
  }

  // ---- (3) CYLINDER side view: two generators at radius + raise ----------
  {
    Scene sc;
    sc.camera = orthoZ();
    Cylinder cyl;
    cyl.p0 = {-3, 0, 0};
    cyl.p1 = {3, 0, 0};  // axis along +x, seen broadside
    cyl.radius = 0.5f;
    cyl.group = 4;
    sc.cylinders.push_back(cyl);

    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.width = 0.02f;
    opt.raise = 0.1f;
    umbreon::generateObjectSpaceEdges(sc, opt);
    s.check_eq("side cylinder: 2 side lines appended", sc.cylinders.size(),
               static_cast<std::size_t>(3));
    const Cylinder& l1 = sc.cylinders[1];
    const Cylinder& l2 = sc.cylinders[2];
    s.check("side cylinder: edge attributes",
            edgeAttrsOk(l1, opt.width, opt.color, 4) &&
                edgeAttrsOk(l2, opt.width, opt.color, 4));
    const Vec3 u{1, 0, 0};
    const Vec3 o1 = l1.p0 - Vec3{l1.p0.x, 0, 0};
    const Vec3 o2 = l2.p0 - Vec3{l2.p0.x, 0, 0};
    s.check("side cylinder: offsets at radius + raise",
            nearf(umbreon::length(o1), cyl.radius + opt.raise) &&
                nearf(umbreon::length(o2), cyl.radius + opt.raise));
    s.check("side cylinder: opposite sides, perpendicular to axis",
            nearf(umbreon::length(o1 + o2), 0.0f) &&
                nearf(umbreon::dot(o1, u), 0.0f));
  }

  // ---- (4) CYLINDER end-on => cap-rim circle, not side lines -------------
  {
    Scene sc;
    sc.camera = orthoZ();
    Cylinder cyl;
    cyl.p0 = {0, 0, -2};
    cyl.p1 = {0, 0, 2};  // axis along the view
    cyl.radius = 0.7f;
    cyl.group = 9;
    sc.cylinders.push_back(cyl);
    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    umbreon::generateObjectSpaceEdges(sc, opt);
    bool rimOk = sc.edgeArcs.size() == 8;
    for (const EdgeArc& a : sc.edgeArcs) {
      // The nearer cap (z = +2) rim, radius r about the axis.
      if (!nearf(a.cp[0].z, 2.0f) ||
          !nearf(umbreon::length(Vec3{a.cp[0].x, a.cp[0].y, 0}), cyl.radius) ||
          a.group != 9)
        rimOk = false;
    }
    s.check("end-on cylinder: cap rim of 8 arcs, no side lines",
            rimOk && sc.cylinders.size() == 1);
  }

  // ---- (5) snapshot: appended edges are not themselves silhouetted -------
  {
    Scene sc;
    sc.camera = orthoZ();
    Sphere sp;
    sp.radius = 1.0f;
    sc.spheres.push_back(sp);
    Cylinder cyl;
    cyl.p0 = {-2, 0, 0};
    cyl.p1 = {2, 0, 0};
    cyl.radius = 0.3f;
    sc.cylinders.push_back(cyl);
    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.clip = false;  // raw geometry (the clip is checked below)
    umbreon::generateObjectSpaceEdges(sc, opt);
    s.check("snapshot: 1 original + 2 side lines, one ring (no recursion)",
            sc.cylinders.size() == 3 && sc.edgeArcs.size() == 8);
  }

  // ---- (6) union clip: same-section solids, thinner bonds included --------
  {
    // A sphere (r 1) with a bond leaving it along +x, both in section 1.
    auto makeScene = [](float bondR, uint16_t bondGroup, bool noEdge) {
      Scene sc;
      sc.camera = orthoZ();
      Sphere sp;
      sp.radius = 1.0f;
      sp.group = 1;
      sc.spheres.push_back(sp);
      Cylinder b;
      b.p0 = {0, 0, 0};
      b.p1 = {3, 0, 0};
      b.radius = bondR;
      b.group = bondGroup;
      b.noEdge = noEdge;
      sc.cylinders.push_back(b);
      return sc;
    };
    ObjectSpaceEdgeOptions o;
    o.enable = true;
    o.clip = true;
    const Vec3 O{0, 0, 0};

    Scene alone;
    alone.camera = orthoZ();
    Sphere sp;
    sp.radius = 1.0f;
    sp.group = 1;
    alone.spheres.push_back(sp);
    umbreon::generateObjectSpaceEdges(alone, o);
    const float full = arcAngle(alone, O, 1);
    s.check("clip: a lone sphere keeps the whole ring", nearf(full, 2 * kPi, 1e-2f));

    // A thinner bond of the same section cuts the ring where it leaves the
    // atom: the arc inside the bond (2 asin(0.3) ~ 35 deg) is gone.
    Scene thin = makeScene(0.3f, 1, false);
    umbreon::generateObjectSpaceEdges(thin, o);
    const float gap = full - arcAngle(thin, O, 1);
    s.check("clip: a thinner same-section bond cuts the ring at the joint",
            gap > 0.45f && gap < 0.75f);
    bool sideInside = false;  // the bond's side lines stop at the atom surface
    for (std::size_t i = 1; i < thin.cylinders.size(); ++i) {
      const Cylinder& c = thin.cylinders[i];
      if (c.radius < 1e-6f || umbreon::length(c.p1 - c.p0) < 1e-3f) continue;
      const Vec3 mid = (c.p0 + c.p1) * 0.5f;
      if (umbreon::length(mid) < 0.9f) sideInside = true;
    }
    s.check("clip: the bond's side lines stop at the atom surface", !sideInside);

    // An equal-radius bond cuts half of the ring.
    Scene lic = makeScene(1.0f, 1, false);
    umbreon::generateObjectSpaceEdges(lic, o);
    s.check("clip: an equal-radius bond cuts half of the ring",
            nearf(arcAngle(lic, O, 1), kPi, 0.1f));

    // Another section's solid and a line-drawing cylinder do not clip.
    Scene other = makeScene(0.3f, 2, false);
    umbreon::generateObjectSpaceEdges(other, o);
    s.check("clip: a bond of another section leaves the ring whole",
            nearf(arcAngle(other, O, 1), full, 1e-2f));
    Scene wire = makeScene(0.3f, 1, true);
    umbreon::generateObjectSpaceEdges(wire, o);
    s.check("clip: a line-drawing cylinder (noEdge) neither clips nor gets lines",
            nearf(arcAngle(wire, O, 1), full, 1e-2f) &&
                wire.cylinders.size() == 1);
  }

  // ---- (7) per-section styles -------------------------------------------
  {
    Scene sc;
    sc.camera = orthoZ();
    for (uint16_t g = 0; g < 3; ++g) {
      Sphere sp;
      sp.center = {static_cast<float>(g) * 3.0f - 3.0f, 0, 0};
      sp.radius = 1.0f;
      sp.group = g;
      sc.spheres.push_back(sp);
    }
    sc.groupObjectEdgeStyle.resize(3);
    sc.groupObjectEdgeStyle[0].color[0] = 1.0f;
    sc.groupObjectEdgeStyle[0].width = 0.04f;
    sc.groupObjectEdgeStyle[1].color[2] = 1.0f;
    sc.groupObjectEdgeStyle[1].width = 0.08f;
    sc.groupObjectEdgeStyle[2].silhouette = false;
    sc.groupObjectEdgeStyle[2].crease = false;
    sc.groupObjectEdgeStyle[2].border = false;  // disabled section
    ObjectSpaceEdgeOptions o;
    o.enable = true;
    umbreon::generateObjectSpaceEdges(sc, o);
    bool ok0 = false, ok1 = false, none2 = true;
    for (const EdgeArc& a : sc.edgeArcs) {
      if (a.group == 0) ok0 = nearf(a.radius, 0.04f) && nearf(a.color.x, 1.0f);
      if (a.group == 1) ok1 = nearf(a.radius, 0.08f) && nearf(a.color.z, 1.0f);
      if (a.group == 2) none2 = false;
    }
    s.check("styles: each section draws with its own color and radius", ok0 && ok1);
    s.check("styles: a disabled section draws nothing", none2);
  }

  // ---- (8) triangle-mesh edges -------------------------------------------
  {
    const Camera cam = orthoZ();
    auto meshEdges = [&](const umbreon::Mesh& m, bool sil, bool crease,
                         bool border) {
      Scene sc;
      sc.camera = cam;
      sc.mesh = m;
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = sil;
      o.meshCrease = crease;
      o.meshBorder = border;
      umbreon::generateObjectSpaceEdges(sc, o);
      return sc.cylinders.size();
    };

    // BORDER: a lone triangle has three single-face boundary edges.
    {
      umbreon::Mesh m;
      m.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
      m.normals.assign(3, Vec3{0, 0, 1});
      s.check("mesh border: a lone triangle emits 3 boundary edges",
              meshEdges(m, false, false, true) == 3);
    }

    // SILHOUETTE on mesh edges: a cube seen along its diagonal outlines with
    // its 6 rim edges (the in-face diagonals and the hidden edges add none).
    {
      umbreon::Mesh m;
      const Vec3 v[8] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                         {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
      const int q[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4},
                           {2, 3, 7, 6}, {1, 2, 6, 5}, {0, 4, 7, 3}};
      for (const auto& f : q) {  // outward winding, two triangles per face
        const int t[6] = {f[0], f[1], f[2], f[0], f[2], f[3]};
        const Vec3 n = umbreon::normalize(umbreon::cross(v[f[1]] - v[f[0]],
                                                         v[f[2]] - v[f[0]]));
        for (int k : t) {
          m.positions.push_back(v[k]);
          m.normals.push_back(n);
        }
      }
      Scene sc;
      sc.camera.orthographic = true;
      sc.camera.direction = umbreon::normalize(Vec3{-1, -1, -1});
      sc.camera.position = {5, 5, 5};
      sc.mesh = m;
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshCrease = false;
      o.meshBorder = false;
      umbreon::generateObjectSpaceEdges(sc, o);
      s.check_eq("mesh silhouette: a cube along its diagonal has 6 rim edges",
                 sc.cylinders.size(), static_cast<std::size_t>(6));
    }

    // SLIVER: a flat square whose diagonal carries a zero-area triangle. The
    // sliver's normal is rounding noise; it must add no line across the face
    // and must not turn the diagonal into a border.
    {
      umbreon::Mesh m;
      const Vec3 a{0, 0, 0}, b{2, 0, 0}, c{2, 2, 0}, d{0, 2, 0};
      const Vec3 mid{1.0f, 1.0f, 1.0e-7f};  // on the diagonal a-c
      const Vec3 tris[9] = {a, b, c, a, c, d, a, mid, c};
      for (const Vec3& p : tris) {
        m.positions.push_back(p);
        m.normals.push_back(Vec3{0, 0, 1});
      }
      s.check("mesh sliver: no silhouette or crease across a flat face",
              meshEdges(m, true, true, false) == 0);
      s.check("mesh sliver: only the 4 outer borders",
              meshEdges(m, false, false, true) == 4);
    }

    // EDGE MODES (CueMol MeshFace nmode): a kTriNoBorder face draws no border,
    // and a kTriNoEdge face no line on any edge (the fold below is a crease).
    {
      umbreon::Mesh tri;
      tri.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
      tri.normals.assign(3, Vec3{0, 0, 1});
      tri.triEdgeMode = {umbreon::Mesh::kTriNoBorder};
      umbreon::Mesh fold;
      fold.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                        {1, 0, 0}, {0, 0, 0}, {0, 0, 1}};
      fold.normals.assign(6, Vec3{0, 0, 1});
      fold.triEdgeMode = {umbreon::Mesh::kTriEdgeNormal, umbreon::Mesh::kTriNoEdge};
      s.check("mesh edge mode: a no-border face draws no border",
              meshEdges(tri, false, false, true) == 0);
      s.check("mesh edge mode: an edge on a no-edge face draws nothing",
              meshEdges(fold, true, true, false) == 0);
    }

    // CREASE: a 90 degree fold fires below the threshold but not above it.
    auto creaseCount = [&](float deg) {
      Scene sc;
      sc.camera = cam;
      sc.mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                           {1, 0, 0}, {0, 0, 0}, {0, 0, 1}};
      sc.mesh.normals.assign(6, Vec3{0, 0, 1});
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = false;
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

  // ---- (9) mesh crease/border geometric gates ------------------------------
  {
    const Camera cam = orthoZ();
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
    const std::vector<Vec3> convex = {{0, 0, 0}, {2, 0, 0}, {1, 1, -0.5f},
                                      {2, 0, 0}, {0, 0, 0}, {1, -1, -0.5f}};
    const std::vector<Vec3> concave = {{0, 0, 0}, {2, 0, 0}, {1, 1, 0.5f},
                                       {2, 0, 0}, {0, 0, 0}, {1, -1, 0.5f}};
    s.check("mesh convex-only: a convex ridge is kept",
            creaseN(convex, {}, 30.0f, true, 0.0f) == 1);
    s.check("mesh convex-only: a concave valley is dropped",
            creaseN(concave, {}, 30.0f, true, 0.0f) == 0);
    const std::vector<Vec3> facet = {{0, 0, 0},    {1, 0, 0}, {0.5f, 1, 0.176f},
                                     {1, 0, 0},    {0, 0, 0}, {0.5f, -1, 0.176f}};
    const std::vector<Vec3> smoothN(6, Vec3{0, 0, 1});
    s.check("mesh smooth-veto OFF: a 20deg fold is a crease",
            creaseN(facet, smoothN, 10.0f, false, 0.0f) == 1);
    s.check("mesh smooth-veto: a smooth-shaded facet seam is dropped",
            creaseN(facet, smoothN, 10.0f, false, 25.0f) == 0);

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

    // Crease-cluster degree filter: a hexagonal cone's six creases meet at the
    // apex (degree 6); a low limit drops the cluster (a tube cap blob).
    auto coneCreases = [&](int maxDeg) {
      Scene sc;
      sc.camera = cam;
      Vec3 r[6];
      for (int i = 0; i < 6; ++i) {
        const float a = static_cast<float>(i) * (kPi / 3.0f);
        r[i] = Vec3{std::cos(a), std::sin(a), 0.0f};
      }
      for (int i = 0; i < 6; ++i) {
        sc.mesh.positions.push_back(Vec3{0, 0, 1});
        sc.mesh.positions.push_back(r[i]);
        sc.mesh.positions.push_back(r[(i + 1) % 6]);
      }
      ObjectSpaceEdgeOptions o;
      o.enable = true;
      o.meshSilhouette = false;
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

  // ---- (R) render() integration --------------------------------------------
  {
    using umbreon::FrameResult;
    using umbreon::RenderOptions;

    auto makeScene = []() {
      Scene sc;
      sc.camera = orthoZ();
      sc.background = {1, 1, 1};
      Sphere sp;
      sp.radius = 1.0f;
      sp.color = {0.3f, 0.4f, 0.5f, 1.0f};
      sc.spheres.push_back(sp);
      return sc;
    };
    ObjectSpaceEdgeOptions opt;
    opt.enable = true;
    opt.width = 0.2f;
    opt.raise = 0.15f;

    // (R1) the render()-integrated pass equals generating first, then rendering.
    {
      Scene pre = makeScene();
      umbreon::generateObjectSpaceEdges(pre, opt);
      RenderOptions roptNo;
      roptNo.width = 32;
      roptNo.height = 32;
      FrameResult fPre = umbreon::render(pre, roptNo);
      RenderOptions roptB = roptNo;
      roptB.objectSpaceEdges = opt;
      FrameResult fInt = umbreon::render(makeScene(), roptB);
      bool identical = !fPre.color.empty() && fPre.color == fInt.color;
      s.check("render(): integrated pass == pre-generate path", identical);
      FrameResult fPlain = umbreon::render(makeScene(), roptNo);
      s.check("render(): edges change the image", fPlain.color != fInt.color);
    }

    // (R2) render() leaves the caller's scene untouched.
    {
      Scene base = makeScene();
      RenderOptions roptB;
      roptB.width = 16;
      roptB.height = 16;
      roptB.objectSpaceEdges = opt;
      (void)umbreon::render(base, roptB);
      s.check("render(): caller scene not mutated",
              base.cylinders.empty() && base.edgeArcs.empty());
    }

    // (R3) both edge methods at once is an error.
    {
      RenderOptions both;
      both.width = 8;
      both.height = 8;
      both.strokeEdges.enable = true;
      both.objectSpaceEdges.enable = true;
      bool threw = false;
      try {
        (void)umbreon::render(makeScene(), both);
      } catch (const std::runtime_error&) {
        threw = true;
      }
      s.check("render(): stroke + object edges both on => throws", threw);
    }

    // White flat sphere (r 0.5, section 0) in front of a white flat plate; the
    // black ring's pixel sits on the +x side, over the plate.
    constexpr int kW = 128;
    const float lineR = 0.5f + 0.025f;  // sphere radius + raise
    const int ringCol = static_cast<int>(0.5f * kW + lineR * kW / 4.0f);
    auto ringDarkness = [&](const FrameResult& f) {
      float darkest = 1.0f;
      for (int x = ringCol - 3; x <= ringCol + 3; ++x) {
        const std::size_t i = (static_cast<std::size_t>(kW / 2) * kW + x) * 4;
        darkest = std::fmin(darkest, f.color[i + 1]);
      }
      return darkest;  // ~0 where the black line covers a pixel
    };
    auto plateScene = [&](uint16_t plateGroup, umbreon::SilhouetteMode mode) {
      Scene sc;
      sc.camera = orthoZ();
      sc.background = {1, 1, 1};
      Sphere sp;
      sp.center = {0, 0, 1};
      sp.radius = 0.5f;
      sp.color = {1, 1, 1, 1};
      sp.material = flatWhite();
      sc.spheres.push_back(sp);
      sc.mesh.positions = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                           {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      sc.mesh.normals.assign(6, Vec3{0, 0, 1});
      sc.mesh.colors.assign(6, Vec4{1, 1, 1, 1});
      sc.mesh.material = flatWhite();
      sc.mesh.triGroupId.assign(2, plateGroup);
      sc.groupObjectEdgeStyle.resize(2);
      for (ObjectEdgeStyle& st : sc.groupObjectEdgeStyle) {
        st.width = 0.05f;
        st.raise = 0.025f;
        st.border = false;
      }
      sc.groupObjectEdgeStyle[0].mode = mode;
      return sc;
    };
    RenderOptions ro;
    ro.width = kW;
    ro.height = kW;
    ro.objectSpaceEdges.enable = true;

    // (R4) silhouette (Outline) mode: a line is dropped only over a surface of
    // its own section; another section behind it keeps the line.
    {
      using umbreon::SilhouetteMode;
      const float full = ringDarkness(umbreon::render(
          plateScene(0, SilhouetteMode::Full), ro));
      const float own = ringDarkness(umbreon::render(
          plateScene(0, SilhouetteMode::Outline), ro));
      const float other = ringDarkness(umbreon::render(
          plateScene(1, SilhouetteMode::Outline), ro));
      s.check("outline: full mode draws the ring over the plate", full < 0.2f);
      s.check("outline: the ring over its own section's plate is dropped",
              own > 0.8f);
      s.check("outline: the ring over another section's plate stays",
              other < 0.2f);
    }

    // (R5) the near clip plane keeps or drops a line by its center: the ring
    // (center view-z 9) is drawn whole when the plane cuts only the front of
    // its tube, and dropped whole when the plane is just behind its center.
    {
      Scene sc = plateScene(1, umbreon::SilhouetteMode::Full);
      sc.clipNear = 9.0f - 0.02f;  // cuts the front of the tube (radius 0.05)
      const float kept = ringDarkness(umbreon::render(sc, ro));
      sc.clipNear = 9.0f + 0.02f;  // the line center is in front of the plane
      const float dropped = ringDarkness(umbreon::render(sc, ro));
      s.check("near clip: a line whose center is behind the plane is whole",
              kept < 0.2f);
      s.check("near clip: a line whose center is in front of the plane is gone",
              dropped > 0.8f);
    }
  }

  return s.report();
}
