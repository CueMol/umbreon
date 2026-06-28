// Unit / integration tests for the ANALYTIC silhouette path of the Freestyle
// STROKE edge pass (render/analytic_silhouette.{hpp,cpp} + the merge in
// render/stroke_edges.cpp applyStrokeEdges).
//
// Guards the contract that makes `--edges` outline ball-and-stick (spheres /
// cylinders), not just the triangle mesh:
//   UNIT (analytic_silhouette):
//     * appendAnalyticFeatureSegs turns a sphere into a CLOSED ring of Silhouette
//       FeatureSegs with fresh node ids from nodeBase, no incident mesh faces,
//       zero nrm, carrying the source group; a cylinder into its two side
//       generators (standalone open segments);
//     * those segs chain into one closed loop (sphere) / standalone chains (cyl);
//     * `circumscribe` scales the sphere ring so the chord polygon stays OUTSIDE
//       the contour (vertices at r/cos(pi/N), chord midpoints on r) -- the anti-
//       self-occlusion fix -- while circumscribe=false is the exact contour;
//     * fromEdgeMacro primitives (baked outlines) are skipped as sources.
//   INTEGRATION (umbreon::render):
//     * a free sphere is outlined by a CONTINUOUS ring (no dashing) with --edges;
//     * the outline is hidden where the sphere is occluded by a NEARER primitive
//       (true cross-primitive QI hidden-line);
//     * with --stroke-analytic OFF the ball-stick gets no outline, yet a mesh
//       (ribbon) edge passing BEHIND a sphere is still correctly HIDDEN -- the
//       analytic silhouette is used as an occluder boundary even when not drawn
//       (regression guard for the ViewEdge-split fix);
//     * the analytic-edge render is deterministic (two renders bit-identical).
#include <cmath>
#include <cstddef>
#include <vector>

#include "edges/analytic_silhouette.hpp"
#include "edges/stroke_edges.hpp"
#include "scene.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

using umbreon::AnalyticLoop;
using umbreon::Camera;
using umbreon::Cylinder;
using umbreon::EdgeChain;
using umbreon::EdgeNature;
using umbreon::FeatureSeg;
using umbreon::FrameResult;
using umbreon::Mesh;
using umbreon::RenderOptions;
using umbreon::Scene;
using umbreon::ScreenProj;
using umbreon::Sphere;
using umbreon::Vec3;
using umbreon::Vec4;

bool nearf(float a, float b, float eps = 1.0e-3f) {
  return std::fabs(a - b) <= eps;
}

// An ortho camera at +z looking down -Z (the standard CueMol framing), so the
// silhouette of a sphere at the origin is its great circle in the z=0 plane.
Camera orthoCam() {
  Camera c;
  c.position = {0, 0, 10};
  c.direction = {0, 0, -1};
  c.up = {0, 1, 0};
  c.orthographic = true;
  c.height = 4.0f;  // world units mapped to the image height
  return c;
}

// Red channel at pixel (x,y); render frames are top-left-origin RGBA.
float lumR(const FrameResult& f, int x, int y) {
  return f.color[(static_cast<std::size_t>(y) * f.width + x) * 4 + 0];
}

bool inBounds(const FrameResult& f, int x, int y) {
  return x >= 0 && y >= 0 && x < f.width && y < f.height;
}

// True iff some pixel within `rad` of (cx,cy) is a dark edge ink (R < thresh).
// Edge color is black (0), the gray surfaces are 0.7 and the background 1.0, so a
// small threshold cleanly separates "an edge is here" from surface/background.
bool darkNear(const FrameResult& f, int cx, int cy, int rad, float thresh = 0.3f) {
  for (int dy = -rad; dy <= rad; ++dy)
    for (int dx = -rad; dx <= rad; ++dx) {
      const int x = cx + dx, y = cy + dy;
      if (inBounds(f, x, y) && lumR(f, x, y) < thresh) return true;
    }
  return false;
}

// A light-gray flat-outline sphere (raw color, no lighting needed).
Sphere graySphere(Vec3 center, float radius, std::uint16_t group = 0) {
  Sphere s;
  s.center = center;
  s.radius = radius;
  s.color = Vec4{0.7f, 0.7f, 0.7f, 1.0f};
  s.group = group;
  return s;
}

// A flat quad (2-triangle soup) in the plane z = `zPlane`, spanning x in [0,xMax]
// and y in [-yHalf, yHalf]. Its LEFT boundary edge runs vertically at x=0 (screen
// center) -- the edge used to test occlusion behind a sphere. flatOutline material
// + gray vertex color => the surface renders as raw 0.7 (no lights required), so
// only the black border edge is "dark".
Mesh quadBehind(float zPlane, float xMax, float yHalf) {
  Mesh m;
  const Vec3 a{0, -yHalf, zPlane}, b{xMax, -yHalf, zPlane}, c{xMax, yHalf, zPlane},
      d{0, yHalf, zPlane};
  m.positions = {a, b, c, a, c, d};       // tri (a,b,c) + tri (a,c,d)
  m.normals.assign(6, Vec3{0, 0, 1});     // faces the camera (flat, no silhouette)
  m.colors.assign(6, Vec4{0.7f, 0.7f, 0.7f, 1.0f});
  m.material = umbreon::Material::flatOutline();
  return m;
}

RenderOptions edgeOpts(int w, int h, bool analytic) {
  RenderOptions o;
  o.width = w;
  o.height = h;
  o.supersample = 1;  // 1:1 pixel mapping so worldToScreen lines up with the frame
  o.strokeEdges.enable = true;
  o.strokeEdges.silhouette = true;
  o.strokeEdges.analytic = analytic;
  return o;
}

}  // namespace

int main() {
  umbreon::test::Suite s("stroke_analytic");

  // ---- (1) appendAnalyticFeatureSegs: a sphere -> a closed Silhouette ring ----
  {
    Scene sc;
    sc.camera = orthoCam();
    sc.spheres.push_back(graySphere({0, 0, 0}, 1.0f, /*group=*/5));
    std::vector<FeatureSeg> segs;
    const int next =
        umbreon::appendAnalyticFeatureSegs(sc, sc.camera, /*N=*/8, /*raise=*/0.0f,
                                           /*nodeBase=*/100, segs);
    s.check_eq("sphere: 8 ring segments emitted", segs.size(),
               static_cast<std::size_t>(8));
    s.check_eq("sphere: node ids consumed (return == base + 8)", next, 108);
    bool idsOk = true, attrOk = true;
    for (int i = 0; i < 8; ++i) {
      const FeatureSeg& f = segs[static_cast<std::size_t>(i)];
      if (f.v0 != 100 + i || f.v1 != 100 + ((i + 1) % 8)) idsOk = false;
      if (f.nature != EdgeNature::Silhouette || f.group != 5 || f.face0 != -1 ||
          f.face1 != -1 || !f.excludeFaces.empty() ||
          !nearf(umbreon::length(f.nrm), 0.0f))
        attrOk = false;
    }
    s.check("sphere: ring node ids form a closed loop from nodeBase", idsOk);
    s.check("sphere: segs are Silhouette / source group / no mesh face / zero nrm",
            attrOk);

    // The ring chains into exactly ONE closed loop of all 8 segments.
    const std::vector<EdgeChain> chains = umbreon::chainFeatureSegs(segs, next);
    s.check_eq("sphere: ring chains into one chain", chains.size(),
               static_cast<std::size_t>(1));
    s.check("sphere: chain is closed and holds all 8 segments",
            !chains.empty() && chains[0].closed && chains[0].segs.size() == 8);
  }

  // ---- (2) appendAnalyticFeatureSegs: a cylinder -> two side generators -------
  {
    Scene sc;
    sc.camera = orthoCam();
    Cylinder cyl;
    cyl.p0 = {-3, 0, 0};
    cyl.p1 = {3, 0, 0};  // axis +x, seen broadside; two side lines along +/- y
    cyl.radius = 0.5f;
    cyl.group = 7;
    sc.cylinders.push_back(cyl);
    std::vector<FeatureSeg> segs;
    const int next = umbreon::appendAnalyticFeatureSegs(sc, sc.camera, 8, 0.0f, 0, segs);
    s.check_eq("cylinder: two side generators emitted", segs.size(),
               static_cast<std::size_t>(2));
    s.check_eq("cylinder: 4 node ids consumed (2 standalone segments)", next, 4);
    s.check("cylinder: side segs are Silhouette / source group / own id pair",
            segs.size() == 2 && segs[0].nature == EdgeNature::Silhouette &&
                segs[0].group == 7 && segs[0].v0 == 0 && segs[0].v1 == 1 &&
                segs[1].v0 == 2 && segs[1].v1 == 3);
    // The two side lines do NOT chain together (disjoint id blocks).
    const std::vector<EdgeChain> chains = umbreon::chainFeatureSegs(segs, next);
    s.check_eq("cylinder: side lines stay two standalone chains", chains.size(),
               static_cast<std::size_t>(2));
  }

  // ---- (3) circumscribe scales the ring out so chords stay on/outside the
  //          surface (anti-self-occlusion); circumscribe=false is the contour ---
  {
    Scene sc;
    sc.camera = orthoCam();
    sc.spheres.push_back(graySphere({0, 0, 0}, 2.0f));
    const int N = 8;
    const float kPi = 3.14159265358979323846f;
    std::vector<AnalyticLoop> exact, circ;
    umbreon::emitAnalyticSilhouettes(sc, sc.camera, N, 0.0f, /*circumscribe=*/false, exact);
    umbreon::emitAnalyticSilhouettes(sc, sc.camera, N, 0.0f, /*circumscribe=*/true, circ);
    s.check("emit: one loop per path",
            exact.size() == 1 && circ.size() == 1 &&
                exact[0].pts.size() == static_cast<std::size_t>(N) &&
                circ[0].pts.size() == static_cast<std::size_t>(N));

    bool exactOnContour = true, circOutside = true, circMidOnContour = true;
    const float rCirc = 2.0f / std::cos(kPi / static_cast<float>(N));
    for (int i = 0; i < N && !exact.empty() && !circ.empty(); ++i) {
      // Exact contour: vertices sit at radius r (the great circle).
      if (!nearf(umbreon::length(exact[0].pts[static_cast<std::size_t>(i)]), 2.0f))
        exactOnContour = false;
      // Circumscribed: vertices sit at r / cos(pi/N) (just outside the surface).
      if (!nearf(umbreon::length(circ[0].pts[static_cast<std::size_t>(i)]), rCirc, 2e-3f))
        circOutside = false;
      // ...and the midpoint of two adjacent circumscribed vertices lands on r
      // (so the QI ray samples the chord at/outside the surface, never inside).
      const Vec3 mid = (circ[0].pts[static_cast<std::size_t>(i)] +
                        circ[0].pts[static_cast<std::size_t>((i + 1) % N)]) *
                       0.5f;
      if (!nearf(umbreon::length(mid), 2.0f, 2e-3f)) circMidOnContour = false;
    }
    s.check("circumscribe off: ring vertices lie ON the contour (radius r)",
            exactOnContour);
    s.check("circumscribe on: ring vertices lie OUTSIDE (radius r/cos(pi/N))",
            circOutside);
    s.check("circumscribe on: chord midpoints lie ON the contour (radius r)",
            circMidOnContour);
  }

  // ---- (4) baked-outline (fromEdgeMacro) primitives are skipped as sources ----
  {
    Scene sc;
    sc.camera = orthoCam();
    Sphere baked = graySphere({0, 0, 0}, 1.0f);
    baked.fromEdgeMacro = true;  // a baked POV outline joint, not a surface
    sc.spheres.push_back(baked);
    std::vector<AnalyticLoop> loops;
    umbreon::emitAnalyticSilhouettes(sc, sc.camera, 8, 0.0f, true, loops);
    s.check("fromEdgeMacro sphere is not silhouetted", loops.empty());
  }

  // ---- (5) INTEGRATION: a free sphere gets a CONTINUOUS outline ring ----------
  {
    const int W = 120, H = 120;
    Scene sc;
    sc.camera = orthoCam();
    sc.background = {1, 1, 1};
    sc.spheres.push_back(graySphere({0, 0, 0}, 1.0f));
    const FrameResult f = umbreon::render(sc, edgeOpts(W, H, /*analytic=*/true));

    // Project the actual silhouette ring vertices and require (almost) every one
    // to land on inked edge -- a dashed ring (self-occlusion) would miss many.
    const ScreenProj sp = umbreon::makeScreenProj(sc.camera, W, H);
    std::vector<AnalyticLoop> loops;
    umbreon::emitAnalyticSilhouettes(sc, sc.camera, 48, 0.0f, true, loops);
    int onRing = 0, total = 0;
    for (const AnalyticLoop& lp : loops)
      for (const Vec3& p : lp.pts) {
        float x, y, vz;
        if (!umbreon::worldToScreen(sp, p, x, y, vz)) continue;
        ++total;
        if (darkNear(f, static_cast<int>(x + 0.5f), static_cast<int>(y + 0.5f), 2))
          ++onRing;
      }
    s.check("free sphere: silhouette ring vertices found", total >= 40);
    s.check("free sphere: outline is continuous (>=90% of ring inked, no dashing)",
            total > 0 && onRing >= (total * 9) / 10);
    // The sphere interior (center) is the gray surface, not an edge.
    s.check("free sphere: center is surface, not edge", !darkNear(f, W / 2, H / 2, 1));
  }

  // ---- (6) INTEGRATION: cross-primitive hidden line --------------------------
  //          A sphere fully occluded by a NEARER, larger sphere has its outline
  //          ring hidden (the QI ray hits the occluder => hidden). Same projected
  //          ring vertices that were inked when free are now NOT inked.
  {
    const int W = 120, H = 120;
    const ScreenProj sp = umbreon::makeScreenProj(orthoCam(), W, H);
    // The test sphere's ring vertices (projected once, reused for both renders).
    Scene probe;
    probe.camera = orthoCam();
    probe.spheres.push_back(graySphere({0, 0, 0}, 1.0f));
    std::vector<AnalyticLoop> loops;
    umbreon::emitAnalyticSilhouettes(probe, probe.camera, 48, 0.0f, true, loops);

    auto countInkedRing = [&](const FrameResult& f) {
      int inked = 0;
      for (const AnalyticLoop& lp : loops)
        for (const Vec3& p : lp.pts) {
          float x, y, vz;
          if (!umbreon::worldToScreen(sp, p, x, y, vz)) continue;
          if (darkNear(f, static_cast<int>(x + 0.5f), static_cast<int>(y + 0.5f), 2))
            ++inked;
        }
      return inked;
    };

    Scene freeS;
    freeS.camera = orthoCam();
    freeS.background = {1, 1, 1};
    freeS.spheres.push_back(graySphere({0, 0, 0}, 1.0f));
    const int inkedFree = countInkedRing(umbreon::render(freeS, edgeOpts(W, H, true)));

    Scene occl = freeS;
    occl.spheres.push_back(graySphere({0, 0, 5}, 3.0f));  // big, NEARER => covers it
    const int inkedOccl = countInkedRing(umbreon::render(occl, edgeOpts(W, H, true)));

    s.check("hidden-line: the free sphere ring is inked", inkedFree >= 40);
    s.check("hidden-line: a sphere behind a nearer sphere has its ring hidden",
            inkedOccl <= inkedFree / 8);
  }

  // ---- (7) INTEGRATION (the fix): mesh edge BEHIND a sphere is hidden even with
  //          --stroke-analytic OFF (analytic silhouette used as occluder boundary,
  //          not drawn). Without this the per-ViewEdge QI majority would leak the
  //          part of the mesh edge that passes behind the sphere. -------------
  {
    const int W = 120, H = 120;
    const ScreenProj sp = umbreon::makeScreenProj(orthoCam(), W, H);
    // The mesh quad's LEFT border runs vertically at world x=0 (screen center).
    // It passes BEHIND the sphere (z=-2 vs the sphere at z=0) for y in ~[-1,1].
    auto buildScene = [&](bool withSphere) {
      Scene sc;
      sc.camera = orthoCam();
      sc.background = {1, 1, 1};
      sc.mesh = quadBehind(/*zPlane=*/-2.0f, /*xMax=*/3.0f, /*yHalf=*/2.5f);
      if (withSphere) sc.spheres.push_back(graySphere({0, 0, 0}, 1.0f));
      return sc;
    };
    // Pixels on the border line: one behind the sphere (center), one above it.
    auto pix = [&](float wy) {
      float x, y, vz;
      umbreon::worldToScreen(sp, Vec3{0.0f, wy, -2.0f}, x, y, vz);
      return std::pair<int, int>(static_cast<int>(x + 0.5f), static_cast<int>(y + 0.5f));
    };
    const auto behind = pix(0.0f);   // screen center: behind the sphere
    const auto above = pix(1.8f);    // above the sphere: never occluded

    // analytic OFF: ball-stick (sphere) is NOT outlined, but still occludes.
    const FrameResult f = umbreon::render(buildScene(true), edgeOpts(W, H, /*analytic=*/false));
    s.check("fix: mesh border ABOVE the sphere is visible (inked)",
            darkNear(f, above.first, above.second, 2));
    s.check("fix: mesh border BEHIND the sphere is HIDDEN (no leak over ball-stick)",
            !darkNear(f, behind.first, behind.second, 2));

    // Control: WITHOUT the sphere the same border point IS inked -> proves the
    // edge exists there and the test above is not vacuous.
    const FrameResult fc = umbreon::render(buildScene(false), edgeOpts(W, H, false));
    s.check("fix control: with no occluder the border point IS inked",
            darkNear(fc, behind.first, behind.second, 2));

    // And the sphere itself draws NO outline ring with analytic off: the ring
    // radius band (~30px from center) carries no ink from the sphere.
    bool ringInked = false;
    for (int a = 0; a < 16; ++a) {
      const float th = 6.2831853f * static_cast<float>(a) / 16.0f;
      const int rx = W / 2 + static_cast<int>(30.0f * std::cos(th) + 0.5f);
      const int ry = H / 2 + static_cast<int>(30.0f * std::sin(th) + 0.5f);
      // skip the x=0 border column (the quad edge), sample only off-axis arcs
      if (std::abs(rx - W / 2) <= 3) continue;
      if (darkNear(f, rx, ry, 1)) ringInked = true;
    }
    s.check("analytic off: the sphere is not outlined", !ringInked);
  }

  // ---- (8) INTEGRATION: analytic-edge render is deterministic -----------------
  {
    const int W = 96, H = 96;
    Scene sc;
    sc.camera = orthoCam();
    sc.background = {1, 1, 1};
    sc.spheres.push_back(graySphere({-0.6f, 0, 0}, 1.0f));
    Cylinder cyl;
    cyl.p0 = {0, 0, 0};
    cyl.p1 = {2.5f, 0, 0};
    cyl.radius = 0.3f;
    sc.cylinders.push_back(cyl);
    const FrameResult a = umbreon::render(sc, edgeOpts(W, H, true));
    const FrameResult b = umbreon::render(sc, edgeOpts(W, H, true));
    bool identical = a.color.size() == b.color.size();
    for (std::size_t i = 0; identical && i < a.color.size(); ++i)
      if (a.color[i] != b.color[i]) identical = false;
    s.check("determinism: two analytic-edge renders are bit-identical", identical);
  }

  return s.report();
}
