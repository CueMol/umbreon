// Silhouette edge cylinder (POV edge_line / edge_line2) integration tests.
// Split out of the monolithic test_render.cpp (same assertions, relocated).
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "render_test_util.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"

int main() {
  umbreon::test::Suite s("render_edges");
  const umbreon::Vec4 pigment{0.5f, 0.6f, 0.7f, 1.0f};

  // ===== Silhouette edge cylinders (POV edge_line / edge_line2) =====
  // POV draws silhouette outlines as a union of `open` cylinders; umbreon
  // renders them as ROUND_LINEAR_CURVE capsules. These tests lock the two
  // properties a prior capless-cylinder rewrite broke: (1) the line is SOLID
  // (the on-axis pixel is fully covered, not stippled/under-covered), and (2)
  // edge_line2's per-endpoint transmit produces a linear opacity fade p0->p1
  // (not a single mean opacity), continuous across joints.
  {
    using umbreon::Vec3;
    using umbreon::Vec4;

    auto cylScene = [&](const umbreon::Cylinder& cyl, Vec3 bg) {
      umbreon::Scene sc;
      sc.camera = makeOrthoCam();          // ortho, frames [-2,2]^2
      sc.background = bg;
      sc.ambientColor = {1, 1, 1};         // flatOutline => raw color shading
      sc.cylinders.push_back(cyl);
      return sc;
    };

    // C1: an opaque black cylinder along x at the view center over a WHITE
    // background. The on-axis center pixel must be (near) black -- a solid,
    // fully-covered line. A thin capless cylinder would under-cover here and
    // leave the pixel gray/white; the capsule covers it solidly.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {2, 0, 0};
      c.radius = 0.5f;                     // ~0.6 px-wide at 5x5; covers center
      c.color = {0, 0, 0, 1.0f};           // opaque black outline
      c.open = true;                       // silhouette edge: ROUND capsule path
      umbreon::Scene sc = cylScene(c, {1, 1, 1});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("C1 solid line: center R black (<0.05)", f.color[kCenterRgba + 0] < 0.05f);
      s.check("C1 solid line: center G black (<0.05)", f.color[kCenterRgba + 1] < 0.05f);
      s.check("C1 solid line: center B black (<0.05)", f.color[kCenterRgba + 2] < 0.05f);
      s.check("C1 solid line: center covered (alpha=1)",
              approx(f.color[kCenterRgba + 3], 1.0f, 1e-4f));
    }

    // C2: edge_line2 gradient. Same geometry, transparent black with opacity 1
    // at p0 (left) fading to 0 at p1 (right). The ray pierces BOTH cylinder
    // walls (front + back at the same x, hence the same axial u and opacity a),
    // composited "over" like POV's open-cylinder transmit, so the black-over-
    // white brightness is (1-a)^2 with a = 1 - u, i.e. u^2 -- a monotone ramp
    // left (dark) to right (bright). Asserts a real lerp, not a single mean.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {2, 0, 0};
      c.radius = 0.5f;
      c.color = {0, 0, 0, 1.0f};           // opacity at p0 = 1 (fully opaque)
      c.opacity1 = 0.0f;                    // opacity at p1 = 0 (fully transmit)
      c.open = true;                       // edge_line2: ROUND capsule path
      umbreon::Scene sc = cylScene(c, {1, 1, 1});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparency = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      // Center row pixels (py=2): the on-axis ray at each px hits the cylinder.
      const std::size_t lft = (2 * 5 + 1) * 4;  // px=1, u=0.3 -> bright u^2=0.09
      const std::size_t ctr = (2 * 5 + 2) * 4;  // px=2, u=0.5 -> bright u^2=0.25
      const std::size_t rgt = (2 * 5 + 3) * 4;  // px=3, u=0.7 -> bright u^2=0.49
      s.check("C2 gradient: left darker than center",
              f.color[lft + 0] + 0.05f < f.color[ctr + 0]);
      s.check("C2 gradient: center darker than right",
              f.color[ctr + 0] + 0.05f < f.color[rgt + 0]);
      // Exact two-wall transmit value at center (u=0.5): (1-a)^2 = u^2 = 0.25.
      s.check("C2 gradient: center brightness = u^2 (0.25)",
              approx(f.color[ctr + 0], 0.25f, 0.02f));
    }

    // C3: a uniform transparent cylinder (opacity1 < 0) uses color.w everywhere,
    // so the fade of C2 is NOT applied: every covered pixel has the same opacity
    // 0.5 on both walls => brightness (1-0.5)^2 = 0.25 over white, constant along
    // the line. Guards that the lerp only triggers for edge_line2 (opacity1 >= 0)
    // and never perturbs a plain edge_line's uniform opacity.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {2, 0, 0};
      c.radius = 0.5f;
      c.color = {0, 0, 0, 0.5f};           // uniform opacity 0.5
      c.opacity1 = -1.0f;                   // no gradient
      c.open = true;                       // edge_line: ROUND capsule path
      umbreon::Scene sc = cylScene(c, {1, 1, 1});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparency = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const std::size_t lft = (2 * 5 + 1) * 4;
      const std::size_t ctr = (2 * 5 + 2) * 4;
      const std::size_t rgt = (2 * 5 + 3) * 4;
      s.check("C3 uniform: center brightness 0.25 (two walls)",
              approx(f.color[ctr + 0], 0.25f, 0.02f));
      s.check("C3 uniform: left == center (no fade)",
              approx(f.color[lft + 0], f.color[ctr + 0], 0.02f));
      s.check("C3 uniform: right == center (no fade)",
              approx(f.color[rgt + 0], f.color[ctr + 0], 0.02f));
    }

    // C4: SEAM GUARD. Two collinear transparent cylinders share the joint at
    // the origin. Rendered as independent capsules, each adds a hemispherical
    // end cap at the joint, so a ray through the joint pierces BOTH caps and the
    // extra transparent black layers darken it -- a dark bead at every segment
    // joint (the POV-vs-umbreon seam). The renderer instead stitches segments
    // that share an endpoint into one connected ROUND_LINEAR_CURVE with
    // RTC_CURVE_FLAG_NEIGHBOR_* so the internal caps are dropped and the joint
    // is a single shared swept-sphere: two walls, brightness (1-0.5)^2 = 0.25,
    // identical to a mid-segment pixel. Asserts the joint is NOT darker than the
    // mid-segments (would fail if the chaining/neighbor-flags were removed).
    {
      umbreon::Cylinder a;
      a.p0 = {-2, 0, 0};
      a.p1 = {0, 0, 0};
      a.radius = 0.5f;
      a.color = {0, 0, 0, 0.5f};             // uniform transparent black
      a.open = true;                         // silhouette edges: chain at joint
      umbreon::Cylinder b = a;               // inherits open=true
      b.p0 = {0, 0, 0};
      b.p1 = {2, 0, 0};                       // shares the joint at the origin
      umbreon::Scene sc = cylScene(a, {1, 1, 1});
      sc.cylinders.push_back(b);
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparency = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const std::size_t lft = (2 * 5 + 1) * 4;  // mid-segment A (x=-0.8)
      const std::size_t ctr = (2 * 5 + 2) * 4;  // the joint (x=0)
      const std::size_t rgt = (2 * 5 + 3) * 4;  // mid-segment B (x=+0.8)
      s.check("C4 seam: joint brightness ~0.25 (single shared sphere)",
              approx(f.color[ctr + 0], 0.25f, 0.05f));
      s.check("C4 seam: joint not darker than mid-segment A",
              f.color[ctr + 0] + 0.03f >= f.color[lft + 0]);
      s.check("C4 seam: joint not darker than mid-segment B",
              f.color[ctr + 0] + 0.03f >= f.color[rgt + 0]);
    }

    // C5: CAP GUARD. POV stick bonds are CLOSED cylinders (open=false) with FLAT
    // disk caps at the exact endpoints; consecutive overlapping bonds must not
    // show a protruding cap. A ROUND_LINEAR_CURVE capsule (open=true) instead
    // bulges a hemisphere ~radius PAST the endpoint, which is what produced the
    // spurious colored arc through an overlapping transparent surface. This test
    // renders the SAME cylinder both ways over a white background and asserts
    // that just BEYOND the p1 endpoint the capped (CONE) cylinder leaves the
    // pixel uncovered (background) while the open (round) one bulges over it.
    // Locks the arc fix: a regression to round caps for bonds would re-cover it.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {0, 0, 0};                       // ends at the view center (x=0)
      c.radius = 0.45f;                       // round cap would reach x=+0.45
      c.color = {0, 0, 0, 1.0f};              // opaque black
      umbreon::RenderOptions o; o.width = 11; o.height = 11;
      // 11-wide ortho over [-2,2]: pixel centers x_i = -2 + (i+0.5)*4/11.
      // i=4 -> x=-0.36 (mid-body, covered both ways); i=6 -> x=+0.36 (beyond p1:
      // inside a round cap's x<=0.45 bulge, outside a flat cap that stops at x=0).
      const std::size_t row = 5;              // center row (py=5, y=0)
      const std::size_t body = (row * 11 + 4) * 4;   // mid-body probe
      const std::size_t past = (row * 11 + 6) * 4;   // just past the endpoint

      c.open = false;                         // capped bond: CONE flat caps
      umbreon::FrameResult fc = umbreon::render(cylScene(c, {1, 1, 1}), o);
      c.open = true;                          // silhouette edge: ROUND hemicap
      umbreon::FrameResult fo = umbreon::render(cylScene(c, {1, 1, 1}), o);

      // Sanity: the body is covered (near black) in BOTH cap modes.
      s.check("C5 cap: capped body covered (R<0.1)", fc.color[body + 0] < 0.1f);
      s.check("C5 cap: open body covered (R<0.1)", fo.color[body + 0] < 0.1f);
      // Discriminator: beyond p1 the flat cap leaves background (near white),
      // while the round cap bulges over it (notably darker).
      s.check("C5 cap: capped leaves background past endpoint (R>0.8)",
              fc.color[past + 0] > 0.8f);
      s.check("C5 cap: round bulges past endpoint (darker than capped)",
              fo.color[past + 0] + 0.2f < fc.color[past + 0]);
    }
  }

  // C6: FAR-SCENE SURFACE-SKIP GUARD. The front-to-back transparency walk steps
  // just past each hit to find the next surface. That step must clear only
  // floating-point jitter, never a whole distinct primitive. A relative step of
  // t*1e-5 was too coarse at a far camera (large t): an opaque surface sitting
  // just behind a transparent one (within ~t*1e-5) was stepped over, so a DEEPER
  // object showed through it (in the real scene an atom sphere appeared through a
  // bond cylinder). Three surfaces along the center ray at t~200: a transparent
  // white quad (z=0), then an opaque BLUE sphere whose near surface is 0.001
  // behind it (z=-0.001), then a deeper opaque RED sphere (z=-0.5). The blue
  // sphere is nearest and must occlude the red one; stepping over its near
  // surface would wrongly reveal red. (A two-surface test would not catch this:
  // skipping a lone sphere's near surface still leaves its far surface to
  // occlude, so a third, deeper, distinctly-colored object is required.)
  {
    using umbreon::Vec3;
    using umbreon::Vec4;
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();
    sc.camera.position = {0, 0, 200};        // far camera => large t (~200)
    sc.background = {0, 0, 0};
    sc.ambientColor = {1, 1, 1};
    sc.assumedGamma = 1.0f;                   // raw values (no gamma encode)
    sc.mesh = makeQuad(Vec4{1, 1, 1, 0.5f});  // transparent quad at z=0
    sc.mesh.material = umbreon::Material::flatOutline();  // raw white * 0.5
    umbreon::Sphere a;                         // nearest opaque: BLUE
    a.center = {0, 0, -5.001f};                // near surface at z=-0.001
    a.radius = 5.0f;
    a.color = Vec4{0, 0, 1, 1};
    a.material = umbreon::Material::flatOutline();
    sc.spheres.push_back(a);
    umbreon::Sphere b;                         // deeper opaque: RED
    b.center = {0, 0, -1.0f};                  // near surface at z=-0.5
    b.radius = 0.5f;
    b.color = Vec4{1, 0, 0, 1};
    b.material = umbreon::Material::flatOutline();
    sc.spheres.push_back(b);
    umbreon::RenderOptions o;
    o.width = 5;
    o.height = 5;
    o.transparency = true;
    umbreon::FrameResult f = umbreon::render(sc, o);
    // fix: quad(0.5 white) over BLUE a => (0.5, 0.5, 1.0). bug: a's near surface
    // is stepped over => quad over RED b => (1.0, 0.5, 0.5). blue-vs-red tells.
    s.check("C6 far skip: nearest opaque (just behind transparent) occludes deeper",
            f.color[kCenterRgba + 2] > f.color[kCenterRgba + 0] + 0.3f);
  }

  // ===== S1: per-group SilhouetteMode (Full vs Outline), stroke edges =====
  // Two spheres of ONE group overlap in screen space at different depths. Full
  // mode inks the front sphere's rim ACROSS the back sphere (same-group
  // self-occlusion); Outline mode suppresses that interior arc and keeps only
  // the group union's outer contour. This locks the EdgeStyle::silhouetteMode
  // -> Scene::groupEdgeStyle -> screen-vector classification wiring end to
  // end. Probes take the min over small windows, not single pixels, because
  // stroke width / Chaikin smoothing shift exact line positions.
  {
    auto edgeScene = [&](umbreon::SilhouetteMode mode) {
      umbreon::Scene sc;
      sc.camera = makeOrthoCam();  // ortho, frames [-2,2]^2
      sc.background = {1, 1, 1};
      umbreon::Sphere a;  // front
      a.center = {-0.5f, 0, 0};
      a.radius = 1.0f;
      a.color = pigment;
      a.group = 1;
      sc.spheres.push_back(a);
      umbreon::Sphere b;  // behind, same group, screen-overlapping
      b.center = {0.7f, 0, -3.0f};
      b.radius = 1.0f;
      b.color = pigment;
      b.group = 1;
      sc.spheres.push_back(b);
      umbreon::EdgeStyle es;
      umbreon::EdgeClassStyle& sil =
          es.cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
      sil.enabled = true;  // black, opacity 1 (defaults), width 2
      sil.width = 2.0f;
      es.silhouetteMode = mode;
      sc.groupEdgeStyle.assign(2, umbreon::EdgeStyle{});
      sc.groupEdgeStyle[1] = es;
      return sc;
    };
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.strokeEdges.enable = true;
    o.strokeEdges.edgesOnly = true;  // full-opacity lines over blank bg
    // Min brightness (R) over a (2r+1)^2 window centered at (cx, cy).
    auto minR = [](const umbreon::FrameResult& f, int cx, int cy, int r) {
      float m = 1.0f;
      for (int y = cy - r; y <= cy + r; ++y)
        for (int x = cx - r; x <= cx + r; ++x)
          m = std::min(m, f.color[(static_cast<std::size_t>(y) * 64 + x) * 4]);
      return m;
    };
    // World (0.5, 0) -- the front rim over the back sphere -- maps to pixel
    // (40, 32); world (-1.5, 0) -- the union's outer rim -- to (8, 32).
    const umbreon::FrameResult ff =
        umbreon::render(edgeScene(umbreon::SilhouetteMode::Full), o);
    s.check("S1 Full: interior same-group arc inked",
            minR(ff, 40, 32, 3) < 0.5f);
    s.check("S1 Full: outer rim inked", minR(ff, 8, 32, 3) < 0.5f);
    const umbreon::FrameResult fo =
        umbreon::render(edgeScene(umbreon::SilhouetteMode::Outline), o);
    s.check("S1 Outline: interior same-group arc suppressed",
            minR(fo, 40, 32, 3) > 0.9f);
    s.check("S1 Outline: outer rim still inked", minR(fo, 8, 32, 3) < 0.5f);
  }

  // ===== S2: Outline contour survives another section BEHIND the group =====
  // An Outline-mode sphere (group 1, only the sil slot enabled -- the shipped
  // `--edge ID=sil:mode=outline` setup, obj slot disabled) in front of a
  // half-plane mesh of the default group 0 (all slots disabled). The sphere/
  // mesh boundary is a cross-section crack; owner-side outline promotion must
  // classify it as Silhouette so the sphere's outer contour is complete even
  // where the mesh, not the background, is behind it.
  {
    using umbreon::Vec3;
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();  // ortho, frames [-2,2]^2
    sc.background = {1, 1, 1};
    umbreon::Sphere a;
    a.center = {0, 0, 0};
    a.radius = 1.0f;
    a.color = pigment;
    a.group = 1;
    sc.spheres.push_back(a);
    // Half quad covering x in [-2,0], y in [-2,2] at z=-2: BEHIND the sphere,
    // under its left rim only. Group 0 (empty triGroupId), styles disabled.
    {
      const Vec3 p00{-2, -2, -2}, p10{0, -2, -2}, p11{0, 2, -2}, p01{-2, 2, -2};
      const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
      for (const Vec3& p : corners) {
        sc.mesh.positions.push_back(p);
        sc.mesh.normals.push_back({0, 0, 1});
        sc.mesh.colors.push_back(pigment);
      }
    }
    umbreon::EdgeStyle es;
    umbreon::EdgeClassStyle& sil =
        es.cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
    sil.enabled = true;  // black, opacity 1 (defaults), width 2
    sil.width = 2.0f;
    es.silhouetteMode = umbreon::SilhouetteMode::Outline;
    sc.groupEdgeStyle.assign(2, umbreon::EdgeStyle{});
    sc.groupEdgeStyle[1] = es;
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.strokeEdges.enable = true;
    o.strokeEdges.edgesOnly = true;
    auto minR = [](const umbreon::FrameResult& f, int cx, int cy, int r) {
      float m = 1.0f;
      for (int y = cy - r; y <= cy + r; ++y)
        for (int x = cx - r; x <= cx + r; ++x)
          m = std::min(m, f.color[(static_cast<std::size_t>(y) * 64 + x) * 4]);
      return m;
    };
    const umbreon::FrameResult f = umbreon::render(sc, o);
    // World (-1,0) -> pixel (16,32): the left rim, mesh behind. This is the
    // regression probe -- without promotion the ObjectId run resolves against
    // the disabled obj slot and drops.
    s.check("S2 Outline: rim over the mesh behind is inked",
            minR(f, 16, 32, 3) < 0.5f);
    // World (1,0) -> pixel (48,32): the right rim over background (control).
    s.check("S2 Outline: rim over background is inked", minR(f, 48, 32, 3) < 0.5f);
    // World (0,1.6) -> pixel (32,6): the quad's own edge against background;
    // group 0's styles are all disabled, so nothing may ink there.
    s.check("S2 Outline: disabled group 0 draws nothing",
            minR(f, 32, 6, 3) > 0.9f);
  }

  // ===== S3: contact lines (strokeEdges.contact) on an intersection =====
  // An Outline-mode sphere (group 1, sil slot only) pierced at its equator by
  // the full-frame quad mesh (group 0, all slots disabled, z=0): the sphere/
  // quad screen boundary is the circle x^2+y^2=1 where the two surfaces meet
  // at CONTINUOUS depth -- the contact veto suppresses it by default, and
  // strokeEdges.contact inks it as the Outline side's Silhouette (the
  // deterministic contact owner), closing the group's contour where it
  // plunges into the mesh.
  {
    using umbreon::Vec3;
    auto contactScene = [&]() {
      umbreon::Scene sc;
      sc.camera = makeOrthoCam();  // ortho, frames [-2,2]^2
      sc.background = {1, 1, 1};
      umbreon::Sphere a;
      a.center = {0, 0, 0};
      a.radius = 1.0f;
      a.color = pigment;
      a.group = 1;
      sc.spheres.push_back(a);
      sc.mesh = makeQuad(pigment);  // full [-2,2]^2 quad at z=0, group 0
      umbreon::EdgeStyle es;
      umbreon::EdgeClassStyle& sil =
          es.cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
      sil.enabled = true;  // black, opacity 1 (defaults), width 2
      sil.width = 2.0f;
      es.silhouetteMode = umbreon::SilhouetteMode::Outline;
      sc.groupEdgeStyle.assign(2, umbreon::EdgeStyle{});
      sc.groupEdgeStyle[1] = es;
      return sc;
    };
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.strokeEdges.enable = true;
    o.strokeEdges.edgesOnly = true;
    auto minR = [](const umbreon::FrameResult& f, int cx, int cy, int r) {
      float m = 1.0f;
      for (int y = cy - r; y <= cy + r; ++y)
        for (int x = cx - r; x <= cx + r; ++x)
          m = std::min(m, f.color[(static_cast<std::size_t>(y) * 64 + x) * 4]);
      return m;
    };
    // Default (contact off): the intersection circle stays silent.
    const umbreon::FrameResult off = umbreon::render(contactScene(), o);
    s.check("S3 contact off: circle top silent", minR(off, 32, 16, 3) > 0.9f);
    s.check("S3 contact off: circle left silent", minR(off, 16, 32, 3) > 0.9f);
    // Contact on: the circle inks (world (0,1) -> px (32,16), world (-1,0) ->
    // px (16,32)) while the interior stays clean.
    o.strokeEdges.contact = true;
    const umbreon::FrameResult on = umbreon::render(contactScene(), o);
    s.check("S3 contact on: circle top inked", minR(on, 32, 16, 3) < 0.5f);
    s.check("S3 contact on: circle left inked", minR(on, 16, 32, 3) < 0.5f);
    s.check("S3 contact on: interior stays clean", minR(on, 32, 32, 2) > 0.9f);
  }

  // ===== S4: the outside-aligned band stops at a nearer surface =====
  // A far cylinder B (same group) stands 2 px right of the near sphere A's
  // rim with only background between them. B's silhouette band (6 px, laid
  // on the background side by the outside alignment) is wider than the gap
  // and must end at A's rim instead of running onto A: before the
  // OuterRoomShader it painted a 4 px bite over the nearer sphere -- black,
  // or with depth fog the far line's fog white. World (0.4, 0), A's rim, is
  // px 38.4; B's rim (0.525, 0) is px 40.4. A is flat-shaded (flatOutline
  // material), so every pixel of A must equal A's center color.
  {
    auto roomScene = [&](bool fog) {
      umbreon::Scene sc;
      sc.camera = makeOrthoCam();  // ortho, frames [-2,2]^2; camera at z=10
      sc.background = {1, 1, 1};
      umbreon::Sphere a;  // near, group 1
      a.center = {-0.6f, 0, 0};
      a.radius = 1.0f;
      a.color = pigment;
      a.group = 1;
      sc.spheres.push_back(a);
      umbreon::Cylinder b;  // far, same group, 2 px of background from A
      b.p0 = {1.125f, -2.5f, -3.0f};
      b.p1 = {1.125f, 2.5f, -3.0f};
      b.radius = 0.6f;
      b.color = {0.9f, 0.9f, 0.3f, 1.0f};
      b.group = 1;
      sc.cylinders.push_back(b);
      if (fog) {  // A (view-z <= 10) unfogged, B (view-z 13) fully white
        sc.fog.enabled = true;
        sc.fog.color = {1, 1, 1};
        sc.fog.start = 10.5f;
        sc.fog.end = 12.0f;
      }
      umbreon::EdgeStyle es;
      umbreon::EdgeClassStyle& sil =
          es.cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
      sil.enabled = true;  // black, opacity 1 (defaults)
      sil.width = 6.0f;    // wider than the 2 px gap
      sc.groupEdgeStyle.assign(2, umbreon::EdgeStyle{});
      sc.groupEdgeStyle[1] = es;
      return sc;
    };
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.strokeEdges.enable = true;
    auto px = [](const umbreon::FrameResult& f, int x, int y, int c) {
      return f.color[(static_cast<std::size_t>(y) * 64 + x) * 4 + c];
    };
    // Every pixel of the window matches A's flat color (sampled at A's
    // center, px (22, 32)): no ink of any color over A's rim interior.
    auto flatLikeA = [&](const umbreon::FrameResult& f, int x0, int x1,
                         int y0, int y1) {
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
          for (int c = 0; c < 3; ++c)
            if (std::fabs(px(f, x, y, c) - px(f, 22, 32, c)) > 0.05f)
              return false;
      return true;
    };
    auto minR = [&](const umbreon::FrameResult& f, int x0, int x1, int y0,
                    int y1) {
      float m = 1.0f;
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) m = std::min(m, px(f, x, y, 0));
      return m;
    };
    // The probe window is 2-3 px inside A's rim: B's unclamped band reached
    // px 34.9, A's own band (pad 0.5 px inside the rim) starts near 37.9.
    const umbreon::FrameResult nf = umbreon::render(roomScene(false), o);
    s.check("S4 room: A's color is a mid tone (probe is meaningful)",
            px(nf, 22, 32, 0) > 0.1f && px(nf, 22, 32, 0) < 0.9f);
    s.check("S4 room: the gap between A and B is inked",
            minR(nf, 39, 40, 30, 34) < 0.5f);
    s.check("S4 room: no black band over A's rim interior",
            flatLikeA(nf, 35, 36, 30, 34));
    const umbreon::FrameResult ff = umbreon::render(roomScene(true), o);
    s.check("S4 room: no fog-white band over A's rim interior",
            flatLikeA(ff, 35, 36, 30, 34));
    s.check("S4 room: A's own rim is still outlined under fog",
            minR(ff, 38, 43, 30, 34) < 0.5f);
  }

  // ===== S5: junction weaving does not join lines at different depths =====
  // A near capsule C (cylinder + end sphere, one group) over a far cylinder
  // D, 12 units deeper, whose top silhouette runs 0.05 above C's bottom edge:
  // C covers D's edge under the stick, and where the sphere's arc meets D's
  // exposed edge three cracks join -- the sphere's arc (Silhouette, near),
  // the arc's continuation over D into the stick's bottom edge (DepthGap,
  // near) and D's top edge against the background (Silhouette, far). In 2D
  // the far edge continues the near contour almost straight while the arc
  // arrives curving, so straightness alone wove far + near into one bar: the
  // bar was split back into two runs at the corner and the re-centering taper
  // at that split painted a bite of ink INTO the sphere, while the sphere's
  // own arc, demoted to a stem, was clipped along the near-parallel bar. The
  // depth gate rejects the far + near pair, so the near contour weaves with
  // itself and the far edge becomes the stem. Frame: ortho [-2,2]^2 over
  // 128 px (32 px per unit), camera at z = 10; the sphere is flat-shaded.
  {
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();
    sc.background = {1, 1, 1};
    umbreon::Cylinder c;  // near stick, group 1
    c.p0 = {-3.0f, 0.0f, 0.0f};
    c.p1 = {0.0f, 0.0f, 0.0f};
    c.radius = 1.0f;
    c.color = pigment;
    c.group = 1;
    sc.cylinders.push_back(c);
    umbreon::Sphere cap;  // its rounded end, center px (64, 64), r 32 px
    cap.center = {0.0f, 0.0f, 0.0f};
    cap.radius = 1.0f;
    cap.color = pigment;
    cap.group = 1;
    sc.spheres.push_back(cap);
    umbreon::Cylinder d;  // far stick: top edge y = -0.95, view-z 22
    d.p0 = {-0.5f, -1.95f, -12.0f};
    d.p1 = {3.0f, -1.95f, -12.0f};
    d.radius = 1.0f;
    d.color = {0.9f, 0.9f, 0.3f, 1.0f};
    d.group = 1;
    sc.cylinders.push_back(d);
    umbreon::EdgeStyle es;
    umbreon::EdgeClassStyle& sil =
        es.cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
    sil.enabled = true;
    sil.width = 6.0f;
    sc.groupEdgeStyle.assign(2, umbreon::EdgeStyle{});
    sc.groupEdgeStyle[1] = es;
    umbreon::RenderOptions o;
    o.width = 128;
    o.height = 128;
    o.strokeEdges.enable = true;
    const umbreon::FrameResult f = umbreon::render(sc, o);
    auto px = [&](int x, int y, int c2) {
      return f.color[(static_cast<std::size_t>(y) * 128 + x) * 4 + c2];
    };
    auto minR = [&](int x0, int x1, int y0, int y1) {
      float m = 1.0f;
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) m = std::min(m, px(x, y, 0));
      return m;
    };
    // Every pixel of the window matches the sphere's flat color at its
    // center (64, 64): no ink inside the sphere.
    auto flatLikeSphere = [&](int x0, int x1, int y0, int y1) {
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
          for (int c2 = 0; c2 < 3; ++c2)
            if (std::fabs(px(x, y, c2) - px(64, 64, c2)) > 0.05f) return false;
      return true;
    };
    // The junction sits at px (75, 93); the woven far + near bar's taper
    // used to paint the rows just above it, inside the sphere.
    s.check("S5 weave: no bite of ink inside the sphere at the junction",
            flatLikeSphere(70, 79, 88, 90));
    // The far edge (a stem now) is still outlined, right of the sphere.
    s.check("S5 weave: far edge stem still outlined",
            minR(108, 118, 89, 93) < 0.5f);
    // The near stick's bottom edge stays outlined left of the sphere.
    s.check("S5 weave: stick bottom edge outlined",
            minR(20, 26, 97, 99) < 0.5f);
  }

  // ===== S6: a sphere in front of a bond of its own section keeps its rim =====
  // Full mode inks every same-section self-occlusion. A sphere over another
  // SPHERE already did (S1: same primitive kind, the same-id branch marks the
  // step strong); a sphere over a BOND -- the mixed-kind branch -- classified
  // as a weak DepthGap that the prune dropped wherever no strong neighbor
  // supported it, so the rim vanished exactly over the bond. Sphere A at the
  // origin, cylinder B of the same group 6 units behind it crossing the view
  // below A's center; A's lower rim over B (world (0, -0.7), px (32, 43))
  // must be inked, as its upper rim over the background is.
  {
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();  // ortho, frames [-2,2]^2
    sc.background = {1, 1, 1};
    umbreon::Sphere a;
    a.center = {0.0f, 0.0f, 0.0f};
    a.radius = 0.7f;
    a.color = pigment;
    a.group = 1;
    sc.spheres.push_back(a);
    umbreon::Cylinder b;  // same section, 6 units behind, closed bond
    b.p0 = {-3.0f, -1.0f, -6.0f};
    b.p1 = {3.0f, -1.0f, -6.0f};
    b.radius = 0.6f;
    b.color = {0.9f, 0.9f, 0.3f, 1.0f};
    b.group = 1;
    sc.cylinders.push_back(b);
    umbreon::EdgeStyle es;
    umbreon::EdgeClassStyle& sil =
        es.cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
    sil.enabled = true;
    sil.width = 2.0f;
    sc.groupEdgeStyle.assign(2, umbreon::EdgeStyle{});
    sc.groupEdgeStyle[1] = es;
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.strokeEdges.enable = true;
    o.strokeEdges.edgesOnly = true;
    const umbreon::FrameResult f = umbreon::render(sc, o);
    auto minR = [&](int cx, int cy, int r) {
      float m = 1.0f;
      for (int y = cy - r; y <= cy + r; ++y)
        for (int x = cx - r; x <= cx + r; ++x)
          m = std::min(m, f.color[(static_cast<std::size_t>(y) * 64 + x) * 4]);
      return m;
    };
    s.check("S6 rim over bond: upper rim over background inked",
            minR(32, 21, 2) < 0.5f);
    s.check("S6 rim over bond: lower rim over the bond inked",
            minR(32, 43, 2) < 0.5f);
  }

  return s.report();
}
