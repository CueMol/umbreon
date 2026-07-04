// Silhouette edge cylinder (POV edge_line / edge_line2) integration tests.
// Split out of the monolithic test_render.cpp (same assertions, relocated).
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


  return s.report();
}
