// Tests for the view-space clipping planes (Scene::clipNear/clipFar): primary
// rays are clamped to the [clipNear, clipFar] view-z range, and with the
// stroke edge pass on the renderer captures the clip-cut G-buffer (clipCut /
// clipNearVz / clipFarVz) the edge pass uses to keep cut boundaries
// line-free. The material-sphere scene of render_test_util frames a radius-1
// sphere at the origin from an ortho camera at z=10, so the center ray's
// front hit is at view-z 9 and the interior back-wall hit at view-z 11.
#include <cstddef>

#include "render_test_util.hpp"
#include "test_util.hpp"

int main() {
  umbreon::test::Suite s("clip_planes");
  const umbreon::Vec4 red{1.0f, 0.0f, 0.0f, 1.0f};
  const umbreon::Vec3 bg{0.0f, 0.0f, 1.0f};  // blue background
  umbreon::Material mat;
  mat.ambient = 0.2f;
  mat.diffuse = 0.8f;
  auto scene = [&]() { return makeMaterialSphereScene(red, mat, bg); };
  umbreon::RenderOptions o;
  o.width = 5;
  o.height = 5;
  o.supersample = 1;

  // --- no clip: the defaults leave the sphere visible (regression guard).
  {
    umbreon::FrameResult f = umbreon::render(scene(), o);
    s.check("no clip: center hits the sphere (red-ish)",
            f.color[kCenterRgba] > 0.3f && f.color[kCenterRgba + 2] < 0.5f);
  }

  // --- far plane in front of the sphere: everything clipped -> background.
  {
    umbreon::Scene sc = scene();
    sc.clipFar = 8.0f;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("clipFar 8: sphere fully clipped -> background",
            approx(f.color[kCenterRgba + 2], bg.z, 1e-4f) &&
                f.color[kCenterRgba] < 1e-4f);
  }

  // --- near plane behind the sphere: everything clipped -> background.
  {
    umbreon::Scene sc = scene();
    sc.clipNear = 12.0f;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("clipNear 12: sphere fully clipped -> background",
            f.color[kCenterRgba] < 1e-4f);
  }

  // --- near plane through the center: the front hemisphere is clipped and
  // the center ray sees the INTERIOR of the back wall (a backface hit at
  // view-z 11). With edges on, the clip-cut G-buffer marks it (confirmed by
  // the any-hit ray finding the clipped front hemisphere).
  {
    umbreon::Scene sc = scene();
    sc.clipNear = 10.0f;
    umbreon::RenderOptions oe = o;
    oe.strokeEdges.enable = true;
    umbreon::FrameResult f = umbreon::render(sc, oe);
    s.check("clipNear 10: center still hits (interior back wall)",
            f.viewZ[kCenterPix] > 10.5f && f.viewZ[kCenterPix] < 11.5f);
    s.check_eq("clipNear 10: clip-cut interior flagged",
               static_cast<int>(f.clipCut[kCenterPix]), 1);
    // A corner ray misses the sphere entirely: nothing was clipped along it.
    s.check_eq("clipNear 10: miss ray far from the sphere stays unflagged",
               static_cast<int>(f.clipCut[0]), 0);
    s.check("clipNear 10: unclipped miss ray records no removed hit",
            f.clipNearVz[0] == 0.0f && f.clipFarVz[0] == 0.0f);
  }

  // --- far plane in front: the center becomes a BACKGROUND pixel whose ray
  // records the removed front hit (view-z 9) in clipFarVz.
  {
    umbreon::Scene sc = scene();
    sc.clipFar = 8.0f;
    umbreon::RenderOptions oe = o;
    oe.strokeEdges.enable = true;
    umbreon::FrameResult f = umbreon::render(sc, oe);
    s.check("clipFar 8: removed hit recorded beyond the plane",
            approx(f.clipFarVz[kCenterPix], 9.0f, 1e-2f));
    s.check("clipFar 8: nothing recorded in front of a near plane",
            f.clipNearVz[kCenterPix] == 0.0f);
  }

  // --- near plane behind: the removed hit lands in clipNearVz instead.
  {
    umbreon::Scene sc = scene();
    sc.clipNear = 12.0f;
    umbreon::RenderOptions oe = o;
    oe.strokeEdges.enable = true;
    umbreon::FrameResult f = umbreon::render(sc, oe);
    s.check("clipNear 12: removed hit recorded in front of the plane",
            approx(f.clipNearVz[kCenterPix], 9.0f, 1e-2f));
  }

  // --- edges off: the clip-cut planes stay unallocated (no extra memory).
  {
    umbreon::Scene sc = scene();
    sc.clipNear = 10.0f;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("edges off: clip-cut planes not allocated", f.clipCut.empty());
  }

  return s.report();
}
