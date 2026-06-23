// Byte-identical-default regression for the screen-space NPR edge feature
// (design doc edge-extraction-screenspace.md, section 10).
//
// The headline guarantee of the whole edge pipeline is that it is OFF by default
// and adds ZERO observable change unless a class is actually enabled:
//   (1) A default-constructed RenderOptions{} allocates NO edge AOV
//       (objectId/viewZ/normal/materialId stay empty), so the renderer does no
//       extra per-pixel work and the legacy code path is untouched.
//   (2) Turning the master gate ON but leaving every class DISABLED must still
//       produce a BIT-IDENTICAL color buffer: the AOVs are captured and
//       applyEdges() runs, but with no class enabled it composites nothing.
//
// This guards the default-off invariant against any future edit to the capture
// or styling code: if enabling the edge AOVs ever perturbs shading, or if an
// "all classes off" render ever inks a pixel, this test fails. A small fixed
// scene with both a mesh and an analytic primitive exercises both id paths.
#include <cstddef>

#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

// A flat quad in z=0 spanning [-2,2]^2 facing +Z, one pigment, material 0.2/0.8
// -- the same minimal lit-mesh fixture the render tests use.
umbreon::Mesh makeQuad(umbreon::Vec4 color) {
  using umbreon::Vec3;
  umbreon::Mesh m;
  const Vec3 p00{-2, -2, 0}, p10{2, -2, 0}, p11{2, 2, 0}, p01{-2, 2, 0};
  const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
  const Vec3 n{0, 0, 1};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;
  return m;
}

// Mesh quad behind a sphere, head-on ortho camera + one key light, so the frame
// contains a mesh region, a primitive region and background -- all three id
// classes the edge AOVs encode.
umbreon::Scene makeScene() {
  umbreon::Scene sc;
  sc.camera.position = {0, 0, 10};
  sc.camera.direction = {0, 0, -1};
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = true;
  sc.camera.height = 4.0f;
  sc.background = {0.1f, 0.2f, 0.3f};
  sc.mesh = makeQuad({0.5f, 0.6f, 0.7f, 1.0f});
  umbreon::DistantLight l;
  l.direction = {0, 0, -1};
  l.intensity = 0.5f;
  sc.lights.push_back(l);
  umbreon::Sphere sp;
  sp.center = {0, 0, 1};  // in front of the quad
  sp.radius = 1.0f;
  sp.color = {0.9f, 0.1f, 0.1f, 1.0f};
  sc.spheres.push_back(sp);
  return sc;
}

bool colorEqual(const umbreon::FrameResult& a, const umbreon::FrameResult& b) {
  if (a.width != b.width || a.height != b.height) return false;
  if (a.color.size() != b.color.size()) return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (a.color[i] != b.color[i]) return false;  // exact, bit-for-bit
  return true;
}

}  // namespace

int main() {
  umbreon::test::Suite s("edges_default");
  const umbreon::Scene scene = makeScene();

  umbreon::RenderOptions base;
  base.width = 24;
  base.height = 24;
  base.supersample = 2;  // exercise the supersample + downsample path too

  // (1) Default path: edges off -> no edge AOV allocated.
  umbreon::FrameResult def = umbreon::render(scene, base);
  s.check("default: objectId AOV not allocated", def.objectId.empty());
  s.check("default: viewZ AOV not allocated", def.viewZ.empty());
  s.check("default: materialId AOV not allocated", def.materialId.empty());
  // `normal` is the legacy AOV; the edge-off path must not size it either.
  s.check("default: normal AOV not allocated", def.normal.empty());

  // (2) Master gate ON but every class disabled: AOVs ARE captured (so the edge
  // path runs end to end), yet the color must be BIT-IDENTICAL to the default
  // render -- enabling the gate alone must change nothing on screen.
  umbreon::RenderOptions edgesOnAllOff = base;
  edgesOnAllOff.edges.enable = true;  // defaultStyle has every class disabled
  umbreon::FrameResult on = umbreon::render(scene, edgesOnAllOff);
  s.check("edges-on/all-off: edge AOV is allocated (path exercised)",
          !on.objectId.empty());
  s.check("edges-on/all-off: color is bit-identical to the default path",
          colorEqual(def, on));

  // Same invariant without supersampling (ss == 1 takes the no-downsample
  // branch, a distinct code path through render()).
  umbreon::RenderOptions baseNoSS = base;
  baseNoSS.supersample = 1;
  umbreon::RenderOptions onNoSS = baseNoSS;
  onNoSS.edges.enable = true;
  umbreon::FrameResult defNoSS = umbreon::render(scene, baseNoSS);
  umbreon::FrameResult onNoSS2 = umbreon::render(scene, onNoSS);
  s.check("ss=1 default: no edge AOV allocated", defNoSS.objectId.empty());
  s.check("ss=1 edges-on/all-off: color is bit-identical",
          colorEqual(defNoSS, onNoSS2));

  return s.report();
}
