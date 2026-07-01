// pt1 integration tests against the render() facade.
//
// Phase 2 (direct-lighting parity): pt1 replaces only the indirect post-pass;
// the direct shading comes from the exact same pixel loop as the cache path.
// Rendering the same GI scene with both integrators at giIntensity = 0 (the
// composite then adds exactly 0.0f) must therefore produce bitwise-identical
// color -- this locks the "direct matches the existing renderer" acceptance
// criterion by construction and guards the dispatch seam against divergence.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

// A GI test scene: a floor quad (mesh, gather-eligible) plus a sphere
// primitive (CSG occluder) under one angled distant light.
umbreon::Mesh makeFloor(umbreon::Vec4 color) {
  using umbreon::Vec3;
  umbreon::Mesh m;
  const Vec3 p00{-4, 0, -4}, p10{4, 0, -4}, p11{4, 0, 4}, p01{-4, 0, 4};
  const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
  const Vec3 n{0, 1, 0};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;
  return m;
}

umbreon::Scene makeGiScene() {
  umbreon::Scene sc;
  sc.mesh = makeFloor(umbreon::Vec4{0.7f, 0.7f, 0.7f, 1.0f});
  sc.camera.position = {0, 3, 8};
  sc.camera.direction = umbreon::normalize(umbreon::Vec3{0, -0.35f, -1});
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = false;
  sc.camera.fovy = 40.0f;
  umbreon::DistantLight l;
  l.direction = umbreon::normalize(umbreon::Vec3{-0.4f, -1.0f, -0.3f});
  l.color = {1, 1, 1};
  l.intensity = 0.8f;
  sc.lights.push_back(l);
  umbreon::Sphere sp;
  sp.center = {0, 1, 0};
  sp.radius = 1.0f;
  sp.color = {0.8f, 0.3f, 0.3f, 1.0f};
  sp.material.ambient = 0.2f;
  sp.material.diffuse = 0.8f;
  sc.spheres.push_back(sp);
  sc.background = {0.1f, 0.1f, 0.15f};
  sc.ambientColor = {0.5f, 0.5f, 0.5f};  // GI energy the gather collects
  return sc;
}

umbreon::RenderOptions makeGiOptions(int integrator, float giIntensity) {
  umbreon::RenderOptions o;
  o.width = 48;
  o.height = 36;
  o.gi = true;
  o.giIntegrator = integrator;
  o.giIntensity = giIntensity;
  o.giSamples = 16;   // keep the cache branch cheap
  o.pt1Spp = 4;       // keep the pt1 branch cheap
  o.pt1HalfRes = false;
  o.pt1Denoise = false;
  o.shadows = true;
  return o;
}

// Open-top Cornell box (plan Phase 3 adaptation: the repo has distant lights
// only, no area lights, so the box is lit from above through the missing
// ceiling by a straight-down distant light). White floor and back wall, red
// left wall, green right wall; bleeding shows on the floor next to each wall.
namespace cornell {

void addQuad(umbreon::Mesh& m, umbreon::Vec3 a, umbreon::Vec3 b,
             umbreon::Vec3 c, umbreon::Vec3 d, umbreon::Vec3 n,
             umbreon::Vec4 color) {
  const umbreon::Vec3 corners[6] = {a, b, c, a, c, d};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
}

umbreon::Scene makeScene() {
  using umbreon::Vec3;
  using umbreon::Vec4;
  const Vec4 white{0.75f, 0.75f, 0.75f, 1.0f};
  const Vec4 red{0.75f, 0.05f, 0.05f, 1.0f};
  const Vec4 green{0.05f, 0.75f, 0.05f, 1.0f};
  umbreon::Mesh m;
  // Box interior [-1,1]^2 x [0,2] in y, open top (no ceiling) and open front
  // (+z, toward the camera). Normals face inward.
  addQuad(m, {-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}, {0, 1, 0}, white);
  addQuad(m, {-1, 0, -1}, {-1, 2, -1}, {1, 2, -1}, {1, 0, -1}, {0, 0, 1}, white);
  addQuad(m, {-1, 0, -1}, {-1, 0, 1}, {-1, 2, 1}, {-1, 2, -1}, {1, 0, 0}, red);
  addQuad(m, {1, 0, -1}, {1, 2, -1}, {1, 2, 1}, {1, 0, 1}, {-1, 0, 0}, green);
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;

  umbreon::Scene sc;
  sc.mesh = std::move(m);
  sc.camera.position = {0, 1, 3.2f};
  sc.camera.direction = {0, 0, -1};
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = false;
  sc.camera.fovy = 45.0f;
  umbreon::DistantLight l;
  l.direction = {0, -1, 0};  // straight down through the open top
  l.color = {1, 1, 1};
  l.intensity = 1.0f;
  sc.lights.push_back(l);
  sc.background = {0, 0, 0};
  sc.ambientColor = {0.25f, 0.25f, 0.25f};  // small sky fill via the open top
  return sc;
}

}  // namespace cornell

}  // namespace

int main() {
  umbreon::test::Suite s("pt1_render");
  const umbreon::Scene sc = makeGiScene();

  // --- direct-lighting parity: giIntensity = 0 makes both integrators add
  // exactly +0.0f indirect, so color must match the shared direct pass bitwise.
  {
    const umbreon::FrameResult a = umbreon::render(sc, makeGiOptions(0, 0.0f));
    const umbreon::FrameResult b = umbreon::render(sc, makeGiOptions(1, 0.0f));
    bool colorSame = a.color.size() == b.color.size();
    if (colorSame)
      for (std::size_t i = 0; i < a.color.size(); ++i)
        if (a.color[i] != b.color[i]) {
          colorSame = false;
          break;
        }
    s.check("direct parity: cache and pt1 color bitwise-identical at gi*0",
            colorSame);
  }

  // --- direct component parity at full GI strength: color - indirect must
  // agree within float rounding (the indirect add/subtract costs <= 1 ulp).
  {
    const umbreon::FrameResult a = umbreon::render(sc, makeGiOptions(0, 1.0f));
    const umbreon::FrameResult b = umbreon::render(sc, makeGiOptions(1, 1.0f));
    const std::size_t npix = static_cast<std::size_t>(a.width) * a.height;
    bool directSame = b.indirect.size() == npix * 3;
    float maxDiff = 0.0f;
    for (std::size_t p = 0; p < npix && directSame; ++p)
      for (int c = 0; c < 3; ++c) {
        const float da = a.color[p * 4 + c] - a.indirect[p * 3 + c];
        const float db = b.color[p * 4 + c] - b.indirect[p * 3 + c];
        maxDiff = std::fmax(maxDiff, std::fabs(da - db));
      }
    s.check("direct parity: color - indirect matches within 1e-5",
            directSame && maxDiff <= 1e-5f);
  }

  // --- Cornell color bleeding (plan Phase 3 acceptance 2): with the pt1
  // full-res gather, the white floor next to the red wall must pick up red
  // indirect light (R > 1.15 * G in the `indirect` AOV), mirrored for green,
  // and the floor center must receive nonzero indirect.
  {
    const umbreon::Scene box = cornell::makeScene();
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.gi = true;
    o.giIntegrator = 1;
    o.pt1Spp = 128;
    o.pt1HalfRes = false;
    o.pt1Denoise = false;
    o.shadows = true;
    const umbreon::FrameResult f = umbreon::render(box, o);

    // Sample the floor in the lower image half: rows ~3/4 down look at the
    // floor; columns near the left/right edges sit next to the red/green wall.
    auto indirectAt = [&](int px, int py) {
      const std::size_t pix = static_cast<std::size_t>(py) * f.width + px;
      return umbreon::Vec3{f.indirect[pix * 3 + 0], f.indirect[pix * 3 + 1],
                           f.indirect[pix * 3 + 2]};
    };
    const int floorRow = 52;
    const umbreon::Vec3 nearRed = indirectAt(6, floorRow);
    const umbreon::Vec3 nearGreen = indirectAt(57, floorRow);
    const umbreon::Vec3 center = indirectAt(32, floorRow);
    s.check("cornell: floor near red wall bleeds red (R > 1.15 G)",
            nearRed.x > 1.15f * nearRed.y && nearRed.x > 0.0f);
    s.check("cornell: floor near green wall bleeds green (G > 1.15 R)",
            nearGreen.y > 1.15f * nearGreen.x && nearGreen.y > 0.0f);
    s.check("cornell: floor center indirect > 0",
            center.x > 0.0f && center.y > 0.0f && center.z > 0.0f);
  }

  return s.report();
}
