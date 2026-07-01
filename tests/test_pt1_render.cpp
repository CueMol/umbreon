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

  return s.report();
}
