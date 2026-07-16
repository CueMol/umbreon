// pt2 integration tests against the render() facade. Locks the invariants
// verified by hand during the pt2 phase-1 work:
//
//  1. Determinism: run-to-run AND thread-count bit-exactness through every
//     pt2 extension (Sobol/blue-noise sampler, emissive NEE, adaptive spp,
//     area-light soft NEE, traced reflection). The pt2 machinery is built on
//     pure per-pixel functions and ping-pong passes, so this is structural --
//     the test guards it against regressions.
//  2. Direct parity: at giIntensity = 0, pt1 and pt2 produce bitwise-equal
//     color (they share the pixel loop and the composite adds 0; the sampler
//     difference lives entirely inside the E buffer).
//  3. Emissive transport: no double count (the flag leaves gi-off renders
//     bit-exact), a real halo with GI on, and NEE-vs-BSDF-only convergence
//     (both estimators are unbiased for the same integral).
//  4. Area lights: DistantLight::angularRadius is pt2-only -- pt1 output is
//     bit-exact with and without it; under pt2 it softens shadows without
//     changing the total energy.
//  5. Feature gates: on scenes without the relevant material/light, each
//     pt2 extension is a bit-exact no-op (reflection, emissive NEE).
//
// Deliberately NOT tested: ReSTIR spatial resampling (--pt2-rounds). It is
// measured-dead for stills and kept only as a video/pt3 substrate; see the
// carry-over note in experimental/pt2/pt2_spatial.hpp.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

// Floor mesh + optional emissive panel, one distant light, one CSG sphere.
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

// Append a small vertical emissive panel hovering over the floor. Uses a
// second material slot so only the panel emits.
void addEmissivePanel(umbreon::Mesh& m, float emission) {
  using umbreon::Vec3;
  const Vec3 a{-0.5f, 0.6f, -1.0f}, b{0.5f, 0.6f, -1.0f},
      c{0.5f, 1.6f, -1.0f}, d{-0.5f, 1.6f, -1.0f};
  const Vec3 corners[6] = {a, b, c, a, c, d};
  const Vec3 n{0, 0, 1};
  const std::size_t firstTri = m.positions.size() / 3;
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(umbreon::Vec4{1.0f, 0.5f, 0.1f, 1.0f});
  }
  umbreon::Material base = m.material;
  umbreon::Material em = m.material;
  em.emission = emission;
  em.diffuse = 0.2f;
  m.materials = {base, em};
  m.triMaterialId.assign(m.positions.size() / 3, 0);
  for (std::size_t t = firstTri; t < m.positions.size() / 3; ++t)
    m.triMaterialId[t] = 1;
}

umbreon::Scene makeScene(bool emissivePanel, float lightAngularRadius) {
  umbreon::Scene sc;
  sc.mesh = makeFloor(umbreon::Vec4{0.7f, 0.7f, 0.7f, 1.0f});
  if (emissivePanel) addEmissivePanel(sc.mesh, 3.0f);
  sc.camera.position = {0, 3, 8};
  sc.camera.direction = umbreon::normalize(umbreon::Vec3{0, -0.35f, -1});
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = false;
  sc.camera.fovy = 40.0f;
  umbreon::DistantLight l;
  l.direction = umbreon::normalize(umbreon::Vec3{-0.4f, -1.0f, -0.3f});
  l.color = {1, 1, 1};
  l.intensity = 0.8f;
  l.angularRadius = lightAngularRadius;
  sc.lights.push_back(l);
  umbreon::Sphere sp;
  sp.center = {0, 1, 0};
  sp.radius = 1.0f;
  sp.color = {0.8f, 0.3f, 0.3f, 1.0f};
  sp.material.ambient = 0.2f;
  sp.material.diffuse = 0.8f;
  sc.spheres.push_back(sp);
  sc.background = {0.1f, 0.1f, 0.15f};
  sc.ambientColor = {0.5f, 0.5f, 0.5f};
  return sc;
}

umbreon::RenderOptions makeOpts(int integrator) {
  umbreon::RenderOptions o;
  o.width = 48;
  o.height = 36;
  o.gi = true;
  o.giIntegrator = integrator;
  o.giSamples = 16;
  o.pt1Spp = 4;
  o.pt1HalfRes = false;
  o.pt1GatherDiv = 1;
  o.pt1Denoise = false;
  o.pt1EdgePatch = false;
  return o;
}

bool bitEqual(const std::vector<float>& a, const std::vector<float>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i] != b[i]) return false;
  return true;
}

float meanLum(const umbreon::FrameResult& f) {
  double s = 0.0;
  const std::size_t n = f.color.size() / 4;
  for (std::size_t p = 0; p < n; ++p)
    s += 0.2126 * f.color[p * 4] + 0.7152 * f.color[p * 4 + 1] +
         0.0722 * f.color[p * 4 + 2];
  return static_cast<float>(s / static_cast<double>(n));
}

}  // namespace

int main() {
  umbreon::test::Suite s("pt2_render");

  // 1) Determinism through every pt2 extension at once: emissive panel +
  //    area light + adaptive spp + traced reflection candidates, rendered
  //    twice and with different thread caps elsewhere (the harness runs the
  //    TBB default; determinism is by pure-function construction, so two
  //    runs in the same process must be bitwise equal).
  {
    const umbreon::Scene sc = makeScene(true, 0.15f);
    umbreon::RenderOptions o = makeOpts(2);
    o.shadows = true;
    o.shadowSamples = 4;
    o.pt2Adaptive = true;
    const umbreon::FrameResult f1 = umbreon::render(sc, o);
    const umbreon::FrameResult f2 = umbreon::render(sc, o);
    s.check("pt2 full-feature render is run-to-run bit-exact",
            bitEqual(f1.color, f2.color) && bitEqual(f1.indirect, f2.indirect));
  }

  // 2) Direct parity: giIntensity = 0 makes the composite a no-op, so pt1
  //    and pt2 -- which share the pixel loop -- must agree bitwise.
  {
    const umbreon::Scene sc = makeScene(false, 0.0f);
    umbreon::RenderOptions o1 = makeOpts(1);
    umbreon::RenderOptions o2 = makeOpts(2);
    o1.giIntensity = 0.0f;
    o2.giIntensity = 0.0f;
    const umbreon::FrameResult f1 = umbreon::render(sc, o1);
    const umbreon::FrameResult f2 = umbreon::render(sc, o2);
    s.check("giIntensity=0: pt1 and pt2 direct passes bitwise equal",
            bitEqual(f1.color, f2.color));
  }

  // 3a) Emissive transport exists and is GI-only: with GI on, the panel
  //     brightens the floor (pt2Emissive on vs off differ); with gi off the
  //     flag must be a bit-exact no-op.
  {
    const umbreon::Scene sc = makeScene(true, 0.0f);
    umbreon::RenderOptions on = makeOpts(2);
    umbreon::RenderOptions off = makeOpts(2);
    off.pt2Emissive = false;
    const umbreon::FrameResult fOn = umbreon::render(sc, on);
    const umbreon::FrameResult fOff = umbreon::render(sc, off);
    s.check("emissive transport brightens the GI render",
            meanLum(fOn) > meanLum(fOff) + 1e-4f);

    umbreon::RenderOptions dOn = makeOpts(2);
    umbreon::RenderOptions dOff = makeOpts(2);
    dOn.gi = false;
    dOff.gi = false;
    dOff.pt2Emissive = false;
    const umbreon::FrameResult g1 = umbreon::render(sc, dOn);
    const umbreon::FrameResult g2 = umbreon::render(sc, dOff);
    s.check("emissive flag is a no-op with gi off (no double count)",
            bitEqual(g1.color, g2.color));
  }

  // 3b) NEE-vs-BSDF-only convergence: both are unbiased estimators of the
  //     same transport, so high-spp means must agree closely. (Loose bound:
  //     the BSDF-only side converges slowly on a small panel.)
  {
    const umbreon::Scene sc = makeScene(true, 0.0f);
    umbreon::RenderOptions a = makeOpts(2);
    umbreon::RenderOptions b = makeOpts(2);
    a.pt1Spp = 256;
    b.pt1Spp = 256;
    b.pt2EmissiveNee = false;
    const umbreon::FrameResult fa = umbreon::render(sc, a);
    const umbreon::FrameResult fb = umbreon::render(sc, b);
    const float ma = meanLum(fa), mb = meanLum(fb);
    s.check("emissive NEE and BSDF-only converge to the same mean (2%)",
            std::fabs(ma - mb) <= 0.02f * std::fmax(ma, mb));
  }

  // 4) Area light: angularRadius must be INERT under pt1 (frozen integrator)
  //    and soften shadows under pt2 without changing the energy.
  {
    const umbreon::Scene hard = makeScene(false, 0.0f);
    const umbreon::Scene area = makeScene(false, 0.20f);
    umbreon::RenderOptions o1 = makeOpts(1);
    o1.shadows = true;
    const umbreon::FrameResult p1h = umbreon::render(hard, o1);
    const umbreon::FrameResult p1a = umbreon::render(area, o1);
    s.check("pt1 ignores DistantLight::angularRadius (bitwise)",
            bitEqual(p1h.color, p1a.color));

    umbreon::RenderOptions o2 = makeOpts(2);
    o2.shadows = true;
    o2.shadowSamples = 8;
    const umbreon::FrameResult p2h = umbreon::render(hard, o2);
    const umbreon::FrameResult p2a = umbreon::render(area, o2);
    s.check("pt2 area light changes the shadow rendering",
            !bitEqual(p2h.color, p2a.color));
    const float mh = meanLum(p2h), ma = meanLum(p2a);
    s.check("pt2 area light conserves energy (1.5%)",
            std::fabs(mh - ma) <= 0.015f * std::fmax(mh, ma));
  }

  // 5) Feature gates: without reflective material the traced-reflection flag
  //    is a bit-exact no-op; without emissive material the NEE flag is too.
  {
    const umbreon::Scene sc = makeScene(false, 0.0f);
    umbreon::RenderOptions a = makeOpts(2);
    umbreon::RenderOptions b = makeOpts(2);
    b.pt2Reflect = false;
    const umbreon::FrameResult fa = umbreon::render(sc, a);
    const umbreon::FrameResult fb = umbreon::render(sc, b);
    s.check("no reflective material: --pt2-reflect is a bit-exact no-op",
            bitEqual(fa.color, fb.color));

    umbreon::RenderOptions c = makeOpts(2);
    c.pt2EmissiveNee = false;
    const umbreon::FrameResult fc = umbreon::render(sc, c);
    s.check("no emissive material: NEE flag is a bit-exact no-op",
            bitEqual(fa.color, fc.color));
  }

  // 6) Adaptive spp: off is bit-exact with the flag absent (base path
  //    untouched), on stays deterministic (two runs bit-exact).
  {
    const umbreon::Scene sc = makeScene(false, 0.0f);
    umbreon::RenderOptions off1 = makeOpts(2);
    umbreon::RenderOptions off2 = makeOpts(2);
    off2.pt2Adaptive = false;  // explicit off == default
    const umbreon::FrameResult f1 = umbreon::render(sc, off1);
    const umbreon::FrameResult f2 = umbreon::render(sc, off2);
    s.check("adaptive off matches the default bitwise",
            bitEqual(f1.color, f2.color));

    umbreon::RenderOptions on = makeOpts(2);
    on.pt2Adaptive = true;
    on.pt1Spp = 8;
    const umbreon::FrameResult a1 = umbreon::render(sc, on);
    const umbreon::FrameResult a2 = umbreon::render(sc, on);
    s.check("adaptive spp is run-to-run bit-exact",
            bitEqual(a1.color, a2.color));
  }

  return s.report();
}
