// pt1 numeric gather tests (plan Phase 3 acceptance): call pt1GatherPoint
// directly on hand-built Embree scenes and check the estimator against
// analytic values. Convention reminder: the gather returns E_stored =
// mean(L_i) = E_true/pi, so the plan's "E = pi" uniform-sky expectation reads
// E_stored = 1.0 here (see docs/pt1_design.md).
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

#include "experimental/pt1/pt1_integrator.hpp"
#include "render/scene_build.hpp"

namespace {

using umbreon::Vec3;
using umbreon::Vec4;

bool approxRel(float got, float expected, float relTol) {
  return std::fabs(got - expected) <= relTol * std::fabs(expected);
}

// A large vertical wall in the x = -0.5 plane (normal +x), spanning
// [-2000, 2000] in y and z. Big enough that every gather direction with
// wi.x < 0 from the origin hits it except a negligible grazing sliver.
umbreon::Mesh makeWall() {
  umbreon::Mesh m;
  const float d = -0.5f, B = 2000.0f;
  const Vec3 p00{d, -B, -B}, p10{d, -B, B}, p11{d, B, B}, p01{d, B, -B};
  const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
  const Vec3 n{1, 0, 0};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(Vec4{0.0f, 0.0f, 0.0f, 1.0f});  // black (and unlit)
  }
  m.material.ambient = 0.0f;
  m.material.diffuse = 0.8f;
  return m;
}

struct TestScene {
  RTCDevice device = nullptr;
  umbreon::Scene scene;
  umbreon::detail::BuiltScene built;
  std::vector<umbreon::detail::Light> lights;  // empty: sky-only tests

  explicit TestScene(umbreon::Mesh mesh) {
    scene.mesh = std::move(mesh);
    device = rtcNewDevice(nullptr);
    built = umbreon::detail::buildEmbreeScene(device, scene, false);
  }
  ~TestScene() {
    if (built.scene) rtcReleaseScene(built.scene);
    if (device) rtcReleaseDevice(device);
  }

  umbreon::detail::IrradianceCacheParams params() const {
    umbreon::detail::IrradianceCacheParams p;
    p.scene = built.scene;
    p.built = &built;
    p.mesh = &scene.mesh;
    p.lights = &lights;
    p.ambLight = Vec3{1.0f, 1.0f, 1.0f};
    p.envUp = Vec3{0.0f, 1.0f, 0.0f};
    p.envIntensity = 1.0f;
    p.maxDistance = std::numeric_limits<float>::infinity();
    p.spacing = 1.0f;
    return p;
  }
};

}  // namespace

int main() {
  umbreon::test::Suite s("pt1_gather");
  const Vec3 P{0, 0, 0};
  const Vec3 N{0, 1, 0};
  const int spp = 1024;
  const float epsT = 10.0f;  // finite eps length scale (scene-diag stand-in)

  // --- 1. up-facing point under a uniform sky = 1, no geometry, no lights:
  // every sample misses with L = 1, so E_stored = 1.0 exactly (plan test 1,
  // "E = pi" in the plan's convention). 1% tolerance per the plan.
  {
    TestScene ts(umbreon::Mesh{});  // empty scene: all rays miss
    umbreon::detail::IrradianceCacheParams p = ts.params();
    p.skyColor = p.groundColor = Vec3{1.0f, 1.0f, 1.0f};
    const Vec3 E =
        umbreon::detail::pt1GatherPoint(p, P, N, N, spp, 12345u, epsT, nullptr);
    s.check("uniform sky: E_stored = 1.0 within 1%", approxRel(E.x, 1.0f, 0.01f));
    s.check("uniform sky: channels equal", E.x == E.y && E.y == E.z);
  }

  // --- 2. gradient sky (sky = 1 at zenith, ground = 0), up axis +y, floor
  // normal +y: L(wi) = 0.5 * (wi.y + 1), and under cosine sampling
  // E[wi.y] = 2/3, so E_stored = 0.5 * (2/3 + 1) = 5/6. This exercises the
  // sampler and the estimator constant (test 1 alone is insensitive to both).
  {
    TestScene ts(umbreon::Mesh{});
    umbreon::detail::IrradianceCacheParams p = ts.params();
    p.skyColor = Vec3{1.0f, 1.0f, 1.0f};
    p.groundColor = Vec3{0.0f, 0.0f, 0.0f};
    const Vec3 E =
        umbreon::detail::pt1GatherPoint(p, P, N, N, spp, 12345u, epsT, nullptr);
    s.check("gradient sky: E_stored = 5/6 within 1%",
            approxRel(E.x, 5.0f / 6.0f, 0.01f));
  }

  // --- 3. half-occluded: a huge unlit black wall at x = -0.5 blocks the
  // wi.x < 0 half of the hemisphere (cosine-weighted measure exactly 1/2 by
  // symmetry). Hits contribute 0 (no lights -> oneBounceRadiance = 0), so
  // E_stored ~= 0.5 and the occlusion output ~= 0.5. The estimate of a binary
  // event has sd = 0.5/sqrt(spp), so use 8192 samples and a 3% tolerance
  // (~5 sd); the deterministic seed keeps the result stable run-to-run.
  {
    TestScene ts(makeWall());
    umbreon::detail::IrradianceCacheParams p = ts.params();
    p.skyColor = p.groundColor = Vec3{1.0f, 1.0f, 1.0f};
    float occ = 0.0f;
    const Vec3 E = umbreon::detail::pt1GatherPoint(p, P, N, N, 8192, 12345u,
                                                   epsT, &occ);
    s.check("half occluded: E_stored = 0.5 within 3%",
            approxRel(E.x, 0.5f, 0.03f));
    s.check("half occluded: occlusion = 0.5 within 3%",
            approxRel(occ, 0.5f, 0.03f));
  }

  // --- 4. denoisePt1E sanity (plan Phase 4): a constant irradiance field must
  // come back (approximately) constant, and a NaN-poisoned buffer must come
  // back finite (the scrub runs before the backend).
  {
    const int w = 32, h = 32;
    const std::size_t npix = static_cast<std::size_t>(w) * h;
    std::vector<float> albedo(npix * 3, 0.7f);
    std::vector<float> normal(npix * 3, 0.0f);
    std::vector<float> pos(npix * 3, 0.0f);
    for (std::size_t pix = 0; pix < npix; ++pix) {
      normal[pix * 3 + 2] = 1.0f;  // flat +z plane
      const std::size_t col = pix % static_cast<std::size_t>(w);
      const std::size_t row = pix / static_cast<std::size_t>(w);
      pos[pix * 3 + 0] = static_cast<float>(col) * 0.1f;
      pos[pix * 3 + 1] = static_cast<float>(row) * 0.1f;
    }
    umbreon::RenderOptions opt;

    std::vector<float> E(npix * 3, 0.5f);
    umbreon::detail::denoisePt1E(w, h, E, albedo.data(), normal.data(),
                                 pos.data(), opt);
    float maxDev = 0.0f;
    for (float v : E) maxDev = std::fmax(maxDev, std::fabs(v - 0.5f));
    s.check("denoise: constant field stays constant (dev < 0.05)",
            maxDev < 0.05f);

    std::vector<float> Enan(npix * 3, 0.5f);
    for (std::size_t i = 0; i < Enan.size(); i += 97)
      Enan[i] = std::numeric_limits<float>::quiet_NaN();
    Enan[5] = std::numeric_limits<float>::infinity();
    umbreon::detail::denoisePt1E(w, h, Enan, albedo.data(), normal.data(),
                                 pos.data(), opt);
    bool allFinite = true;
    for (float v : Enan) allFinite = allFinite && std::isfinite(v);
    s.check("denoise: NaN/Inf input comes back finite", allFinite);
  }

  return s.report();
}
