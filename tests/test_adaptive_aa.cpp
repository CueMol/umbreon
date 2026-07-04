// Adaptive-AA unit tests: the sample-multiplier / RNG-seed plumbing that lets
// a replicated center evaluation carry the effective sample count of the ss^2
// subpixels it replaces (plan: docs/plans/ -- adaptive antialiasing).
// The mask predicates and the renderer integration are tested here as well as
// they land (later steps of the same plan).
#include <cmath>
#include <cstdint>
#include <vector>

#include <tbb/global_control.h>

#include "test_util.hpp"
#include "umbreon.hpp"

#include "ao/ao_shade.hpp"
#include "render/adaptive_aa.hpp"
#include "render/scene_build.hpp"

namespace {

using umbreon::Vec3;
using umbreon::Vec4;

// Append one quad (two triangles, soup layout) with a constant normal/color.
void addQuad(umbreon::Mesh& m, Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n,
             Vec4 color) {
  const Vec3 corners[6] = {a, b, c, a, c, d};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
}

// Floor quad at y = 0 plus a sphere occluder hovering above, so the AO gather
// from the origin is PARTIAL (some rays hit, some miss) and therefore
// sensitive to both the sample count and the RNG seed.
struct AoTestScene {
  RTCDevice device = nullptr;
  umbreon::Scene scene;
  umbreon::detail::BuiltScene built;

  AoTestScene() {
    umbreon::Mesh m;
    const float B = 100.0f;
    addQuad(m, Vec3{-B, 0, -B}, Vec3{B, 0, -B}, Vec3{B, 0, B}, Vec3{-B, 0, B},
            Vec3{0, 1, 0}, Vec4{0.8f, 0.8f, 0.8f, 1.0f});
    scene.mesh = std::move(m);
    scene.spheres.push_back(
        umbreon::Sphere{Vec3{0.3f, 0.8f, 0.0f}, 0.6f,
                        Vec4{1.0f, 1.0f, 1.0f, 1.0f}});
    device = rtcNewDevice(nullptr);
    built = umbreon::detail::buildEmbreeScene(device, scene, false);
  }
  ~AoTestScene() {
    if (built.scene) rtcReleaseScene(built.scene);
    if (device) rtcReleaseDevice(device);
  }
};

bool sameVec3(const Vec3& a, const Vec3& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool sameShade(const umbreon::detail::AoShade& a,
               const umbreon::detail::AoShade& b) {
  return sameVec3(a.aoFactor, b.aoFactor) && sameVec3(a.ambLight, b.ambLight) &&
         a.diffuseAo == b.diffuseAo;
}

// A Wf x Hf grid of identical "flat surface hit" centers: same geometry, same
// color, constant depth, up normal. Every mask predicate is quiet on it.
std::vector<umbreon::detail::PixelResult> flatCenters(int Wf, int Hf) {
  umbreon::detail::PixelResult pr;
  pr.r = pr.g = pr.b = 0.5f;
  pr.a = 1.0f;
  pr.firstGeomID = 1;
  pr.firstGroup = 0;
  pr.viewZ = 10.0f;
  pr.worldNormal = Vec3{0.0f, 0.0f, 1.0f};
  return std::vector<umbreon::detail::PixelResult>(
      static_cast<std::size_t>(Wf) * Hf, pr);
}

int maskCount(const std::vector<uint8_t>& m) {
  int n = 0;
  for (uint8_t v : m) n += v ? 1 : 0;
  return n;
}

// A flat quad in the z = 0 plane spanning [x0, x1] x [-2, 2], facing +Z, with
// one pigment color (soup layout, material 0.2/0.8).
umbreon::Mesh makeQuadX(float x0, float x1, Vec4 color) {
  umbreon::Mesh m;
  const Vec3 p00{x0, -2, 0}, p10{x1, -2, 0}, p11{x1, 2, 0}, p01{x0, 2, 0};
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

// Orthographic camera at +Z looking down -Z, height 4 (frames [-2,2]^2 on a
// square image) -- the test_render.cpp rig.
umbreon::Scene makeSceneX(float x0, float x1) {
  umbreon::Scene sc;
  sc.mesh = makeQuadX(x0, x1, Vec4{0.5f, 0.6f, 0.7f, 1.0f});
  sc.camera.position = {0, 0, 10};
  sc.camera.direction = {0, 0, -1};
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = true;
  sc.camera.height = 4.0f;
  umbreon::DistantLight l;
  l.direction = {0, 0, -1};
  l.color = {1, 1, 1};
  l.intensity = 0.5f;
  sc.lights.push_back(l);
  sc.background = {0.1f, 0.1f, 0.1f};
  return sc;
}

bool framesEqual(const umbreon::FrameResult& a, const umbreon::FrameResult& b) {
  if (a.color.size() != b.color.size() || a.depth.size() != b.depth.size())
    return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (a.color[i] != b.color[i]) return false;
  for (std::size_t i = 0; i < a.depth.size(); ++i)
    if (a.depth[i] != b.depth[i]) return false;
  return true;
}

// Adaptive-vs-grid contract: REPLICATED pixels match the grid box-average only
// up to barycentric-interpolation rounding (identical vertex attributes
// interpolated at 4 different (u,v) differ in the last ulp), so "flat regions
// unchanged" is a ~1-ulp bound, not bitwise. FLAGGED pixels re-shade every
// subpixel exactly like the grid and stay bitwise.
bool framesClose(const umbreon::FrameResult& a, const umbreon::FrameResult& b,
                 float eps) {
  if (a.color.size() != b.color.size()) return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (std::fabs(a.color[i] - b.color[i]) > eps) return false;
  return true;
}

}  // namespace

int main() {
  umbreon::test::Suite s("adaptive_aa");

  AoTestScene ts;
  const Vec3 ambLight{1.0f, 1.0f, 1.0f};
  const Vec3 aoUp{0.0f, 1.0f, 0.0f};
  const Vec3 P{0.0f, 0.0f, 0.0f};
  const Vec3 N{0.0f, 1.0f, 0.0f};
  const Vec3 C{0.8f, 0.8f, 0.8f};
  const float secEps = 1.0e-4f;

  // --- 1. sampleMul plumbing: computeAoShade with (aoSamples = n, mul = 4)
  // must be bitwise-identical to (aoSamples = 4n, mul = 1) -- the multiplier
  // must feed the SAME nSamples into the same estimator, nothing else.
  {
    umbreon::RenderOptions optA;
    optA.width = 64;
    optA.height = 64;
    optA.aoSamples = 8;
    umbreon::RenderOptions optB = optA;
    optB.aoSamples = 32;
    bool allSame = true;
    for (uint32_t py : {0u, 7u, 31u}) {
      for (uint32_t px : {0u, 5u, 63u}) {
        const auto a = umbreon::detail::computeAoShade(
            ts.built.scene, optA, ambLight, aoUp, P, N, N, C, secEps, px, py,
            /*sampleMul=*/4, /*wSeed=*/optA.width);
        const auto b = umbreon::detail::computeAoShade(
            ts.built.scene, optB, ambLight, aoUp, P, N, N, C, secEps, px, py,
            /*sampleMul=*/1, /*wSeed=*/optB.width);
        allSame = allSame && sameShade(a, b);
      }
    }
    s.check("legacy path: sampleMul 4 == 4x aoSamples (bitwise)", allSame);
  }

  // --- 2. same equivalence on the enhanced (multi-scale) estimator path.
  {
    umbreon::RenderOptions optA;
    optA.width = 64;
    optA.height = 64;
    optA.aoSamples = 8;
    optA.aoMultiScale = true;
    umbreon::RenderOptions optB = optA;
    optB.aoSamples = 32;
    const auto a = umbreon::detail::computeAoShade(
        ts.built.scene, optA, ambLight, aoUp, P, N, N, C, secEps, 5u, 9u,
        /*sampleMul=*/4, /*wSeed=*/optA.width);
    const auto b = umbreon::detail::computeAoShade(
        ts.built.scene, optB, ambLight, aoUp, P, N, N, C, secEps, 5u, 9u,
        /*sampleMul=*/1, /*wSeed=*/optB.width);
    s.check("enhanced path: sampleMul 4 == 4x aoSamples (bitwise)",
            sameShade(a, b));
  }

  // --- 3. wSeed plumbing: a different RNG lattice width must change the
  // sample directions (base = px + py*wSeed) for at least one probe pixel
  // with py > 0. Partial occlusion + few samples makes the hit count differ.
  {
    umbreon::RenderOptions opt;
    opt.width = 64;
    opt.height = 64;
    opt.aoSamples = 8;
    bool anyDiff = false;
    for (uint32_t py : {1u, 9u, 31u}) {
      for (uint32_t px : {3u, 17u, 40u}) {
        const auto a = umbreon::detail::computeAoShade(
            ts.built.scene, opt, ambLight, aoUp, P, N, N, C, secEps, px, py,
            /*sampleMul=*/1, /*wSeed=*/opt.width);
        const auto b = umbreon::detail::computeAoShade(
            ts.built.scene, opt, ambLight, aoUp, P, N, N, C, secEps, px, py,
            /*sampleMul=*/1, /*wSeed=*/opt.width * 3);
        anyDiff = anyDiff || !sameShade(a, b);
      }
    }
    s.check("wSeed reaches the RNG (different lattice width changes AO)",
            anyDiff);
  }

  // ------------------------------------------------------------------
  // Refinement mask predicates (buildAaMask on hand-built center grids).
  // ------------------------------------------------------------------
  const umbreon::detail::AaMaskParams mp;  // defaults
  const float pxWorld = 0.1f;              // output-pixel world size stand-in

  // --- 4. flat grid: nothing fires.
  {
    const int Wf = 8, Hf = 8;
    auto centers = flatCenters(Wf, Hf);
    const auto m =
        umbreon::detail::buildAaMask(Wf, Hf, centers.data(), pxWorld, mp);
    s.check_eq("flat grid: empty mask", maskCount(m), 0);
  }

  // --- 5. geomID change in one column: the pair flags both pixels, dilation
  // extends exactly one ring around them (columns 2..5 on every row).
  {
    const int Wf = 8, Hf = 8;
    auto centers = flatCenters(Wf, Hf);
    for (int y = 0; y < Hf; ++y)
      for (int x = 4; x < Wf; ++x)
        centers[static_cast<std::size_t>(y) * Wf + x].firstGeomID = 2;
    const auto m =
        umbreon::detail::buildAaMask(Wf, Hf, centers.data(), pxWorld, mp);
    bool ok = true;
    for (int y = 0; y < Hf && ok; ++y)
      for (int x = 0; x < Wf; ++x) {
        const bool expect = (x >= 2 && x <= 5);  // pair {3,4} + 1-ring dilation
        if ((m[static_cast<std::size_t>(y) * Wf + x] != 0) != expect) {
          ok = false;
          break;
        }
      }
    s.check("geomID boundary: pair flagged + one-ring dilation", ok);
  }

  // --- 6. color contrast around the 0.1 threshold: 0.09 quiet, 0.11 fires.
  // Alpha-only contrast fires too (transparent-background coverage edges).
  {
    const int Wf = 8, Hf = 1;
    auto centers = flatCenters(Wf, Hf);
    centers[4].g += 0.09f;
    auto m = umbreon::detail::buildAaMask(Wf, Hf, centers.data(), pxWorld, mp);
    s.check_eq("color contrast 0.09 < 0.1: quiet", maskCount(m), 0);
    centers[4].g += 0.02f;  // total 0.11
    m = umbreon::detail::buildAaMask(Wf, Hf, centers.data(), pxWorld, mp);
    s.check("color contrast 0.11 > 0.1: fires", maskCount(m) > 0);
    auto centers2 = flatCenters(Wf, Hf);
    centers2[4].a = 0.7f;
    m = umbreon::detail::buildAaMask(Wf, Hf, centers2.data(), pxWorld, mp);
    s.check("alpha-only contrast fires", maskCount(m) > 0);
  }

  // --- 7. normal delta: a fold (1 - dot > 0.3) fires; a mild bend does not.
  {
    const int Wf = 8, Hf = 1;
    auto centers = flatCenters(Wf, Hf);
    for (int x = 4; x < Wf; ++x)  // ~37 deg: 1 - cos = 0.2 < 0.3
      centers[x].worldNormal = Vec3{0.6f, 0.0f, 0.8f};
    auto m = umbreon::detail::buildAaMask(Wf, Hf, centers.data(), pxWorld, mp);
    s.check_eq("mild normal bend: quiet", maskCount(m), 0);
    for (int x = 4; x < Wf; ++x)  // 90 deg: 1 - cos = 1 > 0.3
      centers[x].worldNormal = Vec3{1.0f, 0.0f, 0.0f};
    m = umbreon::detail::buildAaMask(Wf, Hf, centers.data(), pxWorld, mp);
    s.check("occlusion fold normal delta: fires", maskCount(m) > 0);
  }

  // --- 8. depth crack: a same-id step fires; the SAME per-pair delta as part
  // of a constant slope is slope-compensated away.
  {
    const int Wf = 8, Hf = 1;
    // Constant slope: z = 10 + 3*x. Every pair delta (3.0) > depthAbsPx *
    // pxWorld (1.2), but slope compensation (3 * min neighbor slope = 9) wins.
    auto slope = flatCenters(Wf, Hf);
    for (int x = 0; x < Wf; ++x) slope[x].viewZ = 10.0f + 3.0f * x;
    auto m = umbreon::detail::buildAaMask(Wf, Hf, slope.data(), pxWorld, mp);
    s.check_eq("constant depth slope: quiet", maskCount(m), 0);
    // Step: flat at 10, flat at 13 -- neighbor slopes are 0, abs floor 1.2 < 3.
    auto step = flatCenters(Wf, Hf);
    for (int x = 4; x < Wf; ++x) step[x].viewZ = 13.0f;
    m = umbreon::detail::buildAaMask(Wf, Hf, step.data(), pxWorld, mp);
    s.check("same-id depth step: fires", maskCount(m) > 0);
  }

  // --- 9. edges-mode intra-block detector: a hi-res sliver no center sees
  // (one subpixel with a different objectId) flags exactly its block + ring.
  {
    const int Wf = 6, Hf = 6, ss = 3, W = Wf * ss;
    auto centers = flatCenters(Wf, Hf);
    std::vector<uint32_t> objId(static_cast<std::size_t>(W) * Hf * ss, 7u);
    std::vector<float> viewZ(objId.size(), 10.0f);
    // One odd subpixel inside output pixel (2,2), not at its center.
    objId[(static_cast<std::size_t>(2 * ss) * W) + 2 * ss + 1] = 9u;
    const auto m = umbreon::detail::buildAaMask(
        Wf, Hf, centers.data(), pxWorld, mp, objId.data(), viewZ.data(), W, ss);
    bool ok = maskCount(m) == 9;  // block + one-ring dilation
    ok = ok && m[static_cast<std::size_t>(2) * Wf + 2] != 0;
    s.check("edges mode: sub-center sliver flags its block (+ring)", ok);
  }

  // ------------------------------------------------------------------
  // Renderer integration (umbreon::render facade, adaptive vs grid).
  // ------------------------------------------------------------------

  // --- 10. uniform scene (quad fills the whole view), AO off: nothing is
  // flagged; every replicated block matches the grid box-average within
  // interpolation rounding (see framesClose -- constant vertex attributes
  // interpolated at different (u,v) differ in the last ulp).
  {
    const umbreon::Scene sc = makeSceneX(-2.0f, 2.0f);
    umbreon::RenderOptions grid;
    grid.width = 6;
    grid.height = 6;
    grid.supersample = 2;
    umbreon::RenderOptions adap = grid;
    adap.aaMode = 1;
    const auto a = umbreon::render(sc, grid);
    const auto b = umbreon::render(sc, adap);
    s.check("uniform scene: adaptive == grid within 1e-6", framesClose(a, b, 1e-6f));
    s.check_eq("uniform scene: final width", b.width, 6);
    s.check_eq("uniform scene: final height", b.height, 6);
  }

  // --- 11. vertical silhouette (quad covers the left half), ss = 3: the
  // flagged pixels (mask AOV) re-shade every subpixel exactly like the grid,
  // so their downsampled output is BITWISE-equal; the replicated remainder
  // stays within interpolation rounding.
  {
    const umbreon::Scene sc = makeSceneX(-2.0f, 0.0f);
    umbreon::RenderOptions grid;
    grid.width = 9;
    grid.height = 9;
    grid.supersample = 3;
    umbreon::RenderOptions adap = grid;
    adap.aaMode = 1;
    adap.aaDebug = true;
    const auto a = umbreon::render(sc, grid);
    const auto b = umbreon::render(sc, adap);
    s.check("silhouette scene: adaptive == grid within 1e-6",
            framesClose(a, b, 1e-6f));
    bool flaggedBitwise = b.aaMask.size() == static_cast<std::size_t>(9 * 9);
    int flagged = 0;
    for (std::size_t p = 0; p < b.aaMask.size() && flaggedBitwise; ++p) {
      if (b.aaMask[p] == 0.0f) continue;
      ++flagged;
      for (int c = 0; c < 4; ++c)
        if (a.color[p * 4 + c] != b.color[p * 4 + c]) flaggedBitwise = false;
    }
    s.check("silhouette scene: flagged pixels bitwise-equal grid",
            flaggedBitwise && flagged > 0);
  }

  // --- 12. aaDebug mask AOV: sized Wf x Hf (final grid), fires along the
  // silhouette column, quiet far away from it.
  {
    const umbreon::Scene sc = makeSceneX(-2.0f, 0.0f);
    umbreon::RenderOptions adap;
    adap.width = 9;
    adap.height = 9;
    adap.supersample = 3;
    adap.aaMode = 1;
    adap.aaDebug = true;
    const auto f = umbreon::render(sc, adap);
    s.check_eq("aaMask: sized Wf*Hf", f.aaMask.size(),
               static_cast<std::size_t>(9 * 9));
    // Silhouette at x = 0 world = output column 4 (of [-2,2] over 9 px).
    bool edgeFlagged = true, farQuiet = true;
    for (int y = 0; y < 9; ++y) {
      edgeFlagged = edgeFlagged && f.aaMask[static_cast<std::size_t>(y) * 9 + 4] > 0.0f;
      farQuiet = farQuiet && f.aaMask[static_cast<std::size_t>(y) * 9 + 0] == 0.0f &&
                 f.aaMask[static_cast<std::size_t>(y) * 9 + 8] == 0.0f;
    }
    s.check("aaMask: silhouette column flagged", edgeFlagged);
    s.check("aaMask: far columns quiet", farQuiet);
  }

  // --- 13. determinism under thread count: AO + soft shadows on, adaptive;
  // 1 TBB thread bitwise == default threads (mask + seeds are pure functions
  // of pixel coordinates, never of the schedule).
  {
    const umbreon::Scene sc = makeSceneX(-2.0f, 0.0f);
    umbreon::RenderOptions adap;
    adap.width = 9;
    adap.height = 9;
    adap.supersample = 3;
    adap.aaMode = 1;
    adap.aoSamples = 8;
    adap.aoDistance = 100.0f;
    adap.shadows = true;
    adap.shadowSamples = 4;
    adap.lightRadius = 2.0f;
    umbreon::FrameResult a, b;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      a = umbreon::render(sc, adap);
    }
    b = umbreon::render(sc, adap);
    s.check("determinism: 1 thread == N threads (bitwise)", framesEqual(a, b));
  }

  // --- 14. AO compensation smoke test: floor + slab scene, ss = 2. The
  // unflagged floor interior gets ONE 4x-sample center evaluation instead of
  // 4 per-subpixel evaluations; both estimate the same openness, so the
  // adaptive value must sit within a small epsilon of the grid box-average
  // (the exact multiplier plumbing is locked bitwise by test 1).
  {
    umbreon::Scene sc = makeSceneX(-2.0f, 2.0f);
    // Black slab above the +x half of the floor (the AO occluder), oriented
    // facing down so the floor sees it.
    const float z = 0.4f;
    const Vec3 sl[6] = {{0.2f, -2, z}, {4, -2, z}, {4, 2, z},
                        {0.2f, -2, z}, {4, 2, z},  {0.2f, 2, z}};
    for (int i = 0; i < 6; ++i) {
      sc.mesh.positions.push_back(sl[i]);
      sc.mesh.normals.push_back({0, 0, -1});
      sc.mesh.colors.push_back({0, 0, 0, 1});
    }
    sc.mesh.material.ambient = 1.0f;  // ambient-only: isolate AO
    sc.mesh.material.diffuse = 0.0f;
    sc.lights.clear();
    umbreon::RenderOptions grid;
    grid.width = 8;
    grid.height = 8;
    grid.supersample = 2;
    grid.aoSamples = 16;
    grid.aoDistance = 100.0f;
    umbreon::RenderOptions adap = grid;
    adap.aaMode = 1;
    const auto a = umbreon::render(sc, grid);
    const auto b = umbreon::render(sc, adap);
    // Probe a floor pixel under the slab, away from the slab edge: output
    // (6, 4) -> world x ~ +1.25 (occluded half).
    const std::size_t pix = (static_cast<std::size_t>(4) * 8 + 6) * 4;
    s.check("AO compensation: unflagged pixel close to grid box-average",
            std::fabs(a.color[pix] - b.color[pix]) < 0.05f);
  }

  // --- 15. probeGBuffer parity: on a scene with all four geometry kinds'
  // G-buffer variety (mesh quad, sphere, round cylinder with an opacity
  // gradient) the shading-free probe must reproduce integratePixel's first-hit
  // G-buffer fields BITWISE for every ray -- the lock that keeps adaptive
  // edges renders extracting the identical line set.
  {
    umbreon::Scene sc;
    sc.mesh = makeQuadX(-2.0f, 0.0f, Vec4{0.5f, 0.6f, 0.7f, 1.0f});
    umbreon::Sphere sp;
    sp.center = {1.0f, 1.0f, 0.5f};
    sp.radius = 0.6f;
    sp.color = {0.9f, 0.2f, 0.2f, 1.0f};
    sc.spheres.push_back(sp);
    umbreon::Cylinder cy;
    cy.p0 = {0.5f, -1.5f, 0.5f};
    cy.p1 = {1.8f, -0.2f, 0.5f};
    cy.radius = 0.25f;
    cy.color = {0.2f, 0.9f, 0.2f, 0.8f};
    cy.opacity1 = 0.2f;  // axial opacity gradient (edge_line2 path)
    sc.cylinders.push_back(cy);

    RTCDevice device = rtcNewDevice(nullptr);
    umbreon::detail::BuiltScene built =
        umbreon::detail::buildEmbreeScene(device, sc, /*edges=*/true);
    std::vector<umbreon::detail::Light> lights;
    umbreon::RenderOptions opt;
    opt.width = 24;
    opt.height = 24;
    opt.strokeEdges.enable = true;  // shadeHit fills objectId/materialId
    umbreon::detail::ShadeContext sctx{built,
                                       sc.mesh,
                                       lights,
                                       Vec3{1, 1, 1},
                                       Vec3{0.1f, 0.1f, 0.1f},
                                       opt,
                                       Vec3{0, 1, 0}};
    std::atomic<long long> capped{0};
    const Vec3 camDir{0, 0, -1};
    bool parity = true;
    for (int py = 0; py < 24 && parity; ++py) {
      for (int px = 0; px < 24; ++px) {
        const float v = 1.0f - 2.0f * (py + 0.5f) / 24.0f;
        const float u = 2.0f * (px + 0.5f) / 24.0f - 1.0f;
        const Vec3 org{u * 2.0f, v * 2.0f, 10.0f};
        const auto pr = umbreon::detail::integratePixel(
            sctx, org, camDir, static_cast<uint32_t>(px),
            static_cast<uint32_t>(py), camDir, capped);
        const auto gp = umbreon::detail::probeGBuffer(sctx, org, camDir, camDir);
        if (gp.viewZ != pr.viewZ || gp.objectId != pr.objectId ||
            gp.materialId != pr.materialId || gp.surfAlpha != pr.firstOpacity ||
            !sameVec3(gp.normal, pr.worldNormal)) {
          parity = false;
          break;
        }
      }
    }
    s.check("probeGBuffer parity: first-hit G-buffer bitwise == integratePixel",
            parity);
    rtcReleaseScene(built.scene);
    rtcReleaseDevice(device);
  }

  // --- 16. edges integration: with strokeEdges on, the adaptive render's
  // hi-res G-buffer AOVs (probe-filled) are BITWISE the grid render's
  // (shadeHit-filled), so the stroke extraction sees identical input; the
  // composited color stays within interpolation rounding.
  {
    umbreon::Scene sc = makeSceneX(-2.0f, 0.0f);
    umbreon::Sphere sp;
    sp.center = {1.0f, 0.0f, 0.5f};
    sp.radius = 0.7f;
    sp.color = {0.9f, 0.4f, 0.1f, 1.0f};
    sc.spheres.push_back(sp);
    umbreon::RenderOptions grid;
    grid.width = 12;
    grid.height = 12;
    grid.supersample = 3;
    grid.strokeEdges.enable = true;
    umbreon::RenderOptions adap = grid;
    adap.aaMode = 1;
    const auto a = umbreon::render(sc, grid);
    const auto b = umbreon::render(sc, adap);
    s.check("edges: color within 1e-6 of grid", framesClose(a, b, 1e-6f));
    s.check("edges: viewZ AOV bitwise", a.viewZ == b.viewZ);
    s.check("edges: objectId AOV bitwise", a.objectId == b.objectId);
    s.check("edges: materialId AOV bitwise", a.materialId == b.materialId);
    s.check("edges: surfAlpha AOV bitwise", a.surfAlpha == b.surfAlpha);
    s.check("edges: normal AOV bitwise", a.normal == b.normal);
  }

  // --- 17. aa-depth: aaDepth == ss selects k == 1, which is structurally the
  // plain subpixel path -- bitwise-equal to aaDepth = 0.
  {
    const umbreon::Scene sc = makeSceneX(-2.0f, 0.0f);
    umbreon::RenderOptions o1;
    o1.width = 9;
    o1.height = 9;
    o1.supersample = 3;
    o1.aaMode = 1;
    umbreon::RenderOptions o2 = o1;
    o2.aaDepth = 3;
    const auto a = umbreon::render(sc, o1);
    const auto b = umbreon::render(sc, o2);
    s.check("aa-depth == ss: bitwise-equal to aa-depth 0", framesEqual(a, b));
  }

  // --- 18. aa-depth quality mode at ss = 1: interior (unflagged) pixels are
  // the plain 1-sample render bitwise; the silhouette pixels are 2x2 box
  // averages that must DIFFER from the grid's single sample (that is the whole
  // point) and land strictly between the two side colors.
  {
    const umbreon::Scene sc = makeSceneX(-2.0f, 0.0f);
    umbreon::RenderOptions grid;
    grid.width = 9;
    grid.height = 9;  // silhouette x=0 falls mid-pixel of column 4
    umbreon::RenderOptions adap = grid;
    adap.aaMode = 1;
    adap.aaDepth = 2;
    adap.aaDebug = true;
    const auto a = umbreon::render(sc, grid);
    const auto b = umbreon::render(sc, adap);
    s.check_eq("aa-depth ss=1: aaMask full-res", b.aaMask.size(),
               static_cast<std::size_t>(9 * 9));
    bool interiorSame = true, edgeDiffers = false;
    for (std::size_t p = 0; p < b.aaMask.size(); ++p) {
      bool same = true;
      for (int c = 0; c < 4; ++c)
        if (a.color[p * 4 + c] != b.color[p * 4 + c]) same = false;
      if (b.aaMask[p] == 0.0f) {
        interiorSame = interiorSame && same;
      } else if (!same) {
        edgeDiffers = true;
      }
    }
    s.check("aa-depth ss=1: unflagged pixels bitwise == 1-sample grid",
            interiorSame);
    s.check("aa-depth ss=1: some flagged pixels are refined (differ)",
            edgeDiffers);
  }

  // --- 19. GI fallback: adaptive + gi normalizes to the grid path in
  // renderFrame (warning), so the output is bitwise the gi grid render.
  {
    const umbreon::Scene sc = makeSceneX(-2.0f, 0.0f);
    umbreon::RenderOptions grid;
    grid.width = 6;
    grid.height = 6;
    grid.supersample = 2;
    grid.gi = true;
    umbreon::RenderOptions adap = grid;
    adap.aaMode = 1;
    const auto a = umbreon::render(sc, grid);
    const auto b = umbreon::render(sc, adap);
    s.check("gi fallback: adaptive+gi == grid+gi (bitwise)", framesEqual(a, b));
  }

  return s.report();
}
