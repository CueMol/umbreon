// Coarse-grid AO tests (--ao-res out): the per-hit bilateral lookup unit
// tests on synthetic grids, and adaptive-vs-full render equivalences on
// synthetic scenes (added with the renderer wiring).
#include <cmath>
#include <cstdint>
#include <vector>

#include <tbb/global_control.h>

#include "test_util.hpp"
#include "umbreon.hpp"

#include "ao/ao_coarse.hpp"

namespace {

using umbreon::Vec3;
using umbreon::detail::AOResult;
using umbreon::detail::CoarseAoGrid;
using umbreon::detail::sampleCoarseAo;

// A wf x hf grid of eligible flat-surface cells: normal +Z, constant depth,
// per-cell openness supplied by the caller.
CoarseAoGrid makeGrid(int w, int h, float depth,
                      const std::vector<float>& openness) {
  CoarseAoGrid g;
  g.w = w;
  g.h = h;
  g.openness = openness;
  g.normal.assign(static_cast<std::size_t>(w) * h * 3, 0.0f);
  for (std::size_t i = 0; i < static_cast<std::size_t>(w) * h; ++i)
    g.normal[i * 3 + 2] = 1.0f;
  g.depth.assign(static_cast<std::size_t>(w) * h, depth);
  g.hit.assign(static_cast<std::size_t>(w) * h, 1);
  return g;
}

bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

// Ambient-only white floor at z = 0 with a black slab hovering over the +x
// half (the test_render.cpp AO rig): the floor's AO varies smoothly from open
// (-x) to occluded (+x), ideal for coarse-vs-inline comparisons. Ortho camera
// at +z looking down, height 4 -> frames [-2,2]^2.
umbreon::Scene makeAoScene(bool withSlab = true) {
  umbreon::Scene sc;
  umbreon::Mesh m;
  const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                      {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(fl[i]);
    m.normals.push_back({0, 0, 1});
    m.colors.push_back({1, 1, 1, 1});
  }
  if (withSlab) {
    const float z = 0.4f;
    const Vec3 sl[6] = {{0.2f, -2, z}, {4, -2, z}, {4, 2, z},
                        {0.2f, -2, z}, {4, 2, z},  {0.2f, 2, z}};
    for (int i = 0; i < 6; ++i) {
      m.positions.push_back(sl[i]);
      m.normals.push_back({0, 0, -1});
      m.colors.push_back({0, 0, 0, 1});
    }
  }
  m.material.ambient = 1.0f;  // ambient-only: isolate AO's effect
  m.material.diffuse = 0.0f;
  sc.mesh = std::move(m);
  sc.camera.position = {0, 0, 10};
  sc.camera.direction = {0, 0, -1};
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = true;
  sc.camera.height = 4.0f;
  sc.background = {0.1f, 0.1f, 0.1f};
  return sc;
}

bool framesEqual(const umbreon::FrameResult& a, const umbreon::FrameResult& b) {
  if (a.color.size() != b.color.size()) return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (a.color[i] != b.color[i]) return false;
  return true;
}

bool framesClose(const umbreon::FrameResult& a, const umbreon::FrameResult& b,
                 float eps) {
  if (a.color.size() != b.color.size()) return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (std::fabs(a.color[i] - b.color[i]) > eps) return false;
  return true;
}

}  // namespace

int main() {
  umbreon::test::Suite s("ao_coarse");
  const Vec3 up{0, 0, 1};

  // --- 1. bilinear interior: a 2x2 grid {0, 1, 0, 1} sampled at the exact
  // midpoint of the four cell centers must return the plain average 0.5
  // (flat normals and equal depths make the bilateral weights pure bilinear).
  {
    CoarseAoGrid g = makeGrid(2, 2, 10.0f, {0.0f, 1.0f, 0.0f, 1.0f});
    // lattice 4x4 (div 2): hi-res pixel (1,1) center -> s=(1.5)*2/4-0.5 = 0.25;
    // pixel (2,2) would be s=0.75. Midpoint of cells = s=0.5 -> px such that
    // (px+0.5)/latticeW*g.w-0.5 = 0.5 => px+0.5 = 2 => px = 1.5 (not integer).
    // Use lattice 8x8 (div 4): px=3 -> s=(3.5)*2/8-0.5 = 0.375; px=4 -> 0.625.
    // Exact midpoint needs (px+0.5)*0.25 = 1.0 -> px=3.5. So instead check the
    // symmetric pair: lookups at px=3 and px=4 must be mirror weights, and
    // their average must be 0.5.
    float op1 = -1, op2 = -1;
    AOResult aov;
    s.check("bilinear: lookup at px=3 succeeds",
            sampleCoarseAo(g, 8, 8, 3, 3, up, 10.0f, op1, aov));
    s.check("bilinear: lookup at px=4 succeeds",
            sampleCoarseAo(g, 8, 8, 4, 3, up, 10.0f, op2, aov));
    s.check("bilinear: symmetric pair averages to 0.5",
            approx(0.5f * (op1 + op2), 0.5f, 1e-5f));
    s.check("bilinear: values bracket the average", op1 < 0.5f && op2 > 0.5f);
    // Directly over a cell center (px=1 on lattice 8 -> s=(1.5)*0.25-0.5=-0.125
    // -> clamps into cell 0 region with cell 0 dominating): openness stays
    // within [0,1] and below 0.5 (cell 0 = 0.0 dominates).
    float op0 = -1;
    s.check("bilinear: near cell 0 succeeds",
            sampleCoarseAo(g, 8, 8, 1, 1, up, 10.0f, op0, aov));
    s.check("bilinear: near cell 0 leans to its value", op0 < 0.4f);
  }

  // --- 2. depth rejection: a hit far behind the grid surface (transparency
  // back layer) must fail the lookup even with matching normals.
  {
    CoarseAoGrid g = makeGrid(2, 2, 10.0f, {0.5f, 0.5f, 0.5f, 0.5f});
    float op = -1;
    AOResult aov;
    s.check("depth match succeeds",
            sampleCoarseAo(g, 8, 8, 4, 4, up, 10.0f, op, aov));
    s.check("depth mismatch (z=20 vs grid 10) fails",
            !sampleCoarseAo(g, 8, 8, 4, 4, up, 20.0f, op, aov));
  }

  // --- 3. normal rejection: a hit whose normal disagrees with the grid
  // (silhouette rim / different surface) must fail.
  {
    CoarseAoGrid g = makeGrid(2, 2, 10.0f, {0.5f, 0.5f, 0.5f, 0.5f});
    float op = -1;
    AOResult aov;
    const Vec3 side{1, 0, 0};  // 90 degrees off the grid's +Z
    s.check("normal mismatch fails",
            !sampleCoarseAo(g, 8, 8, 4, 4, side, 10.0f, op, aov));
    // A mild bend (dot ~ 0.98, pow32 ~ 0.52) still passes.
    const Vec3 mild = umbreon::normalize(Vec3{0.2f, 0.0f, 1.0f});
    s.check("mild normal bend passes",
            sampleCoarseAo(g, 8, 8, 4, 4, mild, 10.0f, op, aov));
  }

  // --- 4. all-miss quad (background cells): lookup fails -> inline fallback.
  {
    CoarseAoGrid g = makeGrid(2, 2, 10.0f, {0.5f, 0.5f, 0.5f, 0.5f});
    g.hit.assign(4, 0);
    float op = -1;
    AOResult aov;
    s.check("all-miss quad fails",
            !sampleCoarseAo(g, 8, 8, 4, 4, up, 10.0f, op, aov));
  }

  // --- 5. quality channels: bent normal and contact/shape interpolate and
  // bent is renormalized; legacy grid (empty bent) leaves the AOV defaults.
  {
    CoarseAoGrid g = makeGrid(2, 2, 10.0f, {0.5f, 0.5f, 0.5f, 0.5f});
    const std::size_t n = 4;
    g.bent.assign(n * 3, 0.0f);
    for (std::size_t i = 0; i < n; ++i) g.bent[i * 3 + 2] = 1.0f;  // all +Z
    g.contact.assign(n, 0.25f);
    g.shape.assign(n, 0.75f);
    g.avgHitDist.assign(n, 2.0f);
    float op = -1;
    AOResult aov;
    s.check("quality lookup succeeds",
            sampleCoarseAo(g, 8, 8, 4, 4, up, 10.0f, op, aov));
    s.check("bent interpolated (+Z unit)",
            approx(aov.bent.z, 1.0f, 1e-5f) && approx(aov.bent.x, 0.0f, 1e-5f));
    s.check("contact/shape interpolated",
            approx(aov.contact, 0.25f, 1e-5f) && approx(aov.shape, 0.75f, 1e-5f));
    CoarseAoGrid legacy = makeGrid(2, 2, 10.0f, {0.5f, 0.5f, 0.5f, 0.5f});
    AOResult aov2;
    s.check("legacy grid lookup succeeds",
            sampleCoarseAo(legacy, 8, 8, 4, 4, up, 10.0f, op, aov2));
    s.check("legacy grid leaves AOV defaults",
            aov2.contact == 1.0f && aov2.shape == 1.0f &&
                aov2.bent.x == 0.0f && aov2.bent.z == 0.0f);
  }

  // --- 6. border clamp: lookups at the image corners stay in-bounds and
  // succeed on a fully eligible grid.
  {
    CoarseAoGrid g = makeGrid(3, 3, 10.0f, std::vector<float>(9, 0.5f));
    float op = -1;
    AOResult aov;
    bool ok = true;
    for (uint32_t p : {0u, 8u}) {
      ok = ok && sampleCoarseAo(g, 9, 9, p, 0, up, 10.0f, op, aov);
      ok = ok && sampleCoarseAo(g, 9, 9, 0, p, up, 10.0f, op, aov);
      ok = ok && sampleCoarseAo(g, 9, 9, p, p, up, 10.0f, op, aov);
    }
    s.check("corner lookups succeed in-bounds", ok);
  }

  // ------------------------------------------------------------------
  // Renderer integration (umbreon::render facade, coarse vs inline AO).
  // ------------------------------------------------------------------

  // --- 7. inline-path neutrality: aoResDiv 0 and 1 are both the inline path
  // and must be bitwise-equal, for the legacy and enhanced estimators.
  {
    const umbreon::Scene sc = makeAoScene();
    umbreon::RenderOptions o0;
    o0.width = 8;
    o0.height = 8;
    o0.supersample = 2;
    o0.aoSamples = 16;
    o0.aoDistance = 100.0f;
    umbreon::RenderOptions o1 = o0;
    o1.aoResDiv = 1;
    s.check("inline neutrality (legacy): div 0 == div 1 bitwise",
            framesEqual(umbreon::render(sc, o0), umbreon::render(sc, o1)));
    o0.aoMultiScale = true;
    o0.aoMultibounce = true;
    o1 = o0;
    o1.aoResDiv = 1;
    s.check("inline neutrality (enhanced): div 0 == div 1 bitwise",
            framesEqual(umbreon::render(sc, o0), umbreon::render(sc, o1)));
  }

  // --- 8. coarse-vs-inline closeness on the smooth AO scene ("out" sentinel
  // resolved to ss). Two regimes, matching the documented contract (effective
  // samples drop from ss^2*N box-averaged to N interpolated):
  // (a) high sample count + LD (the GTAO-recipe regime): both estimates are
  //     low-noise, so the images must agree within a small epsilon;
  // (b) moderate sample count: per-pixel MC noise dominates (sigma ~ 0.09 at
  //     32 spp), so only the MEAN absolute difference is bounded -- it checks
  //     for systematic bias without being noise-sensitive.
  {
    const umbreon::Scene sc = makeAoScene();
    umbreon::RenderOptions full;
    full.width = 16;
    full.height = 16;
    full.supersample = 3;
    full.aoSamples = 256;
    full.aoDistance = 100.0f;
    full.aoLowDiscrepancy = true;
    umbreon::RenderOptions coarse = full;
    coarse.aoResDiv = -1;  // "out" -> ss
    const auto a = umbreon::render(sc, full);
    const auto b = umbreon::render(sc, coarse);
    s.check("coarse vs inline within 0.05 (256 spp + LD)",
            framesClose(a, b, 0.05f));

    umbreon::RenderOptions full32 = full;
    full32.aoSamples = 32;
    full32.aoLowDiscrepancy = false;
    umbreon::RenderOptions coarse32 = full32;
    coarse32.aoResDiv = -1;
    const auto a32 = umbreon::render(sc, full32);
    const auto b32 = umbreon::render(sc, coarse32);
    double sum = 0.0;
    for (std::size_t i = 0; i < a32.color.size(); ++i)
      sum += std::fabs(a32.color[i] - b32.color[i]);
    const double mean = sum / static_cast<double>(a32.color.size());
    s.check("coarse vs inline mean |diff| < 0.02 (32 spp, no bias)",
            mean < 0.02);
  }

  // --- 9. fallback exactness: ss = 1 with an explicit div 2 grid on a scene
  // with a slab silhouette; patched first-hit pixels (mask == 1) must be
  // BITWISE-equal to the full inline render (same seeds, same estimator),
  // and the mask must fire somewhere but not everywhere.
  {
    const umbreon::Scene sc = makeAoScene();
    umbreon::RenderOptions full;
    full.width = 24;
    full.height = 24;
    full.aoSamples = 16;
    full.aoDistance = 100.0f;
    umbreon::RenderOptions coarse = full;
    coarse.aoResDiv = 2;  // explicit divisor (ss = 1, no downsample)
    coarse.aoResDebug = true;
    const auto a = umbreon::render(sc, full);
    const auto b = umbreon::render(sc, coarse);
    s.check_eq("patch mask allocated at render res", b.aoPatchMask.size(),
               static_cast<std::size_t>(24 * 24));
    int fired = 0;
    bool patchedBitwise = true;
    for (std::size_t p = 0; p < b.aoPatchMask.size(); ++p) {
      if (b.aoPatchMask[p] == 0.0f) continue;
      ++fired;
      for (int c = 0; c < 4; ++c)
        if (a.color[p * 4 + c] != b.color[p * 4 + c]) patchedBitwise = false;
    }
    s.check("patch mask fires somewhere",
            fired > 0 && fired < 24 * 24);
    s.check("patched pixels bitwise-equal the inline render", patchedBitwise);
  }

  // --- 10. determinism: coarse AO render is thread-count invariant.
  {
    const umbreon::Scene sc = makeAoScene();
    umbreon::RenderOptions o;
    o.width = 12;
    o.height = 12;
    o.supersample = 2;
    o.aoSamples = 16;
    o.aoDistance = 100.0f;
    o.aoResDiv = -1;
    umbreon::FrameResult a, b;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      a = umbreon::render(sc, o);
    }
    b = umbreon::render(sc, o);
    s.check("determinism: 1 thread == N threads (bitwise)", framesEqual(a, b));
  }

  // --- 11. GI fallback: --ao-res out with gi falls back to inline AO, so the
  // output is bitwise the gi render without the flag.
  {
    const umbreon::Scene sc = makeAoScene();
    umbreon::RenderOptions full;
    full.width = 8;
    full.height = 8;
    full.supersample = 2;
    full.aoSamples = 8;
    full.aoDistance = 100.0f;
    full.gi = true;
    umbreon::RenderOptions coarse = full;
    coarse.aoResDiv = -1;
    s.check("gi fallback: out+gi == full+gi (bitwise)",
            framesEqual(umbreon::render(sc, full),
                        umbreon::render(sc, coarse)));
  }

  // --- 12. adaptive-AA composition: --aa adaptive + --ao-res out runs,
  // is deterministic, and stays close to the grid + coarse render.
  {
    const umbreon::Scene sc = makeAoScene();
    umbreon::RenderOptions gridOpt;
    gridOpt.width = 12;
    gridOpt.height = 12;
    gridOpt.supersample = 3;
    gridOpt.aoSamples = 16;
    gridOpt.aoDistance = 100.0f;
    gridOpt.aoResDiv = -1;
    umbreon::RenderOptions adap = gridOpt;
    adap.aaMode = 1;
    const auto g = umbreon::render(sc, gridOpt);
    const auto a1 = umbreon::render(sc, adap);
    const auto a2 = umbreon::render(sc, adap);
    s.check("adaptive+coarse: deterministic", framesEqual(a1, a2));
    s.check("adaptive+coarse close to grid+coarse", framesClose(g, a1, 0.05f));
  }

  // --- 13. transparency: a half-transparent shell over the AO floor -- the
  // shell's front hits interpolate, the floor hits behind it fall back
  // inline (depth guide rejects); the whole image stays close to the full
  // inline render.
  {
    umbreon::Scene sc = makeAoScene();
    const float z = 1.0f;
    const Vec3 sh[6] = {{-2, -2, z}, {2, -2, z}, {2, 2, z},
                        {-2, -2, z}, {2, 2, z},  {-2, 2, z}};
    for (int i = 0; i < 6; ++i) {
      sc.mesh.positions.push_back(sh[i]);
      sc.mesh.normals.push_back({0, 0, 1});
      sc.mesh.colors.push_back({0.5f, 0.7f, 0.9f, 0.4f});  // 40% opacity
    }
    umbreon::RenderOptions full;
    full.width = 12;
    full.height = 12;
    full.supersample = 2;
    full.aoSamples = 16;
    full.aoDistance = 100.0f;
    umbreon::RenderOptions coarse = full;
    coarse.aoResDiv = -1;
    const auto a = umbreon::render(sc, full);
    const auto b = umbreon::render(sc, coarse);
    s.check("transparency: coarse within 0.05 of inline",
            framesClose(a, b, 0.05f));
  }

  return s.report();
}
