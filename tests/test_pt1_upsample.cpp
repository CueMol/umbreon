// pt1 joint bilateral upsample tests (plan Phase 5): constant field
// round-trip, no bleeding across a depth edge, and the nearest-valid fallback
// when every guide weight dies.
#include <cmath>
#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

#include "experimental/pt1/pt1_upsample.hpp"

namespace {

using umbreon::detail::Pt1GBuffer;

// A synthetic half G-buffer: all pixels hit, normal +z, depth `depth`.
Pt1GBuffer makeHalf(int w, int h, float depth) {
  Pt1GBuffer g;
  g.w = w;
  g.h = h;
  const std::size_t n = static_cast<std::size_t>(w) * h;
  g.position.assign(n * 3, 0.0f);
  g.normal.assign(n * 3, 0.0f);
  g.geomNormal.assign(n * 3, 0.0f);
  g.albedo.assign(n * 3, 0.5f);
  g.depth.assign(n, depth);
  g.hit.assign(n, 1);
  for (std::size_t p = 0; p < n; ++p) {
    g.normal[p * 3 + 2] = 1.0f;
    g.geomNormal[p * 3 + 2] = 1.0f;
  }
  return g;
}

bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

}  // namespace

int main() {
  umbreon::test::Suite s("pt1_upsample");
  const float kPow = 32.0f, kZScale = 0.02f;

  // --- 1. constant half field -> constant full field.
  {
    const int W = 8, H = 8, hw = 4, hh = 4;
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    Pt1GBuffer g = makeHalf(hw, hh, 5.0f);
    std::vector<float> Eh(static_cast<std::size_t>(hw) * hh * 3, 0.75f);
    std::vector<float> occh(static_cast<std::size_t>(hw) * hh, 0.25f);
    std::vector<float> fullN(npix * 3, 0.0f), fullZ(npix, 5.0f);
    std::vector<uint8_t> eligible(npix, 1);
    for (std::size_t p = 0; p < npix; ++p) fullN[p * 3 + 2] = 1.0f;

    std::vector<float> E, occ;
    umbreon::detail::upsampleJointBilateral(W, H, fullN.data(), fullZ.data(),
                                            eligible.data(), g, Eh, occh, kPow,
                                            kZScale, E, occ);
    bool ok = true;
    for (std::size_t i = 0; i < npix * 3; ++i)
      ok = ok && approx(E[i], 0.75f, 1e-5f);
    for (std::size_t p = 0; p < npix; ++p)
      ok = ok && approx(occ[p], 0.25f, 1e-5f);
    s.check("constant half field upsamples to the same constant", ok);
  }

  // --- 2. depth edge: left half depth 1 / E = 1, right half depth 10 / E = 0.
  // The depth weight kills cross-edge neighbors, so full pixels next to the
  // edge keep their side's value (bilinear alone would blend ~25% across).
  {
    const int W = 8, H = 4, hw = 4, hh = 2;
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    const std::size_t hn = static_cast<std::size_t>(hw) * hh;
    Pt1GBuffer g = makeHalf(hw, hh, 1.0f);
    std::vector<float> Eh(hn * 3, 0.0f), occh(hn, 0.0f);
    for (int hy = 0; hy < hh; ++hy)
      for (int hx = 0; hx < hw; ++hx) {
        const std::size_t p = static_cast<std::size_t>(hy) * hw + hx;
        const bool left = hx < hw / 2;
        g.depth[p] = left ? 1.0f : 10.0f;
        for (int c = 0; c < 3; ++c) Eh[p * 3 + c] = left ? 1.0f : 0.0f;
      }
    std::vector<float> fullN(npix * 3, 0.0f), fullZ(npix, 0.0f);
    std::vector<uint8_t> eligible(npix, 1);
    for (int py = 0; py < H; ++py)
      for (int px = 0; px < W; ++px) {
        const std::size_t p = static_cast<std::size_t>(py) * W + px;
        fullN[p * 3 + 2] = 1.0f;
        fullZ[p] = (px < W / 2) ? 1.0f : 10.0f;
      }

    std::vector<float> E, occ;
    umbreon::detail::upsampleJointBilateral(W, H, fullN.data(), fullZ.data(),
                                            eligible.data(), g, Eh, occh, kPow,
                                            kZScale, E, occ);
    // Columns adjacent to the edge (px = 3 on the near side, px = 4 beyond).
    bool ok = true;
    for (int py = 0; py < H; ++py) {
      const std::size_t pl = static_cast<std::size_t>(py) * W + 3;
      const std::size_t pr = static_cast<std::size_t>(py) * W + 4;
      ok = ok && approx(E[pl * 3 + 0], 1.0f, 1e-3f) &&
           approx(E[pr * 3 + 0], 0.0f, 1e-3f);
    }
    s.check("depth edge: no irradiance bleed across the discontinuity", ok);
  }

  // --- 3. fallback: full normal orthogonal to every half normal kills all
  // weights (w_n = 0), so the nearest valid half pixel is taken verbatim.
  {
    const int W = 4, H = 4, hw = 2, hh = 2;
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    Pt1GBuffer g = makeHalf(hw, hh, 1.0f);
    std::vector<float> Eh(static_cast<std::size_t>(hw) * hh * 3, 0.7f);
    std::vector<float> occh(static_cast<std::size_t>(hw) * hh, 0.4f);
    std::vector<float> fullN(npix * 3, 0.0f), fullZ(npix, 1.0f);
    std::vector<uint8_t> eligible(npix, 1);
    for (std::size_t p = 0; p < npix; ++p) fullN[p * 3 + 0] = 1.0f;  // +x

    std::vector<float> E, occ;
    umbreon::detail::upsampleJointBilateral(W, H, fullN.data(), fullZ.data(),
                                            eligible.data(), g, Eh, occh, kPow,
                                            kZScale, E, occ);
    bool ok = true;
    for (std::size_t p = 0; p < npix; ++p)
      ok = ok && approx(E[p * 3 + 0], 0.7f, 1e-6f) &&
           approx(occ[p], 0.4f, 1e-6f);
    s.check("all-weights-dead: nearest valid half pixel taken verbatim", ok);

    // And with no valid half pixel at all (hit = 0 everywhere) the output
    // must stay zero (hole accepted, not garbage).
    g.hit.assign(g.hit.size(), 0);
    umbreon::detail::upsampleJointBilateral(W, H, fullN.data(), fullZ.data(),
                                            eligible.data(), g, Eh, occh, kPow,
                                            kZScale, E, occ);
    ok = true;
    for (std::size_t i = 0; i < npix * 3; ++i) ok = ok && E[i] == 0.0f;
    s.check("no valid half pixels: output stays zero", ok);
  }

  return s.report();
}
