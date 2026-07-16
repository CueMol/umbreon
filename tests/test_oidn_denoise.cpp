// In-process OIDN denoiser tests (registered only when UMBREON_WITH_OIDN built
// the backend). Exercises the observable contract of denoiseOidn on a small
// synthetic frame -- variance reduction, background preservation, finiteness,
// a real-OIDN-vs-a-trous distinction, the bool return, the maxMemoryMB
// bit-invariance at tiling-free sizes -- plus the render() facade reporting
// FrameResult::denoiserUsed == 2. denoiseOidn is an internal-header symbol.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "experimental/irradiance_cache/denoise.hpp"
#include "render_test_util.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

// Dark-left / bright-right flat halves with deterministic per-pixel noise, a
// normal discontinuity on the seam, flat per-half albedo (clean OIDN guides),
// and a background top strip (zero normal, sentinel color).
umbreon::FrameResult makeNoisyFrame(int W, int H, int bgRows) {
  const std::size_t N = static_cast<std::size_t>(W) * H;
  umbreon::FrameResult f;
  f.width = W;
  f.height = H;
  f.color.assign(N * 4, 0.0f);
  f.normal.assign(N * 3, 0.0f);
  f.albedo.assign(N * 3, 0.0f);
  auto noiseAt = [](std::size_t i) {
    std::uint32_t h = static_cast<std::uint32_t>(i) * 1103515245u + 12345u;
    return (static_cast<float>((h >> 16) & 0x7fffu) / 32768.0f) - 0.5f;
  };
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      const std::size_t p = static_cast<std::size_t>(y) * W + x;
      if (y < bgRows) {
        f.color[p * 4 + 0] = f.color[p * 4 + 1] = f.color[p * 4 + 2] = 0.123f;
        f.color[p * 4 + 3] = 1.0f;
        continue;
      }
      const bool left = x < W / 2;
      f.normal[p * 3 + 0] = left ? 0.0f : 1.0f;
      f.normal[p * 3 + 2] = left ? 1.0f : 0.0f;
      const float base = left ? 0.3f : 0.8f;
      f.albedo[p * 3 + 0] = f.albedo[p * 3 + 1] = f.albedo[p * 3 + 2] = base;
      f.color[p * 4 + 0] = f.color[p * 4 + 1] = f.color[p * 4 + 2] =
          base + 0.08f * noiseAt(p);
      f.color[p * 4 + 3] = 1.0f;
    }
  return f;
}

std::pair<double, double> lumVar(const std::vector<float>& c, int W, int H,
                                 int bgRows, int x0, int x1) {
  double s1 = 0.0, s2 = 0.0;
  int n = 0;
  for (int y = bgRows + 2; y < H - 2; ++y)
    for (int x = x0; x < x1; ++x) {
      const std::size_t p = static_cast<std::size_t>(y) * W + x;
      const float l = 0.2126f * c[p * 4 + 0] + 0.7152f * c[p * 4 + 1] +
                      0.0722f * c[p * 4 + 2];
      s1 += l;
      s2 += l * l;
      ++n;
    }
  const double mean = s1 / n;
  return {mean, s2 / n - mean * mean};
}

bool allFinite(const std::vector<float>& v) {
  for (float x : v)
    if (!std::isfinite(x)) return false;
  return true;
}

bool bgUntouched(const umbreon::FrameResult& got,
                 const umbreon::FrameResult& base, int bgRows) {
  for (int y = 0; y < bgRows; ++y)
    for (int x = 0; x < got.width; ++x) {
      const std::size_t p = static_cast<std::size_t>(y) * got.width + x;
      for (int c = 0; c < 4; ++c)
        if (got.color[p * 4 + c] != base.color[p * 4 + c]) return false;
    }
  return true;
}

}  // namespace

int main() {
  umbreon::test::Suite s("oidn_denoise");

  const int W = 64, H = 48, bgRows = 6;
  const umbreon::FrameResult base = makeNoisyFrame(W, H, bgRows);
  const auto lBefore = lumVar(base.color, W, H, bgRows, 4, W / 2 - 4);
  const auto rBefore = lumVar(base.color, W, H, bgRows, W / 2 + 4, W - 4);

  umbreon::RenderOptions opt;
  opt.denoiseDemodulateAlbedo = false;
  opt.oidnCleanAux = true;
  opt.oidnMaxMemoryMB = 1024;  // default; tiling-free at this size

  umbreon::FrameResult f = base;
  const bool ran = umbreon::denoiseOidn(f, opt);
  s.check("denoiseOidn returns true (OIDN ran)", ran);

  const auto lAfter = lumVar(f.color, W, H, bgRows, 4, W / 2 - 4);
  const auto rAfter = lumVar(f.color, W, H, bgRows, W / 2 + 4, W - 4);
  s.check("flat-region variance reduced",
          lAfter.second < 0.5 * lBefore.second &&
              rAfter.second < 0.5 * rBefore.second);
  s.check("background pixels untouched", bgUntouched(f, base, bgRows));
  s.check("output is finite", allFinite(f.color));

  // OIDN really ran (not a silent a-trous fallback): the two denoisers cannot
  // produce the same image on this input.
  umbreon::FrameResult viaAtrous = base;
  umbreon::denoiseAtrous(viaAtrous, opt);
  s.check("output differs from the a-trous result", f.color != viaAtrous.color);

  // maxMemoryMB invariance at a tiling-free size: the default cap (1024) must
  // give bit-identical output to the uncapped default (-1). This locks the
  // refactor_check assumption that small frames do not tile.
  umbreon::FrameResult fCap = base, fUncap = base;
  umbreon::RenderOptions optUncap = opt;
  optUncap.oidnMaxMemoryMB = -1;
  umbreon::denoiseOidn(fCap, opt);
  umbreon::denoiseOidn(fUncap, optUncap);
  s.check("maxMemoryMB=1024 bit-identical to uncapped at tiling-free size",
          fCap.color == fUncap.color);

  // render() facade: a GI+pt1 render with final-color OIDN reports 2 for both
  // denoise stages.
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad({0.7f, 0.7f, 0.7f, 1.0f});
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0.0f, 0.0f, 0.0f};
    umbreon::RenderOptions o;
    o.width = 32;
    o.height = 24;
    o.gi = true;
    o.giIntegrator = 1;  // pt1
    o.pt1Spp = 4;
    o.pt1Denoise = true;  // pt1 E-buffer denoise via OIDN
    o.denoiser = 2;       // final-color OIDN
    const umbreon::FrameResult rf = umbreon::render(sc, o);
    s.check("render: denoiserUsed == 2 (OIDN)", rf.denoiserUsed == 2);
    s.check("render: pt1DenoiserUsed == 2 (OIDN)", rf.pt1DenoiserUsed == 2);
  }

  return s.report();
}
