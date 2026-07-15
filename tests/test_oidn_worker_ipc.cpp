// Out-of-process OIDN round trip against the real umbreon_oidn_worker binary
// (argv[1], passed by CMake as $<TARGET_FILE:umbreon_oidn_worker>). Denoises a
// synthetic noisy frame and checks the observable contract: flat-region noise
// variance drops, background (zero-normal) pixels stay byte-identical, the
// output is finite, it differs from the a-trous result (i.e. the worker
// really ran -- a silent fallback would produce the a-trous image), and a
// repeat run through the persistent worker is deterministic. Frames of other
// sizes exercise the shared-memory grow (recreate) and shrink (reuse) paths.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "experimental/irradiance_cache/denoise.hpp"
#include "test_util.hpp"

namespace {

// Dark-left / bright-right flat halves with deterministic noise, a normal
// discontinuity on the seam, flat per-half albedo (clean OIDN guides) and a
// background top strip (zero normal, sentinel color).
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
        const float bg = 0.123f;
        f.color[p * 4 + 0] = f.color[p * 4 + 1] = f.color[p * 4 + 2] = bg;
        f.color[p * 4 + 3] = 1.0f;
        continue;
      }
      const bool left = x < W / 2;
      f.normal[p * 3 + 0] = left ? 0.0f : 1.0f;
      f.normal[p * 3 + 2] = left ? 1.0f : 0.0f;
      const float base = left ? 0.3f : 0.8f;
      f.albedo[p * 3 + 0] = f.albedo[p * 3 + 1] = f.albedo[p * 3 + 2] = base;
      const float v = base + 0.08f * noiseAt(p);
      f.color[p * 4 + 0] = f.color[p * 4 + 1] = f.color[p * 4 + 2] = v;
      f.color[p * 4 + 3] = 1.0f;
    }
  return f;
}

// mean/variance of luminance over a flat interior window.
std::pair<double, double> lumStats(const std::vector<float>& c, int W,
                                   int bgRows, int H, int x0, int x1) {
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

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: test_oidn_worker_ipc <worker-path>\n");
    return 1;
  }
  umbreon::test::Suite s("oidn_worker_ipc");

  umbreon::RenderOptions opt;
  opt.denoiser = 2;  // DenoiserBackend::OIDN
  opt.denoiseIters = 5;
  opt.denoiseDemodulateAlbedo = false;
  opt.oidnCleanAux = true;
  opt.oidnWorkerPath = argv[1];

  const int W = 64, H = 48, bgRows = 6;
  const umbreon::FrameResult base = makeNoisyFrame(W, H, bgRows);
  const auto lBefore = lumStats(base.color, W, bgRows, H, 4, W / 2 - 4);
  const auto rBefore = lumStats(base.color, W, bgRows, H, W / 2 + 4, W - 4);

  umbreon::FrameResult f1 = base;
  umbreon::denoiseOidn(f1, opt);

  const auto lAfter = lumStats(f1.color, W, bgRows, H, 4, W / 2 - 4);
  const auto rAfter = lumStats(f1.color, W, bgRows, H, W / 2 + 4, W - 4);
  s.check("flat-region variance reduced",
          lAfter.second < 0.5 * lBefore.second &&
              rAfter.second < 0.5 * rBefore.second);
  s.check("background pixels untouched", bgUntouched(f1, base, bgRows));
  s.check("output is finite", allFinite(f1.color));

  // Distinguish a real worker run from a silent a-trous fallback: the two
  // denoisers cannot produce the same image on this input.
  umbreon::FrameResult viaAtrous = base;
  umbreon::denoiseAtrous(viaAtrous, opt);
  s.check("output differs from the a-trous fallback",
          f1.color != viaAtrous.color);

  // Persistent-worker reuse must be deterministic: a second identical request
  // through the same (already running) worker gives the identical image.
  umbreon::FrameResult f2 = base;
  umbreon::denoiseOidn(f2, opt);
  s.check("repeat run is bitwise deterministic", f1.color == f2.color);

  // Larger frame: outgrows the shared region => recreate under a new name.
  {
    const umbreon::FrameResult big = makeNoisyFrame(96, 64, 6);
    umbreon::FrameResult g = big;
    umbreon::denoiseOidn(g, opt);
    s.check("larger frame (shm recreate): finite and background untouched",
            allFinite(g.color) && bgUntouched(g, big, 6));
    s.check("larger frame: worker ran (output changed)", g.color != big.color);
  }
  // Smaller frame afterwards: fits the enlarged region => reuse path.
  {
    const umbreon::FrameResult small = makeNoisyFrame(32, 24, 4);
    umbreon::FrameResult g = small;
    umbreon::denoiseOidn(g, opt);
    s.check("smaller frame (shm reuse): finite and background untouched",
            allFinite(g.color) && bgUntouched(g, small, 4));
    s.check("smaller frame: worker ran (output changed)", g.color != small.color);
  }

  return s.report();
}
