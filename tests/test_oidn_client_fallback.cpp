// OIDN IPC client fallback contract: with an explicit worker path that cannot
// be spawned, denoiseOidn must degrade to EXACTLY the built-in a-trous result
// for the same options (the fallback is denoiseAtrous(frame, opt), which is
// deterministic). An explicit RenderOptions::oidnWorkerPath is authoritative
// (no env/PATH fall-through), so pointing it at a nonexistent binary makes
// the spawn failure deterministic regardless of the test environment.
// Registered only when the build compiles the IPC client (UMBREON_OIDN_CLIENT).
#include <cstdint>
#include <cstdio>
#include <vector>

#include "experimental/irradiance_cache/denoise.hpp"
#include "test_util.hpp"

namespace {

// Synthetic noisy frame: dark-left / bright-right flat halves with
// deterministic per-pixel noise, a normal discontinuity on the seam, and a
// background top strip (zero normal). Mirrors the denoiseAtrous test scene in
// test_render_shadows_gi.cpp.
umbreon::FrameResult makeNoisyFrame(int W, int H, int bgRows) {
  const std::size_t N = static_cast<std::size_t>(W) * H;
  umbreon::FrameResult f;
  f.width = W;
  f.height = H;
  f.color.assign(N * 4, 0.0f);
  f.normal.assign(N * 3, 0.0f);
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
      const float v = base + 0.08f * noiseAt(p);
      f.color[p * 4 + 0] = f.color[p * 4 + 1] = f.color[p * 4 + 2] = v;
      f.color[p * 4 + 3] = 1.0f;
    }
  return f;
}

}  // namespace

int main() {
  umbreon::test::Suite s("oidn_client_fallback");

  umbreon::RenderOptions opt;
  opt.denoiser = 2;  // DenoiserBackend::OIDN
  opt.denoiseIters = 5;
  opt.denoiseDemodulateAlbedo = false;  // no albedo buffer in this frame
  opt.oidnWorkerPath = "/nonexistent/umbreon_oidn_worker_bogus";

  const umbreon::FrameResult base = makeNoisyFrame(24, 24, 4);

  // The spawn failure path (prints one warning) must fall back to a-trous.
  umbreon::FrameResult viaOidn = base;
  umbreon::denoiseOidn(viaOidn, opt);

  umbreon::FrameResult viaAtrous = base;
  umbreon::denoiseAtrous(viaAtrous, opt);

  s.check("fallback output == denoiseAtrous output (bitwise)",
          viaOidn.color == viaAtrous.color);
  s.check("fallback actually changed the noisy input",
          viaOidn.color != base.color);

  // Second call: the client is now disabled for this path and must stay on
  // the identical a-trous fallback (no respawn storm, no divergence).
  umbreon::FrameResult again = base;
  umbreon::denoiseOidn(again, opt);
  s.check("second call falls back identically",
          again.color == viaAtrous.color);

  return s.report();
}
