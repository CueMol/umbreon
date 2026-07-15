// Worker-death recovery contract (POSIX only; registered on non-Windows).
// SIGKILLs the live worker between denoises and checks the client's failure
// matrix: the request in flight when the worker is found dead falls back to
// exactly the a-trous result, and the NEXT denoise respawns a fresh worker
// and produces the same image as the first successful run. The kill targets
// only children of this test process (pkill -P), so parallel builds or an
// unrelated host application are never hit.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "experimental/irradiance_cache/denoise.hpp"
#include "test_util.hpp"

namespace {

// Same synthetic scene as test_oidn_worker_ipc: flat noisy halves, normal
// seam, clean albedo guide, background top strip.
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: test_oidn_worker_respawn <worker-path>\n");
    return 1;
  }
  umbreon::test::Suite s("oidn_worker_respawn");

  umbreon::RenderOptions opt;
  opt.denoiser = 2;  // DenoiserBackend::OIDN
  opt.denoiseIters = 5;
  opt.denoiseDemodulateAlbedo = false;
  opt.oidnWorkerPath = argv[1];

  const umbreon::FrameResult base = makeNoisyFrame(64, 48, 6);
  umbreon::FrameResult viaAtrous = base;
  umbreon::denoiseAtrous(viaAtrous, opt);

  // 1) Healthy run: spawns the worker.
  umbreon::FrameResult first = base;
  umbreon::denoiseOidn(first, opt);
  s.check("initial run used the worker", first.color != viaAtrous.color);

  // 2) Kill the worker (only our own child) and let the signal land. -f
  // matches the full command line: on Linux the comm name is truncated to 15
  // characters, so a plain name match would miss "umbreon_oidn_worker".
  const std::string kill =
      "pkill -9 -P " + std::to_string(getpid()) + " -f umbreon_oidn_worker";
  s.check("worker process found and killed", std::system(kill.c_str()) == 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 3) The next request finds the worker dead: warning + exact a-trous image.
  umbreon::FrameResult during = base;
  umbreon::denoiseOidn(during, opt);
  s.check("request against the dead worker falls back to a-trous (bitwise)",
          during.color == viaAtrous.color);

  // 4) The denoise after that respawns and matches the first worker output.
  umbreon::FrameResult after = base;
  umbreon::denoiseOidn(after, opt);
  s.check("next run respawned the worker (matches first output)",
          after.color == first.color);

  return s.report();
}
