// Intel Open Image Denoise backend. Compiled into libumbreon only when the build
// located OpenImageDenoise (CMake: UMBREON_WITH_OIDN=ON => UMBREON_HAVE_OIDN). The
// pipeline guards the call with the same macro, so this translation unit is absent
// from the link when OIDN is unavailable.
#include "experimental/irradiance_cache/denoise.hpp"

#ifdef UMBREON_HAVE_OIDN

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <OpenImageDenoise/oidn.hpp>

namespace umbreon {
namespace {

// OIDN's progress monitor is a plain C callback, so the slice travels through
// userPtr rather than a capture. Returning false cancels filter.execute().
bool oidnProgressCb(void* userPtr, double n) {
  const detail::ProgressSlice* s =
      static_cast<const detail::ProgressSlice*>(userPtr);
  if (!s || !s->progress) return true;
  if (s->progress->cancelRequested()) return false;
  // n is an ABSOLUTE fraction and may arrive out of order from OIDN's worker
  // threads; reportAbs() seeks monotonically, so a stale value is ignored.
  s->reportAbs(n);
  return true;
}

}  // namespace

// Denoise frame.color (final-resolution linear HDR RGBA) with the OIDN "RT"
// filter. color is deinterleaved to a contiguous RGB scratch buffer (OIDN wants
// Float3), denoised with albedo/normal as auxiliary guides, and written back to
// the geometry pixels only -- the background (no normal) keeps its flat color so
// the surface/background boundary is not blurred. Returns true when OIDN ran;
// false on a device/execute error (frame left untouched). The a-trous fallback
// on failure is the caller's responsibility (pipeline.cpp / pt1_denoise.cpp),
// which also records FrameResult::denoiserUsed from this return value.
bool denoiseOidn(FrameResult& frame, const RenderOptions& opt,
                 const detail::ProgressSlice* prog) {
  const int W = frame.width;
  const int H = frame.height;
  const std::size_t N = static_cast<std::size_t>(W) * H;
  if (W <= 0 || H <= 0 || frame.color.size() < N * 4) return false;

  const bool haveNormal = frame.normal.size() == N * 3;
  const bool haveAlbedo = frame.albedo.size() == N * 3;

  // Deinterleave RGBA -> RGB scratch (OIDN color/output are Float3).
  std::vector<float> rgb(N * 3);
  for (std::size_t p = 0; p < N; ++p) {
    rgb[p * 3 + 0] = frame.color[p * 4 + 0];
    rgb[p * 3 + 1] = frame.color[p * 4 + 1];
    rgb[p * 3 + 2] = frame.color[p * 4 + 2];
  }
  std::vector<float> out(N * 3);

  using clock = std::chrono::high_resolution_clock;
  const auto tDev0 = clock::now();
  oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
  device.commit();
  const auto tDev1 = clock::now();
  if (device.getError() != oidn::Error::None) {
    std::fprintf(stderr, "warning: OIDN device init failed; skipping denoise\n");
    return false;
  }

  oidn::FilterRef filter = device.newFilter("RT");
  filter.setImage("color", rgb.data(), oidn::Format::Float3, W, H);
  if (haveAlbedo)
    filter.setImage("albedo", frame.albedo.data(), oidn::Format::Float3, W, H);
  if (haveNormal)
    filter.setImage("normal", frame.normal.data(), oidn::Format::Float3, W, H);
  filter.setImage("output", out.data(), oidn::Format::Float3, W, H);
  filter.set("hdr", true);
  // primary-hit albedo/normal are essentially noise-free, so OIDN can use them
  // directly (no aux prefilter). Disable only if the guides are known noisy.
  if (haveAlbedo || haveNormal) filter.set("cleanAux", opt.oidnCleanAux);
  // Cap the scratch arena; uncapped, OIDN allocates a single multi-GiB block
  // at high resolutions (measured 2.5 GiB at 13 MP), which aborts hosts whose
  // allocator rejects huge single allocations (Electron PartitionAlloc,
  // ~2 GiB). OIDN implements the cap as internal overlapping tiling.
  if (opt.oidnMaxMemoryMB >= 0) filter.set("maxMemoryMB", opt.oidnMaxMemoryMB);
  // Installed BEFORE commit. Without it the bar sits frozen through the whole
  // execute() below, which is the single largest slice of a low-spp GI render.
  if (prog && prog->progress)
    filter.setProgressMonitorFunction(
        oidnProgressCb, const_cast<detail::ProgressSlice*>(prog));
  filter.commit();
  const auto tFil1 = clock::now();
  filter.execute();
  const auto tExe1 = clock::now();
  if (opt.pt1Stats)
    std::fprintf(stderr,
                 "oidn: device %.3fs  filter %.3fs  execute %.3fs (%dx%d)\n",
                 std::chrono::duration<double>(tDev1 - tDev0).count(),
                 std::chrono::duration<double>(tFil1 - tDev1).count(),
                 std::chrono::duration<double>(tExe1 - tFil1).count(), W, H);

  const char* msg = nullptr;
  const oidn::Error err = device.getError(msg);
  if (err != oidn::Error::None) {
    // A cancelled filter is not a failure -- the monitor returned false because
    // the UI asked the render to stop. Report it silently; the frame is left
    // untouched either way, and the caller must not run a fallback denoise.
    if (err != oidn::Error::Cancelled)
      std::fprintf(stderr,
                   "warning: OIDN execute failed (%s); skipping denoise\n",
                   msg ? msg : "unknown");
    return false;
  }

  // Write back geometry pixels only; leave the background color untouched.
  for (std::size_t p = 0; p < N; ++p) {
    if (haveNormal) {
      const float nx = frame.normal[p * 3 + 0], ny = frame.normal[p * 3 + 1],
                  nz = frame.normal[p * 3 + 2];
      if ((nx * nx + ny * ny + nz * nz) <= 0.25f) continue;  // background
    }
    frame.color[p * 4 + 0] = out[p * 3 + 0];
    frame.color[p * 4 + 1] = out[p * 3 + 1];
    frame.color[p * 4 + 2] = out[p * 3 + 2];
  }
  return true;
}

}  // namespace umbreon

#endif  // UMBREON_HAVE_OIDN
