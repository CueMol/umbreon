// Intel Open Image Denoise backend, out of process. libumbreon does not link
// OIDN: this translation unit drives the umbreon_oidn_worker executable
// through the IPC client (ipc/oidn_client.hpp) -- buffers move through shared
// memory, so the copy count matches the old in-process implementation
// (deinterleave in, masked write-back out). Compiled into the umbreon target
// only when the build enables the backend (CMake: UMBREON_WITH_OIDN + Boost
// found => UMBREON_HAVE_OIDN); the pipeline guards the call with the same
// macro.
#include "experimental/irradiance_cache/denoise.hpp"

#ifdef UMBREON_HAVE_OIDN

#include <cstddef>
#include <cstdio>
#include <cstring>

#include "ipc/oidn_client.hpp"

namespace umbreon {

// Denoise frame.color (final-resolution linear HDR RGBA) with the OIDN "RT"
// filter in the worker process. color is deinterleaved into the shared
// memory region (OIDN wants Float3), denoised in place with albedo/normal as
// auxiliary guides, and written back to the geometry pixels only -- the
// background (no normal) keeps its flat color so the surface/background
// boundary is not blurred. Any worker failure falls back to the built-in
// a-trous denoiser (the IPC client prints the detailed reason).
//
// pt1Stats timing parity note: the "device" figure is the worker's one-time
// OIDN device init, so it is real on the first denoise of the process and
// 0.000 afterwards (the persistent worker reuses the device).
bool denoiseOidn(FrameResult& frame, const RenderOptions& opt) {
  const int W = frame.width;
  const int H = frame.height;
  const std::size_t N = static_cast<std::size_t>(W) * H;
  // Degenerate frame: nothing to denoise (unreachable via render(); defensive).
  if (W <= 0 || H <= 0 || frame.color.size() < N * 4) return false;

  const bool haveNormal = frame.normal.size() == N * 3;
  const bool haveAlbedo = frame.albedo.size() == N * 3;

  bool ok = false;
  {
    ipc::OidnClient::Session s = ipc::OidnClient::instance().begin(
        W, H, haveAlbedo, haveNormal, opt.oidnWorkerPath);
    if (s.valid()) {
      // Deinterleave RGBA -> RGB straight into the shared region.
      float* rgb = s.color();
      for (std::size_t p = 0; p < N; ++p) {
        rgb[p * 3 + 0] = frame.color[p * 4 + 0];
        rgb[p * 3 + 1] = frame.color[p * 4 + 1];
        rgb[p * 3 + 2] = frame.color[p * 4 + 2];
      }
      if (haveAlbedo)
        std::memcpy(s.albedo(), frame.albedo.data(), N * 3 * sizeof(float));
      if (haveNormal)
        std::memcpy(s.normal(), frame.normal.data(), N * 3 * sizeof(float));

      if (s.run(opt.oidnCleanAux, opt.pt1Stats)) {
        // Write back geometry pixels only; leave the background untouched.
        const float* out = s.color();
        for (std::size_t p = 0; p < N; ++p) {
          if (haveNormal) {
            const float nx = frame.normal[p * 3 + 0],
                        ny = frame.normal[p * 3 + 1],
                        nz = frame.normal[p * 3 + 2];
            if ((nx * nx + ny * ny + nz * nz) <= 0.25f) continue;  // background
          }
          frame.color[p * 4 + 0] = out[p * 3 + 0];
          frame.color[p * 4 + 1] = out[p * 3 + 1];
          frame.color[p * 4 + 2] = out[p * 3 + 2];
        }
        ok = true;
      }
    }
  }
  if (!ok) {
    std::fprintf(stderr,
                 "warning: OIDN denoise unavailable; using the built-in "
                 "a-trous denoiser\n");
    denoiseAtrous(frame, opt);
  }
  return ok;
}

}  // namespace umbreon

#endif  // UMBREON_HAVE_OIDN
