// Intel Open Image Denoise backend. Compiled into libumbreon only when the build
// located OpenImageDenoise (CMake: UMBREON_WITH_OIDN=ON => UMBREON_HAVE_OIDN). The
// pipeline guards the call with the same macro, so this translation unit is absent
// from the link when OIDN is unavailable.
#include "experimental/irradiance_cache/denoise.hpp"

#ifdef UMBREON_HAVE_OIDN

#include <cstddef>
#include <cstdio>
#include <vector>

#include <OpenImageDenoise/oidn.hpp>

namespace umbreon {

// Denoise frame.color (final-resolution linear HDR RGBA) with the OIDN "RT"
// filter. color is deinterleaved to a contiguous RGB scratch buffer (OIDN wants
// Float3), denoised with albedo/normal as auxiliary guides, and written back to
// the geometry pixels only -- the background (no normal) keeps its flat color so
// the surface/background boundary is not blurred.
void denoiseOidn(FrameResult& frame, const RenderOptions& opt) {
  const int W = frame.width;
  const int H = frame.height;
  const std::size_t N = static_cast<std::size_t>(W) * H;
  if (W <= 0 || H <= 0 || frame.color.size() < N * 4) return;

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

  oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
  device.commit();
  if (device.getError() != oidn::Error::None) {
    std::fprintf(stderr, "warning: OIDN device init failed; skipping denoise\n");
    return;
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
  filter.commit();
  filter.execute();

  const char* msg = nullptr;
  if (device.getError(msg) != oidn::Error::None) {
    std::fprintf(stderr, "warning: OIDN execute failed (%s); skipping denoise\n",
                 msg ? msg : "unknown");
    return;
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
}

}  // namespace umbreon

#endif  // UMBREON_HAVE_OIDN
