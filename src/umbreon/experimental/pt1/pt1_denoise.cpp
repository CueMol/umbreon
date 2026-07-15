// pt1 indirect-irradiance denoise: OIDN (preferred) or the built-in a-trous
// fallback applied to the E buffer BEFORE the albedo multiply / composite --
// the direct lighting and albedo are noise-free and must not be smoothed, and
// denoising pre-multiply doubles as albedo demodulation. Compiled into the
// umbreon target (NOT header-inline) so UMBREON_HAVE_OIDN is evaluated exactly
// once: an inline definition would give library and test translation units
// different bodies (ODR violation) since the macro is target-private.
#include "experimental/pt1/pt1_integrator.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

#include "experimental/irradiance_cache/denoise.hpp"

namespace umbreon {
namespace detail {

int denoisePt1E(int w, int h, std::vector<float>& E, const float* albedo,
                const float* normal, const float* position,
                const RenderOptions& opt) {
  const std::size_t npix = static_cast<std::size_t>(w) * h;
  if (w <= 0 || h <= 0 || E.size() < npix * 3) return 0;  // no-op

  // NaN/Inf scrub before handing the buffer to OIDN: a single NaN propagates
  // through the network and corrupts the whole output image.
  for (std::size_t i = 0; i < npix * 3; ++i)
    if (!std::isfinite(E[i])) E[i] = 0.0f;

  // Pack E into a synthetic FrameResult so the existing backends (which
  // operate on frame.color RGBA + albedo/normal guides at arbitrary
  // resolution) run unchanged. Alpha = 1 everywhere; miss pixels carry a zero
  // normal, which denoiseOidn treats as background (no write-back).
  FrameResult tmp;
  tmp.width = w;
  tmp.height = h;
  tmp.color.resize(npix * 4);
  for (std::size_t p = 0; p < npix; ++p) {
    tmp.color[p * 4 + 0] = E[p * 3 + 0];
    tmp.color[p * 4 + 1] = E[p * 3 + 1];
    tmp.color[p * 4 + 2] = E[p * 3 + 2];
    tmp.color[p * 4 + 3] = 1.0f;
  }
  if (albedo) tmp.albedo.assign(albedo, albedo + npix * 3);
  if (normal) tmp.normal.assign(normal, normal + npix * 3);
  if (position) tmp.position.assign(position, position + npix * 3);

  // E is irradiance (pre-albedo), so albedo demodulation must stay off in the
  // fallback -- the albedo buffer is a guide here, not a factor of the signal.
  RenderOptions dopt = opt;
  dopt.denoiseDemodulateAlbedo = false;

  int used;
#ifdef UMBREON_HAVE_OIDN
  used = denoiseOidn(tmp, dopt)
             ? static_cast<int>(DenoiserBackend::OIDN)            // worker ran
             : static_cast<int>(DenoiserBackend::AtrousBilateral);  // fell back
#else
  std::fprintf(stderr,
               "warning: pt1 denoise requested but OIDN is unavailable "
               "(UMBREON_WITH_OIDN=OFF); using the a-trous fallback\n");
  denoiseAtrous(tmp, dopt);
  used = static_cast<int>(DenoiserBackend::AtrousBilateral);
#endif

  for (std::size_t p = 0; p < npix; ++p) {
    E[p * 3 + 0] = tmp.color[p * 4 + 0];
    E[p * 3 + 1] = tmp.color[p * 4 + 1];
    E[p * 3 + 2] = tmp.color[p * 4 + 2];
  }
  return used;
}

}  // namespace detail
}  // namespace umbreon
