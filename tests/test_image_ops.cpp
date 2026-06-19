// Unit tests for the public post-process helpers in the umbreon facade: linear
// box-downsample (the antialiasing average), assumed_gamma, and sRGB encoding
// (the CueMol hand-off encoder, otherwise exercised by nothing). The fog
// post-process is covered separately in test_fog.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

namespace {
bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }
}  // namespace

int main() {
  umbreon::test::Suite s("image_ops");

  // --- boxDownsample: 4x4 single channel, ss=2 -> 2x2; each output pixel is the
  // mean of its 2x2 source block. src[i] = i (row-major), w = 4. ---
  {
    std::vector<float> src(16);
    for (int i = 0; i < 16; ++i) src[i] = static_cast<float>(i);
    std::vector<float> out = umbreon::boxDownsample(src, 4, 4, 1, 2);
    s.check_eq("downsample: output size", out.size(), std::size_t(4));
    s.check("downsample: block (0,0) = mean{0,1,4,5}", approx(out[0], 2.5f, 1e-6f));
    s.check("downsample: block (1,0) = mean{2,3,6,7}", approx(out[1], 4.5f, 1e-6f));
    s.check("downsample: block (0,1) = mean{8,9,12,13}", approx(out[2], 10.5f, 1e-6f));
    s.check("downsample: block (1,1) = mean{10,11,14,15}", approx(out[3], 12.5f, 1e-6f));
  }

  // --- applyAssumedGamma: g=1 is a no-op; g=2 squares the RGB channels and
  // leaves alpha; negative radiance clamps to 0. ---
  {
    umbreon::FrameResult f;
    f.width = 1;
    f.height = 1;
    f.color = {0.5f, 0.4f, 0.2f, 0.7f};
    umbreon::applyAssumedGamma(f, 1.0f);
    s.check("gamma g=1: no-op", approx(f.color[0], 0.5f, 1e-6f));

    umbreon::applyAssumedGamma(f, 2.0f);
    s.check("gamma g=2: R squared", approx(f.color[0], 0.25f, 1e-6f));
    s.check("gamma g=2: G squared", approx(f.color[1], 0.16f, 1e-6f));
    s.check("gamma g=2: B squared", approx(f.color[2], 0.04f, 1e-6f));
    s.check("gamma g=2: alpha unchanged", approx(f.color[3], 0.7f, 1e-6f));

    umbreon::FrameResult g;
    g.width = 1;
    g.height = 1;
    g.color = {-0.5f, 0.0f, 0.0f, 1.0f};
    umbreon::applyAssumedGamma(g, 2.0f);
    s.check("gamma: negative radiance clamps to 0", approx(g.color[0], 0.0f, 1e-6f));
  }

  // --- srgbEncode8: linear -> 8-bit sRGB. 0->0, 1->255, 0.5 linear -> 188 (the
  // standard sRGB midtone); out-of-range clamps; alpha is stored linear. ---
  {
    umbreon::FrameResult f;
    f.width = 1;
    f.height = 1;
    f.color = {0.0f, 0.5f, 1.0f, 0.5f};
    std::vector<std::uint8_t> rgba = umbreon::srgbEncode8(f, 4);
    s.check_eq("srgb: size for 4 channels", rgba.size(), std::size_t(4));
    s.check_eq("srgb: 0.0 -> 0", static_cast<int>(rgba[0]), 0);
    s.check_eq("srgb: 0.5 -> 188", static_cast<int>(rgba[1]), 188);
    s.check_eq("srgb: 1.0 -> 255", static_cast<int>(rgba[2]), 255);
    s.check_eq("srgb: alpha stored linear (0.5 -> 128)", static_cast<int>(rgba[3]), 128);

    umbreon::FrameResult h;
    h.width = 1;
    h.height = 1;
    h.color = {2.0f, -1.0f, 0.0f, 1.0f};
    std::vector<std::uint8_t> rgb = umbreon::srgbEncode8(h, 3);
    s.check_eq("srgb: size for 3 channels", rgb.size(), std::size_t(3));
    s.check_eq("srgb: >1 clamps to 255", static_cast<int>(rgb[0]), 255);
    s.check_eq("srgb: <0 clamps to 0", static_cast<int>(rgb[1]), 0);
  }

  return s.report();
}
