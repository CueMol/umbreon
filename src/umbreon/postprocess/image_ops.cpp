#include "postprocess/image_ops.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace umbreon {

std::vector<float> boxDownsample(const std::vector<float>& src, int w, int h,
                                 int channels, int ss) {
  const int ow = w / ss, oh = h / ss;
  std::vector<float> out(static_cast<std::size_t>(ow) * oh * channels, 0.0f);
  const float inv = 1.0f / static_cast<float>(ss * ss);
  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      for (int c = 0; c < channels; ++c) {
        float acc = 0.0f;
        for (int dy = 0; dy < ss; ++dy) {
          for (int dx = 0; dx < ss; ++dx) {
            std::size_t si =
                (static_cast<std::size_t>(y * ss + dy) * w + (x * ss + dx)) *
                    channels + c;
            // Contain a NaN/Inf subpixel to itself: a non-finite sample
            // contributes 0 instead of poisoning the whole block average (the
            // divisor stays ss*ss, matching OSPRay zeroing NaN before accum).
            const float v = src[si];
            acc += std::isfinite(v) ? v : 0.0f;
          }
        }
        out[(static_cast<std::size_t>(y) * ow + x) * channels + c] = acc * inv;
      }
    }
  }
  return out;
}

void applyAssumedGamma(FrameResult& frame, float g) {
  if (std::fabs(g - 1.0f) <= 1.0e-4f) return;
  const std::size_t px = static_cast<std::size_t>(frame.width) * frame.height;
  for (std::size_t i = 0; i < px; ++i) {
    for (int c = 0; c < 3; ++c) {
      float v = frame.color[i * 4 + c];
      frame.color[i * 4 + c] = (v > 0.0f) ? std::pow(v, g) : 0.0f;
    }
  }
}

float srgbEncodeF(float v) {
  v = std::min(1.0f, std::max(0.0f, v));
  return (v <= 0.0031308f) ? 12.92f * v
                           : 1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f;
}

float srgbDecodeF(float s) {
  s = std::min(1.0f, std::max(0.0f, s));
  return (s <= 0.04045f) ? s / 12.92f
                         : std::pow((s + 0.055f) / 1.055f, 2.4f);
}

namespace {
std::uint8_t toSrgb8(float v) {
  const int i = static_cast<int>(srgbEncodeF(v) * 255.0f + 0.5f);
  return static_cast<std::uint8_t>(std::min(255, std::max(0, i)));
}
}  // namespace

std::vector<std::uint8_t> srgbEncode8(const FrameResult& frame, int channels) {
  const std::size_t px = static_cast<std::size_t>(frame.width) * frame.height;
  std::vector<std::uint8_t> out(px * channels);
  for (std::size_t i = 0; i < px; ++i) {
    for (int c = 0; c < 3 && c < channels; ++c)
      out[i * channels + c] = toSrgb8(frame.color[i * 4 + c]);
    if (channels == 4) {
      const float a = std::min(1.0f, std::max(0.0f, frame.color[i * 4 + 3]));
      out[i * 4 + 3] = static_cast<std::uint8_t>(a * 255.0f + 0.5f);
    }
  }
  return out;
}

}  // namespace umbreon
