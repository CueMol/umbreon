// Image post-process helpers for the umbreon renderer: linear-space box
// downsample (the supersample resolve), assumed-gamma, and 8-bit sRGB encode.
// Split out of umbreon.cpp so the public entry stays a thin facade; also reused
// by the bench harness and the image_ops unit test.
#pragma once

#include <cstdint>
#include <vector>

#include "render/render_types.hpp"

namespace umbreon {

// Box-average a linear-space image from w x h down to (w/ss) x (h/ss).
std::vector<float> boxDownsample(const std::vector<float>& src, int w, int h,
                                 int channels, int ss);

// Raise the RGB channels to the power g in place (POV assumed_gamma; g == 1 is a
// no-op). Alpha is left unchanged.
void applyAssumedGamma(FrameResult& frame, float g);

// Encode the linear RGBA framebuffer to interleaved 8-bit sRGB bytes for display
// or hand-off to CueMol. channels is 3 (RGB) or 4 (RGBA; alpha stored linear).
std::vector<std::uint8_t> srgbEncode8(const FrameResult& frame, int channels);

}  // namespace umbreon
