// Image output (self-contained PNG / PPM writer) and comparison metrics.
// Pure C++17, no external image library.
#pragma once

#include <string>
#include <vector>

namespace umbreon {

// Write a linear-light image to disk. channels must be 3 (RGB) or 4 (RGBA).
// The output format is chosen by the path extension: ".ppm" -> binary PPM,
// anything else -> PNG. RGB channels are sRGB-encoded on write; an alpha
// channel is stored without gamma.
void writeImage(const std::string& path, int width, int height,
                const float* pixels, int channels);

// A linear-light RGB image.
struct ImageRGB {
  int width = 0;
  int height = 0;
  std::vector<float> rgb;  // width * height * 3, linear
};

// Read a binary PPM (P6) file and sRGB-decode it to linear RGB.
ImageRGB readPpm(const std::string& path);

// PSNR (dB) and SSIM between two equally sized images, computed on
// sRGB-encoded luma/values. Returns {0,0} if the sizes differ.
struct CompareResult {
  double psnr = 0.0;
  double ssim = 0.0;
};
CompareResult compareImages(const ImageRGB& a, const ImageRGB& b);

}  // namespace umbreon
