#include "modes.hpp"

#include <cstdio>
#include <exception>

#include "image/image_io.hpp"

namespace umbreon {

int runConvertMode(const Options& opt) {
  try {
    umbreon::ImageRGB img = umbreon::readPpm(opt.convertIn);
    umbreon::writeImage(opt.convertOut, img.width, img.height, img.rgb.data(), 3);
    std::printf("converted %s -> %s\n", opt.convertIn.c_str(),
                opt.convertOut.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}

int runCompareMode(const Options& opt) {
  try {
    umbreon::ImageRGB a = umbreon::readPpm(opt.compareA);
    umbreon::ImageRGB b = umbreon::readPpm(opt.compareB);
    umbreon::CompareResult cr = umbreon::compareImages(a, b);
    std::printf("PSNR %.2f dB   SSIM %.4f   (%s vs %s)\n", cr.psnr, cr.ssim,
                opt.compareA.c_str(), opt.compareB.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}

}  // namespace umbreon
