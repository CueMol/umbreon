// umbreon CLI utility modes: PPM->PNG/PPM convert and PPM PSNR/SSIM compare.
// Split out of main.cpp; both are early-exit modes dispatched before any
// scene work.
#pragma once

#include "cli.hpp"

namespace umbreon {

// --convert <in> <out>: read a PPM, write it in the requested format.
// Returns the process exit code.
int runConvertMode(const Options& opt);

// --compare <a> <b>: print PSNR/SSIM between two PPM files.
// Returns the process exit code.
int runCompareMode(const Options& opt);

}  // namespace umbreon
