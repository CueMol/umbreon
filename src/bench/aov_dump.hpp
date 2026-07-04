// Debug image dumps (verification only), split out of main.cpp: the
// adaptive-AA refinement mask, the coarse-AO fallback mask, and the
// --dump-aov false-color AOV set (edge G-buffer / AO / GI cache).
#pragma once

#include "cli.hpp"
#include "umbreon.hpp"

namespace umbreon {

// Write every debug image the render produced: aaMask / aoPatchMask (written
// whenever their AOV is present) and the --dump-aov set (only with a prefix).
void dumpDebugImages(const Options& opt, const RenderOptions& ropt,
                     const FrameResult& frame);

}  // namespace umbreon
