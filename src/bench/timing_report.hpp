// pt1 stage timing report (stdout table + outputs/timing.json), split out of
// main.cpp.
#pragma once

#include "umbreon.hpp"

namespace umbreon {

// Print the pt1 stage split and write outputs/timing.json. No-op unless the
// pt1 integrator ran (ropt.gi && ropt.giIntegrator == 1). `totalSeconds` is
// the wall time around render().
void reportPt1Timing(const RenderOptions& ropt, const FrameResult& frame,
                     double totalSeconds);

}  // namespace umbreon
