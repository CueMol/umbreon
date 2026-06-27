// OpenGL linear fog reproduced as a depth-based post-process.
//
// CueMol's interactive display uses OpenGL linear fog; its POV-Ray export only
// APPROXIMATED that with an exponential ground-fog hack. We reproduce the GL
// model directly from the plane eye-z (the `viewZ` AOV), so the output matches
// what the user actually sees and tunes in CueMol. Shading is untouched.
#pragma once

#include "scene.hpp"

namespace umbreon {

// Linear fog factor for a surface at PLANE eye-z `z` (= |eye-space z|, the
// slant-corrected view depth, NOT the radial ray length). f = 1 keeps the
// surface color (z <= start), f = 0 is fully the fog color (z >= end).
float fogFactor(const Fog& fog, float z);

// Apply fog to a linear color buffer in place. `viewZ` holds one plane eye-z per
// pixel; pixels with viewZ <= 0 are the background (no geometry) and are
// skipped. `channels` is 3 or 4.
//   - opaque background (transparentBackground == false): mix RGB toward the
//     fog color by (1 - f); alpha is left untouched -- matches CueMol GL, whose
//     background is cleared to the fog color so distant geometry melts into it.
//   - transparent background (true): fade coverage (alpha *= f) and DO NOT bake
//     the fog color, so the straight-alpha output can be composited over ANY
//     backdrop later while the distance fade stays physically correct.
void applyFog(const Fog& fog, int width, int height, int channels, float* color,
              const float* viewZ, bool transparentBackground);

}  // namespace umbreon
