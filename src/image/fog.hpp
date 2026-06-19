// POV-Ray fog reproduced as a depth-based post-process.
//
// POV applies fog by blending each surface color toward the fog color by the
// transmittance along the view ray. Because the renderer has no built-in ground-fog
// primitive, we replay the same model using the rendered depth buffer: this
// keeps the shading untouched and lets the fog match POV exactly.
#pragma once

#include "scene.hpp"

namespace umbreon {

// Transmittance (fraction of the surface color retained) for a ray that starts
// at `camPos`, travels along unit vector `camDir`, and hits a surface at
// distance `depth`. 1.0 = no fog, 0.0 = fully the fog color.
//
// Ground fog (type 2) uses CueMol's regime: a hard density step at `offset`
// along `up` (fog_alt -> 0), so the optical depth is the ray length below the
// offset plane divided by `distance`. Constant fog (type 1) fogs the whole ray.
float fogTransmittance(const Fog& fog, const Vec3& camPos, const Vec3& camDir,
                       float depth);

// Blend a linear color buffer toward the fog color in place. `channels` is 3 or
// 4 (alpha is left untouched). `depth` holds one value per pixel; pixels whose
// depth is non-finite or beyond `maxDepth` (i.e. the background) are skipped.
void applyFog(const Fog& fog, const Camera& camera, int width, int height,
              int channels, float* color, const float* depth,
              float maxDepth = 1.0e19f);

}  // namespace umbreon
