#include "postprocess/fog.hpp"

#include <cstddef>

namespace umbreon {

float fogFactor(const Fog& fog, float z) {
  if (!fog.enabled) return 1.0f;
  const float denom = fog.end - fog.start;
  if (denom <= 0.0f) return z < fog.end ? 1.0f : 0.0f;  // degenerate slab
  float f = (fog.end - z) / denom;
  if (f < 0.0f) return 0.0f;
  if (f > 1.0f) return 1.0f;
  return f;
}

void applyFog(const Fog& fog, int width, int height, int channels, float* color,
              const float* viewZ, bool transparentBackground) {
  if (!fog.enabled || color == nullptr || viewZ == nullptr) return;
  const std::size_t px = static_cast<std::size_t>(width) * height;
  for (std::size_t i = 0; i < px; ++i) {
    const float z = viewZ[i];
    if (z <= 0.0f) continue;  // background sentinel (no geometry)
    const float f = fogFactor(fog, z);
    float* c = color + i * channels;
    if (transparentBackground) {
      // Fade coverage toward the far plane; keep RGB (no fog color baked).
      if (channels == 4) c[3] *= f;
    } else {
      // GL: mix(fogColor, objRGB, f). Alpha unchanged.
      c[0] = fog.color.x * (1.0f - f) + c[0] * f;
      c[1] = fog.color.y * (1.0f - f) + c[1] * f;
      c[2] = fog.color.z * (1.0f - f) + c[2] * f;
    }
  }
}

}  // namespace umbreon
