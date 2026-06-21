#include "image/fog.hpp"

#include <algorithm>
#include <cmath>

namespace umbreon {

float fogTransmittance(const Fog& fog, const Vec3& camPos, const Vec3& camDir,
                       float depth) {
  if (!fog.enabled || fog.distance <= 0.0f || depth <= 0.0f) return 1.0f;

  float fogLength;  // ray length that travels through the fog
  if (fog.type == 2) {
    // Ground fog: density is full where altitude (along `up`) <= offset.
    Vec3 up = normalize(fog.up);
    float a0 = dot(camPos, up);        // altitude at the ray origin
    float ad = dot(camDir, up);        // altitude change per unit length
    if (std::fabs(ad) < 1.0e-8f) {
      fogLength = (a0 <= fog.offset) ? depth : 0.0f;
    } else {
      // altitude(t) = a0 + ad*t crosses the offset plane at t*.
      float tStar = (fog.offset - a0) / ad;
      if (ad < 0.0f) {
        // altitude decreases along the ray: below the plane for t >= t*.
        float lo = std::min(std::max(tStar, 0.0f), depth);
        fogLength = depth - lo;
      } else {
        // altitude increases along the ray: below the plane for t <= t*.
        float hi = std::min(std::max(tStar, 0.0f), depth);
        fogLength = hi;
      }
    }
  } else {
    fogLength = depth;  // constant fog fills the entire ray
  }

  return std::exp(-fogLength / fog.distance);
}

void applyFog(const Fog& fog, const Camera& camera, int width, int height,
              int channels, float* color, const float* depth, float maxDepth) {
  if (!fog.enabled || color == nullptr || depth == nullptr) return;

  const Vec3 camPos = camera.position;
  const Vec3 camDir = normalize(camera.direction);
  const std::size_t px = static_cast<std::size_t>(width) * height;

  for (std::size_t i = 0; i < px; ++i) {
    float d = depth[i];
    if (!std::isfinite(d) || d >= maxDepth) continue;  // background
    float t = fogTransmittance(fog, camPos, camDir, d);
    float* c = color + i * channels;
    c[0] = t * c[0] + (1.0f - t) * fog.color.x;
    c[1] = t * c[1] + (1.0f - t) * fog.color.y;
    c[2] = t * c[2] + (1.0f - t) * fog.color.z;
  }
}

}  // namespace umbreon
