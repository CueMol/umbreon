// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Tiny helpers shared by the screen-vector edge pipeline TUs
// (screen_edge_*.cpp / screen_vector_edges.cpp).
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "edges/screen_projection.hpp"

namespace umbreon {
namespace screen_edge {

// objectId AOV background sentinel.
constexpr std::uint32_t kBackground = 0xFFFFFFFFu;

// World units spanned by one pixel at linear view-z `vz` (identity in vz under
// ortho). Uses the vertical extent; the renderer's pixels are square.
inline float pixelSizeAt(const ScreenProj& sp, float vz) {
  return sp.ortho ? (2.0f * sp.halfH / static_cast<float>(sp.H))
                  : (2.0f * sp.persHalfH * vz / static_cast<float>(sp.H));
}

// |n . viewdir| / |n| at pixel `idx`, i.e. how front-facing the surface is
// (1 = facing the camera, 0 = edge-on / at its silhouette). Zero for a
// degenerate normal.
inline float facingCos(const float* normal, const ScreenProj& sp, int idx) {
  const float* n = normal + 3 * static_cast<std::size_t>(idx);
  const float len2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
  if (len2 <= 1.0e-12f) return 0.0f;
  const float d = n[0] * sp.dir.x + n[1] * sp.dir.y + n[2] * sp.dir.z;
  return std::fabs(d) / std::sqrt(len2);
}

}  // namespace screen_edge
}  // namespace umbreon
