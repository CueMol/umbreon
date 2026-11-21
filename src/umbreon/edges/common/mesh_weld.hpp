// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Positional weld grid shared by the mesh2 reader and the feature-edge
// extractor.
//
// World coordinates are quantized to a ~1e-4-unit grid and the integer cell is
// hashed; vertices landing in the same cell are treated as one TOPOLOGICAL
// vertex. This reconstructs the water-tight ribbon topology that mesh2
// face_indices do not encode -- CueMol emits the ribbon as per-winding triangle
// strips whose seam vertices coincide only positionally. The tolerance is
// deliberately defined ONCE here so the reader's load-time position-class map
// and the extractor's fallback weld cannot drift apart.
//
// NOTE: this is the POSITION-ONLY topological weld. It is distinct from the
// render-vertex dedup (exact position+normal+color), which keeps hard-edge and
// color-seam corners on separate vertices.
#pragma once

#include <cmath>
#include <cstddef>

#include "scene.hpp"

namespace umbreon {

struct WeldKey {
  int x, y, z;
  bool operator==(const WeldKey& o) const {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct WeldKeyHash {
  std::size_t operator()(const WeldKey& k) const {
    // Teschner et al. spatial hash.
    return (static_cast<std::size_t>(k.x) * 73856093u) ^
           (static_cast<std::size_t>(k.y) * 19349663u) ^
           (static_cast<std::size_t>(k.z) * 83492791u);
  }
};

inline constexpr float kWeldQuant = 1.0e4f;  // weld within ~1e-4 world units

inline WeldKey weldKey(const Vec3& p) {
  return WeldKey{static_cast<int>(std::lround(p.x * kWeldQuant)),
                 static_cast<int>(std::lround(p.y * kWeldQuant)),
                 static_cast<int>(std::lround(p.z * kWeldQuant))};
}

}  // namespace umbreon
