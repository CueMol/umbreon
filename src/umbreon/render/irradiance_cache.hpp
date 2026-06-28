// Surface irradiance cache for one-bounce diffuse GI (detail, Embree-aware).
//
// Instead of tracing GI rays at every pixel, indirect irradiance is gathered at
// a small set of representative surface points (records) and interpolated back
// at shading time (Ward-Heckbert / Krivanek irradiance caching). The build runs
// inside EmbreeRenderer::render() AFTER the primary loop, on the hi-res first-hit
// G-buffer (position/normal/componentId AOVs), because the fill pass needs the
// in-scope BuiltScene + lights to evaluate albedo*directLighting at each gather
// hit -- data that does not survive to the post-pass.
//
// Determinism (TBB thread-count independent, run-to-run bit-identical):
//   [B] placement   single-thread canonical raster scan; voxel occupancy is a
//                   set membership test, so the record SET is scan-order free.
//   [C] fill        TBB-parallel, each record seeded from its stable index hash
//                   (never a shared counter); neighbor clamp uses commutative min.
//   [D] lookup      read-only; per shading point the candidate record indices are
//                   sorted before float accumulation so the sum order is fixed.
//
// This header is included by embree_renderer.cpp; the fill/lookup passes (C4/C5)
// are added below as they land. C3 wires the structs + deterministic placement.
#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// One irradiance cache record: a representative surface point where one-bounce
// indirect irradiance is gathered (fill) and later interpolated (lookup).
struct IrradianceRecord {
  Vec3 position{0.0f, 0.0f, 0.0f};    // surface point (cache spatial key)
  Vec3 normal{0.0f, 0.0f, 0.0f};      // surface normal
  Vec3 irradiance{0.0f, 0.0f, 0.0f};  // gathered one-bounce indirect E_i (fill)
  Vec3 bentNormal{0.0f, 0.0f, 0.0f};  // average unoccluded direction (fill)
  float radius = 0.0f;                // harmonic-mean valid radius R_i (fill)
  uint32_t componentId = 0xFFFFFFFFu; // CueMol section: rejects cross-section blend
};

// Pack an integer voxel coordinate into a 63-bit spatial-hash key: 21 bits per
// axis (signed, bias 2^20) => +/- ~1e6 cells/axis, ample at the record spacing.
// Deterministic and collision-free within that range.
inline uint64_t packVoxel(long long ix, long long iy, long long iz) {
  constexpr long long kBias = 1 << 20;
  const uint64_t ux = static_cast<uint64_t>(ix + kBias) & 0x1FFFFFull;
  const uint64_t uy = static_cast<uint64_t>(iy + kBias) & 0x1FFFFFull;
  const uint64_t uz = static_cast<uint64_t>(iz + kBias) & 0x1FFFFFull;
  return ux | (uy << 21) | (uz << 42);
}

// World point -> voxel key at the given inverse spacing.
inline uint64_t voxelKey(const Vec3& p, float invSpacing) {
  return packVoxel(static_cast<long long>(std::floor(p.x * invSpacing)),
                   static_cast<long long>(std::floor(p.y * invSpacing)),
                   static_cast<long long>(std::floor(p.z * invSpacing)));
}

// [B] Deterministic record placement. Scan the hi-res first-hit G-buffer in
// raster order and drop ONE record into each unoccupied world voxel of size
// `spacing`. Voxel occupancy is a set membership test, so the record SET is
// independent of scan order; the raster scan only fixes WHICH pixel (the first
// in raster order) supplies each cell's attributes, giving a stable canonical
// record order whose index is a stable per-record key for the fill seed.
// Non-mesh pixels (componentId == sentinel) seed nothing: GI is mesh-only.
inline std::vector<IrradianceRecord> placeRecordsVoxel(const FrameResult& res,
                                                       float spacing) {
  std::vector<IrradianceRecord> records;
  if (res.position.empty() || res.componentId.empty() || spacing <= 0.0f)
    return records;
  std::unordered_set<uint64_t> occupied;
  const int W = res.width, H = res.height;
  const float inv = 1.0f / spacing;
  for (int py = 0; py < H; ++py) {
    for (int px = 0; px < W; ++px) {
      const std::size_t pix = static_cast<std::size_t>(py) * W + px;
      const uint32_t comp = res.componentId[pix];
      if (comp == 0xFFFFFFFFu) continue;  // background / non-mesh hit: no record
      const Vec3 p{res.position[pix * 3 + 0], res.position[pix * 3 + 1],
                   res.position[pix * 3 + 2]};
      if (occupied.insert(voxelKey(p, inv)).second) {
        IrradianceRecord r;
        r.position = p;
        r.normal = Vec3{res.normal[pix * 3 + 0], res.normal[pix * 3 + 1],
                        res.normal[pix * 3 + 2]};
        r.componentId = comp;
        records.push_back(r);
      }
    }
  }
  return records;
}

// The cache: records in canonical placement order plus a spatial hash from world
// voxel to the record indices whose influence reaches that voxel. The hash is
// built AFTER fill (radii known) and queried read-only at lookup; the fill seed
// uses only a record's stable index, never a shared counter. Grid build/query
// land with the interpolation pass (C5).
struct IrradianceCache {
  std::vector<IrradianceRecord> records;
  std::unordered_map<uint64_t, std::vector<uint32_t>> grid;
  float cellSize = 1.0f;
};

}  // namespace detail
}  // namespace umbreon
