// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Blend-group ghost probing for the group-alpha multipass reuse: while the
// BACKGROUND pass (all blend groups hidden) renders, every traced ray segment
// is additionally tested against small per-group BVHs holding ONLY the blend
// groups' geometry, and a per-pixel bitmask records which groups any of the
// pixel's rays could have intersected. A pixel whose rays never touch group
// i's geometry is guaranteed bit-identical in layer pass i (all sampling is
// deterministic per pixel), so the layer pass can copy it instead of
// recomputing.
//
// Self-contained (only rtcore + scene.hpp) so the ray helpers that need it
// (secondary_rays / transparency / ao / pt1) can include it without cycles;
// the any-hit test mirrors detail::occluded (secondary_rays.hpp) exactly.
#pragma once

#include <cstdint>
#include <vector>

#include <embree4/rtcore.h>

#include "scene.hpp"

namespace umbreon {
namespace detail {

// One committed blend-only BVH PER blend group, indexed by the group's
// position in Scene::groupBlend (exact per-group touch attribution with plain
// any-hit queries; typical group counts are 1-3). Scenes are read-only during
// rendering, so concurrent rtcOccluded1 from the pixel loops is safe.
struct BlendProbeScenes {
  std::vector<RTCScene> groupScene;
  uint32_t allBits() const {
    return groupScene.size() >= 32
               ? 0xFFFFFFFFu
               : ((1u << static_cast<uint32_t>(groupScene.size())) - 1u);
  }
};

// Per-PIXEL probe view: the frame-level scenes plus THIS pixel's touch word.
// Threaded as `const RayProbe* probe = nullptr` through the ray helpers; the
// normal (non-capture) render passes nullptr and the `if (!p)` early-out
// folds away in the inlined hot path. `touch` is written only by the thread
// that owns the pixel in the enclosing parallel loop -- no atomics.
struct RayProbe {
  const BlendProbeScenes* scenes = nullptr;
  uint32_t* touch = nullptr;
};

// Test [tnear, tfar] along (O, dir) against every not-yet-touched group BVH
// and set the corresponding bits. Early-out once every group is touched.
// Callers pass the LIVE part of the segment: up to the accepted hit for
// nearest-hit queries (geometry behind an accepted hit cannot change the
// result), the full segment for visibility queries that came back unoccluded
// (an occluded result cannot be changed by ADDING geometry).
inline void probeSegment(const RayProbe* p, const Vec3& O, const Vec3& dir,
                         float tnear, float tfar) {
  if (!p || !p->scenes || !p->touch) return;
  uint32_t t = *p->touch;
  const uint32_t all = p->scenes->allBits();
  if (t == all) return;
  for (std::size_t i = 0; i < p->scenes->groupScene.size(); ++i) {
    const uint32_t bit = 1u << static_cast<uint32_t>(i);
    if (t & bit) continue;
    // Mirrors detail::occluded (secondary_rays.hpp): binary any-hit.
    RTCRay r;
    r.org_x = O.x;
    r.org_y = O.y;
    r.org_z = O.z;
    r.dir_x = dir.x;
    r.dir_y = dir.y;
    r.dir_z = dir.z;
    r.tnear = tnear;
    r.tfar = tfar;
    r.mask = 0xFFFFFFFFu;
    r.flags = 0;
    r.time = 0.0f;
    RTCOccludedArguments oargs;
    rtcInitOccludedArguments(&oargs);
    rtcOccluded1(p->scenes->groupScene[i], &r, &oargs);
    if (r.tfar < 0.0f) t |= bit;
  }
  *p->touch = t;
}

}  // namespace detail
}  // namespace umbreon
