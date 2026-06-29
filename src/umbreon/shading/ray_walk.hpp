// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Front-to-back all-hit ray walk for the object-space edge extractor's
// visibility / clipping queries.
//
// The transparency integrator (shading/transparency.hpp integratePixel) already
// walks a ray hit-by-hit, advancing tnear past each surface by a scale-adaptive
// self-intersection epsilon (OSPRay calcEpsilon). The edge extractor needs the
// SAME stepping to enumerate every triangle a visibility/clip ray crosses, but
// it (a) runs against a SEPARATE edge-mesh BVH (not the render scene), (b) needs
// the per-hit primID/t rather than shading, and (c) must keep an early-out-free
// "collect all" semantics. Rather than refactor the hot, stateful integratePixel
// loop (and risk its bit-exact output), this header factors out only the generic
// stepping as a visitor template and the integrator is left untouched.
//
// Pure inline header; depends only on Embree + scene.hpp (Vec3).
#pragma once

#include <cmath>
#include <vector>

#include <embree4/rtcore.h>

#include "scene.hpp"

namespace umbreon {
namespace detail {

// One recorded hit along a ray: the struck Embree primitive plus the ray
// parameter t at the hit, so callers can map t -> fsec = t / segmentLength.
struct RayHit {
  unsigned int geomID;
  unsigned int primID;
  float t;
};

enum class WalkAction { Continue, Stop };

// Walk the hits along [org + tnear*dir, org + tfar*dir] front-to-back, calling
// visit(const RTCRayHit&) -> WalkAction at each hit. Owns the scale-adaptive
// epsilon advance identical to transparency.hpp:185-193 (OSPRay calcEpsilon:
// max(|hitP|, |dir|*t) * ulpEps), so an edge query skips the same float jitter a
// primary ray would. Stops at the first miss (ray escaped), when visit returns
// Stop, when tnear reaches tfar, or after maxIters layers (defensive bound; a
// closed surface terminates by the miss break first).
template <class Visit>
inline void walkHitsAlongRay(RTCScene scene, const Vec3& org, const Vec3& dir,
                             float tnear, float tfar, int maxIters,
                             Visit&& visit) {
  // ~4 ULP, OSPRay ulpEpsilon -- MUST match transparency.hpp:192 so the edge
  // query and the primary ray skip self-intersections identically.
  constexpr float kUlpEps = 0x1.0fp-21f;
  for (int iter = 0; iter < maxIters; ++iter) {
    RTCRayHit rh;
    rh.ray.org_x = org.x;
    rh.ray.org_y = org.y;
    rh.ray.org_z = org.z;
    rh.ray.dir_x = dir.x;
    rh.ray.dir_y = dir.y;
    rh.ray.dir_z = dir.z;
    rh.ray.tnear = tnear;
    rh.ray.tfar = tfar;
    rh.ray.mask = 0xFFFFFFFFu;
    rh.ray.flags = 0;
    rh.ray.time = 0.0f;
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    rtcIntersect1(scene, &rh, &iargs);

    if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) break;  // ray escaped
    if (visit(rh) == WalkAction::Stop) break;

    // Advance just past this surface (scale-adaptive epsilon, see transparency.hpp).
    const Vec3 hitP{org.x + dir.x * rh.ray.tfar, org.y + dir.y * rh.ray.tfar,
                    org.z + dir.z * rh.ray.tfar};
    const float dirMax =
        std::fmax(std::fabs(dir.x), std::fmax(std::fabs(dir.y), std::fabs(dir.z)));
    const float epsScale =
        std::fmax(std::fmax(std::fabs(hitP.x), std::fabs(hitP.y)),
                  std::fmax(std::fabs(hitP.z), dirMax * rh.ray.tfar));
    tnear = rh.ray.tfar + epsScale * kUlpEps;
    if (tnear >= tfar) break;
  }
}

// Collect every hit along the ray for which excludePred(primID) is FALSE, in
// front-to-back order. The visibility test reads "any surviving hit == occluded"
// (empty == visible); clipping reads each hit's t as fsec = t / length.
// excludePred reproduces CueMol's contains_id self-face skip: an occluder that is
// the edge's own incident face (or 1-ring) is not counted. maxIters bounds the
// layer count.
template <class ExcludePred>
inline std::vector<RayHit> collectHitsAlongRay(RTCScene scene, const Vec3& org,
                                               const Vec3& dir, float tnear,
                                               float tfar,
                                               ExcludePred&& excludePred,
                                               int maxIters = 256) {
  std::vector<RayHit> hits;
  walkHitsAlongRay(scene, org, dir, tnear, tfar, maxIters,
                   [&](const RTCRayHit& rh) {
                     if (!excludePred(rh.hit.primID))
                       hits.push_back(
                           {rh.hit.geomID, rh.hit.primID, rh.ray.tfar});
                     return WalkAction::Continue;
                   });
  return hits;
}

}  // namespace detail
}  // namespace umbreon
