// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt1 G-buffer half: the camera basis / ray-stats / G-buffer data types and
// the TBB per-pixel primary-ray tracer (tracePt1GBuffer). Split out of
// pt1_integrator.hpp for readability; everything stays header-inline (see the
// umbrella header for the unit-compatibility rationale).
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range2d.h>

#include "experimental/irradiance_cache/irradiance_cache.hpp"
#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {


// Camera basis + image-plane half extents, precomputed once by render() (the
// same math as the main pixel loop) so the pt1 half-res pass can shoot its own
// primary rays through an identical camera.
struct Pt1CameraBasis {
  Vec3 position{0.0f, 0.0f, 0.0f};
  Vec3 dir{0.0f, 0.0f, -1.0f};     // normalized view direction
  Vec3 right{1.0f, 0.0f, 0.0f};
  Vec3 trueUp{0.0f, 1.0f, 0.0f};
  bool orthographic = false;
  float halfW = 1.0f, halfH = 1.0f;          // orthographic half extents
  float persHalfW = 1.0f, persHalfH = 1.0f;  // perspective, at unit distance
};

// Per-pixel ray counters, accumulated as plain integers inside the gather
// loops and flushed ONCE per pixel into the atomic totals (per-ray atomics
// would cost ~0.1-0.3 s at the 50M+ rays of the high preset).
struct Pt1RayStatsLocal {
  uint64_t gatherRays = 0;   // intersectFull calls (all bounces)
  uint64_t gatherHits = 0;   // of which hit geometry
  uint64_t neeRays = 0;      // NEE shadow rays (rtcOccluded1)
  uint64_t neeOccluded = 0;  // of which occluded
};

struct Pt1RayStats {
  std::atomic<uint64_t> gatherRays{0};
  std::atomic<uint64_t> gatherHits{0};
  std::atomic<uint64_t> neeRays{0};
  std::atomic<uint64_t> neeOccluded{0};
  std::atomic<uint64_t> gbufferRays{0};  // half-res G-buffer primary rays
  void flush(const Pt1RayStatsLocal& l) {
    gatherRays.fetch_add(l.gatherRays, std::memory_order_relaxed);
    gatherHits.fetch_add(l.gatherHits, std::memory_order_relaxed);
    neeRays.fetch_add(l.neeRays, std::memory_order_relaxed);
    neeOccluded.fetch_add(l.neeOccluded, std::memory_order_relaxed);
  }
};

// First-hit G-buffer for an independently traced (typically half-res) grid.
// Normals are face-forwarded toward the viewer; misses and outline-decoration
// hits have hit == 0 with zeroed position/normal/albedo (a zero normal marks
// background for the OIDN guide). Mesh hits AND real CSG primitives (atom
// balls / bonds, fromEdge == 0) are gather-eligible.
struct Pt1GBuffer {
  int w = 0, h = 0;
  std::vector<float> position;    // w*h*3 world-space first hit
  std::vector<float> normal;      // w*h*3 shading normal (face-forward)
  std::vector<float> geomNormal;  // w*h*3 geometric normal (face-forward)
  std::vector<float> albedo;      // w*h*3 kd * pigment (OIDN guide)
  std::vector<float> depth;       // w*h   ray distance from camera
  std::vector<uint8_t> hit;       // w*h   1 = mesh hit (gather-eligible)
};

// Trace one primary ray per pixel center of a w x h grid through `basis` and
// fill the G-buffer. The grid's normalized [-1,1] plane coordinates match the
// main render grid's, so a (W+1)/2 x (H+1)/2 grid samples the centers of the
// full grid's 2x2 pixel blocks.
inline void tracePt1GBuffer(const IrradianceCacheParams& p,
                            const Pt1CameraBasis& cam, int w, int h,
                            Pt1GBuffer& g, Pt1RayStats* stats = nullptr) {
  const std::size_t npix = static_cast<std::size_t>(w) * h;
  g.w = w;
  g.h = h;
  g.position.assign(npix * 3, 0.0f);
  g.normal.assign(npix * 3, 0.0f);
  g.geomNormal.assign(npix * 3, 0.0f);
  g.albedo.assign(npix * 3, 0.0f);
  g.depth.assign(npix, 0.0f);
  g.hit.assign(npix, 0);

  if (stats)
    stats->gbufferRays.fetch_add(npix, std::memory_order_relaxed);

  tbb::parallel_for(tbb::blocked_range<int>(0, h),
                    [&](const tbb::blocked_range<int>& rows) {
    for (int py = rows.begin(); py != rows.end(); ++py) {
      const float v =
          1.0f - 2.0f * (static_cast<float>(py) + 0.5f) / static_cast<float>(h);
      for (int px = 0; px < w; ++px) {
        const float u =
            2.0f * (static_cast<float>(px) + 0.5f) / static_cast<float>(w) -
            1.0f;
        Vec3 org, rd;
        if (cam.orthographic) {
          org = cam.position + cam.right * (u * cam.halfW) +
                cam.trueUp * (v * cam.halfH);
          rd = cam.dir;
        } else {
          org = cam.position;
          rd = normalize(cam.dir + cam.right * (u * cam.persHalfW) +
                         cam.trueUp * (v * cam.persHalfH));
        }

        const RTCRayHit rh = intersectFull(
            p.scene, org, rd, 0.0f, std::numeric_limits<float>::infinity());
        if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) continue;
        const GeomRecord& rec = p.built->records[rh.hit.geomID];

        Vec3 N, Ng;
        float albedo[3];
        if (rec.kind == GeomKind::Mesh) {
          // Shading normal / pigment from the mesh vertex attributes (same
          // slots as oneBounceRadiance: 0 = normal, 1 = color).
          float nbuf[3] = {0, 0, 0};
          float cbuf[4] = {0, 0, 0, 1};
          rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                          RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
          rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                          RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
          N = safeNormalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
          Ng = safeNormalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z});
          const float kd = p.mesh->materialForTri(rh.hit.primID).diffuse;
          albedo[0] = kd * cbuf[0];
          albedo[1] = kd * cbuf[1];
          albedo[2] = kd * cbuf[2];
        } else {
          // Real CSG primitive (atom ball / bond): analytic normal from the
          // Embree Ng, color/material from the primID side-tables. Outline
          // decoration (fromEdge) is not a GI receiver -- leave hit == 0.
          const BuiltScene& b = *p.built;
          const bool isSphere = (rec.kind == GeomKind::Sphere);
          const bool isCapped = (rec.kind == GeomKind::CylinderCapped);
          const bool fromEdge =
              isSphere ? (!b.sphereFromEdge.empty() &&
                          b.sphereFromEdge[rh.hit.primID])
              : isCapped ? (!b.cylCapFromEdge.empty() &&
                            b.cylCapFromEdge[rh.hit.primID])
                         : (!b.cylFromEdge.empty() &&
                            b.cylFromEdge[rh.hit.primID]);
          if (fromEdge) continue;
          const Vec4& fc = isSphere ? b.sphereColor[rh.hit.primID]
                           : isCapped ? b.cylCapColor[rh.hit.primID]
                                      : b.cylColor[rh.hit.primID];
          const Material& pm = isSphere ? b.sphereMat[rh.hit.primID]
                               : isCapped ? b.cylCapMat[rh.hit.primID]
                                          : b.cylMat[rh.hit.primID];
          N = safeNormalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z});
          Ng = N;
          albedo[0] = pm.diffuse * fc.x;
          albedo[1] = pm.diffuse * fc.y;
          albedo[2] = pm.diffuse * fc.z;
        }
        // Face-forward toward the viewer (dot(N, -rd) < 0 => flip).
        if (dot(N, rd) > 0.0f) N = Vec3{-N.x, -N.y, -N.z};
        if (dot(Ng, rd) > 0.0f) Ng = Vec3{-Ng.x, -Ng.y, -Ng.z};

        const std::size_t pix = static_cast<std::size_t>(py) * w + px;
        const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                     org.z + rd.z * rh.ray.tfar};
        g.position[pix * 3 + 0] = P.x;
        g.position[pix * 3 + 1] = P.y;
        g.position[pix * 3 + 2] = P.z;
        g.normal[pix * 3 + 0] = N.x;
        g.normal[pix * 3 + 1] = N.y;
        g.normal[pix * 3 + 2] = N.z;
        g.geomNormal[pix * 3 + 0] = Ng.x;
        g.geomNormal[pix * 3 + 1] = Ng.y;
        g.geomNormal[pix * 3 + 2] = Ng.z;
        g.albedo[pix * 3 + 0] = albedo[0];
        g.albedo[pix * 3 + 1] = albedo[1];
        g.albedo[pix * 3 + 2] = albedo[2];
        g.depth[pix] = rh.ray.tfar;
        g.hit[pix] = 1;
      }
    }
  });
}
}  // namespace detail
}  // namespace umbreon
