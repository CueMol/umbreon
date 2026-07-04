// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Front-to-back transparency integration for one primary ray. Walks the hits
// along the ray, compositing every transparent fragment with "over" (fragment
// alpha = intrinsic per-color opacity, POV native transmit) over the nearest
// opaque surface / background. Returns the pixel's linear HDR RGBA and near
// depth. Group alpha (CueMol section transparency) is NOT handled here: it is
// realized as a blendpng-equivalent multi-pass post-blend in render() (see
// Scene::groupBlend), so each pass sees only opaque + fragment-alpha surfaces.
//
// Hot per-pixel path, so it is `inline` here and calls shadeHit() (hit_shader.hpp)
// inline; the renderer's TBB loop inlines the whole thing per pixel.
#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include "shading/hit_shader.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// One integrated pixel: linear HDR RGBA plus the near-hit depth (0 if the ray
// escaped to the background). The trailing fields are the screen-space edge
// G-buffer for the FIRST (nearest) hit; they keep their sentinel initials when
// the ray escapes (no write at the no-hit break) and are only consumed when
// RenderOptions::edges is enabled.
struct PixelResult {
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  float a = 0.0f;
  float depth = 0.0f;
  uint32_t objectId = 0xFFFFFFFFu;    // background sentinel
  uint32_t materialId = 0xFFFFFFFFu;  // background sentinel
  Vec3 worldNormal{0.0f, 0.0f, 0.0f};
  float viewZ = 0.0f;                 // linear view-z (0 == background sentinel)
  // First-hit AO / G-buffer AOVs (only meaningful with RenderOptions::aoWriteAov;
  // keep their open/neutral defaults when the ray escapes). Color is unaffected.
  Vec3 albedo{0.0f, 0.0f, 0.0f};
  Vec3 bentNormal{0.0f, 0.0f, 0.0f};
  float contactAo = 1.0f;
  float shapeAo = 1.0f;
  float avgHitDist = 0.0f;
  // First-hit surface-irradiance-cache seed (only meaningful with
  // RenderOptions::gi; kept at the background sentinels when the ray escapes).
  // Color is unaffected. worldPos = org + rd*nearDepth; firstGroup/firstGeomID
  // tag the record's component/geometry so leaks across sections are rejectable.
  Vec3 worldPos{0.0f, 0.0f, 0.0f};
  int firstGroup = -1;                  // CueMol section; -1 = background
  uint32_t firstGeomID = 0xFFFFFFFFu;   // Embree geomID; sentinel = background
  Vec3 giReflectance{0.0f, 0.0f, 0.0f}; // mat.diffuse * pigment (GI composite)
  uint8_t giEligible = 0;               // 1 = pt1 gather-eligible first hit
  // First-hit fragment opacity (surface alpha) for the edge G-buffer; keeps
  // its opaque default when the ray escapes. Only consumed when strokeEdges
  // is enabled.
  float firstOpacity = 1.0f;
  // Coarse-AO debug: 1 = the FIRST hit's bilateral lookup was rejected and
  // gathered inline. Only meaningful with RenderOptions::aoResDebug.
  uint8_t aoPatched = 0;
};

// First-hit edge G-buffer of one primary ray, WITHOUT shading: what the
// adaptive-AA Phase-0 probe writes for every hi-res subpixel when the stroke
// edge pass is on. Field-for-field the values integratePixel captures at its
// first hit (the G-buffer is first-hit-only, so a single rtcIntersect1 suffices
// -- no transparency walk, no AO/shadow rays). Miss keeps the sentinels.
struct GBufProbe {
  Vec3 normal{0.0f, 0.0f, 0.0f};
  float viewZ = 0.0f;
  uint32_t objectId = 0xFFFFFFFFu;
  uint32_t materialId = 0xFFFFFFFFu;
  float surfAlpha = 1.0f;
};

// Trace one primary ray and fill the first-hit G-buffer. MIRRORS the G-buffer
// branch of shadeHit (hit_shader.hpp): the mesh path interpolates the slot-0
// smooth normal (face-forwarded via the geometric Ng, the silhouette-stable
// flip) and reads the slot-1 pigment alpha; the primitive path uses the
// analytic Ng and the per-kind color/material side-tables, including the
// edge_line2 axial opacity lerp. Locked against shadeHit by a parity test
// (tests/test_adaptive_aa.cpp); any change to shadeHit's G-buffer capture must
// be reflected here.
inline GBufProbe probeGBuffer(const ShadeContext& sc, const Vec3& org,
                              const Vec3& rd, const Vec3& camDir) {
  GBufProbe g;
  RTCRayHit rh;
  rh.ray.org_x = org.x;
  rh.ray.org_y = org.y;
  rh.ray.org_z = org.z;
  rh.ray.dir_x = rd.x;
  rh.ray.dir_y = rd.y;
  rh.ray.dir_z = rd.z;
  rh.ray.tnear = 0.0f;
  rh.ray.tfar = std::numeric_limits<float>::infinity();
  rh.ray.mask = 0xFFFFFFFFu;
  rh.ray.flags = 0;
  rh.ray.time = 0.0f;
  rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  rtcIntersect1(sc.built.scene, &rh, &iargs);
  if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
      rh.hit.geomID >= sc.built.records.size())
    return g;  // background sentinels

  g.viewZ = rh.ray.tfar * dot(rd, camDir);
  const GeomRecord& rec = sc.built.records[rh.hit.geomID];
  if (rec.kind == GeomKind::Mesh) {
    // Smooth shading normal (slot 0), face-forwarded via the GEOMETRIC Ng --
    // the same stable flip shadeHit uses (see the comment there).
    float nbuf[3] = {0, 0, 0};
    float cbuf[4] = {0, 0, 0, 1};
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
    const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
    const Vec3 NgF = faceForward(Ng, rd);
    Vec3 N = normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
    if (dot(N, NgF) < 0.0f) N = Vec3{-N.x, -N.y, -N.z};
    g.normal = N;
    const int group = sc.mesh.groupForTri(rh.hit.primID);
    g.objectId = (static_cast<uint32_t>(group) << 2) |
                 static_cast<uint32_t>(rec.kind);  // kindBits: Mesh == 0
    if (!sc.mesh.triMaterialId.empty()) {
      const std::size_t nt = sc.mesh.triMaterialId.size();
      g.materialId =
          static_cast<uint32_t>(sc.mesh.triMaterialId[rh.hit.primID % nt]);
    } else {
      g.materialId = 0u;
    }
    g.surfAlpha = cbuf[3];
  } else {
    const bool isSphere = (rec.kind == GeomKind::Sphere);
    const bool isCapped = (rec.kind == GeomKind::CylinderCapped);
    g.normal = faceForward(
        normalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z}), rd);
    const BuiltScene& b = sc.built;
    const int group = isSphere ? b.sphereGroup[rh.hit.primID]
                      : isCapped ? b.cylCapGroup[rh.hit.primID]
                                 : b.cylGroup[rh.hit.primID];
    g.objectId = (static_cast<uint32_t>(group) << 2) |
                 static_cast<uint32_t>(rec.kind);  // Sphere=1,Cyl=2,CylCap=3
    if (isSphere)
      g.materialId = b.meshMatCount + b.sphereMatIndex[rh.hit.primID];
    else if (isCapped)
      g.materialId = b.meshMatCount + b.sphereMatCount + b.cylMatCount +
                     b.cylCapMatIndex[rh.hit.primID];
    else
      g.materialId =
          b.meshMatCount + b.sphereMatCount + b.cylMatIndex[rh.hit.primID];
    const Vec4& fc = isSphere ? b.sphereColor[rh.hit.primID]
                     : isCapped ? b.cylCapColor[rh.hit.primID]
                                : b.cylColor[rh.hit.primID];
    g.surfAlpha = fc.w;
    // edge_line2 gradient: opacity varies p0 -> p1 along the segment axis.
    if (!isSphere) {
      const float op1 = isCapped ? b.cylCapOpacity1[rh.hit.primID]
                                 : b.cylOpacity1[rh.hit.primID];
      if (op1 >= 0.0f) {
        float u = rh.hit.u;
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;
        g.surfAlpha = fc.w * (1.0f - u) + op1 * u;
      }
    }
  }
  return g;
}

// Integrate one primary ray (origin `org`, direction `rd`) through the scene,
// compositing transparency. (px, py) is the hi-res pixel for deterministic
// secondary-ray sampling; `camDir` is the camera forward axis (normalized) used
// to convert ray-tfar to linear view-z for the edge G-buffer; `cappedRays`
// counts rays that exhaust the transparent-layer ceiling.
inline PixelResult integratePixel(const ShadeContext& sc, const Vec3& org,
                                  const Vec3& rd, uint32_t px, uint32_t py,
                                  const Vec3& camDir,
                                  std::atomic<long long>& cappedRays) {
  RTCScene rscene = sc.built.scene;
  const RenderOptions& opt = sc.opt;
  const Vec3& bg = sc.bg;

  // Fragment alpha (intrinsic per-color opacity): front-to-back "over", every
  // surface composited (no dedup) -- POV native transmit -- over the nearest
  // opaque floor / background. Reduces to opaque (unchanged) when nothing is
  // transparent along the ray.
  Vec3 Cf{0.0f, 0.0f, 0.0f};  // over (fragment) premultiplied color
  float Af = 0.0f;            // over accumulated coverage
  float nearDepth = 0.0f;
  // Edge G-buffer for the FIRST (nearest) hit. Initialized to the background
  // sentinel; overwritten at the first shadeHit, untouched if the ray escapes.
  uint32_t firstObjectId = 0xFFFFFFFFu;
  uint32_t firstMaterialId = 0xFFFFFFFFu;
  Vec3 firstNormal{0.0f, 0.0f, 0.0f};
  float firstViewZ = 0.0f;
  Vec3 firstAlbedo{0.0f, 0.0f, 0.0f};
  Vec3 firstBent{0.0f, 0.0f, 0.0f};
  float firstContact = 1.0f;
  float firstShape = 1.0f;
  float firstAvgHit = 0.0f;
  Vec3 firstWorldPos{0.0f, 0.0f, 0.0f};
  int firstGroup = -1;
  uint32_t firstGeomID = 0xFFFFFFFFu;
  Vec3 firstGiReflectance{0.0f, 0.0f, 0.0f};
  uint8_t firstGiEligible = 0;
  float firstOpacity = 1.0f;
  uint8_t firstAoPatched = 0;
  Vec3 base = bg;
  float baseCov = opt.transparentBackground ? 0.0f : 1.0f;

  const float kOpaque = 1.0f - 1e-4f;  // opacity at/above this == opaque
  float tnear = 0.0f;
  const int maxIters = opt.transparency ? (opt.maxTransparentLayers + 1) : 1;

  for (int iter = 0; iter < maxIters; ++iter) {
    RTCRayHit rh;
    rh.ray.org_x = org.x;
    rh.ray.org_y = org.y;
    rh.ray.org_z = org.z;
    rh.ray.dir_x = rd.x;
    rh.ray.dir_y = rd.y;
    rh.ray.dir_z = rd.z;
    rh.ray.tnear = tnear;
    rh.ray.tfar = std::numeric_limits<float>::infinity();
    rh.ray.mask = 0xFFFFFFFFu;
    rh.ray.flags = 0;
    rh.ray.time = 0.0f;
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    rtcIntersect1(rscene, &rh, &iargs);

    if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
        rh.hit.geomID >= sc.built.records.size()) {
      break;  // ray escaped: base stays the background
    }
    const HitShade hs = shadeHit(sc, rh, rd, org, px, py);

    // Capture the edge G-buffer from the FIRST (nearest) hit, before any
    // opaque/transparent compositing decision, so edges draw against the
    // frontmost visible surface. nearDepth doubles as the first-hit guard
    // (it is 0 only before the first hit, see PixelResult sentinel rules).
    if (nearDepth == 0.0f) {
      nearDepth = rh.ray.tfar;
      firstNormal = hs.normal;        // face-forwarded toward the viewer
      firstObjectId = hs.objectId;
      firstMaterialId = hs.materialId;
      // Linear view-z = tfar * dot(rd, camDir): identity under ortho,
      // slant-corrected under perspective (Mol* getViewZ analogue).
      firstViewZ = rh.ray.tfar * dot(rd, camDir);
      firstAlbedo = hs.albedo;
      firstBent = hs.bentNormal;
      firstContact = hs.contactAo;
      firstShape = hs.shapeAo;
      firstAvgHit = hs.avgHitDist;
      // Surface-irradiance-cache seed: the world-space first hit plus the
      // hit's component (group) and Embree geometry id. geomID lets the cache
      // restrict placement to mesh hits (GI gate); the position seeds the voxel.
      firstWorldPos = Vec3{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                           org.z + rd.z * rh.ray.tfar};
      firstGroup = hs.group;
      firstGeomID = rh.hit.geomID;
      firstGiReflectance = hs.giReflectance;
      firstGiEligible = hs.giEligible;
      firstOpacity = hs.opacity;
      firstAoPatched = hs.aoPatched;
    }

    if (!opt.transparency || hs.opacity >= kOpaque) {
      base = hs.color;  // nearest opaque surface = the floor
      baseCov = 1.0f;
      break;
    }

    // over: every fragment composited front-to-back (no dedup)
    const float w = (1.0f - Af) * hs.opacity;
    Cf.x += w * hs.color.x;
    Cf.y += w * hs.color.y;
    Cf.z += w * hs.color.z;
    Af += w;
    if (Af >= kOpaque) break;  // fully saturated
    // Advance just past this surface to find the next one. The step is a
    // self-intersection epsilon: it must clear only floating-point jitter,
    // never a distinct surface sitting just behind this one. A hardcoded
    // step is wrong because the hit-point precision degrades with scale --
    // the camera is ~200 units from the molecule, so the hit coordinates
    // carry an absolute error ~ t * 2^-23. Use a scale-adaptive epsilon, as
    // OSPRay does (modules/cpu/common/DifferentialGeometry.ih, calcEpsilon):
    // max(|hit point|, |dir| * t) scaled by a small float-ULP factor. This
    // adapts to any camera distance / scene scale with no magic constant.
    const Vec3 hitP{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                    org.z + rd.z * rh.ray.tfar};
    const float dirMax = std::fmax(
        std::fabs(rd.x), std::fmax(std::fabs(rd.y), std::fabs(rd.z)));
    const float epsScale =
        std::fmax(std::fmax(std::fabs(hitP.x), std::fabs(hitP.y)),
                  std::fmax(std::fabs(hitP.z), dirMax * rh.ray.tfar));
    constexpr float kUlpEps = 0x1.0fp-21f;  // ~5.05e-7 (~4 ULP), OSPRay ulpEpsilon
    tnear = rh.ray.tfar + epsScale * kUlpEps;
    // Reaching the final allowed iteration after compositing a transparent
    // layer means more surfaces may lie behind that we will not trace:
    // record the truncation for the post-loop warning.
    if (iter == maxIters - 1)
      cappedRays.fetch_add(1, std::memory_order_relaxed);
  }

  // Fragments over the opaque floor. The base (floor / background) contributes
  // COLOR only where it is covered (baseCov), so an opaque background
  // (default) leaves opaque scenes byte-unchanged, while a transparent
  // background (baseCov 0) yields a premultiplied result with alpha =
  // accumulated coverage.
  const float floorW = (1.0f - Af) * baseCov;
  float outA = Af + floorW;
  outA = std::fmin(1.0f, std::fmax(0.0f, outA));

  return PixelResult{Cf.x + floorW * base.x,
                     Cf.y + floorW * base.y,
                     Cf.z + floorW * base.z,
                     outA,
                     nearDepth,
                     firstObjectId,
                     firstMaterialId,
                     firstNormal,
                     firstViewZ,
                     firstAlbedo,
                     firstBent,
                     firstContact,
                     firstShape,
                     firstAvgHit,
                     firstWorldPos,
                     firstGroup,
                     firstGeomID,
                     firstGiReflectance,
                     firstGiEligible,
                     firstOpacity,
                     firstAoPatched};
}

}  // namespace detail
}  // namespace umbreon
