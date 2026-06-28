// Front-to-back transparency integration for one primary ray. Walks the hits
// along the ray, compositing CueMol "veil" groups additively (group alpha) and
// every other transparent fragment with "over" (fragment alpha), over the
// nearest opaque surface / background. Returns the pixel's linear HDR RGBA and
// near depth.
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
  Vec3 position{0.0f, 0.0f, 0.0f};  // world first-hit point (cache key; 0 on escape)
  uint32_t componentId = 0xFFFFFFFFu;  // first MESH hit's section (sentinel = none)
};

// Integrate one primary ray (origin `org`, direction `rd`) through the scene,
// compositing transparency. (px, py) is the hi-res pixel for deterministic
// secondary-ray sampling; `isVeil` flags the additive (group-alpha) sections;
// `camDir` is the camera forward axis (normalized) used to convert ray-tfar to
// linear view-z for the edge G-buffer; `cappedRays` counts rays that exhaust the
// transparent-layer ceiling.
inline PixelResult integratePixel(const ShadeContext& sc,
                                  const std::vector<uint8_t>& isVeil,
                                  const Vec3& org, const Vec3& rd, uint32_t px,
                                  uint32_t py, const Vec3& camDir,
                                  std::atomic<long long>& cappedRays) {
  RTCScene rscene = sc.built.scene;
  const RenderOptions& opt = sc.opt;
  const Vec3& bg = sc.bg;

  // Single-pass transparency with two coexisting models, selected per hit
  // by the hit's group:
  //   - VEIL groups (group alpha): additive single-layer, frontmost-per-
  //     group, order-independent -- exactly CueMol's blendpng.
  //   - everything else (fragment alpha = intrinsic per-color opacity):
  //     front-to-back "over", every surface composited (no dedup) -- POV
  //     native transmit; group alpha (if any) already multiplied in.
  // Combine: fragments composite over the opaque floor, then the veils are
  // laid additively on top. Reduces to pure "over" (no veils, e.g. scene5),
  // pure additive (all veils, the group-alpha path), or opaque (unchanged).
  Vec3 Cv{0.0f, 0.0f, 0.0f};  // additive (veil) premultiplied color
  float sumBeta = 0.0f;       // sum of veil weights
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
  Vec3 firstPosition{0.0f, 0.0f, 0.0f};
  uint32_t firstComponentId = 0xFFFFFFFFu;
  Vec3 base = bg;
  float baseCov = opt.transparentBackground ? 0.0f : 1.0f;

  constexpr int kMaxSeen = 64;  // distinct veil groups per ray
  int seen[kMaxSeen];
  int nseen = 0;

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
      firstPosition = hs.position;
      firstComponentId = hs.componentId;
    }

    if (!opt.transparency || hs.opacity >= kOpaque) {
      base = hs.color;  // nearest opaque surface = the floor
      baseCov = 1.0f;
      break;
    }

    const bool veil =
        hs.group >= 0 && static_cast<std::size_t>(hs.group) < isVeil.size() &&
        isVeil[hs.group] != 0;
    if (veil) {
      // additive: only the frontmost surface of each veil group
      bool dup = false;
      for (int sidx = 0; sidx < nseen; ++sidx)
        if (seen[sidx] == hs.group) { dup = true; break; }
      if (!dup) {
        if (nseen < kMaxSeen) seen[nseen++] = hs.group;
        const float a = hs.opacity;
        Cv.x += a * hs.color.x;
        Cv.y += a * hs.color.y;
        Cv.z += a * hs.color.z;
        sumBeta += a;
      }
    } else {
      // over: every fragment composited front-to-back (no dedup)
      const float w = (1.0f - Af) * hs.opacity;
      Cf.x += w * hs.color.x;
      Cf.y += w * hs.color.y;
      Cf.z += w * hs.color.z;
      Af += w;
    }
    if (sumBeta >= kOpaque || Af >= kOpaque) break;  // fully saturated
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

  // Fragments over the opaque floor; veils laid additively on top. The base
  // (floor / background) contributes COLOR only where it is covered
  // (baseCov), so an opaque background (default) leaves opaque scenes
  // byte-unchanged, while a transparent background (baseCov 0) yields a
  // premultiplied result with alpha = accumulated coverage.
  const float floorW = (1.0f - Af) * baseCov;
  const Vec3 baseEff{Cf.x + floorW * base.x,
                     Cf.y + floorW * base.y,
                     Cf.z + floorW * base.z};
  const float covEff = Af + floorW;
  float vw = 1.0f - sumBeta;
  if (vw < 0.0f) vw = 0.0f;
  float outA = sumBeta + vw * covEff;
  outA = std::fmin(1.0f, std::fmax(0.0f, outA));

  return PixelResult{Cv.x + vw * baseEff.x,
                     Cv.y + vw * baseEff.y,
                     Cv.z + vw * baseEff.z,
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
                     firstPosition,
                     firstComponentId};
}

}  // namespace detail
}  // namespace umbreon
