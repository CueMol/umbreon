// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// AO shading composite: turns the raw occlusion gather (ao/ambient_occlusion.hpp)
// into the per-hit shading factors the pixel loop applies -- the per-channel
// ambient aoFactor, the bent-normal sky/ground ambient, and the optional
// direct-diffuse cavity darkening (diffuseAo). Split out of shading/hit_shader.hpp
// so the AO feature lives under ao/; the shared mesh + primitive branches in
// hit_shader.hpp call computeAoShade() so both darken identically.
//
// Hot per-hit path, so `inline`; hit_shader.hpp includes this and the compiler
// inlines it into the pixel loop. Returns neutral factors (no darkening) when AO
// is off (aoSamples == 0), so a flag-less render stays byte-identical.
#pragma once

#include <embree4/rtcore.h>

#include "ao/ambient_occlusion.hpp"
#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// AO shading factors for one hit, shared by the mesh branch and the real-
// primitive (atom/bond) branch so both darken identically. Encapsulates the
// openness gather -> per-channel aoFactor (ambient term), the bent-normal
// sky/ground ambient, and the optional direct-diffuse darkening (diffuseAo).
// Returns neutral factors (no darkening, ambLight == the passed ambient) when AO
// is off (aoSamples == 0), so a flag-less render is byte-identical.
struct AoShade {
  Vec3 aoFactor{1.0f, 1.0f, 1.0f};
  Vec3 ambLight;
  float diffuseAo = 1.0f;
  AOResult aov;  // captured for the G-buffer AOVs (default = fully open)
};

// Raw AO gather for one shading point: estimator selection (legacy binary vs
// enhanced quality), returning the openness that feeds the color and filling
// `aov` with the enhanced-path AOV channels (or, on the legacy path, only when
// aoWriteAov requests the extra non-color pass). Split out of computeAoShade
// VERBATIM so the coarse-grid AO build (ao/ao_coarse.hpp) can run the exact
// same gather at coarse cells; the statement sequence is unchanged, so the
// inline path stays bit-exact.
inline float aoGather(RTCScene rscene, const RenderOptions& opt, const Vec3& P,
                      const Vec3& Ng, const Vec3& N, float secEps, uint32_t px,
                      uint32_t py, int sampleMul, int wSeed, AOResult& aov) {
  const int nSamples = opt.aoSamples * sampleMul;
  float openness;
  if (opt.aoEnhanced()) {
    AOParams ap;
    ap.nSamples = nSamples;
    ap.radius = opt.aoDistance;
    ap.falloffPower = opt.aoFalloffPower;
    ap.multiScale = opt.aoMultiScale;
    ap.lowDiscrepancy = opt.aoLowDiscrepancy;
    aov = computeAOQuality(rscene, P, Ng, N, secEps, ap, px, py, wSeed);
    openness = aov.openness;
  } else {
    openness = computeAO(rscene, P, Ng, N, secEps, nSamples, opt.aoDistance,
                         px, py, wSeed);
    // Color stays on the bit-exact legacy path; if AOVs are requested, derive the
    // contact/shape/bent/avgHitDist channels with one extra (single-scale)
    // quality pass that does NOT feed the color.
    if (opt.aoWriteAov) {
      AOParams ap;
      ap.nSamples = nSamples;
      ap.radius = opt.aoDistance;
      aov = computeAOQuality(rscene, P, Ng, N, secEps, ap, px, py, wSeed);
    }
  }
  return openness;
}

// Turn a gathered openness (+ bent normal in `aov`) into the shading factors:
// the bent-normal sky/ground ambient tint, the (multibounce per-channel)
// aoFactor, and the optional direct-diffuse darkening. Statement-for-statement
// the second half of the pre-split computeAoShade (bit-exact); shared by the
// inline gather path and the coarse-grid interpolated path (which supplies
// openness/bent from the interpolated grid instead of a fresh gather).
inline AoShade aoApplyFactors(const RenderOptions& opt, const Vec3& ambLight,
                              const Vec3& aoUp, float openness,
                              const AOResult& aov, const Vec3& C) {
  AoShade r;
  r.ambLight = ambLight;
  r.aov = aov;
  // Directional ambient: a 2-color sky/ground hemisphere gradient sampled along
  // the bent normal (the average unoccluded direction). White sky == ground
  // collapses to the plain scene ambient (neutral). aoBentNormal implies
  // aoEnhanced(), so `aov.bent` was filled by the quality gather.
  if (opt.aoEnhanced() && opt.aoBentNormal) {
    const float w = 0.5f * (dot(aov.bent, aoUp) + 1.0f);
    const float gx = opt.aoGroundColor[0], sx = opt.aoSkyColor[0];
    const float gy = opt.aoGroundColor[1], sy = opt.aoSkyColor[1];
    const float gz = opt.aoGroundColor[2], sz = opt.aoSkyColor[2];
    r.ambLight = Vec3{ambLight.x * (gx + (sx - gx) * w),
                      ambLight.y * (gy + (sy - gy) * w),
                      ambLight.z * (gz + (sz - gz) * w)};
  }
  if (opt.aoMultibounce) {
    // Lift each channel by its pigment albedo so light cavities don't crush to
    // black (single-bounce AO assumes a black surface).
    const float ox = aoMultibounce(openness, C.x);
    const float oy = aoMultibounce(openness, C.y);
    const float oz = aoMultibounce(openness, C.z);
    r.aoFactor = Vec3{1.0f - opt.aoIntensity * (1.0f - ox),
                      1.0f - opt.aoIntensity * (1.0f - oy),
                      1.0f - opt.aoIntensity * (1.0f - oz)};
  } else {
    const float s = 1.0f - opt.aoIntensity * (1.0f - openness);
    r.aoFactor = Vec3{s, s, s};
  }
  // Optional indirect-shadowing approximation: also darken direct diffuse in
  // cavities (off by default => diffuseAo == 1 => bit-exact).
  if (opt.aoDiffuseFactor > 0.0f)
    r.diffuseAo = 1.0f - opt.aoDiffuseFactor * (1.0f - openness);
  return r;
}

// Compute the AO shading factors for a hit. P/Ng/N/secEps are the hit point,
// geometric normal, face-forwarded shading normal and secondary-ray epsilon; C
// is the pigment (for the albedo-aware multibounce lift). `opt` supplies the AO
// knobs, `ambLight` the scene ambient radiance, and `aoUp` the resolved
// sky/ground gradient axis (both precomputed once per frame by the caller).
// sampleMul multiplies opt.aoSamples (adaptive-AA replicated centers use ss^2 so
// one shared evaluation carries the sample count of the ss^2 subpixels it
// replaces); wSeed is the RNG lattice width. sampleMul == 1 && wSeed ==
// opt.width reproduces the legacy path bit-exactly.
inline AoShade computeAoShade(RTCScene rscene, const RenderOptions& opt,
                              const Vec3& ambLight, const Vec3& aoUp,
                              const Vec3& P, const Vec3& Ng, const Vec3& N,
                              const Vec3& C, float secEps, uint32_t px,
                              uint32_t py, int sampleMul, int wSeed) {
  if (opt.aoSamples <= 0) {
    AoShade r;
    r.ambLight = ambLight;
    return r;
  }
  AOResult aov;
  const float openness = aoGather(rscene, opt, P, Ng, N, secEps, px, py,
                                  sampleMul, wSeed, aov);
  return aoApplyFactors(opt, ambLight, aoUp, openness, aov, C);
}

}  // namespace detail
}  // namespace umbreon
