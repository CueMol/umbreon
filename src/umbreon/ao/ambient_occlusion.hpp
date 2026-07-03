// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Ambient-occlusion estimators for the Embree renderer: the legacy binary
// single-scale computeAO and the enhanced multi-scale / distance-falloff /
// bent-normal computeAOQuality, plus the AO-only sampling helpers they use.
// Split out of shading/secondary_rays.hpp so the AO feature lives under ao/;
// the shared RNG / hemisphere-sampling / occlusion primitives stay there and
// are pulled in via the include below.
//
// Hot per-sample path, so everything is `inline`; the AO shading composite
// (ao/ao_shade.hpp) includes this and the compiler inlines the estimators into
// the pixel loop. Gated off by default (aoSamples == 0 => fully open).
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include <embree4/rtcore.h>

#include "scene.hpp"
#include "shading/secondary_rays.hpp"  // Light, tea2, occluded, cosineSampleHemisphere

namespace umbreon {
namespace detail {

// Ambient-occlusion factor in [0,1] for a hit: 1 = fully open, 0 = fully
// occluded. Casts nSamples cosine-weighted rays over the hemisphere around the
// shading normal Ns; each ray origin is pushed off the surface along the
// GEOMETRIC normal Ng (not Ns: offsetting along an interpolated normal can dip
// the origin below a concave surface and self-hit). aoRadius bounds the search.
// Deterministic from the hi-res pixel (px, py).
inline float computeAO(RTCScene rscene, const Vec3& P, const Vec3& Ng,
                       const Vec3& Ns, float eps, int nSamples, float aoRadius,
                       uint32_t px, uint32_t py, int wHi,
                       const RayProbe* probe = nullptr) {
  if (nSamples <= 0) return 1.0f;
  const Frame f = frameFromNormal(Ns);
  // Offset axis = geometric normal, face-forwarded to the shading side. eps is
  // the primary-ray-scaled self-intersection distance supplied by the caller.
  Vec3 ng = (dot(Ng, Ns) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
  ng = safeNormalize(ng, Ns);
  const Vec3 O = P + ng * eps;
  const uint32_t base = px + py * static_cast<uint32_t>(wHi);
  int hits = 0;
  for (int i = 0; i < nSamples; ++i) {
    uint32_t s0 = base;
    uint32_t s1 = static_cast<uint32_t>(i);
    tea2(s0, s1);
    const Vec3 dir = cosineSampleHemisphere(u32ToUnorm(s0), u32ToUnorm(s1), f);
    if (dot(dir, Ns) < 0.01f) {  // grazing / below the surface: treat as occluded
      ++hits;
      continue;
    }
    if (occluded(rscene, O, dir, eps, aoRadius, probe)) ++hits;
  }
  return 1.0f - static_cast<float>(hits) / static_cast<float>(nSamples);
}

// ---------------------------------------------------------------------------
// AO quality: enhanced multi-scale / distance-falloff / bent-normal estimator.
// The legacy binary single-scale computeAO above is kept verbatim and is what
// runs unless an enhancement is requested (see ao_shade aoEnhanced gate), so
// the default render stays bit-identical. Everything here is deterministic from
// the same hi-res (px,py) tea2 seed, so it is thread-count independent and
// reproducible exactly like the legacy path.
// ---------------------------------------------------------------------------

// Nearest-hit distance along [tnear, tfar]: returns the hit distance (ray.tfar
// after rtcIntersect1) or +inf on a miss. Same any-geometry semantics as
// occluded(), but it keeps the distance so the multi-scale falloff and the
// average-hit-distance AOV can be derived from a single ray set (no extra rays).
// `probe` (blend-reuse capture): the LIVE segment [tnear, min(hit, tfar)] is
// probed -- blend geometry behind an accepted nearest hit cannot change it.
inline float intersectNearest(RTCScene rscene, const Vec3& O, const Vec3& dir,
                              float tnear, float tfar,
                              const RayProbe* probe = nullptr) {
  RTCRayHit rh;
  rh.ray.org_x = O.x;
  rh.ray.org_y = O.y;
  rh.ray.org_z = O.z;
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
  rtcIntersect1(rscene, &rh, &iargs);
  const bool hit = rh.hit.geomID != RTC_INVALID_GEOMETRY_ID;
  probeSegment(probe, O, dir, tnear, hit ? rh.ray.tfar : tfar);
  return hit ? rh.ray.tfar : std::numeric_limits<float>::infinity();
}

// Van der Corput radical inverse in base 2 (bit reversal): the second Hammersley
// coordinate. Returns a value in [0,1).
inline float radicalInverse2(uint32_t bits) {
  bits = (bits << 16) | (bits >> 16);
  bits = ((bits & 0x55555555u) << 1) | ((bits & 0xAAAAAAAAu) >> 1);
  bits = ((bits & 0x33333333u) << 2) | ((bits & 0xCCCCCCCCu) >> 2);
  bits = ((bits & 0x0F0F0F0Fu) << 4) | ((bits & 0xF0F0F0F0u) >> 4);
  bits = ((bits & 0x00FF00FFu) << 8) | ((bits & 0xFF00FF00u) >> 8);
  return static_cast<float>(bits) * 0x1.0p-32f;
}

// Draw the i-th 2D AO sample. Default: the legacy per-sample tea2 stream (white
// noise). Low-discrepancy: the Hammersley point (i/N, radicalInverse2(i)) with a
// per-pixel Cranley-Patterson toroidal shift (cpx,cpy) so neighboring pixels use
// rotated-but-still-stratified sets -- lower AO variance at fixed sample count,
// still fully deterministic.
inline void aoSample2d(bool lowDiscrepancy, uint32_t base, int i, int nSamples,
                       float cpx, float cpy, float& u1, float& u2) {
  if (lowDiscrepancy) {
    u1 = static_cast<float>(i) / static_cast<float>(nSamples) + cpx;
    u2 = radicalInverse2(static_cast<uint32_t>(i)) + cpy;
    if (u1 >= 1.0f) u1 -= 1.0f;
    if (u2 >= 1.0f) u2 -= 1.0f;
  } else {
    uint32_t s0 = base;
    uint32_t s1 = static_cast<uint32_t>(i);
    tea2(s0, s1);
    u1 = u32ToUnorm(s0);
    u2 = u32ToUnorm(s1);
  }
}

// GTAO multi-bounce approximation (Jimenez et al. 2016, "Practical Realtime
// Strategies for Accurate Indirect Occlusion"): a per-channel cubic that lifts
// the AO term toward 1 as albedo rises. Single-bounce AO implicitly assumes a
// black surface, so light-colored cavities over-darken; this restores the energy
// a real surface would bounce back. ao in [0,1] (1 = open), albedo the channel's
// pigment value. The result is >= ao (max guard), so it only ever brightens.
inline float aoMultibounce(float ao, float albedo) {
  const float a = 2.0404f * albedo - 0.3324f;
  const float b = -4.7951f * albedo + 0.6417f;
  const float c = 2.7552f * albedo + 0.6903f;
  return std::fmax(ao, ((ao * a + b) * ao + c) * ao);
}

// Knobs for computeAOQuality (the enhanced estimator).
struct AOParams {
  int nSamples = 0;          // rays per hit (<=0 => fully open)
  float radius = 1.0e20f;    // max search radius R (= aoDistance)
  float falloffPower = 0.0f; // 0 => binary; >0 => (max(0,1-t/r))^power
  bool multiScale = false;   // false => single radius R; true => 3 nested scales
  bool lowDiscrepancy = false;  // false => tea2 white noise; true => Hammersley+CP
};

// Multi-value AO result. contact/shape are kept UNBLENDED (small-radius vs
// mid+large-radius obscurance) so a future surface-irradiance cache can compose
// them as L = L_direct + shape*(L_indirect+L_env); L *= contact. openness is the
// reduced single value the AO-only path feeds into the ambient term (the legacy
// scalar aoFactor analogue). bent is the average unoccluded direction (used for
// directional ambient / cache lookup). avgHitDist is the mean occluder distance
// in world units, to separate near-contact darkening from far-occlusion
// darkening when tuning.
struct AOResult {
  float contact = 1.0f;
  float shape = 1.0f;
  float openness = 1.0f;
  Vec3 bent{0.0f, 0.0f, 0.0f};
  float avgHitDist = 0.0f;
};

// Enhanced AO: one cosine-weighted ray set (rtcIntersect1, nearest hit) yields
// multi-scale distance-falloff obscurance, a bent normal and the mean hit
// distance at once -- same ray count as legacy computeAO, no extra memory. The
// nearest occluder dominates (contact shadows), but with falloff its weight ramps
// down with distance so far geometry no longer darkens uniformly.
inline AOResult computeAOQuality(RTCScene rscene, const Vec3& P, const Vec3& Ng,
                                 const Vec3& Ns, float eps, const AOParams& ap,
                                 uint32_t px, uint32_t py, int wHi,
                                 const RayProbe* probe = nullptr) {
  AOResult r;
  if (ap.nSamples <= 0) {
    r.bent = Ns;
    return r;
  }
  const Frame f = frameFromNormal(Ns);
  Vec3 ng = (dot(Ng, Ns) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
  ng = safeNormalize(ng, Ns);
  const Vec3 O = P + ng * eps;
  const uint32_t base = px + py * static_cast<uint32_t>(wHi);

  // Scale radii (fraction of R) and blend weights. Single-scale degenerates to
  // K=1 / weight 1 / radius R == the legacy single radius. Multi-scale nests
  // small/mid/large so contact shadows and broad obscurance coexist.
  const float R = ap.radius;
  float radius[3];
  float weight[3];
  int K;
  if (ap.multiScale) {
    K = 3;
    radius[0] = 0.08f * R;
    radius[1] = 0.30f * R;
    radius[2] = 1.00f * R;
    weight[0] = 0.55f;
    weight[1] = 0.35f;
    weight[2] = 0.10f;
  } else {
    K = 1;
    radius[0] = radius[1] = radius[2] = R;
    weight[0] = 1.0f;
    weight[1] = weight[2] = 0.0f;
  }

  // Per-pixel Cranley-Patterson rotation offset for the LD sampler (unused by
  // the tea2 path). Seeded from the pixel so it is deterministic.
  uint32_t c0 = px + 1u, c1 = py + 1u;
  tea2(c0, c1);
  const float cpx = u32ToUnorm(c0);
  const float cpy = u32ToUnorm(c1);

  float obsScale[3] = {0.0f, 0.0f, 0.0f};  // Sum_i falloff_ik per scale
  float weightedObs = 0.0f;                // Sum_i Sum_k weight[k]*falloff_ik
  Vec3 bentAccum{0.0f, 0.0f, 0.0f};
  float hitDistSum = 0.0f;
  int nHit = 0;
  const float power = ap.falloffPower;

  for (int i = 0; i < ap.nSamples; ++i) {
    float u1, u2;
    aoSample2d(ap.lowDiscrepancy, base, i, ap.nSamples, cpx, cpy, u1, u2);
    const Vec3 dir = cosineSampleHemisphere(u1, u2, f);
    if (dot(dir, Ns) < 0.01f) {  // grazing/below surface: full contact occlusion
      obsScale[0] += 1.0f;
      obsScale[1] += 1.0f;
      obsScale[2] += 1.0f;
      weightedObs += 1.0f;  // Sum_k weight[k] == 1
      continue;             // no bent / avgHitDist contribution
    }
    const float t = intersectNearest(rscene, O, dir, eps, R, probe);
    float perSample = 0.0f;  // Sum_k weight[k]*falloff_ik for this sample
    for (int k = 0; k < K; ++k) {
      if (t < radius[k]) {
        float fk = 1.0f;
        if (power > 0.0f) {
          float a = 1.0f - t / radius[k];
          if (a < 0.0f) a = 0.0f;
          fk = std::pow(a, power);
        }
        obsScale[k] += fk;
        perSample += weight[k] * fk;
      }
    }
    weightedObs += perSample;
    bentAccum = bentAccum + dir * (1.0f - perSample);  // open dirs steer the bent
    if (t < R) {  // finite hit within the search radius
      hitDistSum += t;
      ++nHit;
    }
  }

  const float invN = 1.0f / static_cast<float>(ap.nSamples);
  r.openness = 1.0f - weightedObs * invN;
  r.contact = 1.0f - obsScale[0] * invN;
  if (ap.multiScale) {
    const float wMid = weight[1] + weight[2];
    r.shape = 1.0f - (weight[1] * obsScale[1] + weight[2] * obsScale[2]) /
                         (wMid * static_cast<float>(ap.nSamples));
  } else {
    r.shape = r.openness;  // single scale: contact == shape == openness
  }
  r.bent = safeNormalize(bentAccum, Ns);
  r.avgHitDist = (nHit > 0) ? hitDistSum / static_cast<float>(nHit) : R;
  return r;
}

}  // namespace detail
}  // namespace umbreon
