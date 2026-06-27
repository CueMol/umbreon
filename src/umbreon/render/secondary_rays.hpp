// Secondary-ray visibility estimators for the Embree renderer: ambient occlusion
// and (soft) shadows, plus the deterministic sampling / RNG / self-intersection
// primitives they share. Ported from OSPRay's scivis/ao renderer (Apache-2.0):
// cosine-weighted hemisphere AO and cone-sampled area-light shadows with
// per-pixel-deterministic sampling (math only, not ISPC).
//
// Hot per-sample path, so everything is `inline`; the local shader (shading.hpp)
// and the per-hit dispatch (hit_shader.hpp) include this and the compiler inlines
// the estimators into the pixel loop. Both effects are gated off by default.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include <embree4/rtcore.h>

#include "scene.hpp"

namespace umbreon {
namespace detail {

// POV-native light: direction from surface toward the light plus its radiance.
struct Light {
  Vec3 L;      // unit direction from surface toward the light
  Vec3 color;  // light color * intensity
  bool highlight = true;  // false for POV fill (shadowless) lights: diffuse only
  float radius = 0.0f;    // angular radius (radians) for soft shadows; 0 = hard
};

// Scale-adaptive self-intersection epsilon (OSPRay calcEpsilon port): the
// distance to push a secondary-ray origin off the surface so it does not re-hit
// the surface it left. Scales with the hit-point magnitude and ray length, so it
// holds at any camera distance / scene scale (a fixed eps fails when the camera
// sits far from the scene). Same formula the transparency walk uses.
inline float selfIntersectEps(const Vec3& P, const Vec3& dir, float t) {
  const float dirMax =
      std::fmax(std::fabs(dir.x), std::fmax(std::fabs(dir.y), std::fabs(dir.z)));
  const float epsScale =
      std::fmax(std::fmax(std::fabs(P.x), std::fabs(P.y)),
                std::fmax(std::fabs(P.z), dirMax * t));
  constexpr float kUlpEps = 0x1.0fp-21f;  // ~5.05e-7 (~4 ULP), OSPRay ulpEpsilon
  return epsScale * kUlpEps;
}

// Tiny Encryption Algorithm, 8 rounds: hash two 32-bit seeds into a decorrelated
// pair. Seeded only from (pixel, sample index), so the stream is identical
// regardless of TBB thread count or grain size (deterministic, reproducible AO).
inline void tea2(uint32_t& v0, uint32_t& v1) {
  uint32_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    sum += 0x9E3779B9u;
    v0 += ((v1 << 4) + 0xA341316Cu) ^ (v1 + sum) ^ ((v1 >> 5) + 0xC8013EA4u);
    v1 += ((v0 << 4) + 0xAD90777Du) ^ (v0 + sum) ^ ((v0 >> 5) + 0x7E95761Eu);
  }
}

// Map a 32-bit hash to a float in [0,1) using the top 24 bits (mantissa width).
inline float u32ToUnorm(uint32_t u) { return (u >> 8) * 0x1.0p-24f; }

// Cosine-weighted hemisphere sample around frame f (f.n is the surface normal),
// Malley's method (a uniform disk lifted to the hemisphere). The cosine weight
// cancels the estimator's 1/cos, so AO needs no per-sample weighting.
inline Vec3 cosineSampleHemisphere(float u1, float u2, const Frame& f) {
  const float phi = 6.2831853072f * u1;  // 2*pi
  const float r = std::sqrt(u2);
  const float z = std::sqrt(std::fmax(0.0f, 1.0f - u2));
  const float x = std::cos(phi) * r;
  const float y = std::sin(phi) * r;
  return f.t * x + f.b * y + f.n * z;
}

// Any-hit visibility test along [tnear, tfar]: true if any geometry is hit
// (rtcOccluded1 sets ray.tfar < 0 on a hit). Transparent geometry counts as an
// opaque occluder (binary), as OSPRay scivis does -- cheaper than a second
// transparency walk and visually close.
inline bool occluded(RTCScene rscene, const Vec3& P, const Vec3& dir, float tnear,
                     float tfar) {
  RTCRay r;
  r.org_x = P.x;
  r.org_y = P.y;
  r.org_z = P.z;
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
  rtcOccluded1(rscene, &r, &oargs);
  return r.tfar < 0.0f;
}

// Ambient-occlusion factor in [0,1] for a hit: 1 = fully open, 0 = fully
// occluded. Casts nSamples cosine-weighted rays over the hemisphere around the
// shading normal Ns; each ray origin is pushed off the surface along the
// GEOMETRIC normal Ng (not Ns: offsetting along an interpolated normal can dip
// the origin below a concave surface and self-hit). aoRadius bounds the search.
// Deterministic from the hi-res pixel (px, py).
inline float computeAO(RTCScene rscene, const Vec3& P, const Vec3& Ng,
                       const Vec3& Ns, float eps, int nSamples, float aoRadius,
                       uint32_t px, uint32_t py, int wHi) {
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
    if (occluded(rscene, O, dir, eps, aoRadius)) ++hits;
  }
  return 1.0f - static_cast<float>(hits) / static_cast<float>(nSamples);
}

// ---------------------------------------------------------------------------
// AO quality: enhanced multi-scale / distance-falloff / bent-normal estimator.
// The legacy binary single-scale computeAO above is kept verbatim and is what
// runs unless an enhancement is requested (see hit_shader aoEnhanced gate), so
// the default render stays bit-identical. Everything here is deterministic from
// the same hi-res (px,py) tea2 seed, so it is thread-count independent and
// reproducible exactly like the legacy path.
// ---------------------------------------------------------------------------

// Nearest-hit distance along [tnear, tfar]: returns the hit distance (ray.tfar
// after rtcIntersect1) or +inf on a miss. Same any-geometry semantics as
// occluded(), but it keeps the distance so the multi-scale falloff and the
// average-hit-distance AOV can be derived from a single ray set (no extra rays).
inline float intersectNearest(RTCScene rscene, const Vec3& O, const Vec3& dir,
                              float tnear, float tfar) {
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
  return (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID)
             ? rh.ray.tfar
             : std::numeric_limits<float>::infinity();
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
                                 uint32_t px, uint32_t py, int wHi) {
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
    const float t = intersectNearest(rscene, O, dir, eps, R);
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

// Sample a direction toward a distant light of angular radius l.radius: a
// uniform-disk perturbation of l.L within a cone of half-angle l.radius, giving
// a soft penumbra. radius 0 returns l.L exactly (the hard-shadow direction).
inline Vec3 sampleLightDir(const Light& l, float u1, float u2) {
  if (l.radius <= 0.0f) return l.L;
  const Frame f = frameFromNormal(l.L);
  const float phi = 6.2831853072f * u1;                 // 2*pi
  const float rr = std::tan(l.radius) * std::sqrt(u2);  // uniform disk radius
  const Vec3 d = f.n + f.t * (rr * std::cos(phi)) + f.b * (rr * std::sin(phi));
  return safeNormalize(d, l.L);
}

// Per-light shadow factor in [0,1]: 1 = lit, 0 = fully shadowed. Casts shadow
// ray(s) from the hit point toward the (distant) light; transparent geometry is
// a binary occluder (as OSPRay scivis). The origin is offset along the geometric
// normal Ng (face-forwarded to the lit side) by the adaptive self-intersection
// epsilon, so a surface does not shadow itself. A single ray gives a hard shadow
// (shadowSamples <= 1 or a point light, radius 0); otherwise shadowSamples cone
// samples are averaged for a soft area-light penumbra.
inline float computeShadow(RTCScene rscene, const Vec3& P, const Vec3& Ng,
                           const Vec3& N, float eps, const Light& l,
                           int shadowSamples, uint32_t& s0, uint32_t& s1) {
  Vec3 ng = (dot(Ng, N) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
  ng = safeNormalize(ng, N);
  const Vec3 O = P + ng * eps;
  const float far = std::numeric_limits<float>::infinity();  // distant light
  if (shadowSamples <= 1 || l.radius <= 0.0f)
    return occluded(rscene, O, l.L, eps, far) ? 0.0f : 1.0f;
  int hits = 0;
  for (int i = 0; i < shadowSamples; ++i) {
    tea2(s0, s1);
    const Vec3 dir = sampleLightDir(l, u32ToUnorm(s0), u32ToUnorm(s1));
    if (occluded(rscene, O, dir, eps, far)) ++hits;
  }
  return 1.0f - static_cast<float>(hits) / static_cast<float>(shadowSamples);
}

}  // namespace detail
}  // namespace umbreon
