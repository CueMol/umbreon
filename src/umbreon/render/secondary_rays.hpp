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
