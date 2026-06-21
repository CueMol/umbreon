// POV-Ray-style local shading and the secondary-ray (ambient occlusion / shadow)
// estimators for the Embree renderer. These helpers sit on the hot per-hit /
// per-sample path, so they live in this header as `inline` functions: the
// renderer's pixel loop (embree_renderer.cpp) includes this and the compiler
// inlines them as if they were still in the same translation unit (no LTO
// needed). Cold one-per-frame work (scene construction) lives in scene_build.*.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include "scene.hpp"

namespace umbreon {
namespace detail {

inline Vec3 faceForward(Vec3 n, Vec3 rayDir) {
  // POV/CueMol normals should point toward the viewer; flip if back-facing.
  return (dot(n, rayDir) > 0.0f) ? Vec3{-n.x, -n.y, -n.z} : n;
}

// POV-native light: direction from surface toward the light plus its radiance.
struct Light {
  Vec3 L;      // unit direction from surface toward the light
  Vec3 color;  // light color * intensity
  bool highlight = true;  // false for POV fill (shadowless) lights: diffuse only
  float radius = 0.0f;    // angular radius (radians) for soft shadows; 0 = hard
};

// Map POV "roughness" to a Blinn-Phong specular exponent. POV-Ray's Blinn
// specular uses pow(N.H, 1/roughness). roughness 0.01 -> exp 100.
inline float blinnExp(float roughness) {
  float r = roughness;
  if (r < 1e-6f) r = 1e-6f;
  float e = 1.0f / r;
  if (e < 1.0f) e = 1.0f;
  if (e > 1.0e6f) e = 1.0e6f;
  return e;
}

// --- Secondary-ray (ambient occlusion) helpers ----------------------------
// Ported from OSPRay's scivis/ao renderer (Apache-2.0): a cosine-weighted
// hemisphere AO estimator with deterministic per-pixel sampling (math only, not
// ISPC). AO is computed on mesh hits and modulates the ambient term; it is gated
// off by default (RenderOptions::aoSamples == 0) so flag-less output is unchanged.

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

// Shared POV local-illumination shader (no 1/pi factor). C is the pigment rgb,
// N the face-forwarded shading normal, V the unit direction toward the viewer.
// out = emission*C + aoFactor*ambient*C*ambLight
//       + sum_lights[ diffuse*C*pow(N.L,brilliance)*Lc + specular/phong lobe ]
//       + reflection*background.
// The FLAT preset (ambient 1, diffuse 0, specular 0, phong 0) with ambLight
// (1,1,1) yields out = C, preserving the flat outline/silhouette behavior.
inline Vec3 shadeLocal(const Material& mat, const Vec3& C, const Vec3& N,
                       const Vec3& V, const std::vector<Light>& lights,
                       const Vec3& ambLight, const Vec3& bg, float specularScale,
                       float aoFactor, const Vec3& P, const Vec3& Ng, float eps,
                       RTCScene rscene, bool shadowsOn, int shadowSamples,
                       uint32_t px, uint32_t py) {
  Vec3 out{mat.emission * C.x + aoFactor * mat.ambient * C.x * ambLight.x,
           mat.emission * C.y + aoFactor * mat.ambient * C.y * ambLight.y,
           mat.emission * C.z + aoFactor * mat.ambient * C.z * ambLight.z};

  const float exp = blinnExp(mat.roughness);
  // Per-pixel deterministic RNG stream for soft-shadow sampling (advanced per
  // light/sample inside computeShadow); seeded from the hi-res pixel only, so
  // the penumbra is identical regardless of TBB thread count.
  uint32_t s0 = px, s1 = py;
  for (const Light& l : lights) {
    const float ndl = dot(N, l.L);
    if (ndl <= 0.0f) continue;

    // Per-light shadow factor folded into a local light color Lc, applied to
    // BOTH the diffuse and the highlight below. With shadowsOn == false (or a
    // fully lit point) Lc == l.color bitwise, so the shadow-off render is
    // byte-identical to the pre-shadow output.
    const float shadowFactor =
        shadowsOn ? computeShadow(rscene, P, Ng, N, eps, l, shadowSamples, s0, s1)
                  : 1.0f;
    const Vec3 Lc{l.color.x * shadowFactor, l.color.y * shadowFactor,
                  l.color.z * shadowFactor};

    // Diffuse with brilliance as the N.L exponent. Guard pow(0,0).
    float d;
    if (mat.brilliance == 1.0f) {
      d = ndl;
    } else if (mat.brilliance == 0.0f) {
      d = 1.0f;
    } else {
      d = std::pow(ndl, mat.brilliance);
    }
    const float dk = mat.diffuse * d;
    out.x += dk * C.x * Lc.x;
    out.y += dk * C.y * Lc.y;
    out.z += dk * C.z * Lc.z;

    // POV fill (shadowless) lights contribute diffuse only -- no specular/phong
    // (trace.cpp gates highlights on Light_Type != FILL_LIGHT_SOURCE).
    if (!l.highlight) continue;

    // Highlight color. POV "metallic" tints the highlight toward the pigment by
    // an empirical Fresnel factor f(N.L): head-on (f=0) the highlight is fully
    // pigment-tinted, at grazing (f=1) it desaturates to the light color
    // (trace.cpp ComputeSpecularColour/ComputePhongColour). Non-metallic uses
    // the plain light color.
    Vec3 hl;
    if (mat.metallic) {
      float c = ndl;
      if (c > 1.0f) c = 1.0f;
      const float x = std::acos(c) * 0.63661977f;  // (angle)/(pi/2), 0..1
      float f = 0.014567225f / ((x - 1.12f) * (x - 1.12f)) - 0.011612903f;
      if (f < 0.0f) f = 0.0f;
      if (f > 1.0f) f = 1.0f;
      // cs = light * (f + (1-f)*pigment): lerp pigment->white by f.
      hl = Vec3{Lc.x * (f + (1.0f - f) * C.x),
                Lc.y * (f + (1.0f - f) * C.y),
                Lc.z * (f + (1.0f - f) * C.z)};
    } else {
      hl = Lc;
    }

    float specW = 0.0f;  // accumulated scalar specular weight
    // Blinn highlight (POV "specular S roughness R"), gated on specular > 0.
    if (mat.specular > 0.0f) {
      const Vec3 H = normalize(Vec3{l.L.x + V.x, l.L.y + V.y, l.L.z + V.z});
      float nh = dot(N, H);
      if (nh > 0.0f) specW += mat.specular * std::pow(nh, exp);
    }
    // Phong highlight (POV "phong P phong_size PS"), gated on phong > 0.
    // POV-faithful: intensity = phong * pow(R.V, phong_size) with no clamp, so a
    // large phong (e.g. 10000) saturates the channel to a white pip exactly as
    // POV-Ray does (ComputePhongColour). The supersample box-average then mirrors
    // POV's antialiasing of that crest. POV skips the term for tiny reflections
    // at high phong_size (phong_size >= 60 && R.V <= 0.0008).
    if (mat.phong > 0.0f) {
      const Vec3 Rr = 2.0f * ndl * N - l.L;
      float rv = dot(Rr, V);
      if (rv > 0.0f && (mat.phongSize < 60.0f || rv > 0.0008f))
        specW += mat.phong * std::pow(rv, mat.phongSize);
    }
    if (specW > 0.0f) {
      const float s = specW * specularScale;
      out.x += s * hl.x;
      out.y += s * hl.y;
      out.z += s * hl.z;
    }
  }

  // Cheap reflection: add reflection * background (no second ray). On scene4's
  // white background with no env geometry this matches POV non-radiosity.
  if (mat.reflection > 0.0f) {
    out.x += mat.reflection * bg.x;
    out.y += mat.reflection * bg.y;
    out.z += mat.reflection * bg.z;
  }
  return out;
}

}  // namespace detail
}  // namespace umbreon
