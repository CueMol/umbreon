// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Principled BSDF subset, direct-pass evaluation (Material::model ==
// ShadingModel::Principled): energy-conserving GGX metallic-roughness model
// replacing the POV finish lobes. See docs/principled_design.md for the
// parameter semantics and the energy convention.
//
// Energy units: the POV direct pass carries no 1/pi on diffuse (the light's
// physical irradiance is effectively pi * Lc), so the specular term is
// pi * f_ggx * (N.L) * Lc = pi * D * G2 * F / (4 cos_v) * Lc -- the pi
// compensates the light units, keeping principled and POV materials
// comparable side by side in one scene.
//
// Layering: shading/ must not include experimental/pt2/, so the Smith Lambda
// here intentionally duplicates pt2GgxLambda (pt2_glossy.hpp) -- keep the two
// in sync if the formula ever changes.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// Floor on the direct-pass GGX alpha: below this D() degenerates toward a
// delta and the float math toward NaN. (The traced-reflection pass has its
// own mirror cut, kPt2GlossyAlphaMin, which is larger.)
constexpr float kGgxDirectAlphaMin = 1.0e-3f;

// Diffuse-lobe weight: Material::diffuse doubles as the principled base
// weight. metallic == 0 reproduces m.diffuse bitwise (x * (1 - 0) == x).
inline float principledDiffuseWeight(const Material& m) {
  return m.diffuse * (1.0f - m.pbr.metallic);
}

// Fresnel F0: dielectric 0.08 * specular, lerped to the pigment for metals.
inline Vec3 principledF0(const Material& m, const Vec3& C) {
  const float d = 0.08f * m.pbr.specular;
  const float t = m.pbr.metallic;
  return Vec3{d + (C.x - d) * t, d + (C.y - d) * t, d + (C.z - d) * t};
}

// Schlick Fresnel with F90 = 1.
inline Vec3 schlickF(const Vec3& F0, float u) {
  float c = 1.0f - u;
  if (c < 0.0f) c = 0.0f;
  if (c > 1.0f) c = 1.0f;
  const float c5 = c * c * c * c * c;
  return Vec3{F0.x + (1.0f - F0.x) * c5, F0.y + (1.0f - F0.y) * c5,
              F0.z + (1.0f - F0.z) * c5};
}

// Isotropic GGX NDF (a2 = alpha^2). Returns 0 below the horizon.
inline float ggxD(float a2, float ndh) {
  if (ndh <= 0.0f) return 0.0f;
  const float t = ndh * ndh * (a2 - 1.0f) + 1.0f;
  return a2 / (3.14159265358979323846f * t * t);
}

// Smith Lambda for isotropic GGX (see pt2GgxLambda -- intentional duplicate,
// keep in sync). cosT is the |cosine| of the direction to the normal.
inline float ggxLambda(float a2, float cosT) {
  const float c2 = std::fmax(cosT * cosT, 1.0e-12f);
  const float t2 = (1.0f - c2) / c2;
  return 0.5f * (std::sqrt(1.0f + a2 * t2) - 1.0f);
}

// Disney anisotropy remap: lobe stretch factor. anisotropy 0 -> aspect 1.
inline float principledAspect(float anisotropy) {
  float a = std::fabs(anisotropy);
  if (a > 1.0f) a = 1.0f;
  return std::sqrt(1.0f - 0.9f * a);
}

// Anisotropic GGX NDF; (hx, hy, hz) are the half vector's components in the
// (t1, t2, N) tangent frame.
inline float ggxDAniso(float ax, float ay, float hx, float hy, float hz) {
  if (hz <= 0.0f) return 0.0f;
  const float sx = hx / ax, sy = hy / ay;
  const float t = sx * sx + sy * sy + hz * hz;
  return 1.0f / (3.14159265358979323846f * ax * ay * t * t);
}

// Anisotropic Smith Lambda; (vx, vy, vz) in the same tangent frame.
inline float ggxLambdaAniso(float ax, float ay, float vx, float vy, float vz) {
  const float z2 = std::fmax(vz * vz, 1.0e-12f);
  const float t2 = (ax * ax * vx * vx + ay * ay * vy * vy) / z2;
  return 0.5f * (std::sqrt(1.0f + t2) - 1.0f);
}

// Per-hit anisotropic shading context: the rotation-baked tangent frame and
// the stretched alphas. Valid only when `active` (a tangent frame exists AND
// the material is anisotropic); otherwise the caller keeps the isotropic
// code path verbatim.
struct PrincipledAniso {
  bool active = false;
  Vec3 t1, t2;      // world-space tangent frame (t1 rotated, t2 = N x t1)
  float ax = 0.0f;  // alpha along t1
  float ay = 0.0f;  // alpha along t2
};

inline PrincipledAniso principledAnisoCtx(const Material& mat, const Vec3& N,
                                          const Vec3* tangent, float alpha) {
  PrincipledAniso c;
  if (tangent == nullptr || mat.pbr.anisotropy == 0.0f) return c;
  c.active = true;
  c.t1 = *tangent;
  c.t2 = cross(N, c.t1);
  const float aspect = principledAspect(mat.pbr.anisotropy);
  c.ax = alpha / aspect;
  if (c.ax > 1.0f) c.ax = 1.0f;
  c.ay = alpha * aspect;
  if (c.ax < kGgxDirectAlphaMin) c.ax = kGgxDirectAlphaMin;
  if (c.ay < kGgxDirectAlphaMin) c.ay = kGgxDirectAlphaMin;
  return c;
}

// Principled direct shading for one hit. Mirrors shadeLocal's contract
// (same ambient/emission init, same fill-light and horizon gates, aoFactor
// on ambient / diffuseAo on diffuse only -- highlights are never occluded,
// as in POV). Differences by design: single energy-conserving GGX lobe with
// Schlick Fresnel instead of Blinn+Phong; under an area light (l.radius > 0,
// shadowSamples > 1) visibility and BRDF are evaluated together per cone
// sample (a non-separable estimator -- the highlight softens with the same
// radius as the penumbra), unlike POV's mean-visibility x center-BRDF.
// `traceReflection` mirrors shadeLocal's flag: when the pt2 traced pass owns
// this pixel's specular indirect, the fake environment term is skipped.
// `tangent` is the rotation-baked anisotropy tangent (world space, unit) for
// sphere/cylinder primitives, or nullptr for the isotropic path (mesh hits,
// pole/cap fallbacks, anisotropy == 0).
inline Vec3 shadePrincipled(const Material& mat, const Vec3& C, const Vec3& N,
                            const Vec3& V, const std::vector<Light>& lights,
                            const Vec3& ambLight, const Vec3& bg,
                            const Vec3& aoFactor, float diffuseAo,
                            const Vec3& P, const Vec3& Ng, float eps,
                            RTCScene rscene, bool shadowsOn, int shadowSamples,
                            uint32_t px, uint32_t py,
                            bool traceReflection = false,
                            const Vec3* tangent = nullptr) {
  // Identical init expression to shadeLocal (shared ambient/emission
  // semantics; Route A zeroes ambLight for GI-eligible hits).
  Vec3 out{mat.emission * C.x + aoFactor.x * mat.ambient * C.x * ambLight.x,
           mat.emission * C.y + aoFactor.y * mat.ambient * C.y * ambLight.y,
           mat.emission * C.z + aoFactor.z * mat.ambient * C.z * ambLight.z};

  const float kd = principledDiffuseWeight(mat);
  const Vec3 F0 = principledF0(mat, C);
  const float f0max = std::fmax(F0.x, std::fmax(F0.y, F0.z));
  float alpha = mat.pbr.roughness * mat.pbr.roughness;
  if (alpha < kGgxDirectAlphaMin) alpha = kGgxDirectAlphaMin;
  const float a2 = alpha * alpha;
  const float ndv = std::fmax(dot(N, V), 1.0e-4f);
  const float lamV = ggxLambda(a2, ndv);
  constexpr float kPi = 3.14159265358979323846f;
  // Anisotropic context (inactive keeps every isotropic expression below
  // verbatim -- anisotropy 0 is bitwise the isotropic path).
  const PrincipledAniso an = principledAnisoCtx(mat, N, tangent, alpha);
  const float lamVA =
      an.active ? ggxLambdaAniso(an.ax, an.ay, dot(V, an.t1), dot(V, an.t2),
                                 ndv)
                : 0.0f;

  // Two per-pixel RNG streams: the single-direction path reuses the POV
  // seeding (px, py) through computeShadow so the hard-shadow diffuse render
  // is bitwise-comparable to POV; the area path draws its own tagged stream
  // (advanced across lights in order -- a pure per-pixel function, so the
  // penumbra/highlight are thread-count invariant).
  uint32_t s0 = px, s1 = py;
  uint32_t t0 = px ^ 0x50425244u /* 'PBRD' */, t1 = py;

  for (const Light& l : lights) {
    const float ndl = dot(N, l.L);
    if (ndl <= 0.0f) continue;

    const bool areaPath = shadowsOn && shadowSamples > 1 && l.radius > 0.0f;
    if (!areaPath) {
      // Single-direction path (hard shadow / shadows off / 1 sample): same
      // structure as shadeLocal, GGX highlight instead of Blinn/Phong.
      const float shadowFactor =
          shadowsOn
              ? computeShadow(rscene, P, Ng, N, eps, l, shadowSamples, s0, s1)
              : 1.0f;
      const Vec3 Lc{l.color.x * shadowFactor, l.color.y * shadowFactor,
                    l.color.z * shadowFactor};
      const float dk = kd * ndl * diffuseAo;
      out.x += dk * C.x * Lc.x;
      out.y += dk * C.y * Lc.y;
      out.z += dk * C.z * Lc.z;
      if (!l.highlight) continue;  // fill lights: diffuse only (POV rule)
      if (f0max > 0.0f) {
        const Vec3 H =
            normalize(Vec3{l.L.x + V.x, l.L.y + V.y, l.L.z + V.z});
        const float ndh = dot(N, H);
        if (ndh > 0.0f) {
          float sW;
          if (!an.active) {
            const float G2 = 1.0f / (1.0f + lamV + ggxLambda(a2, ndl));
            // pi * f * (N.L) = pi * D*G2*F / (4 ndv): the N.L cancels.
            sW = kPi * ggxD(a2, ndh) * G2 / (4.0f * ndv);
          } else {
            const float G2 =
                1.0f / (1.0f + lamVA +
                        ggxLambdaAniso(an.ax, an.ay, dot(l.L, an.t1),
                                       dot(l.L, an.t2), ndl));
            sW = kPi *
                 ggxDAniso(an.ax, an.ay, dot(H, an.t1), dot(H, an.t2), ndh) *
                 G2 / (4.0f * ndv);
          }
          const Vec3 F = schlickF(F0, dot(V, H));
          out.x += sW * F.x * Lc.x;
          out.y += sW * F.y * Lc.y;
          out.z += sW * F.z * Lc.z;
        }
      }
    } else {
      // Area path: per-sample visibility x BRDF over the light cone (the
      // highlight spreads with the same angular radius as the penumbra).
      // Below-horizon samples contribute zero but stay in the denominator.
      Vec3 ng = (dot(Ng, N) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
      ng = safeNormalize(ng, N);
      const Vec3 O = P + ng * eps;
      const float far = std::numeric_limits<float>::infinity();
      float accD = 0.0f;
      Vec3 accS{0.0f, 0.0f, 0.0f};
      for (int s = 0; s < shadowSamples; ++s) {
        tea2(t0, t1);
        const Vec3 w = sampleLightDir(l, u32ToUnorm(t0), u32ToUnorm(t1));
        const float ndlS = dot(N, w);
        if (ndlS <= 0.0f) continue;
        if (occluded(rscene, O, w, eps, far)) continue;
        accD += ndlS;
        if (l.highlight && f0max > 0.0f) {
          const Vec3 H = normalize(Vec3{w.x + V.x, w.y + V.y, w.z + V.z});
          const float ndh = dot(N, H);
          if (ndh > 0.0f) {
            float sW;
            if (!an.active) {
              const float G2 = 1.0f / (1.0f + lamV + ggxLambda(a2, ndlS));
              sW = kPi * ggxD(a2, ndh) * G2 / (4.0f * ndv);
            } else {
              const float G2 =
                  1.0f / (1.0f + lamVA +
                          ggxLambdaAniso(an.ax, an.ay, dot(w, an.t1),
                                         dot(w, an.t2), ndlS));
              sW = kPi *
                   ggxDAniso(an.ax, an.ay, dot(H, an.t1), dot(H, an.t2),
                             ndh) *
                   G2 / (4.0f * ndv);
            }
            const Vec3 F = schlickF(F0, dot(V, H));
            accS.x += sW * F.x;
            accS.y += sW * F.y;
            accS.z += sW * F.z;
          }
        }
      }
      const float inv = 1.0f / static_cast<float>(shadowSamples);
      const float dk = kd * diffuseAo * inv * accD;
      out.x += dk * C.x * l.color.x;
      out.y += dk * C.y * l.color.y;
      out.z += dk * C.z * l.color.z;
      out.x += inv * accS.x * l.color.x;
      out.y += inv * accS.y * l.color.y;
      out.z += inv * accS.z * l.color.z;
    }
  }

  // Fake environment specular (the non-pt2 degradation path, mirroring POV's
  // reflection * bg term and its !traceReflection gate). Under pt2 the
  // traced mirror/glossy pass owns the term (traceReflection == true).
  // Amount rules, tuned for raytracing-mode fidelity of converted scenes:
  //  - Material::reflection > 0 (a converted POV finish carries its original
  //    scalar; dormant otherwise under Principled): use reflection * bg --
  //    BIT-FAITHFUL to the POV fake term, so converted metals/mirrors keep
  //    their POV environment brightness.
  //  - else: constant F0 * bg (NOT the Schlick curve: the grazing rise would
  //    paint a bright rim on every dielectric against a bright background,
  //    a term the POV model never had; the Fresnel curve belongs to the
  //    traced pt2 path, where it is evaluated per sample against real
  //    geometry). Skipped entirely at f0max == 0, which keeps the
  //    diffuse-only bitwise parity with the POV model.
  if (!traceReflection) {
    if (mat.reflection > 0.0f) {
      out.x += mat.reflection * bg.x;
      out.y += mat.reflection * bg.y;
      out.z += mat.reflection * bg.z;
    } else if (f0max > 0.0f) {
      out.x += F0.x * bg.x;
      out.y += F0.y * bg.y;
      out.z += F0.z * bg.z;
    }
  }
  return out;
}

}  // namespace detail
}  // namespace umbreon
