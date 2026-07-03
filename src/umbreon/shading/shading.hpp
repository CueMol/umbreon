// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// POV-Ray-style local illumination model for the Embree renderer: the per-hit
// shader (shadeLocal) that turns a pigment + normal + lights into a shaded
// color, matching POV-Ray's diffuse / Blinn-specular / Phong / metallic /
// reflection terms. The secondary-ray visibility it folds in (per-light shadows)
// comes from secondary_rays.hpp; ambient occlusion is applied by the caller.
//
// On the hot per-hit path, so these are `inline`; the per-hit dispatch
// (hit_shader.hpp) includes this and the compiler inlines shadeLocal into the
// pixel loop (no LTO needed) -- speed is unchanged from the pre-split monolith.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <embree4/rtcore.h>

#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

inline Vec3 faceForward(Vec3 n, Vec3 rayDir) {
  // POV/CueMol normals should point toward the viewer; flip if back-facing.
  return (dot(n, rayDir) > 0.0f) ? Vec3{-n.x, -n.y, -n.z} : n;
}

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

// Shared POV local-illumination shader (no 1/pi factor). C is the pigment rgb,
// N the face-forwarded shading normal, V the unit direction toward the viewer.
// out = emission*C + aoFactor*ambient*C*ambLight
//       + sum_lights[ diffuse*C*pow(N.L,brilliance)*Lc + specular/phong lobe ]
//       + reflection*background.
// The FLAT preset (ambient 1, diffuse 0, specular 0, phong 0) with ambLight
// (1,1,1) yields out = C, preserving the flat outline/silhouette behavior.
// aoFactor is a per-channel Vec3 (1,1,1) = no occlusion; the legacy scalar AO
// passes {s,s,s}, so the same float op sequence runs and the output is
// bit-identical. The per-channel form lets the albedo-aware multibounce term
// (phase 3) lift each channel differently.
// diffuseAo scales ONLY the direct diffuse accumulation (not ambient, not
// specular): 1 = no effect (POV/scivis contract, bit-exact), <1 darkens diffuse
// in occluded cavities as a coarse indirect-shadowing approximation. The outline
// path and the legacy AO path pass 1.0f, so x*1 keeps those outputs unchanged.
inline Vec3 shadeLocal(const Material& mat, const Vec3& C, const Vec3& N,
                       const Vec3& V, const std::vector<Light>& lights,
                       const Vec3& ambLight, const Vec3& bg, float specularScale,
                       const Vec3& aoFactor, float diffuseAo, const Vec3& P,
                       const Vec3& Ng, float eps, RTCScene rscene, bool shadowsOn,
                       int shadowSamples, uint32_t px, uint32_t py,
                       const RayProbe* probe = nullptr) {
  Vec3 out{mat.emission * C.x + aoFactor.x * mat.ambient * C.x * ambLight.x,
           mat.emission * C.y + aoFactor.y * mat.ambient * C.y * ambLight.y,
           mat.emission * C.z + aoFactor.z * mat.ambient * C.z * ambLight.z};

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
        shadowsOn ? computeShadow(rscene, P, Ng, N, eps, l, shadowSamples, s0,
                                  s1, probe)
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
    const float dk = mat.diffuse * d * diffuseAo;
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
