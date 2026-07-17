#include "material_convert.hpp"

#include <algorithm>
#include <cmath>

namespace umbreon {
namespace {

// POV roughness -> Blinn exponent. Local copy of detail::blinnExp
// (src/umbreon/shading/shading.hpp) -- bench_core cannot include the shading
// headers (they pull in Embree); keep the two in sync.
float blinnExpLocal(float roughness) {
  float r = roughness;
  if (r < 1e-6f) r = 1e-6f;
  float e = 1.0f / r;
  if (e < 1.0f) e = 1.0f;
  if (e > 1.0e6f) e = 1.0e6f;
  return e;
}

// Blinn/Phong exponent -> perceptual principled roughness via the classic
// microfacet equivalence alpha = sqrt(2 / (e + 2)), then r = sqrt(alpha)
// (principled alpha = r^2). Same map as the pt2 glossy stage's
// pt2GgxAlphaFromRoughness but WITHOUT the mirror snap -- the snap belongs
// to the render-side lobe gate, not to the material parameters.
float exponentToRoughness(float e) {
  const float alpha = std::sqrt(2.0f / (e + 2.0f));
  return std::sqrt(std::min(alpha, 1.0f));
}

float clamp01(float x) { return std::min(1.0f, std::max(0.0f, x)); }

}  // namespace

Material toPrincipledMaterial(const Material& in) {
  if (in.model == ShadingModel::Principled) return in;
  Material out = in;
  out.model = ShadingModel::Principled;
  // ambient / diffuse / emission carry over unchanged (shared semantics;
  // diffuse doubles as the principled base weight).

  const bool hasHighlight = in.specular > 0.0f || in.phong > 0.0f;
  if (in.specular > 0.0f) {
    // POV Blinn: specular amount + roughness (exponent = 1/roughness).
    out.pbr.roughness = exponentToRoughness(blinnExpLocal(in.roughness));
    out.pbr.specular = clamp01(in.specular);
  } else if (in.phong > 0.0f) {
    // POV Phong: amount + phong_size as the exponent directly.
    out.pbr.roughness = exponentToRoughness(std::max(1.0f, in.phongSize));
    out.pbr.specular = clamp01(in.phong);
  } else {
    out.pbr.specular = 0.0f;  // no highlight -> no dielectric lobe
    out.pbr.roughness = 0.5f;
  }

  // POV `metallic` (incl. the F_Metal* presets): a metal. F0 = pigment gives
  // the colored reflection the POV highlight tint only approximated.
  if (in.metallic) out.pbr.metallic = 1.0f;

  if (in.reflection > 0.0f) {
    if (!hasHighlight) {
      // Pure mirror finish (reflection without a highlight): a polished
      // metal -- roughness 0 keeps the traced lobe a single mirror ray.
      out.pbr.metallic = 1.0f;
      out.pbr.roughness = 0.0f;
    } else {
      // Reflection + highlight: POV's un-Fresneled additive reflection has
      // no dielectric equivalent (F0 caps at 0.08 and would nearly erase a
      // `reflection 0.3` floor), so scale toward metal by the reflection
      // amount -- the closest energy match. FLAGGED HEURISTIC: judged
      // against the alternative (keep metallic 0 = physically-correct
      // plastic, much weaker reflections) in the S4 visual round.
      out.pbr.metallic = std::max(out.pbr.metallic, clamp01(in.reflection));
    }
  }
  // Dropped, by design: brilliance (Lambert diffuse), the empirical metallic
  // highlight tint (F0 = pigment supersedes it) and the raw reflection
  // scalar (principled never reads it).
  return out;
}

}  // namespace umbreon
