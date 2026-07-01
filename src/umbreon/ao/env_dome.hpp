// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Environment dome lighting: a synthetic hemisphere of distant diffuse-only
// lights around the camera-forward axis, approximating sky/environment fill.
// Designed to be used TOGETHER WITH ambient occlusion (ao/) -- the dome makes
// shadows active, so recesses and buried atoms darken via real shadows instead
// of a uniform direct-diffuse dimming, giving AO-style depth without the "flat
// flashlight" look of the frontal key light. WORK IN PROGRESS: the default light
// count / intensity are not yet finalized (see the env-dome banding notes).
//
// Called once per frame from the renderer (not on the per-pixel path), so the
// cost of factoring it into a function is nil. Opt-in via opt.envLights > 0.
#pragma once

#include <cmath>
#include <vector>

#include "render/render_types.hpp"
#include "scene.hpp"
#include "shading/secondary_rays.hpp"  // Light

namespace umbreon {
namespace detail {

// Append the environment-dome lights to `lights`. The scene's own lights are
// first scaled by opt.envKeyScale (0 = dome only) to drop the frontal
// "flashlight" component. Each dome light's travel direction d sits in the cone
// of half-angle opt.envAngle about the camera-forward axis `dir`, so its
// toward-light L = -d is on the camera side and every camera-facing point is
// lit. Per-light intensity is normalized so a fully-exposed point whose normal
// faces the eye (-dir) receives diffuse irradiance == opt.envIntensity. `dir`,
// `right`, `trueUp` are the orthonormal camera basis. No-op if envLights <= 0.
inline void buildEnvDomeLights(std::vector<Light>& lights,
                               const RenderOptions& opt, const Vec3& dir,
                               const Vec3& right, const Vec3& trueUp) {
  if (opt.envLights <= 0) return;
  for (Light& l : lights) l.color = l.color * opt.envKeyScale;
  const int n = opt.envLights;
  const float cosMax = std::cos(radians(opt.envAngle));  // forward..rim cosine
  const float ga = 3.14159265f * (3.0f - std::sqrt(5.0f));  // golden angle
  float cosSum = 0.0f;
  std::vector<Vec3> dirs;
  dirs.reserve(n);
  for (int i = 0; i < n; ++i) {
    // cos(theta) uniformly in [cosMax, 1] (area-uniform cap), azimuth by golden
    // angle; theta measured from the camera-forward axis.
    const float z = 1.0f - (1.0f - cosMax) * ((i + 0.5f) / n);
    const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
    const float t = ga * i;
    Vec3 d = normalize(dir * z + right * (r * std::cos(t)) +
                       trueUp * (r * std::sin(t)));
    dirs.push_back(d);
    cosSum += z;  // n.L for refN = -dir, L = -d  ->  (-dir).(-d) = dir.d = z
  }
  const float perLight = cosSum > 0.0f ? opt.envIntensity / cosSum : 0.0f;
  for (const Vec3& d : dirs) {
    Light l;
    l.L = Vec3{-d.x, -d.y, -d.z};  // unit direction toward the light
    l.color = Vec3{perLight, perLight, perLight};
    l.highlight = false;  // diffuse-only: no specular flash from the dome
    l.radius = radians(opt.lightRadius);
    lights.push_back(l);
  }
}

}  // namespace detail
}  // namespace umbreon
