// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt2 ReSTIR-GI reservoir: weighted reservoir resampling of one-bounce gather
// samples (Ouyang et al. 2021; GRIS theory Lin et al. 2022). A reservoir holds
// ONE candidate sample point -- the first-bounce hit of a gather path, with
// the FULL multi-bounce radiance the path carried from it -- plus the RIS
// bookkeeping (M = candidates seen, W = unbiased contribution weight).
//
// Radiance reuse across pixels is exact for our surfaces: every GI receiver
// and scatterer is Lambertian, so the radiance leaving a sample point is
// direction-independent and a neighbor's sample can be adopted by only
// re-weighting geometry (the reconnection Jacobian), never re-shading.
//
// Convention: the resolved output is E_stored = f(s) * W / pi with
// f = radiance * max(0, cos theta) -- identical to pt1's mean(L_i) (check:
// a single candidate with no reuse gives W = pi/cos, so E_stored = L).
#pragma once

#include <cmath>
#include <cstdint>

#include "scene.hpp"

namespace umbreon {
namespace detail {

// Virtual sample-point distance for gather rays that escape to the sky. A
// finite-but-huge anchor lets the standard reconnection Jacobian handle env
// samples with no special case: the distance ratio and both cosines approach
// 1 for any pair of nearby receivers. Squared it stays well inside float
// range (1e14 << 3.4e38).
constexpr float kPt2EnvDistance = 1.0e7f;

struct Pt2Reservoir {
  Vec3 samplePos{0.0f, 0.0f, 0.0f};   // x_s: first-bounce hit (or env anchor)
  Vec3 sampleNormal{0.0f, 0.0f, 0.0f};  // n_s at x_s (env: -ray direction)
  Vec3 radiance{0.0f, 0.0f, 0.0f};      // FULL path radiance from x_s
  float M = 0.0f;                       // candidates represented
  float W = 0.0f;                       // unbiased contribution weight
};

inline float pt2Luminance(const Vec3& c) noexcept {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

// Target function p_hat at a receiver (position P, shading normal N) for a
// stored sample: luminance of the sample radiance times the receiver cosine,
// NO floor. Falcor floors the cosine at 0.1, but with a cosine-pdf candidate
// stream that floor makes the creation-stage RIS weight p_hat/p diverge as
// 0.1/cos for grazing candidates -- measured as dark speckle on a flat wall
// (grazing in-plane self-hits win reservoirs they contribute nothing to).
// Un-floored, p_hat is exactly proportional to the integrand's luminance and
// the creation weight cancels analytically (see pt1GatherPoint).
inline float pt2TargetPdf(const Vec3& P, const Vec3& N, const Vec3& samplePos,
                          const Vec3& radiance) noexcept {
  const Vec3 d{samplePos.x - P.x, samplePos.y - P.y, samplePos.z - P.z};
  const float len2 = dot(d, d);
  if (len2 <= 0.0f) return 0.0f;
  const float c = dot(N, d) / std::sqrt(len2);
  if (c <= 0.0f) return 0.0f;
  return pt2Luminance(radiance) * c;
}

// Reconnection Jacobian for adopting a sample created at visible point x1_old
// by a receiver at x1_new (the standard form all reference implementations
// share):  J = (||x_s - x1_old||^2 * cos phi_new) / (||x_s - x1_new||^2 *
// cos phi_old), where the cosines are measured at the SAMPLE's normal.
// Degenerate configurations (behind the sample surface, zero distances,
// non-finite) return 0; the result is clamped to [0, clampMax] -- an
// unclamped Jacobian at concave corners is the classic ReSTIR firefly source.
inline float pt2Jacobian(const Vec3& newRecv, const Vec3& oldRecv,
                         const Vec3& samplePos, const Vec3& sampleN,
                         float clampMax) noexcept {
  const Vec3 a{newRecv.x - samplePos.x, newRecv.y - samplePos.y,
               newRecv.z - samplePos.z};
  const Vec3 b{oldRecv.x - samplePos.x, oldRecv.y - samplePos.y,
               oldRecv.z - samplePos.z};
  const float ra2 = dot(a, a);
  const float rb2 = dot(b, b);
  if (ra2 <= 0.0f || rb2 <= 0.0f) return 0.0f;
  // cos at the sample surface toward each receiver (unnormalized dot / len).
  const float ca = dot(sampleN, a) / std::sqrt(ra2);
  const float cb = dot(sampleN, b) / std::sqrt(rb2);
  if (ca <= 0.0f || cb <= 0.0f) return 0.0f;
  const float j = (rb2 * ca) / (ra2 * cb);
  if (!std::isfinite(j) || j < 0.0f) return 0.0f;
  return (j > clampMax) ? clampMax : j;
}

// Streaming reservoir update (the canonical rule): add a candidate carrying
// RIS weight `w` and `m` represented samples; returns true when the candidate
// replaced the current selection. `u` is the caller's uniform draw.
inline bool pt2ReservoirUpdate(double& wSum, float& M, float w, float m,
                               float u) noexcept {
  wSum += static_cast<double>(w);
  M += m;
  return w > 0.0f && static_cast<double>(u) * wSum <= static_cast<double>(w);
}

// Resolve a reservoir to E_stored at receiver (P, N): f(s) * W / pi with
// f = radiance * max(0, cos). W is expected already normalized (and clamped)
// by the finalize step of the pass that produced the reservoir.
inline Vec3 pt2Resolve(const Pt2Reservoir& r, const Vec3& P,
                       const Vec3& N) noexcept {
  if (r.W <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};
  const Vec3 d{r.samplePos.x - P.x, r.samplePos.y - P.y, r.samplePos.z - P.z};
  const float len2 = dot(d, d);
  if (len2 <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};
  float c = dot(N, d) / std::sqrt(len2);
  if (c <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};
  const float k = c * r.W * (1.0f / 3.14159265358979323846f);
  return Vec3{r.radiance.x * k, r.radiance.y * k, r.radiance.z * k};
}

}  // namespace detail
}  // namespace umbreon
