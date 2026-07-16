// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt2 emissive-triangle NEE (full-PT track, stage 3): a power-weighted list
// of the mesh triangles whose material carries POV `finish emission`, sampled
// as light sources at gather ORIGIN points and MIS-combined (balance
// heuristic) with the cosine gather that can also hit them.
//
// Why MIS here and not for the distant lights: a distant light can never be
// hit by a gather ray (it is not geometry), so light sampling alone is the
// complete estimator. An emissive TRIANGLE is geometry -- the cosine gather
// already finds it (that is how the m1 emissive halo works). NEE without MIS
// would double-count; NEE-only (dropping emission at hits) would be noisy
// exactly where BSDF sampling excels (large nearby emitters). The balance
// heuristic keeps both estimators and weighs each where it is strong.
//
// Scope: NEE runs at the gather ORIGIN (path vertex 0) only -- that covers
// the dominant emitter -> receiver transport; emitters seen from DEEPER path
// vertices keep the plain emission-at-hit estimator at full weight (a
// consistent per-vertex partition, unbiased). Mesh triangles only: emissive
// CSG primitives (spheres/bonds) keep BSDF-only sampling.
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

// One emissive triangle: world-space corners, interpolatable radiance, and
// the sampling bookkeeping (area, power CDF position).
struct Pt2EmissiveTri {
  Vec3 p0, p1, p2;
  Vec3 e0, e1, e2;   // per-vertex emitted radiance (emission * pigment)
  Vec3 ng;           // unit geometric normal
  float area = 0.0f;
  std::uint32_t primID = 0;  // mesh triangle index (MIS hit lookup)
};

struct Pt2EmissiveLights {
  std::vector<Pt2EmissiveTri> tris;
  std::vector<float> cdf;        // power-weighted, cdf[i] = P(tri <= i)
  std::vector<float> pdfChoose;  // per-tri selection probability
  // primID -> index into tris (0xFFFFFFFF = not emissive); sized to the mesh
  // triangle count for O(1) MIS pdf lookup at gather hits.
  std::vector<std::uint32_t> triIndex;
  bool empty() const noexcept { return tris.empty(); }
};

inline float pt2EmissiveLuminance(const Vec3& c) noexcept {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

// Scan the mesh for emissive triangles and build the power-weighted CDF.
// Returns an empty list when the scene has no emissive mesh material.
inline Pt2EmissiveLights pt2BuildEmissiveLights(const Mesh& mesh) {
  Pt2EmissiveLights out;
  const std::size_t nt = mesh.triangleCount();
  bool any = false;
  for (const Material& mm : mesh.materials)
    any = any || mm.emission > 0.0f;
  if (!any && mesh.material.emission <= 0.0f) return out;

  out.triIndex.assign(nt, 0xFFFFFFFFu);
  std::vector<float> power;
  for (std::size_t t = 0; t < nt; ++t) {
    const Material& mm = mesh.materialForTri(t);
    if (mm.emission <= 0.0f) continue;
    Pt2EmissiveTri et;
    const std::uint32_t i0 = mesh.cornerVertex(t * 3 + 0);
    const std::uint32_t i1 = mesh.cornerVertex(t * 3 + 1);
    const std::uint32_t i2 = mesh.cornerVertex(t * 3 + 2);
    et.p0 = mesh.positions[i0];
    et.p1 = mesh.positions[i1];
    et.p2 = mesh.positions[i2];
    const Vec4& c0 = mesh.colors[i0];
    const Vec4& c1 = mesh.colors[i1];
    const Vec4& c2 = mesh.colors[i2];
    et.e0 = Vec3{mm.emission * c0.x, mm.emission * c0.y, mm.emission * c0.z};
    et.e1 = Vec3{mm.emission * c1.x, mm.emission * c1.y, mm.emission * c1.z};
    et.e2 = Vec3{mm.emission * c2.x, mm.emission * c2.y, mm.emission * c2.z};
    const Vec3 a{et.p1.x - et.p0.x, et.p1.y - et.p0.y, et.p1.z - et.p0.z};
    const Vec3 b{et.p2.x - et.p0.x, et.p2.y - et.p0.y, et.p2.z - et.p0.z};
    const Vec3 cr{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x};
    const float len = std::sqrt(dot(cr, cr));
    if (len <= 0.0f) continue;
    et.area = 0.5f * len;
    et.ng = Vec3{cr.x / len, cr.y / len, cr.z / len};
    et.primID = static_cast<std::uint32_t>(t);
    const Vec3 eAvg{(et.e0.x + et.e1.x + et.e2.x) / 3.0f,
                    (et.e0.y + et.e1.y + et.e2.y) / 3.0f,
                    (et.e0.z + et.e1.z + et.e2.z) / 3.0f};
    const float p = et.area * pt2EmissiveLuminance(eAvg);
    if (p <= 0.0f) continue;
    out.triIndex[t] = static_cast<std::uint32_t>(out.tris.size());
    out.tris.push_back(et);
    power.push_back(p);
  }
  if (out.tris.empty()) {
    out.triIndex.clear();
    return out;
  }
  double total = 0.0;
  for (float p : power) total += p;
  out.cdf.resize(power.size());
  out.pdfChoose.resize(power.size());
  double acc = 0.0;
  for (std::size_t i = 0; i < power.size(); ++i) {
    acc += power[i];
    out.cdf[i] = static_cast<float>(acc / total);
    out.pdfChoose[i] = static_cast<float>(power[i] / total);
  }
  out.cdf.back() = 1.0f;  // guard against rounding
  return out;
}

// Solid-angle pdf of the NEE strategy for a ray from P that hits emissive
// triangle `ti` at distance `dist` with |cos| `cosL` at the light surface:
// pdfChoose * dist^2 / (cosL * area). Used both to weight the NEE draw and,
// through the triIndex lookup, to MIS-weight a cosine-gather hit.
inline float pt2EmissivePdfSA(const Pt2EmissiveLights& el, std::uint32_t ti,
                              float dist2, float cosL) noexcept {
  if (cosL <= 1.0e-6f) return 0.0f;
  const Pt2EmissiveTri& t = el.tris[ti];
  return el.pdfChoose[ti] * dist2 / (cosL * t.area);
}

// One emissive-NEE sample from receiver (P, N): pick a triangle from the
// power CDF, a uniform point on it, test visibility, and return the
// MIS-weighted contribution to E_stored (the E_true/pi convention: the
// estimator term is L_e * cosThetaP / (pi * pdfSA), balance-weighted against
// the cosine pdf cosThetaP/pi). rng is the caller's tea2 stream.
inline Vec3 pt2EmissiveNee(const Pt2EmissiveLights& el, RTCScene scene,
                           const Vec3& P, const Vec3& N, const Vec3& Ng,
                           float epsT, std::uint32_t& s0, std::uint32_t& s1) {
  // Triangle pick (binary search over the CDF).
  tea2(s0, s1);
  const float u = u32ToUnorm(s0);
  std::uint32_t lo = 0, hi = static_cast<std::uint32_t>(el.cdf.size() - 1);
  while (lo < hi) {
    const std::uint32_t mid = (lo + hi) / 2;
    if (el.cdf[mid] < u)
      lo = mid + 1;
    else
      hi = mid;
  }
  const Pt2EmissiveTri& t = el.tris[lo];

  // Uniform point on the triangle (sqrt warp) + interpolated radiance.
  tea2(s0, s1);
  float b0 = u32ToUnorm(s0), b1 = u32ToUnorm(s1);
  const float sq = std::sqrt(b0);
  const float w1 = 1.0f - sq, w2 = b1 * sq, w0 = 1.0f - w1 - w2;
  const Vec3 S{t.p0.x * w0 + t.p1.x * w1 + t.p2.x * w2,
               t.p0.y * w0 + t.p1.y * w1 + t.p2.y * w2,
               t.p0.z * w0 + t.p1.z * w1 + t.p2.z * w2};
  const Vec3 Le{t.e0.x * w0 + t.e1.x * w1 + t.e2.x * w2,
                t.e0.y * w0 + t.e1.y * w1 + t.e2.y * w2,
                t.e0.z * w0 + t.e1.z * w1 + t.e2.z * w2};

  const Vec3 d{S.x - P.x, S.y - P.y, S.z - P.z};
  const float dist2 = dot(d, d);
  if (dist2 <= 1.0e-12f) return Vec3{0.0f, 0.0f, 0.0f};
  const float dist = std::sqrt(dist2);
  const Vec3 dir{d.x / dist, d.y / dist, d.z / dist};
  const float cosP = dot(N, dir);
  if (cosP <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};
  // Two-sided emitter: light leaves both faces (POV emission has no side).
  const float cosL = std::fabs(dot(t.ng, dir));
  const float pdfSA = pt2EmissivePdfSA(el, lo, dist2, cosL);
  if (pdfSA <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};

  // Visibility (finite ray, backed off both ends).
  const float eps = selfIntersectEps(P, dir, epsT);
  Vec3 ng = (dot(Ng, N) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
  ng = safeNormalize(ng, N);
  const Vec3 O{P.x + ng.x * eps, P.y + ng.y * eps, P.z + ng.z * eps};
  if (occluded(scene, O, dir, eps, dist - 2.0f * eps))
    return Vec3{0.0f, 0.0f, 0.0f};

  // Balance-heuristic MIS against the cosine gather (pdf cosP/pi) that can
  // also find this emitter, then the E_stored estimator term.
  const float pdfCos = cosP * (1.0f / 3.14159265358979323846f);
  const float w = pdfSA / (pdfSA + pdfCos);
  const float k = w * cosP / (3.14159265358979323846f * pdfSA);
  return Vec3{Le.x * k, Le.y * k, Le.z * k};
}

// MIS weight for the COSINE side: a first-bounce gather ray that hit emissive
// mesh triangle `primID` at distance `tfar` with direction `wi` (receiver
// cosine cosP at the origin). 1.0 when the hit is not an NEE-covered emitter.
inline float pt2EmissiveHitWeight(const Pt2EmissiveLights& el,
                                  std::uint32_t primID, float tfar, float cosP,
                                  const Vec3& wi) noexcept {
  if (el.empty() || primID >= el.triIndex.size()) return 1.0f;
  const std::uint32_t ti = el.triIndex[primID];
  if (ti == 0xFFFFFFFFu) return 1.0f;
  const float cosL = std::fabs(dot(el.tris[ti].ng, wi));
  const float pdfSA = pt2EmissivePdfSA(el, ti, tfar * tfar, cosL);
  if (pdfSA <= 0.0f) return 1.0f;
  const float pdfCos = ((cosP > 0.0f) ? cosP : 0.0f) *
                       (1.0f / 3.14159265358979323846f);
  if (pdfCos <= 0.0f) return 1.0f;
  return pdfCos / (pdfCos + pdfSA);
}

}  // namespace detail
}  // namespace umbreon
