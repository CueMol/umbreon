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
// consistent per-vertex partition, unbiased).
//
// Emitter kinds covered: mesh triangles, REAL CSG spheres (uniform
// solid-angle cone sampling) and REAL capped cylinders (uniform-area
// sampling of the lateral surface + the two disk caps, one CDF record each).
// NOT covered (BSDF-only sampling, hit weight 1): open cylinder chains
// (ROUND_LINEAR_CURVE primID mapping would need the chain builder
// replicated, and they are almost always outline decoration) and any
// fromEdgeMacro decoration (pt1EvalVertex absorbs those paths, so the BSDF
// side never collects their emission -- NEE-sampling them would build a
// one-sided, inconsistent estimator).
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include "render/scene_build.hpp"
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

// One emissive CSG record: a whole sphere, a capped cylinder's lateral
// surface, or one of its disk caps. Uniform radiance per primitive (CSG
// pigment is per-primitive, no interpolation), so Le is a constant --
// definitionally identical to the emission * pigment the BSDF side collects
// at a CSG hit (pt1EvalVertex), which MIS requires.
enum class Pt2CsgEmitterKind : std::uint8_t { Sphere, CylSide, CylCap };

struct Pt2EmissiveCsg {
  Pt2CsgEmitterKind kind = Pt2CsgEmitterKind::Sphere;
  std::uint32_t primID = 0;      // in the owning geometry's primID space
  Vec3 p0;                       // sphere center / cyl side base / cap center
  Vec3 axis{0.0f, 0.0f, 1.0f};   // unit cylinder axis (unused for Sphere)
  float len = 0.0f;              // lateral-surface length (CylSide)
  float radius = 0.0f;
  float area = 0.0f;             // this record's surface area
  Vec3 Le;                       // uniform emitted radiance = emission * rgb
};

// A capped cylinder's up-to-three CDF records, looked up by its primID.
struct Pt2CylEmitterRef {
  std::uint32_t side = 0xFFFFFFFFu;
  std::uint32_t cap0 = 0xFFFFFFFFu;
  std::uint32_t cap1 = 0xFFFFFFFFu;
};

struct Pt2EmissiveLights {
  std::vector<Pt2EmissiveTri> tris;
  // CSG records occupy CDF positions [triCount, triCount + csg.size()); the
  // triangles keep the prefix so a zero-CSG scene builds a float-identical
  // CDF (byte-identity anchor for the mesh-only refactor_check cases).
  std::vector<Pt2EmissiveCsg> csg;
  std::uint32_t triCount = 0;    // == tris.size(), the CDF prefix boundary
  std::vector<float> cdf;        // power-weighted, cdf[i] = P(emitter <= i)
  std::vector<float> pdfChoose;  // per-emitter selection probability
  // primID -> index into tris (0xFFFFFFFF = not emissive); sized to the mesh
  // triangle count for O(1) MIS pdf lookup at gather hits.
  std::vector<std::uint32_t> triIndex;
  // sphere primID -> index into csg (0xFFFFFFFF = not emissive), and capped
  // cylinder primID -> its record refs. Sized only when a CSG emitter exists.
  std::vector<std::uint32_t> sphereIndex;
  std::vector<Pt2CylEmitterRef> cylCapIndex;
  bool empty() const noexcept { return tris.empty() && csg.empty(); }
};

inline float pt2EmissiveLuminance(const Vec3& c) noexcept {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

// Scan the mesh for emissive triangles and the scene for emissive REAL CSG
// primitives (spheres + capped cylinders) and build the shared power-weighted
// CDF (triangles first -- see Pt2EmissiveLights::triCount). Returns an empty
// list when nothing emits. The CSG scans replicate the Embree build's primID
// order exactly (buildSpheres / buildCylinderGeometry: bake offsets outer,
// scene order inner; capped = the !open cylinders in scene order).
inline Pt2EmissiveLights pt2BuildEmissiveLights(const Mesh& mesh,
                                                const Scene& scene) {
  Pt2EmissiveLights out;
  const std::size_t nt = mesh.triangleCount();
  bool any = false;
  for (const Material& mm : mesh.materials)
    any = any || mm.emission > 0.0f;
  const bool meshEmissive = any || mesh.material.emission > 0.0f;
  bool anyCsg = false;
  for (const Sphere& s : scene.spheres)
    anyCsg = anyCsg || (!s.fromEdgeMacro && s.material.emission > 0.0f &&
                        s.radius > 0.0f);
  for (const Cylinder& c : scene.cylinders)
    anyCsg = anyCsg || (!c.fromEdgeMacro && !c.open &&
                        c.material.emission > 0.0f && c.radius > 0.0f);
  if (!meshEmissive && !anyCsg) return out;

  std::vector<float> power;
  if (meshEmissive) {
  out.triIndex.assign(nt, 0xFFFFFFFFu);
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
  if (out.tris.empty()) out.triIndex.clear();
  }  // meshEmissive
  out.triCount = static_cast<std::uint32_t>(out.tris.size());

  if (anyCsg) {
    constexpr float kPi = 3.14159265358979323846f;
    std::vector<Vec3> bake = scene.instanceOffsets;
    if (bake.empty()) bake.push_back(Vec3{0.0f, 0.0f, 0.0f});
    // Spheres: primID order = bake offset outer, scene.spheres inner
    // (buildSpheres, scene_build.cpp).
    out.sphereIndex.assign(scene.spheres.size() * bake.size(), 0xFFFFFFFFu);
    std::uint32_t prim = 0;
    for (const Vec3& off : bake) {
      for (const Sphere& s : scene.spheres) {
        if (!s.fromEdgeMacro && s.material.emission > 0.0f &&
            s.radius > 0.0f) {
          Pt2EmissiveCsg ec;
          ec.kind = Pt2CsgEmitterKind::Sphere;
          ec.primID = prim;
          ec.p0 = Vec3{s.center.x + off.x, s.center.y + off.y,
                       s.center.z + off.z};
          ec.radius = s.radius;
          ec.area = 4.0f * kPi * s.radius * s.radius;
          ec.Le = Vec3{s.material.emission * s.color.x,
                       s.material.emission * s.color.y,
                       s.material.emission * s.color.z};
          const float p = ec.area * pt2EmissiveLuminance(ec.Le);
          if (p > 0.0f) {
            out.sphereIndex[prim] =
                static_cast<std::uint32_t>(out.csg.size());
            out.csg.push_back(ec);
            power.push_back(p);
          }
        }
        ++prim;
      }
    }
    // Capped cylinders: primID order = bake offset outer, the !open
    // cylinders in scene order inner (buildCylinderGeometry's capIdx,
    // curve_build.cpp). Each emissive bond contributes up to three records:
    // lateral surface + two disk caps.
    std::vector<std::uint32_t> capIdx;
    for (std::uint32_t i = 0; i < scene.cylinders.size(); ++i)
      if (!scene.cylinders[i].open) capIdx.push_back(i);
    out.cylCapIndex.assign(capIdx.size() * bake.size(), Pt2CylEmitterRef{});
    prim = 0;
    for (const Vec3& off : bake) {
      for (std::uint32_t idx : capIdx) {
        const Cylinder& c = scene.cylinders[idx];
        if (!c.fromEdgeMacro && c.material.emission > 0.0f &&
            c.radius > 0.0f) {
          const Vec3 p0{c.p0.x + off.x, c.p0.y + off.y, c.p0.z + off.z};
          const Vec3 p1{c.p1.x + off.x, c.p1.y + off.y, c.p1.z + off.z};
          const Vec3 d{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
          const float len = std::sqrt(dot(d, d));
          if (len > 0.0f) {
            const Vec3 axis{d.x / len, d.y / len, d.z / len};
            const Vec3 Le{c.material.emission * c.color.x,
                          c.material.emission * c.color.y,
                          c.material.emission * c.color.z};
            const float lum = pt2EmissiveLuminance(Le);
            Pt2CylEmitterRef& ref = out.cylCapIndex[prim];
            Pt2EmissiveCsg ec;
            ec.primID = prim;
            ec.axis = axis;
            ec.radius = c.radius;
            ec.Le = Le;
            // Lateral surface.
            ec.kind = Pt2CsgEmitterKind::CylSide;
            ec.p0 = p0;
            ec.len = len;
            ec.area = 2.0f * kPi * c.radius * len;
            if (ec.area * lum > 0.0f) {
              ref.side = static_cast<std::uint32_t>(out.csg.size());
              out.csg.push_back(ec);
              power.push_back(ec.area * lum);
            }
            // Disk caps at p0 and p1 (two-sided emission, axis as normal).
            ec.kind = Pt2CsgEmitterKind::CylCap;
            ec.len = 0.0f;
            ec.area = kPi * c.radius * c.radius;
            if (ec.area * lum > 0.0f) {
              ec.p0 = p0;
              ref.cap0 = static_cast<std::uint32_t>(out.csg.size());
              out.csg.push_back(ec);
              power.push_back(ec.area * lum);
              ec.p0 = p1;
              ref.cap1 = static_cast<std::uint32_t>(out.csg.size());
              out.csg.push_back(ec);
              power.push_back(ec.area * lum);
            }
          }
        }
        ++prim;
      }
    }
    if (out.csg.empty()) {
      out.sphereIndex.clear();
      out.cylCapIndex.clear();
    }
  }

  if (out.tris.empty() && out.csg.empty()) {
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

// One emissive-NEE sample from receiver (P, N): pick an emitter from the
// power CDF, a uniform point on it (triangles / cylinder surfaces: uniform
// area; spheres: uniform solid-angle cone), test visibility, and return the
// MIS-weighted contribution to E_stored (the E_true/pi convention: the
// estimator term is L_e * cosThetaP / (pi * pdfSA), balance-weighted against
// the cosine pdf cosThetaP/pi). rng is the caller's tea2 stream. EVERY call
// consumes exactly two tea2 draws (pick + point) regardless of the picked
// emitter kind, so the caller's continuation stream is draw-stable.
inline Vec3 pt2EmissiveNee(const Pt2EmissiveLights& el, RTCScene scene,
                           const Vec3& P, const Vec3& N, const Vec3& Ng,
                           float epsT, std::uint32_t& s0, std::uint32_t& s1) {
  // Emitter pick (binary search over the CDF).
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
  if (lo < el.triCount) {
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

  // CSG emitter record (unreachable in mesh-only scenes: the CDF prefix is
  // all triangles, see pt2BuildEmissiveLights).
  constexpr float kPi = 3.14159265358979323846f;
  const Pt2EmissiveCsg& g = el.csg[lo - el.triCount];
  tea2(s0, s1);
  const float u1 = u32ToUnorm(s0), u2 = u32ToUnorm(s1);
  const float phi = 2.0f * kPi * u2;

  Vec3 dir;         // sample direction from P
  float target;     // occlusion-ray length toward the sampled point
  float pdfSA;
  if (g.kind == Pt2CsgEmitterKind::Sphere) {
    // Uniform solid-angle cone toward the sphere (pbrt scheme). The
    // 1 - cos(thetaMax) term uses the cancellation-safe form
    // sin^2 / (1 + cos) so distant small emitters keep precision.
    const Vec3 d{g.p0.x - P.x, g.p0.y - P.y, g.p0.z - P.z};
    const float dc2 = dot(d, d);
    const float r2 = g.radius * g.radius;
    if (dc2 <= r2 * 1.0002f) return Vec3{0.0f, 0.0f, 0.0f};  // inside/on
    const float dc = std::sqrt(dc2);
    const float sin2Max = r2 / dc2;
    const float cosMax = std::sqrt(std::fmax(0.0f, 1.0f - sin2Max));
    const float oneMinusCos = sin2Max / (1.0f + cosMax);
    const float cosT = 1.0f - u1 * oneMinusCos;
    const float sinT = std::sqrt(std::fmax(0.0f, 1.0f - cosT * cosT));
    const Frame fr = frameFromNormal(Vec3{d.x / dc, d.y / dc, d.z / dc});
    const float sx = sinT * std::cos(phi), sy = sinT * std::sin(phi);
    dir = Vec3{fr.t.x * sx + fr.b.x * sy + fr.n.x * cosT,
               fr.t.y * sx + fr.b.y * sy + fr.n.y * cosT,
               fr.t.z * sx + fr.b.z * sy + fr.n.z * cosT};
    pdfSA = el.pdfChoose[lo] / (2.0f * kPi * oneMinusCos);
    // Distance to the near intersection with the sphere along dir.
    const float b = dot(d, dir);
    const float disc = std::fmax(0.0f, b * b - (dc2 - r2));
    target = b - std::sqrt(disc);
  } else {
    // Uniform-area sample on the lateral surface or a disk cap; converted
    // to solid angle exactly like the triangle branch.
    const Frame fr = frameFromNormal(g.axis);
    Vec3 S, nS;
    if (g.kind == Pt2CsgEmitterKind::CylSide) {
      nS = Vec3{fr.t.x * std::cos(phi) + fr.b.x * std::sin(phi),
                fr.t.y * std::cos(phi) + fr.b.y * std::sin(phi),
                fr.t.z * std::cos(phi) + fr.b.z * std::sin(phi)};
      const float h = u1 * g.len;
      S = Vec3{g.p0.x + g.axis.x * h + nS.x * g.radius,
               g.p0.y + g.axis.y * h + nS.y * g.radius,
               g.p0.z + g.axis.z * h + nS.z * g.radius};
    } else {  // CylCap: uniform disk at p0, normal = +/- axis (two-sided)
      const float rr = g.radius * std::sqrt(u1);
      S = Vec3{g.p0.x + (fr.t.x * std::cos(phi) + fr.b.x * std::sin(phi)) * rr,
               g.p0.y + (fr.t.y * std::cos(phi) + fr.b.y * std::sin(phi)) * rr,
               g.p0.z + (fr.t.z * std::cos(phi) + fr.b.z * std::sin(phi)) * rr};
      nS = g.axis;
    }
    const Vec3 d{S.x - P.x, S.y - P.y, S.z - P.z};
    const float dist2 = dot(d, d);
    if (dist2 <= 1.0e-12f) return Vec3{0.0f, 0.0f, 0.0f};
    const float dist = std::sqrt(dist2);
    dir = Vec3{d.x / dist, d.y / dist, d.z / dist};
    const float cosL = std::fabs(dot(nS, dir));
    if (cosL <= 1.0e-6f) return Vec3{0.0f, 0.0f, 0.0f};
    pdfSA = el.pdfChoose[lo] * dist2 / (cosL * g.area);
    target = dist;
  }
  const float cosP = dot(N, dir);
  if (cosP <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};
  if (pdfSA <= 0.0f) return Vec3{0.0f, 0.0f, 0.0f};

  // Visibility + balance-heuristic estimator, same form as the triangles.
  const float eps = selfIntersectEps(P, dir, epsT);
  Vec3 ng = (dot(Ng, N) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
  ng = safeNormalize(ng, N);
  const Vec3 O{P.x + ng.x * eps, P.y + ng.y * eps, P.z + ng.z * eps};
  if (occluded(scene, O, dir, eps, target - 2.0f * eps))
    return Vec3{0.0f, 0.0f, 0.0f};
  const float pdfCos = cosP * (1.0f / kPi);
  const float w = pdfSA / (pdfSA + pdfCos);
  const float k = w * cosP / (kPi * pdfSA);
  return Vec3{g.Le.x * k, g.Le.y * k, g.Le.z * k};
}

// MIS weight for the COSINE side: a first-bounce gather ray from origin `O`
// that hit an emissive primitive of geometry kind `kind` (mesh triangle /
// CSG sphere / capped cylinder) with primID `primID` at distance `tfar`,
// direction `wi` (receiver cosine cosP at the origin) and geometric hit
// normal `ngHit` (non-unit, straight from Embree). 1.0 when the hit is not
// an NEE-covered emitter (incl. open cylinder chains and fromEdge
// decoration, which the emitter scan skips).
inline float pt2EmissiveHitWeight(const Pt2EmissiveLights& el, GeomKind kind,
                                  std::uint32_t primID, const Vec3& O,
                                  const Vec3& wi, float tfar, float cosP,
                                  const Vec3& ngHit) noexcept {
  if (el.empty()) return 1.0f;
  const float pdfCos = ((cosP > 0.0f) ? cosP : 0.0f) *
                       (1.0f / 3.14159265358979323846f);
  if (pdfCos <= 0.0f) return 1.0f;
  if (kind == GeomKind::Mesh) {
    if (primID >= el.triIndex.size()) return 1.0f;
    const std::uint32_t ti = el.triIndex[primID];
    if (ti == 0xFFFFFFFFu) return 1.0f;
    const float cosL = std::fabs(dot(el.tris[ti].ng, wi));
    const float pdfSA = pt2EmissivePdfSA(el, ti, tfar * tfar, cosL);
    if (pdfSA <= 0.0f) return 1.0f;
    return pdfCos / (pdfCos + pdfSA);
  }
  if (kind == GeomKind::Sphere) {
    if (primID >= el.sphereIndex.size()) return 1.0f;
    const std::uint32_t ci = el.sphereIndex[primID];
    if (ci == 0xFFFFFFFFu) return 1.0f;
    const Pt2EmissiveCsg& g = el.csg[ci];
    const Vec3 d{g.p0.x - O.x, g.p0.y - O.y, g.p0.z - O.z};
    const float dc2 = dot(d, d);
    const float r2 = g.radius * g.radius;
    if (dc2 <= r2 * 1.0002f) return 1.0f;  // origin inside/on the emitter
    // Same cancellation-safe cone pdf as the NEE draw (the two must match
    // for the balance weights to sum to 1).
    const float sin2Max = r2 / dc2;
    const float cosMax = std::sqrt(std::fmax(0.0f, 1.0f - sin2Max));
    const float oneMinusCos = sin2Max / (1.0f + cosMax);
    if (oneMinusCos <= 0.0f) return 1.0f;
    const float pdfSA = el.pdfChoose[el.triCount + ci] /
                        (2.0f * 3.14159265358979323846f * oneMinusCos);
    if (pdfSA <= 0.0f) return 1.0f;
    return pdfCos / (pdfCos + pdfSA);
  }
  if (kind == GeomKind::CylinderCapped) {
    if (primID >= el.cylCapIndex.size()) return 1.0f;
    const Pt2CylEmitterRef ref = el.cylCapIndex[primID];
    // Classify the hit part from the analytic normal: on a flat cap the
    // geometric normal is (anti)parallel to the axis, on the lateral
    // surface it is perpendicular. The 0.99 threshold leaves a bounded
    // MIS-weight error only on the measure-zero cap rim.
    std::uint32_t part = ref.side;
    const std::uint32_t probe =
        (ref.side != 0xFFFFFFFFu)   ? ref.side
        : (ref.cap0 != 0xFFFFFFFFu) ? ref.cap0
                                    : ref.cap1;
    if (probe == 0xFFFFFFFFu) return 1.0f;
    const Vec3 axis = el.csg[probe].axis;
    const Vec3 ng = safeNormalize(ngHit, axis);
    const float nu = dot(ng, axis);
    if (nu >= 0.99f)
      part = ref.cap1;
    else if (nu <= -0.99f)
      part = ref.cap0;
    if (part == 0xFFFFFFFFu) return 1.0f;
    const Pt2EmissiveCsg& g = el.csg[part];
    const float cosL = std::fabs(dot(ng, wi));
    if (cosL <= 1.0e-6f) return 1.0f;
    const float pdfSA =
        el.pdfChoose[el.triCount + part] * tfar * tfar / (cosL * g.area);
    if (pdfSA <= 0.0f) return 1.0f;
    return pdfCos / (pdfCos + pdfSA);
  }
  return 1.0f;  // open cylinder chains: BSDF-only, full weight
}

}  // namespace detail
}  // namespace umbreon
