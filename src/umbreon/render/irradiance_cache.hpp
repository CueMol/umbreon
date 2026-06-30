// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Adaptive surface irradiance cache (Ward-Heckbert / Krivanek style) for the
// umbreon Embree renderer's diffuse GI. Three deterministic, thread-count-
// independent stages live here:
//   [B] placement   -- visible-surface voxel seeding of cache records (order-
//                       independent: a record per unoccupied world voxel).
//   [C] fill        -- per-record cosine-weighted hemisphere gather (TBB
//                       parallel, per-record fixed RNG seed), giving the
//                       occluded one-bounce irradiance E_i, a bent normal and
//                       a harmonic-mean influence radius R_i; neighbor clamping
//                       (commutative min) bounds R_i across adjacent records.
//   [D] interpolate -- read-only Ward-weighted blend of nearby records at a
//                       shading point (used here to fill the debug `indirect`
//                       AOV; the final composite is wired in a later step).
//
// Determinism: placement is single-threaded canonical raster order over the
// first-hit G-buffer; the occupied-voxel set makes it order-independent. Fill
// seeds each record only from its index, so TBB thread count never changes the
// result. Interpolation sorts its candidate record indices before summation so
// the float accumulation order is fixed run-to-run.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// One cached irradiance sample on a surface. irradiance is the occluded one-
// bounce indirect (the final composite multiplies it by the receiver's kd; AO
// is never applied on top -- occlusion already lives inside this value).
struct IrradianceRecord {
  Vec3 position;       // surface point (cache spatial key)
  Vec3 normal;         // shading normal at the record
  Vec3 irradiance;     // RGB indirect irradiance E_i
  Vec3 bentNormal;     // average unoccluded direction (env lookup; debug)
  float radius = 0.0f; // effective radius R_i (harmonic mean, clamped)
  float occlusion = 0.0f;  // cosine-weighted gather hit fraction (AO-like; debug)
  uint32_t geomID = 0xFFFFFFFFu;
  uint16_t componentID = 0;  // CueMol section (group); rejects cross-section blend
};

// Inputs the cache build/lookup share. Bundled so the (long) signatures stay
// readable and a later multi-bounce step can thread extra state through one type.
struct IrradianceCacheParams {
  RTCScene scene = nullptr;            // live committed BVH (gather rays)
  const BuiltScene* built = nullptr;   // geom records (mesh interpolation handles)
  const Mesh* mesh = nullptr;          // per-triangle material / group
  const std::vector<Light>* lights = nullptr;
  Vec3 ambLight{1.0f, 1.0f, 1.0f};     // environment radiance scale (scene ambient)
  Vec3 envUp{0.0f, 1.0f, 0.0f};        // sky/ground gradient axis
  Vec3 skyColor{1.0f, 1.0f, 1.0f};     // up-hemisphere env tint
  Vec3 groundColor{1.0f, 1.0f, 1.0f};  // down-hemisphere env tint
  float envIntensity = 1.0f;           // environment (miss) fill scale
  int samples = 64;                    // hemisphere gather rays per record
  float maxDistance = 0.0f;            // gather tfar (> 0)
  float spacing = 0.0f;                // voxel seed world spacing (> 0)
  float accuracy = 0.15f;              // interpolation accuracy a
  float normalReject = 0.85f;          // min dot(n_x, n_rec)
  bool componentReject = true;         // reject cross-section records
  bool shadows = false;                // include direct shadows in the gather
  int shadowSamples = 1;
};

// Uniform-grid spatial index over the records, keyed by world voxel. Each record
// is registered into every cell its influence sphere (accuracy * R_i) overlaps,
// so a lookup at a point reads back exactly the records that can influence it
// from the single cell containing that point. cellSize == placement spacing.
class IrradianceCache {
 public:
  std::vector<IrradianceRecord> records;

  void setCellSize(float s) { cellSize_ = (s > 0.0f) ? s : 1.0f; }
  float cellSize() const { return cellSize_; }

  // Hash a world point to its integer cell coordinate.
  static int64_t cellCoord(float x, float inv) {
    return static_cast<int64_t>(std::floor(x * inv));
  }

  // Register record `idx` into every cell its influence radius spans (capped so a
  // huge open-surface R_i cannot explode the registration cost).
  void registerRecord(uint32_t idx) {
    const IrradianceRecord& r = records[idx];
    const float inv = 1.0f / cellSize_;
    float infl = accuracyForReg_ * r.radius;
    if (infl < 0.0f) infl = 0.0f;
    const int64_t cx = cellCoord(r.position.x, inv);
    const int64_t cy = cellCoord(r.position.y, inv);
    const int64_t cz = cellCoord(r.position.z, inv);
    int span = static_cast<int>(std::ceil(infl * inv));
    if (span < 0) span = 0;
    if (span > kMaxRegSpan) span = kMaxRegSpan;  // cap blow-up on open surfaces
    for (int dz = -span; dz <= span; ++dz)
      for (int dy = -span; dy <= span; ++dy)
        for (int dx = -span; dx <= span; ++dx)
          grid_[key(cx + dx, cy + dy, cz + dz)].push_back(idx);
  }

  // Build the grid from the current records (call after fill + neighbor clamp).
  void buildGrid(float accuracy) {
    accuracyForReg_ = accuracy;
    grid_.clear();
    for (uint32_t i = 0; i < records.size(); ++i) registerRecord(i);
  }

  // Candidate record indices that may influence a point, gathered from the 3x3x3
  // block of cells around it (records sit ~one cell apart, and a small-radius
  // concave record only registers into its own cell, so a single-cell query
  // misses the local records a neighbor point needs). Returned sorted+unique so
  // any downstream float summation is deterministic.
  void query(const Vec3& p, std::vector<uint32_t>& out) const {
    out.clear();
    const float inv = 1.0f / cellSize_;
    const int64_t cx = cellCoord(p.x, inv);
    const int64_t cy = cellCoord(p.y, inv);
    const int64_t cz = cellCoord(p.z, inv);
    for (int dz = -1; dz <= 1; ++dz)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          auto it = grid_.find(key(cx + dx, cy + dy, cz + dz));
          if (it == grid_.end()) continue;
          out.insert(out.end(), it->second.begin(), it->second.end());
        }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
  }

 private:
  static constexpr int kMaxRegSpan = 8;
  static uint64_t key(int64_t x, int64_t y, int64_t z) {
    // Mix three signed cell coords into one 64-bit key (offset to unsigned, then
    // a cheap multiplicative hash). Collisions only cost an extra membership
    // check, never correctness (each cell stores explicit indices).
    const uint64_t ux = static_cast<uint64_t>(x + 0x40000000LL);
    const uint64_t uy = static_cast<uint64_t>(y + 0x40000000LL);
    const uint64_t uz = static_cast<uint64_t>(z + 0x40000000LL);
    uint64_t h = ux * 0x9E3779B185EBCA87ull;
    h ^= (uy + 0x9E3779B97F4A7C15ull) * 0xC2B2AE3D27D4EB4Full;
    h ^= (uz + 0x165667B19E3779F9ull) * 0x27D4EB2F165667C5ull;
    return h;
  }

  float cellSize_ = 1.0f;
  float accuracyForReg_ = 0.15f;
  std::unordered_map<uint64_t, std::vector<uint32_t>> grid_;
};

// 32-bit integer hash (used to seed each record's gather RNG from its index
// alone, so the fill is independent of TBB scheduling).
inline uint32_t hashU32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7FEB352Du;
  x ^= x >> 15;
  x *= 0x846CA68Bu;
  x ^= x >> 16;
  return x;
}

// 2-color sky/ground environment radiance along a gather direction (the miss
// term). With the default white sky == ground this collapses to the plain scene
// ambient, so an open point gathers ~ambLight and an occluded one gathers less.
inline Vec3 environmentRadiance(const IrradianceCacheParams& p, const Vec3& wi) {
  const float w = 0.5f * (dot(wi, p.envUp) + 1.0f);
  const float e = p.envIntensity;
  return Vec3{e * p.ambLight.x * (p.groundColor.x + (p.skyColor.x - p.groundColor.x) * w),
              e * p.ambLight.y * (p.groundColor.y + (p.skyColor.y - p.groundColor.y) * w),
              e * p.ambLight.z * (p.groundColor.z + (p.skyColor.z - p.groundColor.z) * w)};
}

// One-bounce outgoing radiance of a gather-ray hit toward the record: the hit
// surface's diffuse reflectance (mat.diffuse * pigment) times its direct
// irradiance (sum of lit lights, optionally shadowed). Restricted to MESH hits
// (the GI gate); a non-mesh hit (flat outline sphere/cylinder) is a pure occluder
// and contributes zero, matching the AO/GI "mesh only" scope.
inline Vec3 oneBounceRadiance(const IrradianceCacheParams& p, const RTCRayHit& rh,
                              const Vec3& O, const Vec3& wi) {
  const GeomRecord& rec = p.built->records[rh.hit.geomID];
  if (rec.kind != GeomKind::Mesh) return Vec3{0.0f, 0.0f, 0.0f};

  float nbuf[3] = {0, 0, 0};
  float cbuf[4] = {0, 0, 0, 1};
  rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                  RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
  rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                  RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
  Vec3 Ny = normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
  // Face the normal toward the record (the gather origin lies on the -wi side).
  if (dot(Ny, wi) > 0.0f) Ny = Vec3{-Ny.x, -Ny.y, -Ny.z};
  const Vec3 Cy{cbuf[0], cbuf[1], cbuf[2]};
  const Material& mat = p.mesh->materialForTri(rh.hit.primID);

  const Vec3 Py{O.x + wi.x * rh.ray.tfar, O.y + wi.y * rh.ray.tfar,
                O.z + wi.z * rh.ray.tfar};
  const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
  const float eps = selfIntersectEps(Py, wi, rh.ray.tfar);

  // Direct irradiance at y (no albedo, no 1/pi): sum of lit lights' N.L * color,
  // each shadow-tested. The bounce-source visibility is NOT optional: a wall deep
  // in a cavity is in shadow, so it must bounce no light -- otherwise enclosed
  // regions get filled by phantom-lit walls and never darken. (This is decoupled
  // from the primary render's `shadows` flag: the GI gather is always physically
  // shadow-correct even when the direct pass draws no hard shadows.) Diffuse
  // reflectance mat.diffuse * Cy then multiplies.
  Vec3 E{0.0f, 0.0f, 0.0f};
  uint32_t s0 = hashU32(rh.hit.primID), s1 = hashU32(rh.hit.geomID + 0x9E3779B9u);
  for (const Light& l : *p.lights) {
    const float ndl = dot(Ny, l.L);
    if (ndl <= 0.0f) continue;
    const float sh = computeShadow(p.scene, Py, Ng, Ny, eps, l, 1, s0, s1);
    E.x += ndl * l.color.x * sh;
    E.y += ndl * l.color.y * sh;
    E.z += ndl * l.color.z * sh;
  }
  const float kd = mat.diffuse;
  return Vec3{kd * Cy.x * E.x, kd * Cy.y * E.y, kd * Cy.z * E.z};
}

// rtcIntersect1 returning the full hit record (geom/prim/uv/tfar), or geomID ==
// RTC_INVALID_GEOMETRY_ID on a miss. Used by the gather to shade the hit point.
inline RTCRayHit intersectFull(RTCScene scene, const Vec3& O, const Vec3& dir,
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
  // Incoherent: GI gather rays diverge, so the BVH traversal is fastest in the
  // single-ray incoherent mode (Embree guidance).
  iargs.flags = RTC_RAY_QUERY_FLAG_INCOHERENT;
  rtcIntersect1(scene, &rh, &iargs);
  return rh;
}

// [B] Placement: one record per unoccupied world voxel over the first-hit
// G-buffer. `pos`/`nrm` are W*H*3 world position/normal, `group`/`geom` are W*H
// per-pixel component id / Embree geomID (geom == sentinel or non-mesh => skip).
// Order-independent: the occupied-voxel set is the same regardless of pixel
// order, and raster order picks a deterministic seed position per voxel.
inline void placeRecordsVoxel(IrradianceCache& cache,
                              const IrradianceCacheParams& p, int W, int H,
                              const float* pos, const float* nrm,
                              const int* group, const uint32_t* geom) {
  const float inv = 1.0f / p.spacing;
  std::unordered_map<uint64_t, uint8_t> occupied;
  const std::size_t np = static_cast<std::size_t>(W) * H;
  for (std::size_t i = 0; i < np; ++i) {
    const uint32_t g = geom[i];
    if (g == 0xFFFFFFFFu) continue;  // background
    if (p.built->records[g].kind != GeomKind::Mesh) continue;  // GI = mesh only
    const Vec3 P{pos[i * 3 + 0], pos[i * 3 + 1], pos[i * 3 + 2]};
    const int64_t cx = IrradianceCache::cellCoord(P.x, inv);
    const int64_t cy = IrradianceCache::cellCoord(P.y, inv);
    const int64_t cz = IrradianceCache::cellCoord(P.z, inv);
    const uint64_t k = (static_cast<uint64_t>(cx + 0x40000000LL) * 73856093ull) ^
                       (static_cast<uint64_t>(cy + 0x40000000LL) * 19349663ull) ^
                       (static_cast<uint64_t>(cz + 0x40000000LL) * 83492791ull);
    if (occupied.emplace(k, 1).second) {
      IrradianceRecord r;
      r.position = P;
      r.normal = safeNormalize(Vec3{nrm[i * 3 + 0], nrm[i * 3 + 1], nrm[i * 3 + 2]},
                               Vec3{0.0f, 0.0f, 1.0f});
      r.geomID = g;
      r.componentID = static_cast<uint16_t>(group[i] < 0 ? 0 : group[i]);
      cache.records.push_back(r);
    }
  }
}

// [B] alt: per-vertex seeding (view-independent). One record per welded vertex
// (posClass) of the mesh, or per raw vertex when no weld map exists. componentID
// is left 0 (a vertex is not owned by a single section); used only when
// giSeedPerVertex is requested.
inline void placeRecordsPerVertex(IrradianceCache& cache,
                                  const IrradianceCacheParams& p) {
  const Mesh& m = *p.mesh;
  const std::size_t nv = m.vertexCount();
  const bool haveNrm = m.normals.size() == nv;
  std::unordered_map<int32_t, uint8_t> seenClass;
  for (std::size_t v = 0; v < nv; ++v) {
    if (!m.posClass.empty()) {
      if (!seenClass.emplace(m.posClass[v], 1).second) continue;  // dedup weld
    }
    IrradianceRecord r;
    r.position = m.positions[v];
    r.normal = haveNrm ? safeNormalize(m.normals[v], Vec3{0.0f, 0.0f, 1.0f})
                       : Vec3{0.0f, 0.0f, 1.0f};
    r.geomID = p.built->records.empty() ? 0u : 0u;  // mesh is geomID 0
    r.componentID = 0;
    cache.records.push_back(r);
  }
}

// [C] Fill: gather one-bounce irradiance at each record in parallel. Each record
// seeds its RNG only from its index, so the result is independent of TBB thread
// count and bit-reproducible. Writes E_i, bent normal and harmonic-mean radius.
inline void fillRecords(IrradianceCache& cache, const IrradianceCacheParams& p) {
  const int N = p.samples;
  const float R = p.maxDistance;
  const float Rmin = 0.25f * p.spacing;
  const std::size_t n = cache.records.size();
  tbb::parallel_for(
      tbb::blocked_range<std::size_t>(0, n),
      [&](const tbb::blocked_range<std::size_t>& rg) {
        std::vector<uint32_t> tmp;
        (void)tmp;
        for (std::size_t ri = rg.begin(); ri != rg.end(); ++ri) {
          IrradianceRecord& rec = cache.records[ri];
          const Frame f = frameFromNormal(rec.normal);
          const float eps = selfIntersectEps(rec.position, rec.normal, R);
          const Vec3 O = rec.position + rec.normal * eps;
          const uint32_t seed = hashU32(static_cast<uint32_t>(ri) + 1u);
          Vec3 E{0.0f, 0.0f, 0.0f};
          Vec3 bentAccum{0.0f, 0.0f, 0.0f};
          float invDistSum = 0.0f;
          int nHit = 0;
          for (int s = 0; s < N; ++s) {
            uint32_t s0 = seed;
            uint32_t s1 = static_cast<uint32_t>(s);
            tea2(s0, s1);
            const Vec3 wi =
                cosineSampleHemisphere(u32ToUnorm(s0), u32ToUnorm(s1), f);
            const RTCRayHit rh = intersectFull(p.scene, O, wi, eps, R);
            if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
              const Vec3 Ly = oneBounceRadiance(p, rh, O, wi);
              E = E + Ly;
              if (rh.ray.tfar > 0.0f) invDistSum += 1.0f / rh.ray.tfar;
              ++nHit;
            } else {
              E = E + environmentRadiance(p, wi);
              bentAccum = bentAccum + wi;  // open directions steer the bent normal
            }
          }
          const float invN = 1.0f / static_cast<float>(N);
          rec.irradiance = Vec3{E.x * invN, E.y * invN, E.z * invN};
          rec.occlusion = static_cast<float>(nHit) * invN;  // hit fraction
          rec.bentNormal = safeNormalize(bentAccum, rec.normal);
          // Harmonic-mean occluder distance (Ward / Jarosz Eq 3.8):
          //   R_i = N / sum_k(1/d_k),  d_k = inf for a miss (1/d_k = 0).
          // The NUMERATOR is the TOTAL sample count N, not the hit count -- a
          // mostly-open record (few hits) then gets a LARGE radius (sparse
          // influence in open space) and a crevice record (many near hits) a
          // SMALL one (dense in corners), which is the whole point of the harmonic
          // mean. Using nHit here instead collapsed that contrast. Clamp the
          // zero/over-large extremes; neighbor clamping bounds it further.
          float Ri = (invDistSum > 0.0f) ? static_cast<float>(N) / invDistSum : R;
          if (Ri < Rmin) Ri = Rmin;
          if (Ri > R) Ri = R;
          rec.radius = Ri;
        }
      });
}

// [C] Neighbor clamping (Krivanek 2006): R_i = min over neighbors of (R_j +
// ||x_i - x_j||). min is commutative, so a single pass over record pairs is
// order-independent. Bounds an over-large radius next to a small-radius record
// (the main leak guard at thin features). Uses a temporary grid keyed at the
// max radius so neighbors within reach are found.
inline void neighborClamp(IrradianceCache& cache, float accuracy) {
  const std::size_t n = cache.records.size();
  if (n == 0) return;
  // Grid sized to the largest radius so every clamping neighbor is reachable.
  float maxR = 0.0f;
  for (const IrradianceRecord& r : cache.records) maxR = std::fmax(maxR, r.radius);
  if (maxR <= 0.0f) return;
  IrradianceCache tmp;
  tmp.setCellSize(maxR);
  tmp.records = cache.records;  // copy positions/radii for the lookup
  tmp.buildGrid(1.0f);          // register each record across its own radius span

  std::vector<float> clamped(n);
  std::vector<uint32_t> cand;
  for (std::size_t i = 0; i < n; ++i) {
    const IrradianceRecord& ri = cache.records[i];
    float best = ri.radius;
    tmp.query(ri.position, cand);
    for (uint32_t j : cand) {
      if (j == i) continue;
      const IrradianceRecord& rj = cache.records[j];
      const float d = length(ri.position - rj.position);
      const float bound = rj.radius + d;
      if (bound < best) best = bound;
    }
    clamped[i] = best;
  }
  for (std::size_t i = 0; i < n; ++i) cache.records[i].radius = clamped[i];
  (void)accuracy;
}

// [D] Interpolate the indirect irradiance at a shading point from nearby records
// (read-only). Ward weight w = 1 / (||x - x_i|| / R_i + sqrt(max(0, 1 - n.n_i))),
// records with w <= 1/accuracy are out of influence. componentReject /
// normalReject drop cross-section and steeply-tilted records (leak guard).
// Returns false (and leaves E at the environment fallback) when no record is in
// range, so the field has no holes. Candidates are summed in sorted index order
// for run-to-run bit reproducibility.
inline bool interpolateIrradiance(const IrradianceCache& cache,
                                  const IrradianceCacheParams& p, const Vec3& x,
                                  const Vec3& nx, int componentX, Vec3& outE,
                                  float* outOcclusion = nullptr,
                                  float* outRadius = nullptr) {
  std::vector<uint32_t> cand;
  cache.query(x, cand);
  const float wMin = 1.0f / p.accuracy;
  Vec3 Esum{0.0f, 0.0f, 0.0f};
  float occSum = 0.0f;
  float radSum = 0.0f;
  float wSum = 0.0f;
  // Track the single best (highest-weight) component/normal-compatible record so
  // a point with records nearby but none passing the strict Ward accuracy cutoff
  // (sparse records relative to their radius) still gets the nearest cached value
  // instead of a hole. Only a point with NO compatible record at all falls back
  // to the environment.
  uint32_t bestIdx = 0xFFFFFFFFu;
  float bestW = 0.0f;
  for (uint32_t i : cand) {
    const IrradianceRecord& r = cache.records[i];
    if (p.componentReject && componentX >= 0 &&
        r.componentID != static_cast<uint16_t>(componentX))
      continue;
    const float nd = dot(nx, r.normal);
    if (nd < p.normalReject) continue;
    const float d = length(x - r.position);
    // Ward weight 1/(d/R_i + sqrt(1 - n.n_i)). A record sitting exactly on the
    // shading point with an aligned normal gives denom 0 (infinite weight); floor
    // the denominator at a tiny epsilon so it yields a large finite weight that
    // dominates the blend instead of being discarded.
    float denom = d / r.radius + std::sqrt(std::fmax(0.0f, 1.0f - nd));
    if (denom < 1.0e-4f) denom = 1.0e-4f;
    const float w = 1.0f / denom;
    if (w > bestW) { bestW = w; bestIdx = i; }
    if (w <= wMin) continue;
    Esum = Esum + r.irradiance * w;
    occSum += r.occlusion * w;
    radSum += r.radius * w;
    wSum += w;
  }
  if (wSum > 0.0f) {
    outE = Vec3{Esum.x / wSum, Esum.y / wSum, Esum.z / wSum};
    if (outOcclusion) *outOcclusion = occSum / wSum;
    if (outRadius) *outRadius = radSum / wSum;
    return true;
  }
  if (bestIdx != 0xFFFFFFFFu) {  // nearest compatible record (no hole)
    const IrradianceRecord& r = cache.records[bestIdx];
    outE = r.irradiance;
    if (outOcclusion) *outOcclusion = r.occlusion;
    if (outRadius) *outRadius = r.radius;
    return true;
  }
  outE = environmentRadiance(p, nx);
  if (outOcclusion) *outOcclusion = 0.0f;  // no record => assume open
  if (outRadius) *outRadius = p.maxDistance;  // no record => maximally open
  return false;
}

// Build the whole cache: [B] placement -> [C] fill -> neighbor clamp -> grid.
// `pos`/`nrm`/`group`/`geom` are the W*H first-hit G-buffer (see placeRecords).
inline IrradianceCache buildIrradianceCache(const IrradianceCacheParams& p, int W,
                                            int H, const float* pos,
                                            const float* nrm, const int* group,
                                            const uint32_t* geom,
                                            bool seedPerVertex) {
  IrradianceCache cache;
  cache.setCellSize(p.spacing);
  if (seedPerVertex)
    placeRecordsPerVertex(cache, p);
  else
    placeRecordsVoxel(cache, p, W, H, pos, nrm, group, geom);
  fillRecords(cache, p);
  neighborClamp(cache, p.accuracy);
  cache.buildGrid(p.accuracy);
  return cache;
}

}  // namespace detail
}  // namespace umbreon
