// Surface irradiance cache for one-bounce diffuse GI (detail, Embree-aware).
//
// Instead of tracing GI rays at every pixel, indirect irradiance is gathered at
// a small set of representative surface points (records) and interpolated back
// at shading time (Ward-Heckbert / Krivanek irradiance caching). The build runs
// inside EmbreeRenderer::render() AFTER the primary loop, on the hi-res first-hit
// G-buffer (position/normal/componentId AOVs), because the fill pass needs the
// in-scope BuiltScene + lights to evaluate albedo*directLighting at each gather
// hit -- data that does not survive to the post-pass.
//
// Determinism (TBB thread-count independent, run-to-run bit-identical):
//   [B] placement   single-thread canonical raster scan; voxel occupancy is a
//                   set membership test, so the record SET is scan-order free.
//   [C] fill        TBB-parallel, each record seeded from its stable index hash
//                   (never a shared counter); neighbor clamp uses commutative min.
//   [D] lookup      read-only; per shading point the candidate record indices are
//                   sorted before float accumulation so the sum order is fixed.
//
// This header is included by embree_renderer.cpp; the fill/lookup passes (C4/C5)
// are added below as they land. C3 wires the structs + deterministic placement.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "shading/hit_shader.hpp"
#include "shading/secondary_rays.hpp"
#include "shading/shading.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// One irradiance cache record: a representative surface point where one-bounce
// indirect irradiance is gathered (fill) and later interpolated (lookup).
struct IrradianceRecord {
  Vec3 position{0.0f, 0.0f, 0.0f};    // surface point (cache spatial key)
  Vec3 normal{0.0f, 0.0f, 0.0f};      // surface normal
  Vec3 irradiance{0.0f, 0.0f, 0.0f};  // gathered one-bounce indirect E_i (fill)
  Vec3 bentNormal{0.0f, 0.0f, 0.0f};  // average unoccluded direction (fill)
  float radius = 0.0f;                // harmonic-mean valid radius R_i (fill)
  uint32_t componentId = 0xFFFFFFFFu; // CueMol section: rejects cross-section blend
};

// Pack an integer voxel coordinate into a 63-bit spatial-hash key: 21 bits per
// axis (signed, bias 2^20) => +/- ~1e6 cells/axis, ample at the record spacing.
// Deterministic and collision-free within that range.
inline uint64_t packVoxel(long long ix, long long iy, long long iz) {
  constexpr long long kBias = 1 << 20;
  const uint64_t ux = static_cast<uint64_t>(ix + kBias) & 0x1FFFFFull;
  const uint64_t uy = static_cast<uint64_t>(iy + kBias) & 0x1FFFFFull;
  const uint64_t uz = static_cast<uint64_t>(iz + kBias) & 0x1FFFFFull;
  return ux | (uy << 21) | (uz << 42);
}

// World point -> voxel key at the given inverse spacing.
inline uint64_t voxelKey(const Vec3& p, float invSpacing) {
  return packVoxel(static_cast<long long>(std::floor(p.x * invSpacing)),
                   static_cast<long long>(std::floor(p.y * invSpacing)),
                   static_cast<long long>(std::floor(p.z * invSpacing)));
}

// [B] Deterministic record placement. Scan the hi-res first-hit G-buffer in
// raster order and drop ONE record into each unoccupied world voxel of size
// `spacing`. Voxel occupancy is a set membership test, so the record SET is
// independent of scan order; the raster scan only fixes WHICH pixel (the first
// in raster order) supplies each cell's attributes, giving a stable canonical
// record order whose index is a stable per-record key for the fill seed.
// Non-mesh pixels (componentId == sentinel) seed nothing: GI is mesh-only.
inline std::vector<IrradianceRecord> placeRecordsVoxel(const FrameResult& res,
                                                       float spacing) {
  std::vector<IrradianceRecord> records;
  if (res.position.empty() || res.componentId.empty() || spacing <= 0.0f)
    return records;
  std::unordered_set<uint64_t> occupied;
  const int W = res.width, H = res.height;
  const float inv = 1.0f / spacing;
  for (int py = 0; py < H; ++py) {
    for (int px = 0; px < W; ++px) {
      const std::size_t pix = static_cast<std::size_t>(py) * W + px;
      const uint32_t comp = res.componentId[pix];
      if (comp == 0xFFFFFFFFu) continue;  // background / non-mesh hit: no record
      const Vec3 p{res.position[pix * 3 + 0], res.position[pix * 3 + 1],
                   res.position[pix * 3 + 2]};
      if (occupied.insert(voxelKey(p, inv)).second) {
        IrradianceRecord r;
        r.position = p;
        r.normal = Vec3{res.normal[pix * 3 + 0], res.normal[pix * 3 + 1],
                        res.normal[pix * 3 + 2]};
        r.componentId = comp;
        records.push_back(r);
      }
    }
  }
  return records;
}

// Two-color sky/ground hemisphere gradient along a miss-ray direction (the GI
// environment fill). Mirrors the bent-normal directional ambient in the hit
// shader: a direction toward `up` sees the sky tint, away sees the ground tint.
// `ambient` is the scene ambient color (the gradient is modulated by it).
inline Vec3 environmentRadiance(const Vec3& wi, const Vec3& up,
                                const Vec3& ambient, const float sky[3],
                                const float ground[3]) {
  const float w = 0.5f * (dot(wi, up) + 1.0f);
  return Vec3{ambient.x * (ground[0] + (sky[0] - ground[0]) * w),
              ambient.y * (ground[1] + (sky[1] - ground[1]) * w),
              ambient.z * (ground[2] + (sky[2] - ground[2]) * w)};
}

// One-bounce outgoing radiance at a gather hit (point y, incoming dir rayDir).
// Mesh hits reflect emission + diffuse direct light only: specular indirect is
// dropped (this is diffuse GI) and ambient is excluded so the cache does not
// double-count itself. Non-mesh (outline primitive) hits are black occluders --
// they block indirect light but bounce none, keeping GI a mesh-only light source.
inline Vec3 gatherMeshRadiance(const BuiltScene& built, const Mesh& mesh,
                               const std::vector<Light>& lights, RTCScene rscene,
                               const NearestHit& hit, const Vec3& y,
                               const Vec3& rayDir, float eps, bool shadowsOn,
                               int shadowSamples, uint32_t& s0, uint32_t& s1) {
  const GeomRecord& rec = built.records[hit.geomID];
  if (rec.kind != GeomKind::Mesh) return Vec3{0.0f, 0.0f, 0.0f};
  float nbuf[3] = {0, 0, 0};
  float cbuf[4] = {0, 0, 0, 1};
  rtcInterpolate0(rec.geom, hit.primID, hit.u, hit.v,
                  RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
  rtcInterpolate0(rec.geom, hit.primID, hit.u, hit.v,
                  RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
  const Vec3 ny = faceForward(normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]}), rayDir);
  const Vec3 Cy{cbuf[0], cbuf[1], cbuf[2]};
  const Material& my = mesh.materialForTri(hit.primID);
  Vec3 L{my.emission * Cy.x, my.emission * Cy.y, my.emission * Cy.z};
  for (const Light& l : lights) {
    const float ndl = dot(ny, l.L);
    if (ndl <= 0.0f) continue;
    float d;
    if (my.brilliance == 1.0f)
      d = ndl;
    else if (my.brilliance == 0.0f)
      d = 1.0f;
    else
      d = std::pow(ndl, my.brilliance);
    const float shadow =
        shadowsOn
            ? computeShadow(rscene, y, hit.ng, ny, eps, l, shadowSamples, s0, s1)
            : 1.0f;
    const float dk = my.diffuse * d * shadow;
    L.x += dk * Cy.x * l.color.x;
    L.y += dk * Cy.y * l.color.y;
    L.z += dk * Cy.z * l.color.z;
  }
  return L;
}

// Knobs for the [C] fill pass.
struct GIFillParams {
  int samples = 64;
  float maxDistance = 1.0f;       // gather ray tfar (indirect search radius)
  Vec3 up{0.0f, 1.0f, 0.0f};      // env gradient axis (camera up)
  Vec3 ambient{1.0f, 1.0f, 1.0f}; // scene ambient color
  float sky[3] = {1.0f, 1.0f, 1.0f};
  float ground[3] = {1.0f, 1.0f, 1.0f};
  bool shadows = false;
  int shadowSamples = 1;
  float radiusMin = 0.0f;
  float radiusMax = 1.0e20f;
};

// [C] Fill: gather one-bounce diffuse irradiance at each record over a cosine-
// weighted hemisphere (the same sampling as computeAO, but with a closest hit so
// the bounce point's radiance can be evaluated). TBB-parallel and per-record
// independent; each record seeds its RNG from its STABLE index (never a shared
// counter) and accumulates samples in fixed order, so the result is identical
// regardless of thread count. The valid radius R_i is the harmonic mean of the
// gather hit distances (near surfaces dominate => dense records in cavities),
// clamped to [radiusMin, radiusMax].
inline void fillRecords(std::vector<IrradianceRecord>& records,
                        const BuiltScene& built, const Mesh& mesh,
                        const std::vector<Light>& lights, RTCScene rscene,
                        const GIFillParams& gp) {
  if (records.empty()) return;
  tbb::parallel_for(
      tbb::blocked_range<std::size_t>(0, records.size()),
      [&](const tbb::blocked_range<std::size_t>& rr) {
        for (std::size_t ri = rr.begin(); ri != rr.end(); ++ri) {
          IrradianceRecord& rec = records[ri];
          const Frame f = frameFromNormal(rec.normal);
          const float eps =
              selfIntersectEps(rec.position, rec.normal, gp.maxDistance);
          const Vec3 O = rec.position + rec.normal * eps;
          // Per-record fixed seed from the stable index (no shared counter).
          uint32_t seed0 = static_cast<uint32_t>(ri);
          uint32_t seed1 = 0x9E3779B9u;
          tea2(seed0, seed1);
          uint32_t sh0 = seed0 ^ 0x68bc21ebu;  // decorrelated shadow RNG stream
          uint32_t sh1 = seed1;
          Vec3 E{0.0f, 0.0f, 0.0f};
          Vec3 bentAccum{0.0f, 0.0f, 0.0f};
          float invDistSum = 0.0f;
          int nHit = 0;
          for (int sidx = 0; sidx < gp.samples; ++sidx) {
            uint32_t s0 = seed0;
            uint32_t s1 = static_cast<uint32_t>(sidx);
            tea2(s0, s1);
            const Vec3 wi =
                cosineSampleHemisphere(u32ToUnorm(s0), u32ToUnorm(s1), f);
            if (dot(wi, rec.normal) < 0.01f) continue;  // grazing / below surface
            const NearestHit hit = intersectHit(rscene, O, wi, eps, gp.maxDistance);
            if (hit.hit) {
              const Vec3 y = O + wi * hit.t;
              E = E + gatherMeshRadiance(built, mesh, lights, rscene, hit, y, wi,
                                         eps, gp.shadows, gp.shadowSamples, sh0,
                                         sh1);
              invDistSum += 1.0f / hit.t;
              ++nHit;
            } else {
              E = E + environmentRadiance(wi, gp.up, gp.ambient, gp.sky,
                                          gp.ground);
              bentAccum = bentAccum + wi;
            }
          }
          const float inv = 1.0f / static_cast<float>(gp.samples);
          rec.irradiance = E * inv;
          rec.bentNormal = safeNormalize(bentAccum, rec.normal);
          float R =
              (nHit > 0) ? static_cast<float>(nHit) / invDistSum : gp.maxDistance;
          if (R < gp.radiusMin) R = gp.radiusMin;
          if (R > gp.radiusMax) R = gp.radiusMax;
          rec.radius = R;
        }
      });
}

// Neighbor clamping (Krivanek 2006), the key leak-prevention step: a record's
// valid radius cannot exceed a nearer record's radius plus their separation,
// R_i = min(R_i, R_j + ||x_i - x_j||). min is commutative and reads the ORIGINAL
// radii (writing clamped values to a separate array), so the result is order- and
// thread-independent. Records are bucketed at `cellSize`; the neighborhood sweep
// is capped at `maxReach` cells (leaks are local, so a bounded sweep suffices and
// keeps an over-large open-region radius from blowing up the scan).
inline void neighborClamp(std::vector<IrradianceRecord>& records, float cellSize,
                          int maxReach = 3) {
  const std::size_t n = records.size();
  if (n == 0 || cellSize <= 0.0f) return;
  const float inv = 1.0f / cellSize;
  std::unordered_map<uint64_t, std::vector<uint32_t>> grid;
  for (uint32_t i = 0; i < n; ++i)
    grid[voxelKey(records[i].position, inv)].push_back(i);
  std::vector<float> clamped(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec3& xi = records[i].position;
    float R = records[i].radius;
    const int reach = std::min(
        maxReach, std::max(1, static_cast<int>(std::ceil(R * inv))));
    const long long cx = static_cast<long long>(std::floor(xi.x * inv));
    const long long cy = static_cast<long long>(std::floor(xi.y * inv));
    const long long cz = static_cast<long long>(std::floor(xi.z * inv));
    for (long long dz = -reach; dz <= reach; ++dz)
      for (long long dy = -reach; dy <= reach; ++dy)
        for (long long dx = -reach; dx <= reach; ++dx) {
          auto it = grid.find(packVoxel(cx + dx, cy + dy, cz + dz));
          if (it == grid.end()) continue;
          for (uint32_t j : it->second) {
            if (static_cast<std::size_t>(j) == i) continue;
            const float dd = length(xi - records[j].position);
            const float cand = records[j].radius + dd;  // ORIGINAL radius of j
            if (cand < R) R = cand;
          }
        }
    clamped[i] = R;
  }
  for (std::size_t i = 0; i < n; ++i) records[i].radius = clamped[i];
}

// The cache: records in canonical placement order plus a spatial hash from world
// voxel to the record indices whose influence reaches that voxel. The hash is
// built AFTER fill (radii known) and queried read-only at lookup; the fill seed
// uses only a record's stable index, never a shared counter. Grid build/query
// land with the interpolation pass (C5).
struct IrradianceCache {
  std::vector<IrradianceRecord> records;
  std::unordered_map<uint64_t, std::vector<uint32_t>> grid;
  float cellSize = 1.0f;
};

// Build the spatial hash AFTER fill (radii known): register each record into
// every voxel its influence box [pos +/- accuracy*R_i] overlaps, so a lookup at
// x only needs x's home cell. Records are visited in ascending index order, so
// each cell's list ends up sorted by index -- the lookup then accumulates in a
// fixed float order with no per-pixel sort.
inline void buildGrid(IrradianceCache& cache, float cellSize, float accuracy) {
  cache.cellSize = cellSize;
  cache.grid.clear();
  const float inv = 1.0f / cellSize;
  for (uint32_t i = 0; i < cache.records.size(); ++i) {
    const IrradianceRecord& r = cache.records[i];
    const float reach = accuracy * r.radius;  // a*R_i influence radius
    const long long lox =
        static_cast<long long>(std::floor((r.position.x - reach) * inv));
    const long long hix =
        static_cast<long long>(std::floor((r.position.x + reach) * inv));
    const long long loy =
        static_cast<long long>(std::floor((r.position.y - reach) * inv));
    const long long hiy =
        static_cast<long long>(std::floor((r.position.y + reach) * inv));
    const long long loz =
        static_cast<long long>(std::floor((r.position.z - reach) * inv));
    const long long hiz =
        static_cast<long long>(std::floor((r.position.z + reach) * inv));
    for (long long iz = loz; iz <= hiz; ++iz)
      for (long long iy = loy; iy <= hiy; ++iy)
        for (long long ix = lox; ix <= hix; ++ix)
          cache.grid[packVoxel(ix, iy, iz)].push_back(i);
  }
}

// [D] Interpolate the cached irradiance at shading point x (normal nx, section
// comp). Gathers the home cell's candidate records, rejects cross-section and
// normal-divergent ones, and forms the Ward weight w = 1/(dist/R + sqrt(1-n.n)).
// Records with w <= 1/accuracy are out of influence. valid=false when no record
// is trusted (the caller then leaves the indirect at 0 for that pixel).
inline Vec3 interpolateIrradiance(const IrradianceCache& cache, const Vec3& x,
                                  const Vec3& nx, uint32_t comp,
                                  const RenderOptions& opt, bool& valid) {
  valid = false;
  const float inv = 1.0f / cache.cellSize;
  auto it = cache.grid.find(voxelKey(x, inv));
  if (it == cache.grid.end()) return Vec3{0.0f, 0.0f, 0.0f};
  // pbrt-style separable error: err = max(positional, angular), each normalized
  // so err < 1 is the influence boundary. Positional uses the FULL influence
  // radius a*R_i (not the old 0.15*R_i, which under-covered cavities and left
  // holes); angular maps the normal-reject cosine floor to a unit error. The
  // weight wt = 1 - err falls off CONTINUOUSLY to 0 at the boundary, removing
  // the lattice step the old 1/denom + hard threshold produced.
  const float a = opt.giAccuracy;  // influence-radius multiplier (a*R_i)
  const float nrejScale =
      std::sqrt(std::fmax(1.0e-6f, 1.0f - opt.giNormalReject));
  Vec3 Esum{0.0f, 0.0f, 0.0f};
  float wSum = 0.0f;
  // Lowest-error in-section record, for the no-coverage fallback.
  float bestErr = 1.0e30f;
  uint32_t bestIdx = 0xFFFFFFFFu;
  // The cell list is in ascending record-index order (buildGrid visits records
  // in order), so this float accumulation is order- and thread-independent.
  for (uint32_t idx : it->second) {
    const IrradianceRecord& ri = cache.records[idx];
    if (opt.giComponentReject && ri.componentId != comp) continue;
    const float ndot = dot(nx, ri.normal);
    const float perr = length(x - ri.position) / (a * ri.radius);
    const float nerr = std::sqrt(std::fmax(0.0f, 1.0f - ndot)) / nrejScale;
    const float err = std::fmax(perr, nerr);
    if (err < bestErr) {
      bestErr = err;
      bestIdx = idx;
    }
    if (err >= 1.0f) continue;
    const float w = 1.0f - err;  // continuous falloff to 0 at the boundary
    Esum = Esum + ri.irradiance * w;
    wSum += w;
  }
  // Minimum support: a single weak record is too splotchy. Below it, fall back
  // to the nearest in-section record so a cavity / curved spot never punches a
  // hole (indirect = 0) where GI matters most -- the classic Ward failure the
  // fixed-grid placement made worse.
  constexpr float kMinSupport = 0.5f;
  if (wSum >= kMinSupport) {
    valid = true;
    return Esum * (1.0f / wSum);
  }
  if (bestIdx != 0xFFFFFFFFu) {
    valid = true;
    return cache.records[bestIdx].irradiance;
  }
  return Vec3{0.0f, 0.0f, 0.0f};
}

// Full GI pass: build the irradiance cache from the hi-res first-hit G-buffer
// (placement -> fill -> neighbor clamp -> grid) and add the interpolated
// indirect to the color. ADDITIVE -- indirect = giIntensity * albedo * E_cached
// * shapeAo is added to the existing direct color, so giIntensity == 0 (or gi
// off) leaves the color byte-identical. Mesh pixels only. Runs inside
// EmbreeRenderer::render() where the BuiltScene + lights are still live.
inline void applyIndirectGI(const ShadeContext& sc, FrameResult& res) {
  const RenderOptions& opt = sc.opt;
  if (res.position.empty() || res.componentId.empty() || res.normal.empty() ||
      res.albedo.empty())
    return;  // cache AOVs must be captured (the gi gate forces them on)
  RTCBounds bounds;
  rtcGetSceneBounds(sc.built.scene, &bounds);
  const Vec3 lo{bounds.lower_x, bounds.lower_y, bounds.lower_z};
  const Vec3 hi{bounds.upper_x, bounds.upper_y, bounds.upper_z};
  const float diagonal = length(hi - lo);
  if (!(diagonal > 0.0f)) return;
  const float spacing =
      opt.giRecordSpacing > 0.0f ? opt.giRecordSpacing : diagonal * 0.01f;
  const float maxDist =
      opt.giMaxDistance > 0.0f ? opt.giMaxDistance : diagonal * 0.5f;
  if (spacing <= 0.0f) return;

  // [B] placement, [C] fill, neighbor clamp, grid build.
  IrradianceCache cache;
  cache.records = placeRecordsVoxel(res, spacing);
  if (cache.records.empty()) return;
  GIFillParams gp;
  gp.samples = opt.giSamples;
  gp.maxDistance = maxDist;
  gp.up = sc.aoUp;
  gp.ambient = sc.ambLight;
  gp.sky[0] = opt.aoSkyColor[0];
  gp.sky[1] = opt.aoSkyColor[1];
  gp.sky[2] = opt.aoSkyColor[2];
  gp.ground[0] = opt.aoGroundColor[0];
  gp.ground[1] = opt.aoGroundColor[1];
  gp.ground[2] = opt.aoGroundColor[2];
  gp.shadows = opt.shadows;
  gp.shadowSamples = opt.shadowSamples;
  gp.radiusMin = spacing * 0.5f;
  // Cap the radius so the full-R influence box (a*R_i, registered per cell in
  // buildGrid) stays a few cells wide -- an open-region record with a huge
  // harmonic-mean radius would otherwise blow up the grid registration.
  gp.radiusMax = std::min(diagonal, spacing * 4.0f);
  fillRecords(cache.records, sc.built, sc.mesh, sc.lights, sc.built.scene, gp);
  neighborClamp(cache.records, spacing);
  buildGrid(cache, spacing, opt.giAccuracy);

  // [D] interpolation + [E] additive composite, parallel over pixels.
  const int W = res.width, H = res.height;
  res.indirect.assign(static_cast<std::size_t>(W) * H * 3, 0.0f);
  tbb::parallel_for(
      tbb::blocked_range<int>(0, H), [&](const tbb::blocked_range<int>& rows) {
        for (int py = rows.begin(); py != rows.end(); ++py)
          for (int px = 0; px < W; ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            const uint32_t comp = res.componentId[pix];
            if (comp == 0xFFFFFFFFu) continue;  // non-mesh pixel: no indirect
            const Vec3 x{res.position[pix * 3 + 0], res.position[pix * 3 + 1],
                         res.position[pix * 3 + 2]};
            const Vec3 nx{res.normal[pix * 3 + 0], res.normal[pix * 3 + 1],
                          res.normal[pix * 3 + 2]};
            bool valid = false;
            const Vec3 E = interpolateIrradiance(cache, x, nx, comp, opt, valid);
            if (!valid) continue;  // no trusted record: indirect stays 0
            const float shape = res.shapeAo.empty() ? 1.0f : res.shapeAo[pix];
            const float g = opt.giIntensity;
            const float ind0 = g * res.albedo[pix * 3 + 0] * E.x * shape;
            const float ind1 = g * res.albedo[pix * 3 + 1] * E.y * shape;
            const float ind2 = g * res.albedo[pix * 3 + 2] * E.z * shape;
            res.indirect[pix * 3 + 0] = ind0;
            res.indirect[pix * 3 + 1] = ind1;
            res.indirect[pix * 3 + 2] = ind2;
            res.color[pix * 4 + 0] += ind0;
            res.color[pix * 4 + 1] += ind1;
            res.color[pix * 4 + 2] += ind2;
          }
      });
}

}  // namespace detail
}  // namespace umbreon
