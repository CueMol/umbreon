// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Irradiance-cache BUILD pipeline + interpolation: the cold once-per-frame
// stages ([B] placement, [C] fill/neighbor clamp, outlier rejection,
// buildIrradianceCache) and the [D] Ward-weighted interpolateIrradiance
// definition. Split out of irradiance_cache.hpp for readability; that header
// keeps the data model and the HOT inline evaluators (oneBounceRadiance /
// intersectFull / hashU32 / environmentRadiance) whose identical inlining in
// embree_renderer.cpp and pt1_integrator.hpp carries the pt1<->cache A/B
// bit-exact contract. Everything stays header-inline; irradiance_cache.hpp
// includes this at its end, so including either header yields the same
// declarations.
#pragma once

#include "experimental/irradiance_cache/irradiance_cache.hpp"

namespace umbreon {
namespace detail {

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

// Stratified-hemisphere gather with Ward-Heckbert irradiance gradients (Ward &
// Heckbert 1992, "Irradiance Gradients", Eq 3/4). The hemisphere is split into an
// M (polar) x Naz (azimuthal) cosine-weighted grid (N ~= pi*M, M*N ~= samples) so
// neighboring cells share a wall whose radiance difference and nearest-occluder
// distance give the gradient analytically -- the same rays that estimate E also
// estimate how E changes with translation and rotation, which is what lets the
// interpolation vary smoothly between records instead of stepping per voxel.
//
// All gradient magnitudes are divided by pi: Ward's E = (pi/MN) sum L, while this
// cache stores E = mean(L) (no 1/pi), so every derivative of E scales by 1/pi too.
// Lc/rc are caller-owned scratch buffers (per-cell radiance and hit distance),
// reused across records to avoid per-record allocation.
inline void fillRecordStratified(IrradianceRecord& rec,
                                 const IrradianceCacheParams& p, uint32_t ri,
                                 std::vector<Vec3>& Lc, std::vector<float>& rc,
                                 const IrradianceCache* prevCache) {
  const float R = p.maxDistance;
  const float Rmin = 0.25f * p.spacing;
  // Cap the gradient grid at ~256 cells and average several rays per cell when
  // more samples are available. A single ray per cell makes each cell radiance
  // (and hence the finite-difference gradient between cells) very noisy on high-
  // frequency occlusion geometry, which the linear extrapolation then turns into
  // per-record speckle. Averaging spp rays per cell denoises the per-cell
  // radiance so the gradient is stable, while the total ray count still ~= samples
  // (so E_i quality is unchanged). Naz ~= pi*M (Ward's near-square cell aspect).
  const int cellsTarget = std::min(p.samples, 256);
  int M = static_cast<int>(std::lround(std::sqrt(cellsTarget / 3.14159265f)));
  if (M < 1) M = 1;
  int Naz = static_cast<int>(std::lround(static_cast<float>(cellsTarget) / M));
  if (Naz < 1) Naz = 1;
  const int total = M * Naz;
  const int spp = std::max(1, p.samples / total);
  Lc.assign(total, Vec3{0.0f, 0.0f, 0.0f});
  rc.assign(total, R);

  const Frame f = frameFromNormal(rec.normal);
  const float eps = selfIntersectEps(rec.position, rec.normal, R);
  const Vec3 O = rec.position + rec.normal * eps;
  const uint32_t seed = hashU32(ri + 1u);

  Vec3 E{0.0f, 0.0f, 0.0f};
  Vec3 bentAccum{0.0f, 0.0f, 0.0f};
  float invDistSum = 0.0f;
  int nHit = 0;
  const float invSpp = 1.0f / static_cast<float>(spp);
  for (int j = 0; j < M; ++j) {
    for (int k = 0; k < Naz; ++k) {
      const int idx = j * Naz + k;
      Vec3 Lsum{0.0f, 0.0f, 0.0f};
      float rClose = R;  // nearest occluder in the cell (drives the 1/Min(r) term)
      for (int sp = 0; sp < spp; ++sp) {
        uint32_t s0 = seed;
        uint32_t s1 = static_cast<uint32_t>(idx * spp + sp);
        tea2(s0, s1);
        // Stratified cosine sample: u2 = sin^2(theta) over the polar band
        // [j,j+1)/M, u1 = phi over the sector [k,k+1)/Naz, jittered within the
        // cell. Reusing cosineSampleHemisphere keeps the same cosine distribution.
        const float u2 = (j + u32ToUnorm(s0)) / static_cast<float>(M);
        const float u1 = (k + u32ToUnorm(s1)) / static_cast<float>(Naz);
        const Vec3 wi = cosineSampleHemisphere(u1, u2, f);
        const RTCRayHit rh = intersectFull(p.scene, O, wi, eps, R);
        if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
          Lsum = Lsum + oneBounceRadiance(p, rh, O, wi, prevCache);
          if (rh.ray.tfar > 0.0f) {
            invDistSum += 1.0f / rh.ray.tfar;
            rClose = std::fmin(rClose, rh.ray.tfar);
          }
          ++nHit;
        } else {
          Lsum = Lsum + environmentRadiance(p, wi);
          bentAccum = bentAccum + wi;
        }
      }
      Lc[idx] = Vec3{Lsum.x * invSpp, Lsum.y * invSpp, Lsum.z * invSpp};
      rc[idx] = rClose;  // background stays at R (1/r small => no gradient)
      E = E + Lc[idx];
    }
  }
  const float invT = 1.0f / static_cast<float>(total);
  const int totalRays = total * spp;
  rec.irradiance = Vec3{E.x * invT, E.y * invT, E.z * invT};
  rec.occlusion = static_cast<float>(nHit) / static_cast<float>(totalRays);
  rec.bentNormal = safeNormalize(bentAccum, rec.normal);
  float Ri = (invDistSum > 0.0f) ? static_cast<float>(totalRays) / invDistSum : R;
  if (Ri < Rmin) Ri = Rmin;
  if (Ri > R) Ri = R;
  rec.radius = Ri;

  // Tangent-plane direction at azimuth phi: cos(phi)*t + sin(phi)*b.
  auto planeDir = [&](float phi) {
    return f.t * std::cos(phi) + f.b * std::sin(phi);
  };
  const float twoPi = 6.2831853072f;
  const float invPi = 1.0f / 3.14159265f;
  Vec3 gR[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  Vec3 gT[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  for (int k = 0; k < Naz; ++k) {
    const float phiCenter = twoPi * (k + 0.5f) / Naz;  // cell-center azimuth
    const float phiWall = twoPi * k / Naz;             // boundary k|k-1
    const Vec3 uHat = planeDir(phiCenter);             // radial (phi_k)
    const Vec3 vHatRot = planeDir(phiCenter + 1.5707963f);  // phi_k + pi/2
    const Vec3 vHatWall = planeDir(phiWall + 1.5707963f);   // phi_{k-} + pi/2
    const int km1 = (k + Naz - 1) % Naz;               // azimuthal wrap

    // Rotational gradient (Eq 3): sum_j -tan(theta_j) * L_{j,k}, steered by vHat.
    float rotSum[3] = {0, 0, 0};
    // Translational polar term (Eq 4, first sum, j=1..M-1), steered by uHat.
    float polR[3] = {0, 0, 0};
    // Translational azimuthal term (Eq 4, second sum, j=0..M-1), steered by vHatWall.
    float aziR[3] = {0, 0, 0};
    for (int j = 0; j < M; ++j) {
      const int idx = j * Naz + k;
      const float sin2c = (j + 0.5f) / M;  // sin^2(theta) at cell center
      const float sinc = std::sqrt(sin2c);
      const float cosc = std::sqrt(std::fmax(0.0f, 1.0f - sin2c));
      const float tanc = (cosc > 1.0e-6f) ? (sinc / cosc) : 0.0f;
      rotSum[0] += -tanc * Lc[idx].x;
      rotSum[1] += -tanc * Lc[idx].y;
      rotSum[2] += -tanc * Lc[idx].z;

      if (j >= 1) {
        const int idxm = (j - 1) * Naz + k;
        const float sinLo = std::sqrt(static_cast<float>(j) / M);  // sin(theta_{j-})
        const float cos2Lo = 1.0f - static_cast<float>(j) / M;     // cos^2(theta_{j-})
        const float rmin = std::fmax(std::fmin(rc[idx], rc[idxm]), 1.0e-6f);
        const float coef = (twoPi / Naz) * (sinLo * cos2Lo) / rmin;
        polR[0] += coef * (Lc[idx].x - Lc[idxm].x);
        polR[1] += coef * (Lc[idx].y - Lc[idxm].y);
        polR[2] += coef * (Lc[idx].z - Lc[idxm].z);
      }
      const int idxk = j * Naz + km1;
      const float sinHi = std::sqrt(static_cast<float>(j + 1) / M);  // sin(theta_{j+})
      const float sinLo2 = std::sqrt(static_cast<float>(j) / M);     // sin(theta_{j-})
      const float rmin2 = std::fmax(std::fmin(rc[idx], rc[idxk]), 1.0e-6f);
      const float coef2 = (sinHi - sinLo2) / rmin2;
      aziR[0] += coef2 * (Lc[idx].x - Lc[idxk].x);
      aziR[1] += coef2 * (Lc[idx].y - Lc[idxk].y);
      aziR[2] += coef2 * (Lc[idx].z - Lc[idxk].z);
    }
    for (int c = 0; c < 3; ++c) {
      gR[c] = gR[c] + vHatRot * rotSum[c];
      gT[c] = gT[c] + uHat * polR[c] + vHatWall * aziR[c];
    }
  }
  // Ward prefactors converted to this cache's no-1/pi irradiance convention
  // (E_umbreon = E_Ward/pi): rotational Eq 3 carries pi/(MN), so /pi leaves
  // 1/(MN) = 1/total; translational Eq 4 has no MN prefactor (only the explicit
  // per-term coefficients already accumulated), so it just gets the 1/pi.
  const float rotScale = 1.0f / static_cast<float>(total);
  // Clamp each gradient to the split-sphere upper bound (Jarosz Eq 3.12-3.14):
  // a translation by dx changes E by at most (4/pi)(E/R)*dx, and a rotation by
  // dphi by at most E*sin(dphi). The analytic gradient can far exceed this in
  // dense geometry where a gather ray hits a neighbor at tiny distance r (the
  // 1/Min(r) term explodes), so without this clamp the linear extrapolation
  // overshoots into per-record speckle. Bounding |grad_t| <= (4/pi)E/R and
  // |grad_r| <= E keeps the extrapolation within the physically possible range.
  const float invRi = (Ri > 1.0e-6f) ? 1.0f / Ri : 0.0f;
  const float Ec[3] = {std::fabs(rec.irradiance.x), std::fabs(rec.irradiance.y),
                       std::fabs(rec.irradiance.z)};
  auto clampMag = [](const Vec3& g, float maxMag) -> Vec3 {
    const float m = length(g);
    return (m > maxMag && m > 1.0e-12f) ? g * (maxMag / m) : g;
  };
  for (int c = 0; c < 3; ++c) {
    rec.gradR[c] = clampMag(gR[c] * rotScale, Ec[c]);
    rec.gradT[c] = clampMag(gT[c] * invPi, (4.0f * invPi) * Ec[c] * invRi);
  }
}

// [C] Fill: gather irradiance at each record in parallel. Each record seeds its
// RNG only from its index, so the result is independent of TBB thread count and
// bit-reproducible. Writes E_i, bent normal and harmonic-mean radius. prevCache
// is null for the one-bounce stage; for later bounces it is the read-only
// previous-bounce snapshot whose indirect is added at each gather hit.
inline void fillRecords(IrradianceCache& cache, const IrradianceCacheParams& p,
                        const IrradianceCache* prevCache) {
  const int N = p.samples;
  const float R = p.maxDistance;
  const float Rmin = 0.25f * p.spacing;
  const std::size_t n = cache.records.size();
  tbb::parallel_for(
      tbb::blocked_range<std::size_t>(0, n),
      [&](const tbb::blocked_range<std::size_t>& rg) {
        // Per-cell scratch for the stratified/gradient path, reused across the
        // block's records (avoids per-record allocation in the hot loop).
        std::vector<Vec3> Lc;
        std::vector<float> rc;
        for (std::size_t ri = rg.begin(); ri != rg.end(); ++ri) {
          IrradianceRecord& rec = cache.records[ri];
          if (p.gradients) {
            fillRecordStratified(rec, p, static_cast<uint32_t>(ri), Lc, rc,
                                 prevCache);
            continue;
          }
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
              const Vec3 Ly = oneBounceRadiance(p, rh, O, wi, prevCache);
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
  // Ward gradient extrapolation (Eq 5): adjust a record's stored E by its
  // translational gradient along the displacement and its rotational gradient
  // along the normal-mismatch axis, so the blend varies smoothly between records
  // instead of stepping per voxel. Clamp each channel to >= 0 (irradiance cannot
  // be negative; the linear extrapolation can otherwise overshoot into dark halos
  // at large displacements). A no-gradient record (gradients off) extrapolates to
  // its plain irradiance.
  auto extrapolate = [&](const IrradianceRecord& r) -> Vec3 {
    if (!p.gradients) return r.irradiance;
    const Vec3 disp = x - r.position;
    const Vec3 rotAxis = cross(r.normal, nx);
    const float d[3] = {dot(disp, r.gradT[0]) + dot(rotAxis, r.gradR[0]),
                        dot(disp, r.gradT[1]) + dot(rotAxis, r.gradR[1]),
                        dot(disp, r.gradT[2]) + dot(rotAxis, r.gradR[2])};
    // Limit each channel's extrapolation to a fraction of the stored irradiance.
    // The split-sphere bound at fill time is loose for small-radius (concave)
    // records, so a single noisy gradient direction can still swing the value
    // hard near a record's influence edge, producing per-cell speckle on bumpy
    // surfaces. Capping the correction at +-kLim*E keeps gradients to their job
    // (linearizing the piecewise-constant steps) without letting them dominate.
    constexpr float kLim = 0.5f;
    const float Ei[3] = {r.irradiance.x, r.irradiance.y, r.irradiance.z};
    Vec3 e{0, 0, 0};
    float* eo = &e.x;
    for (int c = 0; c < 3; ++c) {
      const float lim = kLim * std::fabs(Ei[c]);
      const float dc = std::fmax(-lim, std::fmin(lim, d[c]));
      eo[c] = std::fmax(0.0f, Ei[c] + dc);
    }
    return e;
  };
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
    Esum = Esum + extrapolate(r) * w;
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
    // Plain value (no gradient) at the coverage edge: the fallback fires only when
    // no record passes the weight cutoff, i.e. the point is poorly covered, where
    // linear extrapolation is least trustworthy and would speckle. Gradients are
    // applied only inside the trusted weighted blend above.
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

// [C] Dark-outlier rejection. A record whose hemisphere is (near) fully self-
// occluded gathers ~0 in one-bounce -- all its rays hit immediate neighbors that
// are themselves shadowed, so they re-emit no light. On high-frequency molecular
// surfaces this aliases: a seed that lands in a sub-spacing micro-pocket (an atom-
// sphere valley) reads black while its rim neighbors see open sky and read bright,
// so it paints an isolated dark voxel square. Replace each such dark outlier's
// irradiance with the distance-weighted mean of its same-component neighbors: an
// isolated dark record is lifted to its bright surroundings, while a genuine dark
// cavity (its neighbors are dark too, so their median is dark) is NOT flagged and
// stays dark -- preserving the real depth cue. Only near-fully-occluded records
// (occlusion >= 0.9) are candidates, so gentle concavities (the smooth shading the
// effect is meant to keep) are never touched. The pass reads a snapshot (new values
// computed from the old records, assigned after the scan), so it is order-
// independent and bit-reproducible like the neighbor radius clamp.
inline void rejectDarkOutliers(IrradianceCache& cache,
                               const IrradianceCacheParams& p) {
  const std::size_t n = cache.records.size();
  if (n == 0) return;
  const float reach = 2.0f * p.spacing;
  IrradianceCache tg;
  tg.setCellSize(reach);
  tg.records = cache.records;  // positions only; radius unused here
  tg.buildGrid(0.0f);          // own-cell registration; the 3x3x3 query spans reach
  auto lumOf = [](const Vec3& e) {
    return 0.2126f * e.x + 0.7152f * e.y + 0.0722f * e.z;
  };
  std::vector<Vec3> newE(n);
  std::vector<uint8_t> changed(n, 0);
  std::vector<uint32_t> cand;
  std::vector<float> nbLum;
  for (std::size_t i = 0; i < n; ++i) {
    const IrradianceRecord& ri = cache.records[i];
    newE[i] = ri.irradiance;
    if (ri.occlusion < 0.9f) continue;  // only near-fully-occluded candidates
    tg.query(ri.position, cand);
    Vec3 wMeanE{0.0f, 0.0f, 0.0f};
    float wSum = 0.0f;
    nbLum.clear();
    for (uint32_t j : cand) {
      if (j == static_cast<uint32_t>(i)) continue;
      const IrradianceRecord& rj = cache.records[j];
      if (rj.componentID != ri.componentID) continue;
      const float d = length(ri.position - rj.position);
      if (d > reach) continue;
      if (dot(ri.normal, rj.normal) < p.normalReject) continue;
      nbLum.push_back(lumOf(rj.irradiance));
      const float w = 1.0f / (d + 1.0e-4f);
      wMeanE = wMeanE + rj.irradiance * w;
      wSum += w;
    }
    if (nbLum.size() < 4 || wSum <= 0.0f) continue;  // too few neighbors to judge
    std::sort(nbLum.begin(), nbLum.end());
    const float med = nbLum[nbLum.size() / 2];
    // Flag only a clear dark outlier: well below a bright neighbor median. A
    // genuine cavity has a dark median (med <= 0.05), so it is never flagged.
    if (med > 0.05f && lumOf(ri.irradiance) < 0.5f * med) {
      newE[i] = wMeanE * (1.0f / wSum);
      changed[i] = 1;
    }
  }
  for (std::size_t i = 0; i < n; ++i)
    if (changed[i]) cache.records[i].irradiance = newE[i];
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

  // Bounce 1: one-bounce (direct + environment), no previous cache to read.
  fillRecords(cache, p, nullptr);
  if (p.outlierReject) rejectDarkOutliers(cache, p);
  neighborClamp(cache, p.accuracy);
  cache.buildGrid(p.accuracy);

  // Additional bounces (giBounces > 1): each stage re-gathers irradiance, adding
  // the previous bounce's interpolated indirect at every gather hit. The previous
  // cache is a read-only snapshot, so each stage stays per-record independent and
  // bit-reproducible (the determinism contract). Placement and the geometry-only
  // harmonic-mean radius are identical across bounces; only irradiance changes.
  const int bounces = (p.bounces < 1) ? 1 : p.bounces;
  for (int b = 2; b <= bounces; ++b) {
    const IrradianceCache prev = cache;  // snapshot of bounce b-1 (records + grid)
    fillRecords(cache, p, &prev);
    if (p.outlierReject) rejectDarkOutliers(cache, p);
    neighborClamp(cache, p.accuracy);
    cache.buildGrid(p.accuracy);
  }
  return cache;
}

}  // namespace detail
}  // namespace umbreon
