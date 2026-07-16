// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt2 ReSTIR-GI spatial resampling: each round, every gather pixel restarts
// its reservoir from its own previous-round state and streams in K neighbor
// reservoirs, re-weighted to its own geometry (target pdf at the receiver
// times the reconnection Jacobian). Two rounds at kajiya-like kernel sizes
// multiply the effective sample count by roughly an order of magnitude at
// the cost of pure buffer arithmetic -- no new gather rays.
//
// Determinism: ping-pong purity. Round k reads ONLY the completed round k-1
// buffer plus the immutable G-buffer, and writes its own pixel; the neighbor
// pattern and the reservoir selection draws come from a per-(pixel, round)
// tea2 chain. Tile parallelism therefore cannot affect the result -- the
// same 2-pass argument that keeps the rest of pt1/pt2 bit-exact across
// thread counts. (No reference implementation addresses this; they are all
// GPU passes with framebuffer barriers, which this reproduces on the CPU.)
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "experimental/pt2/pt2_reservoir.hpp"
#include "experimental/pt2/pt2_sampler.hpp"
#include "render/progress_slice.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

struct Pt2SpatialParams {
  int rounds = 2;          // RenderOptions::pt2Rounds
  float radius = 16.0f;    // round-0 kernel radius in gather-grid pixels
  bool unbiased = false;   // Z-normalization with visibility re-check rays
  float mCap = 100.0f;     // clamp on a streamed reservoir's M
  float wClamp = 0.0f;     // firefly clamp on the finalized W (0 = off)
  uint32_t seedMix = 0;    // hashed frame seed (same value the gather used)
  RTCScene scene = nullptr;  // for the unbiased visibility re-check only
};

// One spatial round over a W x H gather grid. `position/normal/depth/eligible`
// are the grid G-buffer channels; `in` is round k-1, `out` is written.
//
// When `Eout` is non-null (the FINAL round), the pixel's E_stored is ALSO
// computed -- not from the single winning sample, but as the balance-MIS
// weighted sum over ALL contributor samples:
//   E(q) = (1/pi) * sum_k  M_k p_k(s_k) W_k J_k f_q(s_k) / sum_j M_j p_j(s_k)
// A single-winner resolve keeps the exact luminance (the target is luminance-
// proportional) but hands the COLOR to one sample -- measured as strong
// blue-channel speckle wherever white sky light and colored bounce light mix.
// Shading every contributor removes that winner-take-all chroma noise for
// (K+1)^2 target evaluations of pure G-buffer arithmetic, no rays.
inline void pt2SpatialRound(int W, int H, const float* position,
                            const float* normal, const float* depth,
                            const uint8_t* eligible,
                            const std::vector<Pt2Reservoir>& in,
                            std::vector<Pt2Reservoir>& out, int round,
                            const Pt2SpatialParams& sp,
                            const ProgressSlice* prog, float* Eout = nullptr) {
  // kajiya's two-pass shape: 8 neighbors then 5, kernel radius halving.
  const int K = (round == 0) ? 8 : 5;
  float radius = sp.radius / static_cast<float>(1 << round);
  if (radius < 3.0f) radius = 3.0f;
  constexpr float kGoldenAngle = 2.399963230f;
  constexpr float kNormalThreshold = 0.5f;   // Falcor default
  constexpr float kDepthThreshold = 0.1f;    // relative depth difference
  constexpr float kJacobianClamp = 10.0f;
  constexpr float kPt2ReuseDistanceRatio = 4.0f;  // see the distance gate

  const uint32_t roundSalt =
      pt2HashHp(sp.seedMix ^ (0x53504154u + static_cast<uint32_t>(round)));

  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
        if (prog && prog->cancelled()) return;
        for (int py = r.rows().begin(); py != r.rows().end(); ++py) {
          for (int px = r.cols().begin(); px != r.cols().end(); ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            if (!eligible[pix]) {
              out[pix] = Pt2Reservoir{};
              continue;
            }
            const Vec3 P{position[pix * 3 + 0], position[pix * 3 + 1],
                         position[pix * 3 + 2]};
            const Vec3 N{normal[pix * 3 + 0], normal[pix * 3 + 1],
                         normal[pix * 3 + 2]};
            const float d0 = depth ? depth[pix] : 0.0f;

            uint32_t s0 = pt2HashHp(static_cast<uint32_t>(pix) ^ roundSalt);
            uint32_t s1 = 0x52535452u;

            Pt2Reservoir dst;
            double wSum = 0.0;
            float Mtot = 0.0f;
            // Contributor log (own + up to K neighbors): the reuse point's
            // receiver geometry (for the MIS denominator) and its SAMPLE
            // (for the final-round multi-sample shading).
            Vec3 cPos[9], cN[9];
            float cM[9];
            Vec3 sPos[9], sRad[9];
            float sW[9], sJ[9];
            int nContrib = 0;

            // Stream the pixel's own previous-round reservoir (Jacobian = 1:
            // its W is already normalized at this receiver).
            {
              const Pt2Reservoir& own = in[pix];
              const float m = (own.M > sp.mCap) ? sp.mCap : own.M;
              const float pHat =
                  pt2TargetPdf(P, N, own.samplePos, own.radiance);
              const float w = pHat * own.W * m;
              tea2(s0, s1);
              if (pt2ReservoirUpdate(wSum, Mtot, w, m, u32ToUnorm(s0))) {
                dst.samplePos = own.samplePos;
                dst.sampleNormal = own.sampleNormal;
                dst.radiance = own.radiance;
              }
              cPos[nContrib] = P;
              cN[nContrib] = N;
              cM[nContrib] = m;
              sPos[nContrib] = own.samplePos;
              sRad[nContrib] = own.radiance;
              sW[nContrib] = own.W;
              sJ[nContrib] = 1.0f;
              ++nContrib;
            }

            // Golden-angle spiral of K neighbors, rotated per pixel.
            tea2(s0, s1);
            const float rot = u32ToUnorm(s0) * 6.2831853f;
            for (int k = 0; k < K; ++k) {
              const float ang = rot + static_cast<float>(k) * kGoldenAngle;
              const float rad =
                  std::sqrt((static_cast<float>(k) + 0.5f) /
                            static_cast<float>(K)) *
                  radius;
              const int nx = px + static_cast<int>(std::lround(
                                      std::cos(ang) * rad));
              const int ny = py + static_cast<int>(std::lround(
                                      std::sin(ang) * rad));
              if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
              const std::size_t npx = static_cast<std::size_t>(ny) * W + nx;
              if (npx == pix || !eligible[npx]) continue;

              // Geometric rejection: reuse only across compatible surfaces.
              const Vec3 Nn{normal[npx * 3 + 0], normal[npx * 3 + 1],
                            normal[npx * 3 + 2]};
              if (dot(N, Nn) < kNormalThreshold) continue;
              if (depth) {
                const float dn = depth[npx];
                if (std::fabs(d0 - dn) > kDepthThreshold * d0) continue;
              }

              const Pt2Reservoir& nb = in[npx];
              if (nb.M <= 0.0f) continue;
              const Vec3 Pn{position[npx * 3 + 0], position[npx * 3 + 1],
                            position[npx * 3 + 2]};
              // Distance gate: reconnection reuse is only well-conditioned
              // when the sample is FAR relative to the receiver spacing
              // (Jacobian ~ 1). Molecular GI is dominated by contact-range
              // hits whose distance is comparable to the world spacing of
              // gather pixels -- reusing those explodes/starves the Jacobian
              // and was measured to ADD noise. Far samples (sky, distant
              // surfaces) are where the variance lives anyway; keep close
              // samples own-pixel only.
              {
                const Vec3 pq{Pn.x - P.x, Pn.y - P.y, Pn.z - P.z};
                const Vec3 ps{nb.samplePos.x - Pn.x, nb.samplePos.y - Pn.y,
                              nb.samplePos.z - Pn.z};
                const float spacing2 = dot(pq, pq);
                const float sdist2 = dot(ps, ps);
                if (sdist2 < kPt2ReuseDistanceRatio * kPt2ReuseDistanceRatio *
                                 spacing2)
                  continue;
              }
              const float j = pt2Jacobian(P, Pn, nb.samplePos,
                                          nb.sampleNormal, kJacobianClamp);
              const float m = (nb.M > sp.mCap) ? sp.mCap : nb.M;
              const float pHat =
                  pt2TargetPdf(P, N, nb.samplePos, nb.radiance);
              const float w = pHat * nb.W * m * j;
              tea2(s0, s1);
              if (pt2ReservoirUpdate(wSum, Mtot, w, m, u32ToUnorm(s0))) {
                dst.samplePos = nb.samplePos;
                dst.sampleNormal = nb.sampleNormal;
                dst.radiance = nb.radiance;
              }
              cPos[nContrib] = Pn;
              cN[nContrib] = Nn;
              cM[nContrib] = m;
              sPos[nContrib] = nb.samplePos;
              sRad[nContrib] = nb.radiance;
              sW[nContrib] = nb.W;
              sJ[nContrib] = j;
              ++nContrib;
            }

            // Normalization: balance-heuristic MIS over the contributor
            // mixture (RTXDI's BASIC bias correction), NOT the naive 1/M.
            // With 1/M, the geometric reweighting jitter (cosine ratio x
            // Jacobian) on curved receivers goes straight into the estimate
            // -- measured ~7 dB WORSE than no reuse on molecular surfaces.
            // The mixture pdf piSum = sum_k M_k * p_hat_k(selected) evaluates
            // the selected sample's target at each contributor's surface
            // (pure G-buffer arithmetic, no rays):  W = wSum / piSum.
            // The unbiased mode additionally zeroes a contributor whose
            // surface cannot SEE the selected sample (one occlusion ray per
            // contributor -- RTXDI's RAY_TRACED mode), which removes the
            // reuse visibility bias at silhouettes.
            float piSum = 0.0f;
            if (wSum > 0.0) {
              for (int c = 0; c < nContrib; ++c) {
                float ps = pt2TargetPdf(cPos[c], cN[c], dst.samplePos,
                                        dst.radiance);
                if (ps <= 0.0f) continue;
                if (sp.unbiased && sp.scene) {
                  const Vec3 toS{dst.samplePos.x - cPos[c].x,
                                 dst.samplePos.y - cPos[c].y,
                                 dst.samplePos.z - cPos[c].z};
                  const float dist2 = dot(toS, toS);
                  if (dist2 <= 0.0f) continue;
                  const float dist = std::sqrt(dist2);
                  const Vec3 dir{toS.x / dist, toS.y / dist, toS.z / dist};
                  if (dot(cN[c], dir) <= 0.0f) {
                    ps = 0.0f;
                  } else {
                    const float eps = selfIntersectEps(cPos[c], dir, dist);
                    const Vec3 O{cPos[c].x + cN[c].x * eps,
                                 cPos[c].y + cN[c].y * eps,
                                 cPos[c].z + cN[c].z * eps};
                    if (occluded(sp.scene, O, dir, eps, dist - 2.0f * eps))
                      ps = 0.0f;
                  }
                }
                piSum += ps * cM[c];
              }
            }
            float Wnew = 0.0f;
            if (piSum > 0.0f && wSum > 0.0)
              Wnew = static_cast<float>(wSum) / piSum;
            dst.W = (sp.wClamp > 0.0f && Wnew > sp.wClamp) ? sp.wClamp : Wnew;
            dst.M = (Mtot > sp.mCap) ? sp.mCap : Mtot;
            out[pix] = dst;

            // Final round: shade EVERY contributor sample under balance MIS
            // (see the function comment). E_q = (1/pi) * sum_k m_k * W_k *
            // J_k * f_q(s_k), with m_k = M_k p_q(s_k) / sum_j M_j p_j(s_k).
            if (Eout) {
              Vec3 E{0.0f, 0.0f, 0.0f};
              for (int k = 0; k < nContrib; ++k) {
                if (sW[k] <= 0.0f || sJ[k] <= 0.0f) continue;
                const float pq = pt2TargetPdf(P, N, sPos[k], sRad[k]);
                if (pq <= 0.0f) continue;
                float denom = 0.0f;
                for (int j2 = 0; j2 < nContrib; ++j2)
                  denom += cM[j2] *
                           pt2TargetPdf(cPos[j2], cN[j2], sPos[k], sRad[k]);
                if (denom <= 0.0f) continue;
                const float mis = cM[k] * pq / denom;
                // f_q(s_k) = radiance * cos_q; cos_q = pq / lum(radiance).
                const float lum = pt2Luminance(sRad[k]);
                if (lum <= 0.0f) continue;
                const float cosq = pq / lum;
                const float g = mis * sW[k] * sJ[k] * cosq *
                                (1.0f / 3.14159265358979323846f);
                E.x += sRad[k].x * g;
                E.y += sRad[k].y * g;
                E.z += sRad[k].z * g;
              }
              Eout[pix * 3 + 0] = E.x;
              Eout[pix * 3 + 1] = E.y;
              Eout[pix * 3 + 2] = E.z;
            }
          }
        }
        if (prog)
          prog->addWork(
              static_cast<std::uint64_t>(r.rows().end() - r.rows().begin()) *
                  static_cast<std::uint64_t>(r.cols().end() -
                                             r.cols().begin()),
              static_cast<std::uint64_t>(W) * static_cast<std::uint64_t>(H) *
                  static_cast<std::uint64_t>(sp.rounds));
      });
}

// Run all spatial rounds (ping-pong) and write the final E buffer (E_stored
// convention; occ is untouched). The last round shades all its contributor
// samples directly into `E` (see pt2SpatialRound); `E` must already be sized
// W*H*3 by the gather, and keeps its gather values on ineligible pixels.
inline void pt2SpatialResampleAndResolve(
    int W, int H, const float* position, const float* normal,
    const float* depth, const uint8_t* eligible,
    std::vector<Pt2Reservoir>& reservoirs, const Pt2SpatialParams& sp,
    std::vector<float>& E, const ProgressSlice* prog) {
  std::vector<Pt2Reservoir> pong(reservoirs.size());
  std::vector<Pt2Reservoir>* cur = &reservoirs;
  std::vector<Pt2Reservoir>* nxt = &pong;
  for (int round = 0; round < sp.rounds; ++round) {
    if (prog && prog->cancelled()) return;
    const bool last = (round == sp.rounds - 1);
    pt2SpatialRound(W, H, position, normal, depth, eligible, *cur, *nxt,
                    round, sp, prog, last ? E.data() : nullptr);
    std::swap(cur, nxt);
  }
}

}  // namespace detail
}  // namespace umbreon
