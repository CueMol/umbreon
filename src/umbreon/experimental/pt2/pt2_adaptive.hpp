// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt2 variance-adaptive spp (the Cycles split-buffer scheme): the base gather
// records, alongside the full mean E, the mean of its first half of samples.
// The normalized L1 luminance difference between the two estimates a pixel's
// remaining Monte-Carlo error; pixels above threshold (mask dilated so a
// converged pixel wedged between noisy ones is not left behind -- Cycles'
// filter_x/y) get one refinement pass that CONTINUES the pixel's sample
// sequence (Sobol index / tea2 counter offset by the base spp) and is merged
// as a weighted mean. Deterministic by the 2-pass argument: the mask is a
// pure function of the completed base buffers, and the refinement pass is a
// pure function of the mask.
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

namespace umbreon {
namespace detail {

// Convergence check (Cycles' film_adaptive_sampling_convergence_check,
// adapted to the E buffer): error = |mean_all - mean_half|_lum, normalized by
// sqrt(intensity) for dark pixels and intensity above 1 -- darker regions get
// proportionally tighter absolute thresholds, matching perception after the
// display transform. E holds mean(spp), halfE holds SUM of the first
// floor(spp/2) samples.
inline void pt2AdaptiveMask(int W, int H, const std::vector<float>& E,
                            const std::vector<float>& halfE,
                            const uint8_t* eligible, int spp, float threshold,
                            std::vector<uint8_t>& mask) {
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  mask.assign(npix, 0);
  const int nHalf = spp / 2;
  if (nHalf < 1) {  // spp == 1: no split estimate; refine every eligible pixel
    for (std::size_t i = 0; i < npix; ++i) mask[i] = eligible[i] ? 1 : 0;
    return;
  }
  const float invHalf = 1.0f / static_cast<float>(nHalf);
  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
        for (int py = r.rows().begin(); py != r.rows().end(); ++py) {
          for (int px = r.cols().begin(); px != r.cols().end(); ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            if (!eligible[pix]) continue;
            const float lumA = 0.2126f * E[pix * 3 + 0] +
                               0.7152f * E[pix * 3 + 1] +
                               0.0722f * E[pix * 3 + 2];
            const float lumH = (0.2126f * halfE[pix * 3 + 0] +
                                0.7152f * halfE[pix * 3 + 1] +
                                0.0722f * halfE[pix * 3 + 2]) *
                               invHalf;
            const float diff = std::fabs(lumA - lumH);
            const float norm =
                (lumA < 1.0f) ? std::sqrt(std::fmax(lumA, 1.0e-4f)) : lumA;
            if (diff > threshold * norm) mask[pix] = 1;
          }
        }
      });
}

// 3x3 dilation of the refinement mask (one Cycles filter pass; call twice for
// the historical 5x5 reach). Reads `in`, writes `out` -- ping-pong purity.
inline void pt2DilateMask(int W, int H, const std::vector<uint8_t>& in,
                          const uint8_t* eligible,
                          std::vector<uint8_t>& out) {
  out.assign(in.size(), 0);
  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
        for (int py = r.rows().begin(); py != r.rows().end(); ++py) {
          for (int px = r.cols().begin(); px != r.cols().end(); ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            if (!eligible[pix]) continue;
            uint8_t v = in[pix];
            for (int dy = -1; dy <= 1 && !v; ++dy) {
              const int ny = py + dy;
              if (ny < 0 || ny >= H) continue;
              for (int dx = -1; dx <= 1 && !v; ++dx) {
                const int nx = px + dx;
                if (nx < 0 || nx >= W) continue;
                v = in[static_cast<std::size_t>(ny) * W + nx];
              }
            }
            out[pix] = v;
          }
        }
      });
}

// Merge the refinement gather into the base: E = (E*spp + Er*extra)/(spp+extra)
// on masked pixels (occ likewise). Er/occr are the refinement pass MEANS.
inline void pt2MergeRefined(int W, int H, const std::vector<uint8_t>& mask,
                            int spp, int extra, const std::vector<float>& Er,
                            const std::vector<float>& occr,
                            std::vector<float>& E, std::vector<float>& occ) {
  const float wB = static_cast<float>(spp) / static_cast<float>(spp + extra);
  const float wR = 1.0f - wB;
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
        for (int py = r.rows().begin(); py != r.rows().end(); ++py) {
          for (int px = r.cols().begin(); px != r.cols().end(); ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            if (pix >= npix || !mask[pix]) continue;
            for (int c = 0; c < 3; ++c)
              E[pix * 3 + c] = E[pix * 3 + c] * wB + Er[pix * 3 + c] * wR;
            occ[pix] = occ[pix] * wB + occr[pix] * wR;
          }
        }
      });
}

}  // namespace detail
}  // namespace umbreon
