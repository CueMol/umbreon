#include "render/denoise.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "scene.hpp"  // Vec3, dot, length

namespace umbreon {

namespace {

constexpr float kEps = 1.0e-3f;

inline float luminance(float r, float g, float b) {
  return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

}  // namespace

// Edge-avoiding a-trous wavelet denoiser (Dammertz 2010) with SVGF edge-stops.
// frame.color is final-resolution linear HDR RGBA. The smoothing is steered by
// the world-space position (depth), the shading normal and the color luminance,
// so flat noisy regions converge while depth/normal/illumination discontinuities
// are kept. Background pixels (no geometry) and missing guides degrade gracefully.
void denoiseAtrous(FrameResult& frame, const RenderOptions& opt) {
  const int W = frame.width;
  const int H = frame.height;
  const std::size_t N = static_cast<std::size_t>(W) * H;
  if (W <= 0 || H <= 0 || frame.color.size() < N * 4) return;
  const int iters = std::max(0, opt.denoiseIters);
  if (iters == 0) return;

  const bool haveNormal = frame.normal.size() == N * 3;
  const bool havePos = frame.position.size() == N * 3;
  const bool haveAlbedo =
      opt.denoiseDemodulateAlbedo && frame.albedo.size() == N * 3;
  // No usable edge guide => skip rather than blur indiscriminately across edges.
  if (!haveNormal && !havePos) return;

  const float sigmaN = opt.denoiseSigmaN;
  const float sigmaZ = std::max(1.0e-4f, opt.denoiseSigmaZ);
  const float sigmaL = std::max(1.0e-4f, opt.denoiseSigmaL);

  // Validity: a pixel carries geometry if its normal is unit-length (hits) / its
  // position is non-zero. Background stays untouched (and never contributes a tap).
  std::vector<unsigned char> valid(N, 0);
  for (std::size_t p = 0; p < N; ++p) {
    bool v;
    if (haveNormal) {
      const float nx = frame.normal[p * 3 + 0], ny = frame.normal[p * 3 + 1],
                  nz = frame.normal[p * 3 + 2];
      v = (nx * nx + ny * ny + nz * nz) > 0.25f;
    } else {
      const float px = frame.position[p * 3 + 0], py = frame.position[p * 3 + 1],
                  pz = frame.position[p * 3 + 2];
      v = (px * px + py * py + pz * pz) > 0.0f;
    }
    valid[p] = v ? 1 : 0;
  }

  // Working illumination buffer: demodulated (color/albedo) or raw color. Only
  // valid pixels are processed; invalid keep their color verbatim on write-back.
  std::vector<float> irr(N * 3);
  for (std::size_t p = 0; p < N; ++p) {
    for (int c = 0; c < 3; ++c) {
      float v = frame.color[p * 4 + c];
      if (haveAlbedo) {
        const float a = frame.albedo[p * 3 + c];
        v /= (a > kEps ? a : kEps);
      }
      irr[p * 3 + c] = v;
    }
  }

  // Per-pixel world-space "depth gradient" magnitude: how far the surface point
  // moves between adjacent pixels, so the position edge-stop adapts to scene scale
  // (a flat plane has a small gradient => only co-planar taps pass; a silhouette a
  // large jump => the far side is rejected). Computed once at unit pixel spacing.
  std::vector<float> posGrad;
  if (havePos) {
    posGrad.assign(N, kEps);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const std::size_t p = static_cast<std::size_t>(y) * W + x;
        if (!valid[p]) continue;
        const Vec3 pp{frame.position[p * 3 + 0], frame.position[p * 3 + 1],
                      frame.position[p * 3 + 2]};
        float g = kEps;
        const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (auto& d : nb) {
          const int qx = x + d[0], qy = y + d[1];
          if (qx < 0 || qx >= W || qy < 0 || qy >= H) continue;
          const std::size_t q = static_cast<std::size_t>(qy) * W + qx;
          if (!valid[q]) continue;
          const Vec3 pq{frame.position[q * 3 + 0], frame.position[q * 3 + 1],
                        frame.position[q * 3 + 2]};
          g = std::max(g, length(pp - pq));
        }
        posGrad[p] = g;
      }
    }
  }

  // Per-pixel luminance variance over a 3x3 valid neighborhood (single-frame
  // spatial estimate; SVGF uses temporal but we have one frame). Fixed across
  // iterations -- it scales the luminance edge-stop to the local noise level.
  std::vector<float> var(N, 0.0f);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const std::size_t p = static_cast<std::size_t>(y) * W + x;
      if (!valid[p]) continue;
      float s = 0.0f, s2 = 0.0f;
      int n = 0;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          const int qx = x + dx, qy = y + dy;
          if (qx < 0 || qx >= W || qy < 0 || qy >= H) continue;
          const std::size_t q = static_cast<std::size_t>(qy) * W + qx;
          if (!valid[q]) continue;
          const float l = luminance(irr[q * 3 + 0], irr[q * 3 + 1],
                                    irr[q * 3 + 2]);
          s += l;
          s2 += l * l;
          ++n;
        }
      if (n > 0) {
        const float mean = s / n;
        var[p] = std::max(0.0f, s2 / n - mean * mean);
      }
    }
  }

  // B3 spline a-trous kernel (Dammertz): outer product of {1/16,1/4,3/8,1/4,1/16}.
  const float h5[5] = {0.0625f, 0.25f, 0.375f, 0.25f, 0.0625f};
  std::vector<float> next(N * 3);
  std::vector<float> lum(N);

  for (int it = 0; it < iters; ++it) {
    const int step = 1 << it;  // 2^i hole spacing
    for (std::size_t p = 0; p < N; ++p)
      lum[p] = luminance(irr[p * 3 + 0], irr[p * 3 + 1], irr[p * 3 + 2]);

    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const std::size_t p = static_cast<std::size_t>(y) * W + x;
        if (!valid[p]) {
          next[p * 3 + 0] = irr[p * 3 + 0];
          next[p * 3 + 1] = irr[p * 3 + 1];
          next[p * 3 + 2] = irr[p * 3 + 2];
          continue;
        }
        const Vec3 np = haveNormal
                            ? Vec3{frame.normal[p * 3 + 0],
                                   frame.normal[p * 3 + 1],
                                   frame.normal[p * 3 + 2]}
                            : Vec3{0, 0, 0};
        const Vec3 pp = havePos
                            ? Vec3{frame.position[p * 3 + 0],
                                   frame.position[p * 3 + 1],
                                   frame.position[p * 3 + 2]}
                            : Vec3{0, 0, 0};
        const float lp = lum[p];
        const float lDenom = sigmaL * std::sqrt(var[p]) + kEps;
        const float zDenom = havePos ? (sigmaZ * posGrad[p] * step + kEps) : 1.0f;

        float sum[3] = {0, 0, 0};
        float wsum = 0.0f;
        for (int dy = -2; dy <= 2; ++dy) {
          const int qy = y + dy * step;
          if (qy < 0 || qy >= H) continue;
          for (int dx = -2; dx <= 2; ++dx) {
            const int qx = x + dx * step;
            if (qx < 0 || qx >= W) continue;
            const std::size_t q = static_cast<std::size_t>(qy) * W + qx;
            if (!valid[q]) continue;
            float w = h5[dx + 2] * h5[dy + 2];
            if (haveNormal) {
              const Vec3 nq{frame.normal[q * 3 + 0], frame.normal[q * 3 + 1],
                            frame.normal[q * 3 + 2]};
              const float nd = std::max(0.0f, dot(np, nq));
              w *= std::pow(nd, sigmaN);
            }
            if (havePos) {
              const Vec3 pq{frame.position[q * 3 + 0], frame.position[q * 3 + 1],
                            frame.position[q * 3 + 2]};
              w *= std::exp(-length(pp - pq) / zDenom);
            }
            w *= std::exp(-std::fabs(lp - lum[q]) / lDenom);
            sum[0] += w * irr[q * 3 + 0];
            sum[1] += w * irr[q * 3 + 1];
            sum[2] += w * irr[q * 3 + 2];
            wsum += w;
          }
        }
        if (wsum > 0.0f) {
          next[p * 3 + 0] = sum[0] / wsum;
          next[p * 3 + 1] = sum[1] / wsum;
          next[p * 3 + 2] = sum[2] / wsum;
        } else {
          next[p * 3 + 0] = irr[p * 3 + 0];
          next[p * 3 + 1] = irr[p * 3 + 1];
          next[p * 3 + 2] = irr[p * 3 + 2];
        }
      }
    }
    irr.swap(next);
  }

  // Re-modulate and write back (valid pixels only; background color untouched).
  for (std::size_t p = 0; p < N; ++p) {
    if (!valid[p]) continue;
    for (int c = 0; c < 3; ++c) {
      float v = irr[p * 3 + c];
      if (haveAlbedo) v *= frame.albedo[p * 3 + c];
      frame.color[p * 4 + c] = std::max(0.0f, v);
    }
  }
}

}  // namespace umbreon
