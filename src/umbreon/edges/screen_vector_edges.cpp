#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace umbreon {
namespace {

constexpr std::uint32_t kBackground = 0xFFFFFFFFu;

// World units spanned by one pixel at linear view-z `vz` (identity in vz under
// ortho). Uses the vertical extent; the renderer's pixels are square.
inline float pixelSizeAt(const ScreenProj& sp, float vz) {
  return sp.ortho ? (2.0f * sp.halfH / static_cast<float>(sp.H))
                  : (2.0f * sp.persHalfH * vz / static_cast<float>(sp.H));
}

// One-sided slope of the viewZ field at pixel `a` looking away from the crack
// (toward `outer`), clamped to +-clampS. Background / off-image outer neighbors
// contribute zero slope (flat extrapolation).
inline float sideSlope(const float* viewZ, const std::uint32_t* objectId,
                       int idxA, int idxOuter, bool outerValid, float clampS) {
  if (!outerValid || objectId[idxOuter] == kBackground) return 0.0f;
  const float s = viewZ[idxA] - viewZ[idxOuter];
  return std::max(-clampS, std::min(clampS, s));
}

// Classify ONE crack between pixel indices ia (first: left/top) and ib
// (second: right/bottom). iOutA / iOutB are the outer straight-line neighbors
// (a's far side, b's far side) with validity flags. Returns the packed crack
// byte (0 = no edge).
inline std::uint8_t classifyPair(const float* viewZ,
                                 const std::uint32_t* objectId,
                                 const float* normal, int ia, int ib, int iOutA,
                                 bool outAValid, int iOutB, bool outBValid,
                                 const ScreenProj& sp, float cosCreaseBase,
                                 const ScreenClassifyParams& p) {
  const bool bgA = objectId[ia] == kBackground;
  const bool bgB = objectId[ib] == kBackground;
  if (bgA && bgB) return 0;

  // 1. Silhouette: exactly one side background; the foreground pixel owns.
  if (bgA != bgB) {
    if (!p.silhouette) return 0;
    const std::uint8_t owner = bgA ? kCrackOwnerBit : 0;
    return static_cast<std::uint8_t>(CrackClass::Silhouette) | owner;
  }

  // 2. ObjectId boundary: both foreground, different id; nearer side owns.
  if (objectId[ia] != objectId[ib]) {
    if (!p.objectBoundary) return 0;
    const std::uint8_t owner = viewZ[ia] <= viewZ[ib] ? 0 : kCrackOwnerBit;
    return static_cast<std::uint8_t>(CrackClass::ObjectId) | owner;
  }

  // 3. DepthGap: same id, both one-sided planar extrapolations miss the far
  // pixel (slope-adaptive second-derivative test; see header).
  const float vzA = viewZ[ia], vzB = viewZ[ib];
  if (p.silhouette) {
    const float vzNear = std::min(vzA, vzB);
    const float px = pixelSizeAt(sp, vzNear);
    const float clampS = p.slopeClampPx * px;
    const float predA =
        vzA + sideSlope(viewZ, objectId, ia, iOutA, outAValid, clampS);
    const float predB =
        vzB + sideSlope(viewZ, objectId, ib, iOutB, outBValid, clampS);
    const float gapA = std::fabs(vzB - predA);
    const float gapB = std::fabs(vzA - predB);
    if (std::min(gapA, gapB) > p.depthGapPx * px) {
      const std::uint8_t owner = vzA <= vzB ? 0 : kCrackOwnerBit;
      return static_cast<std::uint8_t>(CrackClass::DepthGap) | owner;
    }
  }

  // 4. Crease: shading-normal fold, angle widened at grazing incidence. The
  // view axis V is the camera forward (exact under ortho; a per-pixel ray
  // direction refinement is a later option -- CueMol scenes are mostly ortho).
  if (p.crease) {
    const float* nA = normal + 3 * static_cast<std::size_t>(ia);
    const float* nB = normal + 3 * static_cast<std::size_t>(ib);
    const float lA = nA[0] * nA[0] + nA[1] * nA[1] + nA[2] * nA[2];
    const float lB = nB[0] * nB[0] + nB[1] * nB[1] + nB[2] * nB[2];
    if (lA > 1.0e-12f && lB > 1.0e-12f) {
      const float d = nA[0] * nB[0] + nA[1] * nB[1] + nA[2] * nB[2];
      float cosT = cosCreaseBase;
      if (p.grazeK > 0.0f) {
        const float nvA = std::fabs(nA[0] * sp.dir.x + nA[1] * sp.dir.y +
                                    nA[2] * sp.dir.z);
        const float nvB = std::fabs(nB[0] * sp.dir.x + nB[1] * sp.dir.y +
                                    nB[2] * sp.dir.z);
        const float widen = 1.0f + p.grazeK * (1.0f - std::min(nvA, nvB));
        const float deg = std::min(179.0f, p.creaseAngleDeg * widen);
        cosT = std::cos(deg * 3.14159265358979323846f / 180.0f);
      }
      if (d < cosT) {
        const std::uint8_t owner = vzA <= vzB ? 0 : kCrackOwnerBit;
        return static_cast<std::uint8_t>(CrackClass::Crease) | owner;
      }
    }
  }
  return 0;
}

}  // namespace

CrackField classifyCracks(int W, int H, const float* viewZ,
                          const std::uint32_t* objectId, const float* normal,
                          const ScreenProj& sp,
                          const ScreenClassifyParams& params) {
  CrackField cf;
  cf.W = W;
  cf.H = H;
  if (W <= 0 || H <= 0) return cf;
  cf.right.assign(static_cast<std::size_t>(W) * H, 0);
  cf.down.assign(static_cast<std::size_t>(W) * H, 0);
  if (W < 2 && H < 2) return cf;

  const float cosCreaseBase =
      std::cos(params.creaseAngleDeg * 3.14159265358979323846f / 180.0f);

  // Row-parallel: row y writes right[y*W+..] and down[y*W+..] only (disjoint);
  // reads of rows y-1 .. y+2 are read-only.
  tbb::parallel_for(
      tbb::blocked_range<int>(0, H),
      [&](const tbb::blocked_range<int>& rows) {
        for (int y = rows.begin(); y != rows.end(); ++y) {
          const int row = y * W;
          for (int x = 0; x < W; ++x) {
            const int ia = row + x;
            if (x + 1 < W) {  // right crack (x,y)-(x+1,y)
              cf.right[static_cast<std::size_t>(ia)] = classifyPair(
                  viewZ, objectId, normal, ia, ia + 1, ia - 1, x - 1 >= 0,
                  ia + 2, x + 2 < W, sp, cosCreaseBase, params);
            }
            if (y + 1 < H) {  // down crack (x,y)-(x,y+1)
              cf.down[static_cast<std::size_t>(ia)] = classifyPair(
                  viewZ, objectId, normal, ia, ia + W, ia - W, y - 1 >= 0,
                  ia + 2 * W, y + 2 < H, sp, cosCreaseBase, params);
            }
          }
        }
      });
  return cf;
}

}  // namespace umbreon
