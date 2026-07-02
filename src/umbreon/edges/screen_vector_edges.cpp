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

namespace {

// ---------------------------------------------------------------------------
// Stage 2 tracer internals. Corners are lattice nodes (cx,cy), cx in [0..W],
// cy in [0..H]. The four incident lattice edges of a corner map to cracks:
//   E: corners (cx,cy)-(cx+1,cy)   == down [(cy-1)*W + cx]
//   S: corners (cx,cy)-(cx,cy+1)   == right[ cy   *W + (cx-1)]
//   W: corners (cx-1,cy)-(cx,cy)   == down [(cy-1)*W + (cx-1)]
//   N: corners (cx,cy-1)-(cx,cy)   == right[(cy-1)*W + (cx-1)]
// (right[y*W+x] joins corners (x+1,y)-(x+1,y+1); down[y*W+x] joins
// (x,y+1)-(x+1,y+1).)

// One incident lattice edge of a corner: the crack cell it maps to (plane +
// index), and the far corner reached by walking it.
struct CornerEdge {
  bool valid = false;
  bool isRight = false;  // true: cf.right, false: cf.down
  int cell = -1;         // index into the plane
  int farCx = 0, farCy = 0;
};

// The fixed E, S, W, N direction order (the tracer's determinism contract).
inline CornerEdge cornerEdge(const CrackField& cf, int cx, int cy, int dir) {
  CornerEdge e;
  const int W = cf.W, H = cf.H;
  switch (dir) {
    case 0:  // E
      if (cy >= 1 && cy <= H - 1 && cx >= 0 && cx <= W - 1) {
        e.valid = true;
        e.isRight = false;
        e.cell = (cy - 1) * W + cx;
        e.farCx = cx + 1;
        e.farCy = cy;
      }
      break;
    case 1:  // S
      if (cx >= 1 && cx <= W - 1 && cy >= 0 && cy <= H - 1) {
        e.valid = true;
        e.isRight = true;
        e.cell = cy * W + (cx - 1);
        e.farCx = cx;
        e.farCy = cy + 1;
      }
      break;
    case 2:  // W
      if (cy >= 1 && cy <= H - 1 && cx >= 1 && cx <= W) {
        e.valid = true;
        e.isRight = false;
        e.cell = (cy - 1) * W + (cx - 1);
        e.farCx = cx - 1;
        e.farCy = cy;
      }
      break;
    default:  // 3: N
      if (cx >= 1 && cx <= W - 1 && cy >= 1 && cy <= H) {
        e.valid = true;
        e.isRight = true;
        e.cell = (cy - 1) * W + (cx - 1);
        e.farCx = cx;
        e.farCy = cy - 1;
      }
      break;
  }
  return e;
}

inline std::uint8_t crackByte(const CrackField& cf, const CornerEdge& e) {
  return e.isRight ? cf.right[static_cast<std::size_t>(e.cell)]
                   : cf.down[static_cast<std::size_t>(e.cell)];
}

inline void markConsumed(CrackField& cf, const CornerEdge& e) {
  if (e.isRight)
    cf.right[static_cast<std::size_t>(e.cell)] |= kCrackConsumedBit;
  else
    cf.down[static_cast<std::size_t>(e.cell)] |= kCrackConsumedBit;
}

// Active-crack degree of a corner (consumed bit ignored -- stable during the
// trace).
inline int cornerDegree(const CrackField& cf, int cx, int cy) {
  int deg = 0;
  for (int dir = 0; dir < 4; ++dir) {
    const CornerEdge e = cornerEdge(cf, cx, cy, dir);
    if (e.valid && (crackByte(cf, e) & kCrackClassMask)) ++deg;
  }
  return deg;
}

// Owner PIXEL index of a crack (the side flagged by the owner bit).
inline int crackOwnerPixel(const CrackField& cf, const CornerEdge& e,
                           std::uint8_t byte) {
  const int W = cf.W;
  const int x = e.cell % W, y = e.cell / W;
  const bool second = (byte & kCrackOwnerBit) != 0;
  if (e.isRight) return y * W + (second ? x + 1 : x);  // (x,y) vs (x+1,y)
  return (second ? y + 1 : y) * W + x;                 // (x,y) vs (x,y+1)
}

// Walk one chain from `cx,cy` through starting edge `e0`, consuming cracks and
// collecting vertices/edgel attributes. Stops when the far corner is a
// TERMINAL (degree != 2) or, for `loopSeed` >= 0, when the far corner id
// equals loopSeed (closed loop). Returns the finished chain.
ScreenChain walkChain(CrackField& cf, int cx, int cy, CornerEdge e0,
                      long loopSeed, const float* viewZ,
                      const std::uint32_t* objectId) {
  ScreenChain ch;
  std::vector<float> edgeVz;  // per-edgel owner view-z (attribution below)
  auto pushCorner = [&](int px, int py) {
    ch.pts.push_back({static_cast<float>(px) - 0.5f,
                      static_cast<float>(py) - 0.5f, 0.0f});
  };
  pushCorner(cx, cy);

  CornerEdge e = e0;
  for (;;) {
    const std::uint8_t byte = crackByte(cf, e);
    markConsumed(cf, e);
    ch.edgeClass.push_back(byte & kCrackClassMask);
    const int owner = crackOwnerPixel(cf, e, byte);
    ch.edgeGroup.push_back(
        objectId ? static_cast<std::uint16_t>(objectId[owner] >> 2) : 0);
    edgeVz.push_back(viewZ ? viewZ[owner] : 0.0f);

    cx = e.farCx;
    cy = e.farCy;
    pushCorner(cx, cy);
    const long cid = static_cast<long>(cy) * (cf.W + 1) + cx;
    if (cid == loopSeed) {
      ch.closed = true;
      break;
    }
    if (loopSeed < 0 && cornerDegree(cf, cx, cy) != 2) break;
    // Continue through the unique other unconsumed crack at this corner.
    CornerEdge next;
    bool found = false;
    for (int dir = 0; dir < 4 && !found; ++dir) {
      const CornerEdge cand = cornerEdge(cf, cx, cy, dir);
      if (!cand.valid) continue;
      const std::uint8_t b = crackByte(cf, cand);
      if ((b & kCrackClassMask) && !(b & kCrackConsumedBit)) {
        next = cand;
        found = true;
      }
    }
    if (!found) break;  // defensive: dead end (all consumed)
    e = next;
  }

  // Vertex view-z = mean of the adjacent edgels' owner view-z; a closed loop's
  // duplicated seed vertex averages the last and first edgel.
  const std::size_t nE = edgeVz.size();
  for (std::size_t vi = 0; vi < ch.pts.size(); ++vi) {
    float vz;
    if (nE == 0) {
      vz = 0.0f;
    } else if (ch.closed && (vi == 0 || vi == ch.pts.size() - 1)) {
      vz = 0.5f * (edgeVz[0] + edgeVz[nE - 1]);
    } else if (vi == 0) {
      vz = edgeVz[0];
    } else if (vi >= nE) {
      vz = edgeVz[nE - 1];
    } else {
      vz = 0.5f * (edgeVz[vi - 1] + edgeVz[vi]);
    }
    ch.pts[vi].vz = vz;
  }
  return ch;
}

}  // namespace

std::vector<ScreenChain> traceCrackChains(CrackField& cf, const float* viewZ,
                                          const std::uint32_t* objectId) {
  std::vector<ScreenChain> chains;
  const int W = cf.W, H = cf.H;
  if (W <= 0 || H <= 0) return chains;

  // Pass 1: open chains, seeded at TERMINAL corners (degree 1, 3 or 4) in
  // row-major corner order, each walking its unconsumed incident cracks in the
  // fixed E, S, W, N order. Junction-to-junction chains are maximal and every
  // crack is emitted exactly once.
  for (int cy = 0; cy <= H; ++cy) {
    for (int cx = 0; cx <= W; ++cx) {
      const int deg = cornerDegree(cf, cx, cy);
      if (deg == 0 || deg == 2) continue;
      for (int dir = 0; dir < 4; ++dir) {
        const CornerEdge e = cornerEdge(cf, cx, cy, dir);
        if (!e.valid) continue;
        const std::uint8_t b = crackByte(cf, e);
        if (!(b & kCrackClassMask) || (b & kCrackConsumedBit)) continue;
        chains.push_back(walkChain(cf, cx, cy, e, -1, viewZ, objectId));
      }
    }
  }

  // Pass 2: closed loops. Every remaining unconsumed crack lies on a pure
  // degree-2 cycle; scan the right plane then the down plane in array order
  // and walk each loop from the crack's first corner back to itself.
  for (int pass = 0; pass < 2; ++pass) {
    const std::vector<std::uint8_t>& plane = pass == 0 ? cf.right : cf.down;
    for (std::size_t i = 0; i < plane.size(); ++i) {
      const std::uint8_t b = plane[i];
      if (!(b & kCrackClassMask) || (b & kCrackConsumedBit)) continue;
      const int x = static_cast<int>(i) % W, y = static_cast<int>(i) / W;
      // Seed corner and the walk direction along this crack: a right crack
      // starts at (x+1,y) walking S; a down crack starts at (x,y+1) walking E.
      const int cx = pass == 0 ? x + 1 : x;
      const int cy = pass == 0 ? y : y + 1;
      const CornerEdge e = cornerEdge(cf, cx, cy, pass == 0 ? 1 : 0);
      const long seed = static_cast<long>(cy) * (W + 1) + cx;
      chains.push_back(walkChain(cf, cx, cy, e, seed, viewZ, objectId));
    }
  }
  return chains;
}

}  // namespace umbreon
