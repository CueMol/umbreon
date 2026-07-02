#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <cstdlib>
#include <cstdint>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "edges/stroke_render.hpp"

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

// True when any pixel within Chebyshev distance `r` of (x,y) is background.
// Used to suppress DepthGap cracks hugging the outline: a grazing surface
// (tube rim, near-edge-on facet) piles huge depth slopes into the last few
// pixels before the silhouette, indistinguishable there from an occlusion
// step -- and the silhouette class already inks that boundary.
inline bool nearBackground(const std::uint32_t* objectId, int W, int H, int x,
                           int y, int r) {
  const int x0 = std::max(0, x - r), x1 = std::min(W - 1, x + r);
  const int y0 = std::max(0, y - r), y1 = std::min(H - 1, y + r);
  for (int yy = y0; yy <= y1; ++yy)
    for (int xx = x0; xx <= x1; ++xx)
      if (objectId[yy * W + xx] == kBackground) return true;
  return false;
}

// Classify ONE crack between pixel indices ia (first: left/top) and ib
// (second: right/bottom). iOutA / iOutB are the outer straight-line neighbors
// (a's far side, b's far side) with validity flags. Returns the packed crack
// byte (0 = no edge).
inline std::uint8_t classifyPair(const float* viewZ,
                                 const std::uint32_t* objectId,
                                 const float* normal, int W, int H, int ia,
                                 int ib, int iOutA,
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

  // 2. Object boundary: both foreground, ids differ.
  if (objectId[ia] != objectId[ib]) {
    // Same CueMol section (group), different primitive kind (a sphere,
    // cylinder and mesh mixed in one section): the section renders as a
    // SINGLE seamless object -- never ink an internal boundary between its
    // primitives, and do NOT fall through to the depth-gap / crease tests
    // either (a bond embedded in an atom is continuous by construction).
    // objectId == (group << 2) | kind, so equal high bits mean same section.
    if ((objectId[ia] >> 2) == (objectId[ib] >> 2)) return 0;
    if (!p.objectBoundary) return 0;
    const std::uint8_t owner = viewZ[ia] <= viewZ[ib] ? 0 : kCrackOwnerBit;
    return static_cast<std::uint8_t>(CrackClass::ObjectId) | owner;
  }

  // 3. DepthGap: same id, both one-sided planar extrapolations miss the far
  // pixel (slope-adaptive second-derivative test; see header), AND the raw
  // |dvz| is the LOCAL MAX among the three parallel pixel pairs (non-maximum
  // suppression perpendicular to the crack). Without the NMS a grazing rim --
  // where the surface depth rises tangentially over a 1-2 px annulus (e.g. a
  // tube edge) -- fires a dense band of cracks that T-junctions the silhouette
  // into confetti; with it only the strongest pair on the profile fires, so
  // the boundary is one crack thin. A parallel pair that is itself a fg/bg
  // boundary counts as infinitely strong (the silhouette class owns the
  // profile there), which kills the rim annulus next to the outline.
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
      const float g0 = std::fabs(vzB - vzA);
      // Parallel-pair strength on a's far side (pair outA-a) and b's far side
      // (pair b-outB): bg neighbor => that pair is a silhouette boundary =>
      // +inf (suppress); off-image => no pair => 0. STRICT on the a side to
      // break plateau ties (a near-edge-on facet gives a run of equal-gap
      // pairs; >= on both sides would fire the whole run as a band).
      const float inf = std::numeric_limits<float>::infinity();
      float gLeft = 0.0f, gRight = 0.0f;
      if (outAValid)
        gLeft = objectId[iOutA] == kBackground ? inf
                                               : std::fabs(vzA - viewZ[iOutA]);
      if (outBValid)
        gRight = objectId[iOutB] == kBackground ? inf
                                                : std::fabs(viewZ[iOutB] - vzB);
      if (g0 > gLeft && g0 >= gRight) {
        // Silhouette-clearance kill: within bgClearancePx of the outline the
        // depth signal is grazing-dominated and the silhouette class already
        // inks the boundary.
        const int ax = ia % W, ay = ia / W;
        const int bx = ib % W, by = ib / W;
        if (p.bgClearancePx <= 0 ||
            (!nearBackground(objectId, W, H, ax, ay, p.bgClearancePx) &&
             !nearBackground(objectId, W, H, bx, by, p.bgClearancePx))) {
          const std::uint8_t owner = vzA <= vzB ? 0 : kCrackOwnerBit;
          return static_cast<std::uint8_t>(CrackClass::DepthGap) | owner;
        }
      }
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
                  viewZ, objectId, normal, W, H, ia, ia + 1, ia - 1,
                  x - 1 >= 0, ia + 2, x + 2 < W, sp, cosCreaseBase, params);
            }
            if (y + 1 < H) {  // down crack (x,y)-(x,y+1)
              cf.down[static_cast<std::size_t>(ia)] = classifyPair(
                  viewZ, objectId, normal, W, H, ia, ia + W, ia - W,
                  y - 1 >= 0, ia + 2 * W, y + 2 < H, sp, cosCreaseBase,
                  params);
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
        ScreenChain ch = walkChain(cf, cx, cy, e, -1, viewZ, objectId);
        ch.deg0 = deg;
        const int lx = static_cast<int>(ch.pts.back().x + 0.5f);
        const int ly = static_cast<int>(ch.pts.back().y + 0.5f);
        ch.deg1 = cornerDegree(cf, lx, ly);
        chains.push_back(std::move(ch));
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
      ScreenChain ch = walkChain(cf, cx, cy, e, seed, viewZ, objectId);
      ch.deg0 = ch.deg1 = 2;
      chains.push_back(std::move(ch));
    }
  }
  return chains;
}

// ---------------------------------------------------------------------------
// Stage 3: geometry cleanup.

float polylineLength2d(const std::vector<ScreenChainVert>& pts) {
  float len = 0.0f;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const float dx = pts[i].x - pts[i - 1].x;
    const float dy = pts[i].y - pts[i - 1].y;
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}

void collapseCollinear(std::vector<ScreenChainVert>& pts, bool /*closed*/) {
  if (pts.size() < 3) return;
  std::vector<ScreenChainVert> out;
  out.reserve(pts.size());
  out.push_back(pts.front());
  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    const ScreenChainVert& a = out.back();
    const ScreenChainVert& b = pts[i];
    const ScreenChainVert& c = pts[i + 1];
    const float abx = b.x - a.x, aby = b.y - a.y;
    const float bcx = c.x - b.x, bcy = c.y - b.y;
    const float cross = abx * bcy - aby * bcx;
    const float dot = abx * bcx + aby * bcy;
    if (cross == 0.0f && dot > 0.0f) continue;  // same direction: drop b
    out.push_back(b);
  }
  out.push_back(pts.back());
  pts.swap(out);
}

namespace {

inline ScreenChainVert lerpVert(const ScreenChainVert& a,
                                const ScreenChainVert& b, float t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
          a.vz + (b.vz - a.vz) * t};
}

}  // namespace

void chaikinSmooth(std::vector<ScreenChainVert>& pts, bool closed, int iters) {
  for (int it = 0; it < iters; ++it) {
    const std::size_t n = pts.size();
    if (n < 3) return;
    std::vector<ScreenChainVert> out;
    if (closed) {
      // Cut every cyclic segment of the n-1 unique vertices, then
      // re-duplicate the new seam.
      const std::size_t m = n - 1;  // unique vertices (front()==back())
      out.reserve(2 * m + 1);
      for (std::size_t i = 0; i < m; ++i) {
        const ScreenChainVert& a = pts[i];
        const ScreenChainVert& b = pts[(i + 1) % m];
        out.push_back(lerpVert(a, b, 0.25f));
        out.push_back(lerpVert(a, b, 0.75f));
      }
      out.push_back(out.front());
    } else {
      // Endpoints pinned (junction continuity across separate chains).
      out.reserve(2 * n);
      out.push_back(pts.front());
      for (std::size_t i = 0; i + 1 < n; ++i) {
        const ScreenChainVert& a = pts[i];
        const ScreenChainVert& b = pts[i + 1];
        out.push_back(lerpVert(a, b, 0.25f));
        out.push_back(lerpVert(a, b, 0.75f));
      }
      out.push_back(pts.back());
    }
    pts.swap(out);
  }
}

namespace {

// Iterative Douglas-Peucker on the OPEN range [first, last] of pts; marks kept
// vertices. Perpendicular distance to the chord (degenerate chord: distance to
// the endpoint).
void rdpMark(const std::vector<ScreenChainVert>& pts, std::size_t first,
             std::size_t last, float eps, std::vector<char>& keep) {
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  stack.push_back({first, last});
  while (!stack.empty()) {
    const auto [i0, i1] = stack.back();
    stack.pop_back();
    if (i1 <= i0 + 1) continue;
    const float ax = pts[i0].x, ay = pts[i0].y;
    const float bx = pts[i1].x, by = pts[i1].y;
    const float dx = bx - ax, dy = by - ay;
    const float len2 = dx * dx + dy * dy;
    float dMax = -1.0f;
    std::size_t iMax = i0;
    for (std::size_t i = i0 + 1; i < i1; ++i) {
      float d;
      if (len2 <= 0.0f) {
        const float ex = pts[i].x - ax, ey = pts[i].y - ay;
        d = std::sqrt(ex * ex + ey * ey);
      } else {
        d = std::fabs(dy * pts[i].x - dx * pts[i].y + bx * ay - by * ax) /
            std::sqrt(len2);
      }
      if (d > dMax) {
        dMax = d;
        iMax = i;
      }
    }
    if (dMax > eps) {
      keep[iMax] = 1;
      stack.push_back({i0, iMax});
      stack.push_back({iMax, i1});
    }
  }
}

}  // namespace

void simplifyRdp(std::vector<ScreenChainVert>& pts, bool closed, float eps) {
  if (eps <= 0.0f || pts.size() < 3) return;
  if (!closed) {
    std::vector<char> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    rdpMark(pts, 0, pts.size() - 1, eps, keep);
    std::vector<ScreenChainVert> out;
    for (std::size_t i = 0; i < pts.size(); ++i)
      if (keep[i]) out.push_back(pts[i]);
    pts.swap(out);
    return;
  }
  // Closed: rotate so the polyline starts at vertex A of an approximate-
  // diameter pair (two farthest sweeps from vertex 0 -- deterministic), split
  // at B, simplify both halves, rejoin with the seam duplicated.
  const std::size_t m = pts.size() - 1;  // unique vertices
  if (m < 3) return;
  auto farthestFrom = [&](std::size_t s) {
    std::size_t best = s;
    float bd = -1.0f;
    for (std::size_t i = 0; i < m; ++i) {
      const float dx = pts[i].x - pts[s].x, dy = pts[i].y - pts[s].y;
      const float d = dx * dx + dy * dy;
      if (d > bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  };
  const std::size_t a = farthestFrom(0);
  const std::size_t b = farthestFrom(a);
  const std::size_t lo = std::min(a, b), hi = std::max(a, b);
  if (lo == hi) return;
  // Rotate the unique ring so it starts at lo; B lands at index (hi - lo).
  std::vector<ScreenChainVert> ring;
  ring.reserve(m + 1);
  for (std::size_t i = 0; i < m; ++i) ring.push_back(pts[(lo + i) % m]);
  ring.push_back(ring.front());  // duplicated seam at the rotated start
  const std::size_t split = hi - lo;
  std::vector<char> keep(ring.size(), 0);
  keep.front() = keep.back() = 1;
  keep[split] = 1;
  rdpMark(ring, 0, split, eps, keep);
  rdpMark(ring, split, ring.size() - 1, eps, keep);
  std::vector<ScreenChainVert> out;
  for (std::size_t i = 0; i < ring.size(); ++i)
    if (keep[i]) out.push_back(ring[i]);
  pts.swap(out);
}

// ---------------------------------------------------------------------------
// Stage 4: class runs -> style slots -> shared draw stage.

namespace {

// Style slot (EdgeStyle::cls[] index) and paint precedence for a crack class.
// The screen analogue of the mesh path's natureStyleSlot/naturePrecedence:
// Silhouette and DepthGap paint on top (precedence 2), the object-id boundary
// like the mesh Border (1), Crease underneath (0). DepthGap draws with the
// Disconnected slot -- unreachable from the mesh path -- and applyScreen
// VectorEdges falls back to the Silhouette slot when a section leaves
// Disconnected unconfigured.
inline int classStyleSlot(CrackClass c) {
  switch (c) {
    case CrackClass::ObjectId:
      return static_cast<int>(EdgeClass::Object);
    case CrackClass::DepthGap:
      return static_cast<int>(EdgeClass::Disconnected);
    case CrackClass::Crease:
      return static_cast<int>(EdgeClass::Crease);
    default:
      return static_cast<int>(EdgeClass::Silhouette);
  }
}

inline int classPrecedence(CrackClass c) {
  switch (c) {
    case CrackClass::Crease:
      return 0;
    case CrackClass::ObjectId:
      return 1;
    default:
      return 2;  // Silhouette, DepthGap
  }
}

// Relabel a class run shorter than minLen edgels when bracketed by two runs
// of one same class (style-flicker suppression along a boundary whose
// classification alternates, e.g. silhouette <-> depth gap where an object
// edge grazes the background). Geometry is untouched -- only the labels move,
// so the continuity guarantee is intact. Linear scan; a closed chain's
// seam-straddling run pair is left as-is (harmless: one extra style split).
void mergeShortClassRuns(std::vector<std::uint8_t>& cls, int minLen) {
  if (minLen <= 1 || cls.size() < 3) return;
  std::size_t i = 0;
  while (i < cls.size()) {
    std::size_t j = i;
    while (j < cls.size() && cls[j] == cls[i]) ++j;
    const std::size_t runLen = j - i;
    if (i > 0 && j < cls.size() && runLen < static_cast<std::size_t>(minLen) &&
        cls[i - 1] == cls[j]) {
      for (std::size_t k = i; k < j; ++k) cls[k] = cls[i - 1];
      // Re-scan from the previous run start would be quadratic; extending the
      // left run and continuing forward keeps the pass linear and the result
      // deterministic.
    }
    i = j;
  }
}

}  // namespace

void applyScreenVectorEdges(FrameResult& frame, const Scene& scene,
                            const RenderOptions& opt) {
  const StrokeEdgeOptions& se = opt.strokeEdges;
  const int W = frame.width, H = frame.height;
  if (W <= 0 || H <= 0) return;
  if (frame.viewZ.empty() || frame.objectId.empty()) return;

  const ScreenProj sp = makeScreenProj(scene.camera, W, H);
  const float ssScale = static_cast<float>(std::max(1, opt.supersample));

  // Stage 1: classify. The nature master toggles gate the classes here (the
  // shared draw stage applies only the per-section style table).
  ScreenClassifyParams cp;
  cp.silhouette = se.silhouette;
  cp.objectBoundary = se.border;
  cp.crease = se.crease;
  cp.depthGapPx = se.screenDepthGapPx;
  cp.slopeClampPx = se.screenSlopeClampPx;
  cp.creaseAngleDeg = se.creaseAngleDeg;
  cp.grazeK = se.screenGrazeK;
  cp.bgClearancePx = static_cast<int>(std::lround(ssScale));
  const float* normalPtr = frame.normal.empty() ? nullptr : frame.normal.data();
  if (cp.crease && !normalPtr) cp.crease = false;
  CrackField cf = classifyCracks(W, H, frame.viewZ.data(),
                                 frame.objectId.data(), normalPtr, sp, cp);

  // Stage 2: trace.
  const std::vector<ScreenChain> traced =
      traceCrackChains(cf, frame.viewZ.data(), frame.objectId.data());

  // Stage 3+4 per chain: speck filter (whole chain), class-run relabel +
  // split, per-run geometry cleanup, slot mapping.
  const float minChainLen = se.screenMinLenPx * ssScale;
  const int mergeLen = std::max(
      0, static_cast<int>(std::lround(se.screenClassMergeLen * ssScale)));
  const float rdpEps = se.screenSimplifyPx * ssScale;
  const bool perSection = !scene.groupEdgeStyle.empty();

  std::vector<StrokeChainInput> drawChains;
  for (const ScreenChain& ch : traced) {
    if (ch.pts.size() < 2 || ch.edgeClass.empty()) continue;
    // Speck filter on the RAW chain: every edgel is one hi-res px long, so the
    // edgel count IS the arc length. JUNCTION-AWARE: a short chain whose ends
    // are both junctions (degree >= 3) is a piece of a larger boundary chopped
    // by side-branches (e.g. grazing-rim depth-gap spurs T-ing into the
    // silhouette) and is KEPT -- dropping it would dash the outline. Only a
    // short chain with a free end (a spur) or a tiny closed loop is an
    // isolated speckle and is dropped.
    if (minChainLen > 0.0f &&
        static_cast<float>(ch.edgeClass.size()) < minChainLen &&
        !(ch.deg0 >= 3 && ch.deg1 >= 3))
      continue;

    // Class-run relabel (labels only), then split into maximal same-class
    // runs. Adjacent runs share their boundary vertex, so the drawn geometry
    // stays continuous across a style change.
    std::vector<std::uint8_t> cls = ch.edgeClass;
    mergeShortClassRuns(cls, mergeLen);

    std::size_t e0 = 0;
    while (e0 < cls.size()) {
      std::size_t e1 = e0;
      while (e1 < cls.size() && cls[e1] == cls[e0]) ++e1;
      const CrackClass runClass = static_cast<CrackClass>(cls[e0]);
      // A run spanning the whole closed loop keeps the cyclic treatment.
      const bool runClosed = ch.closed && e0 == 0 && e1 == cls.size();

      StrokeChainInput in;
      in.group = ch.edgeGroup[e0];
      in.precedence = classPrecedence(runClass);
      in.styleSlot = classStyleSlot(runClass);
      // DepthGap falls back to the Silhouette slot when the section never
      // configured the Disconnected class (the default style table ships all
      // slots disabled except those the CLI enables; without the fallback a
      // same-id occlusion boundary would silently vanish).
      if (perSection &&
          runClass == CrackClass::DepthGap) {
        float h, c[3], o;
        if (!resolveStrokeStyle(scene, se, ssScale, in.styleSlot, in.group, h,
                                c, o))
          in.styleSlot = static_cast<int>(EdgeClass::Silhouette);
      }

      // Geometry cleanup on the run's vertex slice [e0, e1].
      std::vector<ScreenChainVert> pts(ch.pts.begin() + e0,
                                       ch.pts.begin() + e1 + 1);
      collapseCollinear(pts, runClosed);
      chaikinSmooth(pts, runClosed, se.screenSmoothIters);
      simplifyRdp(pts, runClosed, rdpEps);
      if (pts.size() < 2) {
        e0 = e1;
        continue;
      }
      in.pts.reserve(pts.size());
      for (const ScreenChainVert& v : pts)
        in.pts.push_back({v.x, v.y, v.vz, true});
      drawChains.push_back(std::move(in));
      e0 = e1;
    }
  }

  // Tuning aid (env-gated, zero-cost when unset): one stats line per frame.
  if (std::getenv("UMBREON_SCREEN_EDGE_DEBUG")) {
    std::size_t nEdgels = 0, nPts = 0;
    int clsCount[5] = {0, 0, 0, 0, 0};
    for (const ScreenChain& ch : traced) {
      nEdgels += ch.edgeClass.size();
      for (std::uint8_t c : ch.edgeClass)
        if (c < 5) ++clsCount[c];
    }
    for (const StrokeChainInput& in : drawChains) nPts += in.pts.size();
    std::fprintf(stderr,
                 "[screen-edges] traced=%zu edgels=%zu (sil=%d obj=%d gap=%d "
                 "crease=%d) drawn=%zu pts=%zu\n",
                 traced.size(), nEdgels, clsCount[1], clsCount[2], clsCount[3],
                 clsCount[4], drawChains.size(), nPts);
  }

  renderStrokeChains(frame, scene, opt, drawChains);
}

}  // namespace umbreon
