// SCREEN-SPACE VECTOR edges, Stage 2: deterministic lattice tracing of the
// classified cracks into maximal continuous chains.
// See screen_vector_edges.hpp for the pipeline overview.
#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace umbreon {

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
                      const std::uint32_t* objectId,
                      const float* surfAlpha) {
  ScreenChain ch;
  std::vector<float> edgeVz;  // per-edgel owner view-z (attribution below)
  std::vector<float> edgeA;   // per-edgel owner surface alpha
  auto pushCorner = [&](int px, int py) {
    ch.pts.push_back({static_cast<float>(px) - 0.5f,
                      static_cast<float>(py) - 0.5f, 0.0f, 1.0f});
  };
  pushCorner(cx, cy);

  CornerEdge e = e0;
  for (;;) {
    const std::uint8_t byte = crackByte(cf, e);
    markConsumed(cf, e);
    ch.edgeClass.push_back(byte & kCrackClassMask);
    ch.edgeFlags.push_back((byte & kCrackStrongBit) ? 1 : 0);
    const int owner = crackOwnerPixel(cf, e, byte);
    ch.edgeGroup.push_back(
        objectId ? static_cast<std::uint16_t>(objectId[owner] >> 2) : 0);
    edgeVz.push_back(viewZ ? viewZ[owner] : 0.0f);
    edgeA.push_back(surfAlpha ? surfAlpha[owner] : 1.0f);

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

  ch.edgeAlpha = edgeA;

  // Vertex view-z / surface alpha = mean of the adjacent edgels' owner
  // values; a closed loop's duplicated seed vertex averages the last and
  // first edgel. (Vertex alpha is chain-level only -- the Stage-4 driver
  // re-attributes it per class run from edgeAlpha, see the header.)
  const std::size_t nE = edgeVz.size();
  for (std::size_t vi = 0; vi < ch.pts.size(); ++vi) {
    float vz, a;
    if (nE == 0) {
      vz = 0.0f;
      a = 1.0f;
    } else if (ch.closed && (vi == 0 || vi == ch.pts.size() - 1)) {
      vz = 0.5f * (edgeVz[0] + edgeVz[nE - 1]);
      a = 0.5f * (edgeA[0] + edgeA[nE - 1]);
    } else if (vi == 0) {
      vz = edgeVz[0];
      a = edgeA[0];
    } else if (vi >= nE) {
      vz = edgeVz[nE - 1];
      a = edgeA[nE - 1];
    } else {
      vz = 0.5f * (edgeVz[vi - 1] + edgeVz[vi]);
      a = 0.5f * (edgeA[vi - 1] + edgeA[vi]);
    }
    ch.pts[vi].vz = vz;
    ch.pts[vi].alpha = a;
  }
  return ch;
}

}  // namespace

std::vector<ScreenChain> traceCrackChains(CrackField& cf, const float* viewZ,
                                          const std::uint32_t* objectId,
                                          const float* surfAlpha) {
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
        ScreenChain ch = walkChain(cf, cx, cy, e, -1, viewZ, objectId,
                                   surfAlpha);
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
      ScreenChain ch = walkChain(cf, cx, cy, e, seed, viewZ, objectId,
                                 surfAlpha);
      ch.deg0 = ch.deg1 = 2;
      chains.push_back(std::move(ch));
    }
  }
  return chains;
}

}  // namespace umbreon
