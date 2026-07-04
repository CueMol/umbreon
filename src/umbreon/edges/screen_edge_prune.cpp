// SCREEN-SPACE VECTOR edges, Stage 2.5: hysteresis prune + retrace of the
// traced chains (weak-DepthGap support propagation, run-level tail trim).
// See screen_vector_edges.hpp for the pipeline overview.
#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace umbreon {

bool keepScreenChain(const ScreenChain& ch, int minStrong) {
  int strong = 0;
  for (std::size_t i = 0; i < ch.edgeClass.size(); ++i) {
    if (ch.edgeClass[i] !=
        static_cast<std::uint8_t>(CrackClass::DepthGap))
      return true;
    if (i < ch.edgeFlags.size() && (ch.edgeFlags[i] & 1) &&
        ++strong >= std::max(1, minStrong))
      return true;
  }
  return false;
}

void eraseChainCracks(CrackField& cf, const ScreenChain& ch, std::size_t e0,
                      std::size_t e1) {
  // Consecutive corner pairs map back to lattice cells (see the cornerEdge
  // mapping): a horizontal lattice edge (cx,cy)-(cx+1,cy) is
  // down[(cy-1)*W + cx], a vertical one (cx,cy)-(cx,cy+1) is
  // right[cy*W + (cx-1)]. Chain points store corner - 0.5; edgel i spans
  // pts[i] -> pts[i+1].
  if (ch.pts.size() < 2) return;
  e1 = std::min(e1, ch.pts.size() - 1);
  for (std::size_t i = e0; i < e1; ++i) {
    const int cx0 = static_cast<int>(std::lround(ch.pts[i].x + 0.5f));
    const int cy0 = static_cast<int>(std::lround(ch.pts[i].y + 0.5f));
    const int cx1 = static_cast<int>(std::lround(ch.pts[i + 1].x + 0.5f));
    const int cy1 = static_cast<int>(std::lround(ch.pts[i + 1].y + 0.5f));
    if (cy0 == cy1)
      cf.down[static_cast<std::size_t>(cy0 - 1) * cf.W + std::min(cx0, cx1)] =
          0;
    else
      cf.right[static_cast<std::size_t>(std::min(cy0, cy1)) * cf.W +
               (cx0 - 1)] = 0;
  }
}

void eraseChainCracks(CrackField& cf, const ScreenChain& ch) {
  if (ch.pts.size() < 2) return;
  eraseChainCracks(cf, ch, 0, ch.pts.size() - 1);
}

namespace {

// Lattice corner id of a chain endpoint vertex (pts store corner - 0.5).
inline long cornerIdOf(const ScreenChainVert& v, int W) {
  const long cx = std::lround(v.x + 0.5f);
  const long cy = std::lround(v.y + 0.5f);
  return cy * (W + 1) + cx;
}

}  // namespace

std::vector<ScreenChain> pruneWeakChains(CrackField& cf,
                                         std::vector<ScreenChain> traced,
                                         const float* viewZ,
                                         const std::uint32_t* objectId,
                                         int minStrong,
                                         const float* surfAlpha) {
  // Outer loop: prune, retrace, re-evaluate. Bounded: every round erases at
  // least one chain's cracks for good; the cap only bounds the cost of
  // pathological peeling cascades (leftovers are then kept, not lost).
  for (int round = 0; round < 8; ++round) {
    const std::size_t n = traced.size();
    std::vector<char> kept(n, 0), interior(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
      kept[i] = keepScreenChain(traced[i], minStrong) ? 1 : 0;
      // INTERIOR support = a non-outline feature: strong DepthGap, ObjectId
      // or Crease. A pure-Silhouette chain supports weak neighbors only as
      // an anchor, never on its own: otherwise any weak line whose both ends
      // happen to land on the outline (the grazing rim band running parallel
      // to it) would survive.
      const ScreenChain& ch = traced[i];
      for (std::size_t e = 0; e < ch.edgeClass.size() && !interior[i]; ++e) {
        const std::uint8_t c = ch.edgeClass[e];
        if (c == static_cast<std::uint8_t>(CrackClass::ObjectId) ||
            c == static_cast<std::uint8_t>(CrackClass::Crease) ||
            (c == static_cast<std::uint8_t>(CrackClass::DepthGap) &&
             e < ch.edgeFlags.size() && (ch.edgeFlags[e] & 1)))
          interior[i] = 1;
      }
    }

    // Support propagation to fixpoint: a pure-weak OPEN chain survives when
    // BOTH endpoint corners junction into an already-kept chain AND at least
    // one of those junctions carries INTERIOR support (directly or through a
    // previously rescued weak chain, so a contour's weak tail can extend
    // across several junction-chopped fragments). Junction corners are
    // endpoints of every chain meeting there (the tracer splits at degree
    // != 2), so endpoint-to-endpoint matching is exhaustive.
    std::vector<long> end0(n, -1), end1(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
      if (traced[i].pts.empty() || traced[i].closed) continue;
      end0[i] = cornerIdOf(traced[i].pts.front(), cf.W);
      end1[i] = cornerIdOf(traced[i].pts.back(), cf.W);
    }
    for (;;) {
      std::vector<long> keptEnds, interiorEnds;
      for (std::size_t i = 0; i < n; ++i) {
        if (!kept[i] || end0[i] < 0) continue;
        keptEnds.push_back(end0[i]);
        keptEnds.push_back(end1[i]);
        if (interior[i]) {
          interiorEnds.push_back(end0[i]);
          interiorEnds.push_back(end1[i]);
        }
      }
      std::sort(keptEnds.begin(), keptEnds.end());
      std::sort(interiorEnds.begin(), interiorEnds.end());
      auto touches = [](const std::vector<long>& v, long cid) {
        return std::binary_search(v.begin(), v.end(), cid);
      };
      bool changed = false;
      for (std::size_t i = 0; i < n; ++i) {
        if (kept[i] || end0[i] < 0) continue;
        if (touches(keptEnds, end0[i]) && touches(keptEnds, end1[i]) &&
            (touches(interiorEnds, end0[i]) ||
             touches(interiorEnds, end1[i]))) {
          kept[i] = 1;
          interior[i] = 1;  // a rescued weak fragment extends the contour
          changed = true;
        }
      }
      if (!changed) break;
    }

    bool dropped = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (!kept[i]) {
        eraseChainCracks(cf, traced[i]);
        dropped = true;
      }
    }

    // RUN-LEVEL weak-tail trim on the kept OPEN chains WITHOUT strong self-
    // support. Chain-level support from a NON-DepthGap class must not extend
    // to a weak DepthGap run dangling toward an unsupported endpoint: without
    // a junction at the class transition, a weak sliver FUSED to a supported
    // run (e.g. a stick's cross-section ObjectId border continuing straight
    // into the mesh strand's same-id grazing-fade line) would ride the
    // whole-chain keep, while the SAME cracks are pruned as a free-end spur
    // when a junction separates them. Mirror the pure-weak-chain rule at run
    // granularity: a leading/trailing weak run survives only when its outer
    // endpoint corner junctions into another kept chain (its interior side
    // is bracket-supported by the adjacent non-weak run by construction).
    //
    // A chain carrying >= minStrong STRONG DepthGap edgels of its own is
    // EXEMPT: its weak end runs are the genuine tapering tails of a real
    // occlusion contour (classic hysteresis -- strong evidence extends the
    // connected weak cracks), and such contours routinely END FREE mid-
    // surface at a cusp over another surface behind, with no junction to
    // support them. Trimming those dashed the ribbon fold contours of
    // edge_ribbon2/stick3. The transp1 leak this trim exists for has ZERO
    // strong edgels (it rode an ObjectId run's keep), so it stays trimmed.
    //
    // Interior weak runs are always bracketed; closed chains are cyclically
    // bracketed; a whole-weak open chain kept here passed the both-ends
    // rescue above, so both its ends are supported.
    std::vector<long> keptEnds;
    for (std::size_t i = 0; i < n; ++i) {
      if (!kept[i] || end0[i] < 0) continue;
      keptEnds.push_back(end0[i]);
      keptEnds.push_back(end1[i]);
    }
    std::sort(keptEnds.begin(), keptEnds.end());
    // Endpoint corner `cid` of chain i is supported iff some OTHER kept
    // chain also ends there (subtract chain i's own contributions).
    auto supportedEnd = [&](std::size_t i, long cid) {
      const auto pr = std::equal_range(keptEnds.begin(), keptEnds.end(), cid);
      const int own = (end0[i] == cid ? 1 : 0) + (end1[i] == cid ? 1 : 0);
      return static_cast<int>(pr.second - pr.first) - own > 0;
    };
    for (std::size_t i = 0; i < n; ++i) {
      if (!kept[i] || traced[i].closed || end0[i] < 0) continue;
      const ScreenChain& ch = traced[i];
      const std::size_t nE = ch.edgeClass.size();
      // Strong self-support exemption: a chain that would be kept on its own
      // strong DepthGap evidence keeps its weak end runs too.
      {
        int strong = 0;
        bool exempt = false;
        for (std::size_t e = 0; e < ch.edgeFlags.size(); ++e) {
          if (ch.edgeClass[e] ==
                  static_cast<std::uint8_t>(CrackClass::DepthGap) &&
              (ch.edgeFlags[e] & 1) && ++strong >= std::max(1, minStrong)) {
            exempt = true;
            break;
          }
        }
        if (exempt) continue;
      }
      auto weakAt = [&](std::size_t e) {
        return ch.edgeClass[e] ==
                   static_cast<std::uint8_t>(CrackClass::DepthGap) &&
               !(e < ch.edgeFlags.size() && (ch.edgeFlags[e] & 1));
      };
      std::size_t lead = 0;
      while (lead < nE && weakAt(lead)) ++lead;
      if (lead == nE) continue;  // whole-weak: rescued, both ends supported
      if (lead > 0 && !supportedEnd(i, end0[i])) {
        eraseChainCracks(cf, ch, 0, lead);
        dropped = true;
      }
      std::size_t tail = nE;
      while (tail > 0 && weakAt(tail - 1)) --tail;
      if (tail < nE && !supportedEnd(i, end1[i])) {
        eraseChainCracks(cf, ch, tail, nE);
        dropped = true;
      }
    }
    if (!dropped) break;
    for (std::vector<std::uint8_t>* plane : {&cf.right, &cf.down})
      for (std::uint8_t& b : *plane)
        b &= static_cast<std::uint8_t>(~kCrackConsumedBit);
    traced = traceCrackChains(cf, viewZ, objectId, surfAlpha);
  }
  return traced;
}

}  // namespace umbreon
