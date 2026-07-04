// SCREEN-SPACE VECTOR edges, Stage 3: chain geometry cleanup (collinear
// collapse, Chaikin corner cutting, Douglas-Peucker simplification).
// See screen_vector_edges.hpp for the pipeline overview.
#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace umbreon {
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
          a.vz + (b.vz - a.vz) * t, a.alpha + (b.alpha - a.alpha) * t};
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

}  // namespace umbreon
