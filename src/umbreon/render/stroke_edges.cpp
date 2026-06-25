#include "render/stroke_edges.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "render/mesh_feature_edges.hpp"

namespace umbreon {
namespace {

// Minimal 2D screen-space point (pixel coords); local to the ribbon rasterizer.
struct Vec2 {
  float x = 0.0f, y = 0.0f;
};
inline Vec2 operator-(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator+(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator*(const Vec2& a, float s) { return {a.x * s, a.y * s}; }

// Strip-creation constants ported from Freestyle StrokeRep.cpp:87-90.
constexpr float kZero = 1.0e-5f;             // ZERO
constexpr float kMaxRatioLengthSingu = 2.0f; // MAX_RATIO_LENGTH_SINGU
constexpr float kEpsSingularity = 0.05f;     // EPS_SINGULARITY_RENDERER
constexpr float kHugeCoord = 1.0e4f;         // HUGE_COORD

inline float dot2(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline float norm2(const Vec2& a) { return std::sqrt(dot2(a, a)); }

inline bool notValid(const Vec2& p) {
  return (p.x != p.x) || (p.y != p.y) || (std::fabs(p.x) > kHugeCoord) ||
         (std::fabs(p.y) > kHugeCoord);
}

// 2D line-line intersection, ported from GeomUtils::intersect2dLine2dLine. Lines
// through (p1,p2) and (p3,p4); returns true with `res` set iff not (near-)
// parallel.
bool intersect2dLine2dLine(const Vec2& p1, const Vec2& p2, const Vec2& p3,
                           const Vec2& p4, Vec2& res) {
  const float a1 = p2.y - p1.y;
  const float b1 = p1.x - p2.x;
  const float c1 = p2.x * p1.y - p1.x * p2.y;
  const float a2 = p4.y - p3.y;
  const float b2 = p3.x - p4.x;
  const float c2 = p4.x * p3.y - p3.x * p4.y;
  const float denom = a1 * b2 - a2 * b1;
  if (std::fabs(denom) < 1.0e-6f) return false;  // COLINEAR / parallel
  res.x = (b1 * c2 - b2 * c1) / denom;
  res.y = (a2 * c1 - a1 * c2) / denom;
  return true;
}

// Linear "over" composite of a solid edge color onto an RGB pixel (alpha
// untouched).
inline void compositeOver(float* rgba, const float color[3], float a) {
  if (a <= 0.0f) return;
  a = std::min(1.0f, a);
  const float ia = 1.0f - a;
  rgba[0] = rgba[0] * ia + color[0] * a;
  rgba[1] = rgba[1] * ia + color[1] * a;
  rgba[2] = rgba[2] * ia + color[2] * a;
}

// nextSeg(node, cur): the UNIQUE incident, unconsumed, NOT-cur segment at `node`
// (ChainingIterators.cpp:161-178). Returns its index iff exactly one such
// segment exists; -1 at a branch (>1 candidates) or dead-end (0).
int nextSeg(const std::vector<std::vector<int>>& incident,
            const std::vector<char>& consumed, int node, int cur) {
  int cand = -1, cnt = 0;
  for (int si : incident[static_cast<std::size_t>(node)]) {
    if (si == cur || consumed[static_cast<std::size_t>(si)]) continue;
    ++cnt;
    cand = si;
  }
  return cnt == 1 ? cand : -1;
}

// One projected, resampled backbone vertex of a chain: 2D pixel position, linear
// view-z (carried for future depth use) and a visibility flag.
struct Pt2 {
  Vec2 p;
  float vz = 0.0f;
  bool visible = true;
};

// Project a chain backbone to 2D, dropping vertices that fail to project. Keeps
// per-vertex visibility. A perspective vertex at/behind the eye is dropped (it
// also breaks the polyline run, like the prior 1px path).
std::vector<Pt2> projectChain(const ScreenProj& sp, const std::vector<Vec3>& pts,
                              const std::vector<char>& visible) {
  std::vector<Pt2> out;
  out.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    float x, y, vz;
    if (!worldToScreen(sp, pts[i], x, y, vz)) continue;  // unprojectable
    Pt2 q;
    q.p = {x, y};
    q.vz = vz;
    q.visible = visible.empty() || visible[i] != 0;
    out.push_back(q);
  }
  return out;
}

// Resample a projected polyline by 2D arc length every `stepPx` pixels, linearly
// interpolating position/view-z/visibility. A segment whose endpoints differ in
// visibility is SPLIT at the midpoint so the boundary between a visible and a
// hidden run is exact (Freestyle Stroke::Resample + the visibility split). Each
// emitted vertex inherits the visibility of the sub-span it starts. Mirrors
// Stroke.cpp:636-691 (uniform arc-length stepping) but visibility-aware.
std::vector<Pt2> resampleChain(const std::vector<Pt2>& in, float stepPx) {
  if (in.size() < 2 || stepPx <= 0.0f) return in;
  std::vector<Pt2> out;
  out.reserve(in.size() * 2);
  out.push_back(in.front());
  for (std::size_t i = 1; i < in.size(); ++i) {
    const Pt2& a = in[i - 1];
    const Pt2& b = in[i];
    const Vec2 d = b.p - a.p;
    const float len = norm2(d);
    // The sub-span carries the visibility of its START vertex; a change in
    // visibility is realized exactly at the next original vertex (b), which is
    // emitted with b.visible, so a visible run ends and a hidden one starts on a
    // true backbone vertex.
    if (len <= kZero) {
      out.push_back(b);
      continue;
    }
    const int n = static_cast<int>(std::floor(len / stepPx));
    for (int k = 1; k <= n; ++k) {
      const float t = (stepPx * static_cast<float>(k)) / len;
      if (t >= 1.0f) break;
      Pt2 q;
      q.p = a.p + d * t;
      q.vz = a.vz + (b.vz - a.vz) * t;
      q.visible = a.visible;  // interior of the span keeps the start visibility
      out.push_back(q);
    }
    out.push_back(b);
  }
  return out;
}

// A ribbon strip: a flat list of offset border vertices in pairs (left,right)
// per backbone vertex (2*N entries), consumed as a triangle strip.
using Strip = std::vector<Vec2>;

// A strip plus the resolved (per-section) style to ink it with: linear RGB color
// and opacity. The strip geometry already baked in the per-section half-width, so
// only color/opacity travel here.
struct StyledStrip {
  Strip strip;
  float color[3] = {0.0f, 0.0f, 0.0f};
  float opacity = 1.0f;
  int precedence = 0;  // composite order key (lower paints first)
};

// Map an EdgeNature onto the EdgeStyle::cls[] styling slot it draws with. The
// screen-space EdgeClass enum is reused as the style table key (Scene::
// groupEdgeStyle): Silhouette -> Silhouette, Crease -> Crease, Border -> the
// Object slot (the closest object/border boundary class). See the §5 plan.
inline int natureStyleSlot(EdgeNature nat) {
  switch (nat) {
    case EdgeNature::Silhouette:
      return static_cast<int>(EdgeClass::Silhouette);
    case EdgeNature::Border:
      return static_cast<int>(EdgeClass::Object);
    case EdgeNature::Crease:
      return static_cast<int>(EdgeClass::Crease);
  }
  return static_cast<int>(EdgeClass::Silhouette);
}

// Composite precedence ORDER for the natures: Crease < Border < Silhouette
// (most-specific structural edge wins). Strips are rasterized in this order so a
// later (higher-precedence) nature paints over an earlier one where they overlap.
inline int naturePrecedence(EdgeNature nat) {
  switch (nat) {
    case EdgeNature::Crease:
      return 0;
    case EdgeNature::Border:
      return 1;
    case EdgeNature::Silhouette:
      return 2;
  }
  return 2;
}

// Build a miter-joined ribbon strip for a backbone polyline `bb` (>= 2 points)
// with constant half-thickness `halfThick`. Ports Freestyle Strip::createStrip
// (StrokeRep.cpp:105-293): per vertex emit p +/- halfThick*n with n the segment
// normal; interior vertices MITER-join by intersecting the prev/next offset
// lines, SPIKE-CLAMPED to the averaged normal when the miter overruns
// MAX_RATIO_LENGTH_SINGU*halfThick (or the join is near-degenerate). Returns
// 2*bb.size() vertices: [2k]=left(+), [2k+1]=right(-).
Strip buildStrip(const std::vector<Vec2>& bb, float halfThick) {
  Strip out;
  const std::size_t n = bb.size();
  if (n < 2) return out;
  out.resize(2 * n);

  auto orth = [](const Vec2& d) { return Vec2{-d.y, d.x}; };
  auto unit = [](const Vec2& d) {
    const float l = norm2(d);
    return l > kZero ? Vec2{d.x / l, d.y / l} : Vec2{0.0f, 0.0f};
  };

  // First vertex: normal of the first segment.
  {
    const Vec2 dir = unit(bb[1] - bb[0]);
    const Vec2 sd = orth(dir);
    out[0] = bb[0] + sd * halfThick;   // left (+)
    out[1] = bb[0] - sd * halfThick;   // right (-)
  }

  // Interior vertices: miter join.
  for (std::size_t k = 1; k + 1 < n; ++k) {
    const Vec2& p = bb[k];
    const Vec2& pPrev = bb[k - 1];
    const Vec2& pNext = bb[k + 1];
    const Vec2 dirN = pNext - p;        // to next
    const Vec2 dirP = p - pPrev;        // from prev
    const float dirNNorm = norm2(dirN);
    const float dirPNorm = norm2(dirP);
    const Vec2 udirN = unit(dirN);
    const Vec2 udirP = unit(dirP);
    const Vec2 sdN = orth(udirN);       // normal of next segment
    const Vec2 sdP = orth(udirP);       // normal of prev segment

    // Left (+) miter: intersect the two offset lines.
    Vec2 inter;
    if (intersect2dLine2dLine(pPrev + sdP * halfThick, p + sdP * halfThick,
                              p + sdN * halfThick, pNext + sdN * halfThick,
                              inter))
      out[2 * k] = inter;
    else
      out[2 * k] = p + sdN * halfThick;

    // Right (-) miter.
    if (intersect2dLine2dLine(pPrev - sdP * halfThick, p - sdP * halfThick,
                              p - sdN * halfThick, pNext - sdN * halfThick,
                              inter))
      out[2 * k + 1] = inter;
    else
      out[2 * k + 1] = p - sdN * halfThick;

    // Averaged (bevel) normal for the spike clamp.
    Vec2 sdAvg = sdN + sdP;
    const bool degenerate =
        (dirNNorm < kZero) || (dirPNorm < kZero) || (norm2(sdAvg) < kZero);
    if (degenerate)
      sdAvg = {0.0f, 0.0f};
    else
      sdAvg = unit(sdAvg);

    // SPIKE-CLAMP: if the miter overruns MAX_RATIO_LENGTH_SINGU*halfThick from p,
    // or the join is near-degenerate / a near-180 fold, fall back to the averaged
    // normal offset (StrokeRep.cpp:278-292).
    const float spikeLimit = halfThick * kMaxRatioLengthSingu;
    const float foldDot = std::fabs(dot2(sdAvg, udirN));
    auto overruns = [&](const Vec2& v) {
      const Vec2 t = v - p;
      return (norm2(t) > spikeLimit) || degenerate || notValid(v) ||
             (foldDot < kEpsSingularity);
    };
    if (overruns(out[2 * k])) out[2 * k] = p + sdAvg * halfThick;
    if (overruns(out[2 * k + 1])) out[2 * k + 1] = p - sdAvg * halfThick;
  }

  // Last vertex: normal of the last segment.
  {
    const Vec2 dir = unit(bb[n - 1] - bb[n - 2]);
    const Vec2 sd = orth(dir);
    out[2 * (n - 1)] = bb[n - 1] + sd * halfThick;
    out[2 * (n - 1) + 1] = bb[n - 1] - sd * halfThick;
  }
  return out;
}

// Hard-fill one 2D triangle into the framebuffer with a linear over-composite
// (coverage 1 inside). Pixel centers at integer (x,y); a pixel is inside iff its
// center is on the inside of all three edges (top-left-agnostic; the hi-res box
// downsample antialiases). Only rows in [rowBegin,rowEnd) are touched, so callers
// can tile deterministically over screen rows with TBB.
void fillTriangle(std::vector<float>& color, int W, int rowBegin, int rowEnd,
                  const Vec2& a, const Vec2& b, const Vec2& c,
                  const float col[3], float opacity) {
  if (notValid(a) || notValid(b) || notValid(c)) return;
  float minXf = std::min({a.x, b.x, c.x});
  float maxXf = std::max({a.x, b.x, c.x});
  float minYf = std::min({a.y, b.y, c.y});
  float maxYf = std::max({a.y, b.y, c.y});
  int minX = std::max(0, static_cast<int>(std::floor(minXf)));
  int maxX = std::min(W - 1, static_cast<int>(std::ceil(maxXf)));
  int minY = std::max(rowBegin, static_cast<int>(std::floor(minYf)));
  int maxY = std::min(rowEnd - 1, static_cast<int>(std::ceil(maxYf)));
  if (minX > maxX || minY > maxY) return;

  // Signed area; skip degenerate triangles. Orientation handled by abs/compare.
  const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  if (std::fabs(area) < 1.0e-7f) return;
  const float inv = 1.0f / area;

  for (int y = minY; y <= maxY; ++y) {
    const float py = static_cast<float>(y);
    for (int x = minX; x <= maxX; ++x) {
      const float px = static_cast<float>(x);
      // Barycentric coordinates of the pixel center.
      const float w0 =
          ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inv;
      const float w1 =
          ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv;
      const float w2 = 1.0f - w0 - w1;
      if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;  // outside
      const std::size_t idx = (static_cast<std::size_t>(y) * W + x) * 4;
      compositeOver(&color[idx], col, opacity);
    }
  }
}

// Rasterize one ribbon strip (2*N border vertices, pairs per backbone vertex) as
// a triangle strip, restricted to rows [rowBegin,rowEnd). Two triangles per quad
// between consecutive backbone vertices.
void rasterizeStrip(std::vector<float>& color, int W, int rowBegin, int rowEnd,
                    const Strip& strip, const float col[3], float opacity) {
  const std::size_t pairs = strip.size() / 2;
  for (std::size_t k = 0; k + 1 < pairs; ++k) {
    const Vec2& l0 = strip[2 * k];
    const Vec2& r0 = strip[2 * k + 1];
    const Vec2& l1 = strip[2 * (k + 1)];
    const Vec2& r1 = strip[2 * (k + 1) + 1];
    fillTriangle(color, W, rowBegin, rowEnd, l0, r0, l1, col, opacity);
    fillTriangle(color, W, rowBegin, rowEnd, r0, r1, l1, col, opacity);
  }
}

}  // namespace

std::vector<EdgeChain> chainFeatureSegs(const std::vector<FeatureSeg>& segs,
                                        int nodeCount) {
  std::vector<EdgeChain> chains;
  if (nodeCount <= 0 || segs.empty()) return chains;

  // Per-nature node->incident-segment adjacency over chain node ids. A segment
  // is added to the adjacency of its own nature only, so chaining never crosses
  // natures (Freestyle chains same-nature edges). Out-of-range/negative ids skip.
  // 3 natures: Silhouette, Border, Crease.
  constexpr int kNatures = 3;
  std::vector<std::vector<std::vector<int>>> incident(
      kNatures, std::vector<std::vector<int>>(static_cast<std::size_t>(nodeCount)));
  std::vector<char> consumed(segs.size(), 0);
  std::vector<char> chainable(segs.size(), 0);

  auto validNode = [&](int n) { return n >= 0 && n < nodeCount; };
  for (std::size_t i = 0; i < segs.size(); ++i) {
    const FeatureSeg& s = segs[i];
    if (!validNode(s.v0) || !validNode(s.v1)) continue;  // unchainable seg
    chainable[i] = 1;
    const int nat = static_cast<int>(s.nature);
    incident[static_cast<std::size_t>(nat)][static_cast<std::size_t>(s.v0)]
        .push_back(static_cast<int>(i));
    incident[static_cast<std::size_t>(nat)][static_cast<std::size_t>(s.v1)]
        .push_back(static_cast<int>(i));
  }

  // The far endpoint node of segment `si` relative to entering it at `from`.
  auto farNode = [&](int si, int from) {
    const FeatureSeg& s = segs[static_cast<std::size_t>(si)];
    return s.v0 == from ? s.v1 : s.v0;
  };

  for (std::size_t seed = 0; seed < segs.size(); ++seed) {
    if (!chainable[seed] || consumed[seed]) continue;
    const int nat = static_cast<int>(segs[seed].nature);
    const std::vector<std::vector<int>>& inc = incident[static_cast<std::size_t>(nat)];

    std::deque<int> chainSegs;
    consumed[seed] = 1;
    chainSegs.push_back(static_cast<int>(seed));

    // Grow FORWARD from v1.
    int cur = static_cast<int>(seed);
    int node = segs[seed].v1;
    int seedV0 = segs[seed].v0;
    for (int nx = nextSeg(inc, consumed, node, cur); nx >= 0;
         nx = nextSeg(inc, consumed, node, cur)) {
      consumed[static_cast<std::size_t>(nx)] = 1;
      chainSegs.push_back(nx);
      node = farNode(nx, node);
      cur = nx;
      if (node == seedV0) break;  // closed loop: stop before re-walking the seed
    }
    const bool closed = (node == seedV0) && chainSegs.size() > 1;

    // Grow BACKWARD from v0 (only if not already a closed loop).
    if (!closed) {
      cur = static_cast<int>(seed);
      node = segs[seed].v0;
      for (int nx = nextSeg(inc, consumed, node, cur); nx >= 0;
           nx = nextSeg(inc, consumed, node, cur)) {
        consumed[static_cast<std::size_t>(nx)] = 1;
        chainSegs.push_front(nx);
        node = farNode(nx, node);
        cur = nx;
      }
    }

    // Materialize the ordered world-point backbone. Walk the segment list,
    // orienting each segment so its tail meets the previous head.
    EdgeChain ch;
    ch.closed = closed;
    ch.segs.assign(chainSegs.begin(), chainSegs.end());
    int prevNode = -1;
    for (std::size_t k = 0; k < ch.segs.size(); ++k) {
      const FeatureSeg& s = segs[static_cast<std::size_t>(ch.segs[k])];
      // Orient: for the first segment pick the endpoint that connects to the
      // next; thereafter the tail node equals prevNode.
      int tail, head;
      Vec3 ptail, phead;
      if (k == 0) {
        // Choose orientation so the head node is shared with seg[1] (if any).
        if (ch.segs.size() > 1) {
          const FeatureSeg& s1 = segs[static_cast<std::size_t>(ch.segs[1])];
          if (s.v1 == s1.v0 || s.v1 == s1.v1) {
            tail = s.v0; head = s.v1; ptail = s.p0; phead = s.p1;
          } else {
            tail = s.v1; head = s.v0; ptail = s.p1; phead = s.p0;
          }
        } else {
          tail = s.v0; head = s.v1; ptail = s.p0; phead = s.p1;
        }
        ch.pts.push_back(ptail);
        ch.pts.push_back(phead);
        prevNode = head;
      } else {
        if (s.v0 == prevNode) {
          head = s.v1; phead = s.p1;
        } else {
          head = s.v0; phead = s.p0;
        }
        ch.pts.push_back(phead);
        prevNode = head;
      }
      (void)tail;
    }

    // Per-backbone-vertex incident mesh faces: the union of the incident faces
    // (FeatureSeg::face0/face1) of the chain segments meeting at each vertex, so
    // the visibility ray-cast can exclude the vertex's own surface (Freestyle
    // self-face exclusion). Backbone vertex k borders segs[k-1] and segs[k] for
    // an interior vertex; the endpoints border one segment (the seam wraps for a
    // closed loop). Up to kMaxIncidentFaces ids, -1-padded.
    const std::size_t nPts = ch.pts.size();
    ch.incidentFaces.assign(
        nPts, std::array<int, EdgeChain::kMaxIncidentFaces>{-1, -1, -1, -1});
    auto addFace = [](std::array<int, EdgeChain::kMaxIncidentFaces>& dst, int f) {
      if (f < 0) return;
      for (int& slot : dst) {
        if (slot == f) return;     // already present
        if (slot < 0) { slot = f; return; }
      }
      // More than kMaxIncidentFaces distinct faces at a hub: drop the overflow.
      // (A near-degenerate hub; the dropped face is a rare extra self-occluder.)
    };
    auto addSegFaces =
        [&](std::array<int, EdgeChain::kMaxIncidentFaces>& dst, int segIdx) {
          const FeatureSeg& s = segs[static_cast<std::size_t>(ch.segs[static_cast<std::size_t>(segIdx)])];
          addFace(dst, s.face0);
          addFace(dst, s.face1);
        };
    const int M = static_cast<int>(ch.segs.size());
    for (std::size_t k = 0; k < nPts; ++k) {
      std::array<int, EdgeChain::kMaxIncidentFaces>& dst = ch.incidentFaces[k];
      const int ki = static_cast<int>(k);
      if (ki - 1 >= 0 && ki - 1 < M) addSegFaces(dst, ki - 1);
      if (ki < M) addSegFaces(dst, ki);
      if (ch.closed) {
        // Seam vertex (first == last) borders the first and last segment.
        if (ki == 0) addSegFaces(dst, M - 1);
        if (ki == static_cast<int>(nPts) - 1) addSegFaces(dst, 0);
      }
    }
    chains.push_back(std::move(ch));
  }
  return chains;
}

void closeVisibilityMask(std::vector<char>& vis, int maxBridge, bool closed) {
  const std::size_t n = vis.size();
  if (maxBridge <= 0 || n == 0) return;
  // For a CLOSED loop a hidden run may straddle the index 0 / n-1 seam; rotate
  // the scan to start at a visible vertex so that seam run is contiguous and
  // gets bracketed/merged like any interior run. If every vertex is hidden
  // there is nothing to bracket, so leave the mask untouched.
  std::size_t start = 0;
  if (closed) {
    while (start < n && vis[start] == 0) ++start;
    if (start == n) return;  // all hidden: no visible bracket anywhere
  }
  // Walk the (rotated for closed) sequence of n vertices, bridging each maximal
  // hidden run that is short enough and bracketed by visible vertices on both
  // sides. Indices are taken modulo n for the closed case.
  const auto at = [&](std::size_t k) -> char& { return vis[(start + k) % n]; };
  std::size_t i = 0;
  while (i < n) {
    if (at(i) != 0) {
      ++i;
      continue;
    }
    std::size_t j = i;
    while (j < n && at(j) == 0) ++j;  // [i,j) is a maximal hidden run
    const std::size_t runLen = j - i;
    // For closed loops `start` is visible, so a run never touches index 0 here;
    // i>0 and j<n always hold (the rotation guarantees a visible bracket exists
    // on both sides). For open loops a run at an end has no bracket -> kept.
    const bool leftVisible = (i > 0) ? (at(i - 1) != 0) : false;
    const bool rightVisible = (j < n)  ? (at(j) != 0)
                              : closed ? (at(0) != 0)  // wrap: start is visible
                                       : false;
    if (runLen <= static_cast<std::size_t>(maxBridge) && leftVisible &&
        rightVisible) {
      for (std::size_t k = i; k < j; ++k) at(k) = 1;
    }
    i = j;
  }
}

std::vector<char> computeChainVisibility(const EdgeChain& chain,
                                         const ScreenProj& sp,
                                         const OcclusionQuery& occluded) {
  std::vector<char> visible(chain.pts.size(), 1);
  if (!occluded) return visible;  // no live BVH: treat everything as visible

  const bool haveFaces = chain.incidentFaces.size() == chain.pts.size();
  // For the ORTHOGRAPHIC case the camera-ward target is at the viewer's z over
  // the point (the ray is the constant -view direction); use a far point along
  // -view from P. Freestyle uses the actual viewpoint for perspective and a
  // point at the viewer's z for ortho; here sp.pos is the eye, and for ortho the
  // ray direction is sp.dir so any far target along it suffices for the segment
  // occlusion test the binding performs (it trims and excludes the self-faces).
  for (std::size_t i = 0; i < chain.pts.size(); ++i) {
    const Vec3& P = chain.pts[i];
    Vec3 target;
    if (sp.ortho) {
      // Push the target far along the camera-ward (-view) axis so the segment
      // P->target spans everything between P and the camera plane.
      target = P + (sp.dir * -1.0f) * 1.0e4f;
    } else {
      target = sp.pos;  // the eye
    }
    // Exclude P's own incident mesh faces (the feature surface the edge sits on)
    // so a self-occlusion hit is not counted -- Freestyle self/adjacent-face
    // exclusion. The ray origin is P itself (no along-view nudge).
    const int* faces = nullptr;
    int nFaces = 0;
    std::array<int, EdgeChain::kMaxIncidentFaces> buf{-1, -1, -1, -1};
    if (haveFaces) {
      for (int f : chain.incidentFaces[i])
        if (f >= 0) buf[static_cast<std::size_t>(nFaces++)] = f;
      faces = buf.data();
    }
    visible[i] = occluded(P, target, faces, nFaces) ? 0 : 1;
  }
  return visible;
}

std::vector<std::vector<Vec2f>> buildRibbonStrips(
    const std::vector<Vec2f>& backbone2d, const std::vector<char>& visible,
    float halfThick, float resampleStepPx) {
  std::vector<std::vector<Vec2f>> out;
  if (backbone2d.size() < 2) return out;
  // Wrap the public input into the internal Pt2 list.
  std::vector<Pt2> proj;
  proj.reserve(backbone2d.size());
  for (std::size_t i = 0; i < backbone2d.size(); ++i) {
    Pt2 q;
    q.p = {backbone2d[i].x, backbone2d[i].y};
    q.visible = visible.empty() || visible[i] != 0;
    proj.push_back(q);
  }
  const std::vector<Pt2> rs = resampleChain(proj, resampleStepPx);
  // Split into maximal runs of consecutive VISIBLE vertices; one strip per run.
  std::vector<Vec2> run;
  auto flush = [&]() {
    if (run.size() >= 2) {
      Strip strip = buildStrip(run, halfThick);
      std::vector<Vec2f> sv;
      sv.reserve(strip.size());
      for (const Vec2& v : strip) sv.push_back({v.x, v.y});
      out.push_back(std::move(sv));
    }
    run.clear();
  };
  for (const Pt2& q : rs) {
    if (q.visible)
      run.push_back(q.p);
    else
      flush();
  }
  flush();
  return out;
}

// STEP 4: variable-width ribbon strokes. Extract topology-tagged feature edges,
// chain them per nature into continuous polylines, mark each backbone vertex
// visible/hidden by ray-cast QI against the live Embree BVH (via `occluded`),
// project + arc-length resample, build a miter-joined offset RIBBON per maximal
// visible run, then hard-fill the triangle strips composited over frame.color in
// LINEAR space at hi-res (the box downsample antialiases). Rasterization is
// row-tiled with TBB and is deterministic (each tile rasterizes every strip
// restricted to its rows). The default (edges off) path never reaches here, so
// the no-edge render stays byte-identical.
void applyStrokeEdges(FrameResult& frame, const Scene& scene,
                      const RenderOptions& opt, const OcclusionQuery& occluded) {
  const StrokeEdgeOptions& se = opt.strokeEdges;
  if (!se.enable) return;
  const int W = frame.width, H = frame.height;
  if (W <= 0 || H <= 0) return;

  // Build the per-frame projection at the (hi-res) frame resolution.
  const ScreenProj sp = makeScreenProj(scene.camera, W, H);

  // Extract feature edges (ray-cast visibility needs no 3D lift, so raise 0).
  ExtractParams ep;
  ep.raise = se.raise;
  ep.width = 0.0f;
  ep.silhouette = se.silhouette;
  ep.crease = se.crease;
  ep.border = se.border;
  ep.meshHardEdgeDeg = se.meshHardEdgeDeg;
  ep.creaseAngleDeg = se.creaseAngleDeg;
  ep.meshCreaseSmoothVetoDeg = se.meshCreaseSmoothVetoDeg;
  ep.meshCreaseConvexOnly = se.meshCreaseConvexOnly;
  ep.meshBorderCoplanarVetoDeg = se.meshBorderCoplanarVetoDeg;
  ep.meshCreaseMaxDegree = se.meshCreaseMaxDegree;
  const FeatureMesh fm = extractMeshFeatureEdges(scene.mesh, scene.camera, ep);

  // Chain per nature into continuous polylines (each segment used once).
  const std::vector<EdgeChain> chains = chainFeatureSegs(fm.segs, fm.nodeCount);

  // Supersample-aware stroke geometry. The stroke pass runs on the HI-RES frame
  // (frame.width == final*ss, pre-downsample), but thickness/resample are
  // specified in FINAL-resolution pixels. Scale every length by the supersample
  // factor so a "--stroke-thickness 2" line keeps a ~2px final width at any ss:
  // without this, a 2px band drawn hi-res box-downsamples to ~2/ss px and fades
  // to faint gray on white (the "outline missing at --supersample 4" bug).
  const float ssScale = static_cast<float>(std::max(1, opt.supersample));

  // Global fallback half-thickness / resample step (hi-res px). thickness is the
  // FULL ribbon width target (final px); halfThick is half of it (Freestyle
  // thickness[0]/[1] each == halfThick gives a 2*halfThick band on a straight
  // run). Both are scaled to hi-res px by ssScale.
  const float globalHalf =
      std::max(0.5f, 0.5f * static_cast<float>(se.thickness) * ssScale);
  const float stepPx =
      std::max(1.0f, static_cast<float>(se.resampleStepPx) * ssScale);
  const bool perSection = !scene.groupEdgeStyle.empty();

  // Resolve the (per-section) style for one chain by its group + nature: pick the
  // EdgeStyle for the chain's group (Scene::groupEdgeStyle, else the global
  // strokeEdges fallback) and read the slot the nature maps to. Returns false to
  // SKIP the chain when its section explicitly disables that nature's class.
  // `outHalf`/`outColor`/`outOpacity` are the resolved ribbon style.
  auto resolveStyle = [&](EdgeNature nat, std::uint16_t group, float& outHalf,
                          float outColor[3], float& outOpacity) -> bool {
    // Master per-nature gate first (global toggle): a nature switched off never
    // draws regardless of section overrides.
    switch (nat) {
      case EdgeNature::Silhouette:
        if (!se.silhouette) return false;
        break;
      case EdgeNature::Border:
        if (!se.border) return false;
        break;
      case EdgeNature::Crease:
        if (!se.crease) return false;
        break;
    }
    if (!perSection) {
      // No section table: use the single global stroke style.
      outHalf = globalHalf;
      outColor[0] = se.color[0];
      outColor[1] = se.color[1];
      outColor[2] = se.color[2];
      outOpacity = se.opacity;
      return true;
    }
    const std::vector<EdgeStyle>& table = scene.groupEdgeStyle;
    const EdgeStyle& es = (group < table.size()) ? table[group]
                                                 : se.defaultStyle;
    const EdgeClassStyle& cs = es.cls[natureStyleSlot(nat)];
    // A section that disables this nature's class inks nothing for it.
    if (!cs.enabled) return false;
    // width is the per-class FULL band width (FINAL px); half it and scale to
    // hi-res px by the supersample factor (>= 0.5 hi-res px).
    outHalf = std::max(0.5f, 0.5f * cs.width * ssScale);
    outColor[0] = cs.color[0];
    outColor[1] = cs.color[1];
    outColor[2] = cs.color[2];
    outOpacity = cs.opacity;
    return true;
  };

  // Build all ribbon strips for the frame up front; rasterize them tiled.
  std::vector<StyledStrip> strips;
  for (const EdgeChain& ch : chains) {
    if (ch.segs.empty()) continue;
    // The chain is single-nature (chaining never crosses natures); gate/style on
    // the first segment's nature and group.
    const FeatureSeg& s0 = fm.segs[static_cast<std::size_t>(ch.segs[0])];
    const EdgeNature nat = s0.nature;
    float halfThick = globalHalf, col[3] = {0.0f, 0.0f, 0.0f}, opacity = 1.0f;
    if (!resolveStyle(nat, s0.group, halfThick, col, opacity)) continue;

    // Ray-cast Quantitative Invisibility per backbone vertex, with the vertex's
    // OWN incident mesh faces excluded as self-occluders (Freestyle self-face
    // exclusion). With true exclusion in place the prior heuristic patch stack
    // (along-view origin nudge + morphological close of the mask + short-run
    // drop) is no longer needed: visibility is decided by incident-face
    // exclusion, not by offset+bridging.
    const std::vector<char> visible = computeChainVisibility(ch, sp, occluded);
    // Project + resample (visibility-aware) into a 2D polyline.
    const std::vector<Pt2> proj = projectChain(sp, ch.pts, visible);
    const std::vector<Pt2> rs = resampleChain(proj, stepPx);

    // Emit a strip per maximal run of consecutive VISIBLE vertices
    // (StrokeRep.cpp:837-867: hidden vertices break the strip). A strip needs at
    // least 2 vertices.
    const std::size_t minRun = 2;
    std::vector<Vec2> run;
    auto flush = [&]() {
      if (run.size() >= minRun) {
        StyledStrip ss;
        ss.strip = buildStrip(run, halfThick);
        ss.color[0] = col[0];
        ss.color[1] = col[1];
        ss.color[2] = col[2];
        ss.opacity = opacity;
        ss.precedence = naturePrecedence(nat);
        strips.push_back(std::move(ss));
      }
      run.clear();
    };
    for (const Pt2& q : rs) {
      if (q.visible)
        run.push_back(q.p);
      else
        flush();
    }
    flush();
  }

  if (strips.empty()) return;

  // Stable-sort strips by nature PRECEDENCE (Crease < Border < Silhouette) so a
  // higher-precedence stroke inks OVER a lower one where they overlap. Stable so
  // strips of equal precedence keep their build (chain) order -> deterministic.
  std::stable_sort(strips.begin(), strips.end(),
                   [](const StyledStrip& a, const StyledStrip& b) {
                     return a.precedence < b.precedence;
                   });

  // Composite all strips over frame.color, row-tiled with TBB. Each tile
  // rasterizes EVERY strip in PRECEDENCE order but only the rows in its range, so
  // the result is independent of tile boundaries / thread scheduling
  // (deterministic).
  tbb::parallel_for(
      tbb::blocked_range<int>(0, H),
      [&](const tbb::blocked_range<int>& rows) {
        const int rb = rows.begin(), re = rows.end();
        for (const StyledStrip& ss : strips)
          rasterizeStrip(frame.color, W, rb, re, ss.strip, ss.color, ss.opacity);
      });
}

}  // namespace umbreon
