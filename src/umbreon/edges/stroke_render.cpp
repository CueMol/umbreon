// Shared stroke DRAW stage implementation: 2D ribbon raster primitives, the
// Freestyle Stroke/StrokeShader stylization layer, stroke assembly, and the
// renderStrokeChains driver. See stroke_render.hpp; any edge-chain SOURCE
// (today the screen vector source) feeds this stage.
#include "edges/stroke_render.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "postprocess/fog.hpp"

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

// One projected, resampled backbone vertex of a chain: 2D pixel position, linear
// view-z (carried for future depth use), the surface alpha multiplier (first-
// hit fragment opacity under the vertex; 1 = opaque) and a visibility flag.
struct Pt2 {
  Vec2 p;
  float vz = 0.0f;
  float surfA = 1.0f;
  bool visible = true;
};

// A ribbon strip: a flat list of offset border vertices in pairs (left,right)
// per backbone vertex (2*N entries), consumed as a triangle strip.
using Strip = std::vector<Vec2>;

// A strip plus the resolved (per-section) style to ink it with: linear RGB color
// and opacity. The strip geometry already baked in the per-section half-width, so
// only color/opacity travel here. `alphas` (one entry per backbone vertex ==
// strip pair) carries a per-vertex effective opacity when the run's alpha
// varies along it (surface-alpha gradient under the edge); EMPTY means the
// constant `opacity` applies to the whole strip -- the exact legacy path.
// Per-pixel end-clip disc (StrokeEndClip resolved for rasterization): a pixel
// within r2 of (px, py) whose offset has a positive dot with (nx, ny) is
// culled. nClips == 0 on every legacy path.
struct ClipDisc {
  float px = 0.0f, py = 0.0f;
  float nx = 0.0f, ny = 0.0f;
  float r2 = 0.0f;
};

inline bool clippedPx(float x, float y, const ClipDisc* clips, int nClips) {
  for (int c = 0; c < nClips; ++c) {
    const float dx = x - clips[c].px, dy = y - clips[c].py;
    if (dx * dx + dy * dy > clips[c].r2) continue;
    if (dx * clips[c].nx + dy * clips[c].ny > 0.0f) return true;
  }
  return false;
}

struct StyledStrip {
  Strip strip;
  float color[3] = {0.0f, 0.0f, 0.0f};
  float opacity = 1.0f;
  std::vector<float> alphas;
  // Active end-clip discs of the source chain (0 on legacy paths).
  std::array<ClipDisc, 2> clips;
  int nClips = 0;
  // Per-backbone-vertex ink color (one entry per strip pair) when the color
  // varies along the run -- the depth-fog gradient path (ink melts toward the
  // fog color with distance). EMPTY means the constant `color` applies to the
  // whole strip (the exact legacy path; no fog, or fog with a transparent
  // background where only the alpha fades).
  std::vector<std::array<float, 3>> colors;
  int precedence = 0;     // nature tie-break key (lower paints first)
  float depthKey = 0.0f;  // min view-z over the run (FARTHER = larger); primary sort
  // Round cap / round join ARC FANS: a triangle soup rasterized after the
  // body quads (3*i vertices in fanPts; triangle i carries the constant ink
  // alpha/color of its backbone vertex in fanAlpha[i]/fanColor[i]). Empty --
  // and the body path byte-identical -- unless --stroke-cap/--stroke-join
  // round is on.
  std::vector<Vec2> fanPts;
  std::vector<float> fanAlpha;
  std::vector<std::array<float, 3>> fanColor;
};

// Build a miter-joined ribbon strip for a backbone polyline `bb` (>= 2 points)
// with PER-VERTEX left/right half-widths `L[k]`/`R[k]` (Freestyle Strip::createStrip,
// StrokeRep.cpp:105-293, asymmetric thickness[1]=L on +normal, [0]=R on -normal):
// per vertex emit p + L*n (left) and p - R*n (right) with n the segment normal;
// interior vertices MITER-join by intersecting the prev/next offset lines (each side
// offset by the CURRENT vertex width), SPIKE-CLAMPED per side to the averaged normal
// when the miter overruns MAX_RATIO_LENGTH_SINGU*(L|R) or the join is near-degenerate.
// Returns 2*bb.size() vertices: [2k]=left(+), [2k+1]=right(-). A symmetric stroke
// passes L[k]==R[k]==halfThick (scalar overload below), reducing this to the
// constant-width path expression-for-expression (byte-identical).
Strip buildStrip(const std::vector<Vec2>& bb, const std::vector<float>& L,
                 const std::vector<float>& R) {
  Strip out;
  const std::size_t n = bb.size();
  if (n < 2 || L.size() != n || R.size() != n) return out;
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
    out[0] = bb[0] + sd * L[0];   // left (+)
    out[1] = bb[0] - sd * R[0];   // right (-)
  }

  // Interior vertices: miter join (each side offset by the CURRENT vertex width).
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
    const float lw = L[k], rw = R[k];

    // Left (+) miter: intersect the two offset lines.
    Vec2 inter;
    if (intersect2dLine2dLine(pPrev + sdP * lw, p + sdP * lw,
                              p + sdN * lw, pNext + sdN * lw, inter))
      out[2 * k] = inter;
    else
      out[2 * k] = p + sdN * lw;

    // Right (-) miter.
    if (intersect2dLine2dLine(pPrev - sdP * rw, p - sdP * rw,
                              p - sdN * rw, pNext - sdN * rw, inter))
      out[2 * k + 1] = inter;
    else
      out[2 * k + 1] = p - sdN * rw;

    // Averaged (bevel) normal for the spike clamp.
    Vec2 sdAvg = sdN + sdP;
    const bool degenerate =
        (dirNNorm < kZero) || (dirPNorm < kZero) || (norm2(sdAvg) < kZero);
    if (degenerate)
      sdAvg = {0.0f, 0.0f};
    else
      sdAvg = unit(sdAvg);

    // SPIKE-CLAMP per side: if the miter overruns MAX_RATIO_LENGTH_SINGU*(L|R) from
    // p, or the join is near-degenerate / a near-180 fold, fall back to the averaged
    // normal offset (StrokeRep.cpp:278-292).
    const float foldDot = std::fabs(dot2(sdAvg, udirN));
    auto overruns = [&](const Vec2& v, float limit) {
      const Vec2 t = v - p;
      return (norm2(t) > limit) || degenerate || notValid(v) ||
             (foldDot < kEpsSingularity);
    };
    if (overruns(out[2 * k], lw * kMaxRatioLengthSingu))
      out[2 * k] = p + sdAvg * lw;
    if (overruns(out[2 * k + 1], rw * kMaxRatioLengthSingu))
      out[2 * k + 1] = p - sdAvg * rw;
  }

  // Last vertex: normal of the last segment.
  {
    const Vec2 dir = unit(bb[n - 1] - bb[n - 2]);
    const Vec2 sd = orth(dir);
    out[2 * (n - 1)] = bb[n - 1] + sd * L[n - 1];
    out[2 * (n - 1) + 1] = bb[n - 1] - sd * R[n - 1];
  }
  return out;
}

// ---------------------------------------------------------------------------
// Round caps / round joins (--stroke-cap round, --stroke-join round).

// Append an arc fan (triangle soup) around `p` sweeping from offset vector
// `a0` to `a1`, subdivided at ~1 px arc steps. The sweep normally takes the
// SHORT way; `via` breaks the tie for a 180-degree cap sweep and guards
// against the wrong side in general: when the sweep midpoint points away
// from `via`, the direction is flipped. The radius lerps |a0| -> |a1| along
// the sweep, so asymmetric (tapered) widths blend smoothly. Every triangle
// (p, q_i, q_i+1) is recorded with its backbone vertex `src` for the
// attribute lookup at rep-build time.
void appendArcFan(const Vec2& p, const Vec2& a0, const Vec2& a1,
                  const Vec2& via, std::size_t src, std::vector<Vec2>& fanPts,
                  std::vector<std::size_t>& fanSrc) {
  const float r0 = norm2(a0), r1 = norm2(a1);
  if (r0 <= kZero || r1 <= kZero) return;
  const Vec2 u0{a0.x / r0, a0.y / r0};
  const Vec2 u1{a1.x / r1, a1.y / r1};
  const float c = std::max(-1.0f, std::min(1.0f, dot2(u0, u1)));
  const float ang = std::acos(c);
  if (ang <= 1.0e-3f) return;
  const float crossU = u0.x * u1.y - u0.y * u1.x;
  float sgn = crossU >= 0.0f ? 1.0f : -1.0f;
  {  // orient the sweep toward `via` (the outward / outer-wedge side)
    const float half = 0.5f * sgn * ang;
    const float cs = std::cos(half), sn = std::sin(half);
    const Vec2 mid{u0.x * cs - u0.y * sn, u0.x * sn + u0.y * cs};
    if (dot2(mid, via) < 0.0f) sgn = -sgn;
  }
  const int steps = std::max(
      2, static_cast<int>(std::ceil(ang * std::max(r0, r1))));
  Vec2 prev = p + a0;
  for (int i = 1; i <= steps; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(steps);
    const float th = sgn * ang * t;
    const float cs = std::cos(th), sn = std::sin(th);
    const Vec2 u{u0.x * cs - u0.y * sn, u0.x * sn + u0.y * cs};
    const float r = r0 + (r1 - r0) * t;
    const Vec2 q = p + u * r;
    fanPts.push_back(p);
    fanPts.push_back(prev);
    fanPts.push_back(q);
    fanSrc.push_back(src);
    prev = q;
  }
}

// Round-join variant of buildStrip: an interior vertex whose turn exceeds
// ~10 degrees emits TWO segment-aligned pairs (prev-normal, then next-normal)
// instead of one miter pair, so both adjacent quads end square at the corner
// -- no miter spike -- and the OUTER wedge between them is filled with an arc
// fan. Near-straight corners keep a single averaged pair (visually identical
// to a round join there and it keeps the strip compact). `pairSrc` maps each
// emitted pair to its backbone vertex so the caller can duplicate the
// per-vertex attributes (alphas / fog colors); fan triangles land in
// fanPts/fanSrc via appendArcFan.
Strip buildStripRound(const std::vector<Vec2>& bb, const std::vector<float>& L,
                      const std::vector<float>& R,
                      std::vector<std::size_t>& pairSrc,
                      std::vector<Vec2>& fanPts,
                      std::vector<std::size_t>& fanSrc) {
  Strip out;
  const std::size_t n = bb.size();
  if (n < 2 || L.size() != n || R.size() != n) return out;

  auto orth = [](const Vec2& d) { return Vec2{-d.y, d.x}; };
  auto unit = [](const Vec2& d) {
    const float l = norm2(d);
    return l > kZero ? Vec2{d.x / l, d.y / l} : Vec2{0.0f, 0.0f};
  };
  auto pushPair = [&](const Vec2& l, const Vec2& r, std::size_t src) {
    out.push_back(l);
    out.push_back(r);
    pairSrc.push_back(src);
  };
  const float kStraightCos = 0.9848f;  // cos(10 deg)

  {  // first vertex: normal of the first segment
    const Vec2 sd = orth(unit(bb[1] - bb[0]));
    pushPair(bb[0] + sd * L[0], bb[0] - sd * R[0], 0);
  }
  for (std::size_t k = 1; k + 1 < n; ++k) {
    const Vec2& p = bb[k];
    const Vec2 udirP = unit(p - bb[k - 1]);
    const Vec2 udirN = unit(bb[k + 1] - p);
    const Vec2 sdP = orth(udirP);
    const Vec2 sdN = orth(udirN);
    const float lw = L[k], rw = R[k];
    const float turnCos = dot2(udirP, udirN);
    Vec2 sdAvg = sdP + sdN;
    const bool degenerate =
        norm2(udirP) <= kZero || norm2(udirN) <= kZero || norm2(sdAvg) <= kZero;
    if (degenerate || turnCos >= kStraightCos) {
      sdAvg = degenerate ? Vec2{0.0f, 0.0f} : unit(sdAvg);
      pushPair(p + sdAvg * lw, p - sdAvg * rw, k);
      continue;
    }
    // Square segment ends on both sides of the corner...
    pushPair(p + sdP * lw, p - sdP * rw, k);
    pushPair(p + sdN * lw, p - sdN * rw, k);
    // ...plus the arc fan on the OUTER side of the turn (cross(dirP, dirN)
    // > 0 turns toward +y in raster space, putting the exterior wedge on the
    // RIGHT (-normal) side; < 0 on the LEFT).
    const float crossD = udirP.x * udirN.y - udirP.y * udirN.x;
    const Vec2 a0 = crossD > 0.0f ? Vec2{-sdP.x * rw, -sdP.y * rw}
                                  : Vec2{sdP.x * lw, sdP.y * lw};
    const Vec2 a1 = crossD > 0.0f ? Vec2{-sdN.x * rw, -sdN.y * rw}
                                  : Vec2{sdN.x * lw, sdN.y * lw};
    appendArcFan(p, a0, a1, a0 + a1, k, fanPts, fanSrc);
  }
  {  // last vertex: normal of the last segment
    const Vec2 sd = orth(unit(bb[n - 1] - bb[n - 2]));
    pushPair(bb[n - 1] + sd * L[n - 1], bb[n - 1] - sd * R[n - 1], n - 1);
  }
  return out;
}

// Half-disk cap fans beyond the run's two endpoints (--stroke-cap round):
// sweep from the left offset through the OUTWARD pole to the right offset,
// radius lerping between the endpoint's left/right half-widths (a tapered
// end keeps its thin tip). capStart/capEnd let the caller skip an end: a
// junction-tapered end (outside alignment) must stay a butt -- its cap
// would poke a half-width past the line it meets.
void appendCapFans(const std::vector<Vec2>& bb, const std::vector<float>& L,
                   const std::vector<float>& R, bool capStart, bool capEnd,
                   std::vector<Vec2>& fanPts,
                   std::vector<std::size_t>& fanSrc) {
  const std::size_t n = bb.size();
  if (n < 2) return;
  auto orth = [](const Vec2& d) { return Vec2{-d.y, d.x}; };
  auto unit = [](const Vec2& d) {
    const float l = norm2(d);
    return l > kZero ? Vec2{d.x / l, d.y / l} : Vec2{0.0f, 0.0f};
  };
  if (capStart) {  // start: outward = against the first segment
    const Vec2 d = unit(bb[1] - bb[0]);
    const Vec2 sd = orth(d);
    appendArcFan(bb[0], sd * L[0], Vec2{-sd.x * R[0], -sd.y * R[0]},
                 Vec2{-d.x, -d.y}, 0, fanPts, fanSrc);
  }
  if (capEnd) {  // end: outward = along the last segment
    const Vec2 d = unit(bb[n - 1] - bb[n - 2]);
    const Vec2 sd = orth(d);
    appendArcFan(bb[n - 1], sd * L[n - 1],
                 Vec2{-sd.x * R[n - 1], -sd.y * R[n - 1]}, d, n - 1, fanPts,
                 fanSrc);
  }
}

// Hard-fill one 2D triangle into the framebuffer with a linear over-composite
// (coverage 1 inside). Pixel centers at integer (x,y); a pixel is inside iff its
// center is on the inside of all three edges (top-left-agnostic; the hi-res box
// downsample antialiases). Only rows in [rowBegin,rowEnd) are touched, so callers
// can tile deterministically over screen rows with TBB.
void fillTriangle(std::vector<float>& color, int W, int rowBegin, int rowEnd,
                  const Vec2& a, const Vec2& b, const Vec2& c,
                  const float col[3], float opacity, const ClipDisc* clips,
                  int nClips) {
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
      if (nClips && clippedPx(px, py, clips, nClips)) continue;
      const std::size_t idx = (static_cast<std::size_t>(y) * W + x) * 4;
      compositeOver(&color[idx], col, opacity);
    }
  }
}

// As fillTriangle, but with a PER-VERTEX opacity (aA at `a`, aB at `b`, aC at
// `c`) interpolated barycentrically per pixel -- the alpha-gradient path for a
// stroke whose surface alpha varies along the backbone. The constant-alpha
// strips keep the fillTriangle path above (bit-identical legacy output).
void fillTriangleAlpha(std::vector<float>& color, int W, int rowBegin,
                       int rowEnd, const Vec2& a, const Vec2& b, const Vec2& c,
                       const float col[3], float aA, float aB, float aC,
                       const ClipDisc* clips, int nClips) {
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

  const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  if (std::fabs(area) < 1.0e-7f) return;
  const float inv = 1.0f / area;

  for (int y = minY; y <= maxY; ++y) {
    const float py = static_cast<float>(y);
    for (int x = minX; x <= maxX; ++x) {
      const float px = static_cast<float>(x);
      const float w0 =
          ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inv;
      const float w1 =
          ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv;
      const float w2 = 1.0f - w0 - w1;
      if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;  // outside
      if (nClips && clippedPx(px, py, clips, nClips)) continue;
      const std::size_t idx = (static_cast<std::size_t>(y) * W + x) * 4;
      compositeOver(&color[idx], col, w0 * aA + w1 * aB + w2 * aC);
    }
  }
}

// As fillTriangleAlpha, but with a PER-VERTEX color (colA at `a`, colB at `b`,
// colC at `c`) interpolated barycentrically per pixel as well -- the depth-fog
// gradient path for a stroke whose ink color varies along the backbone (the ink
// melting toward the fog color with distance). The constant-color strips keep
// the fillTriangle / fillTriangleAlpha paths above (bit-identical legacy output).
void fillTriangleColorAlpha(std::vector<float>& color, int W, int rowBegin,
                            int rowEnd, const Vec2& a, const Vec2& b,
                            const Vec2& c, const float colA[3],
                            const float colB[3], const float colC[3], float aA,
                            float aB, float aC, const ClipDisc* clips,
                            int nClips) {
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

  const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  if (std::fabs(area) < 1.0e-7f) return;
  const float inv = 1.0f / area;

  for (int y = minY; y <= maxY; ++y) {
    const float py = static_cast<float>(y);
    for (int x = minX; x <= maxX; ++x) {
      const float px = static_cast<float>(x);
      const float w0 =
          ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inv;
      const float w1 =
          ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv;
      const float w2 = 1.0f - w0 - w1;
      if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;  // outside
      if (nClips && clippedPx(px, py, clips, nClips)) continue;
      const std::size_t idx = (static_cast<std::size_t>(y) * W + x) * 4;
      const float col[3] = {w0 * colA[0] + w1 * colB[0] + w2 * colC[0],
                            w0 * colA[1] + w1 * colB[1] + w2 * colC[1],
                            w0 * colA[2] + w1 * colB[2] + w2 * colC[2]};
      compositeOver(&color[idx], col, w0 * aA + w1 * aB + w2 * aC);
    }
  }
}

// DEBUG (--stroke-node-dots): hard-fill a disc of radius r around (cx, cy),
// restricted to rows [rowBegin, rowEnd) like the triangle fills. Used only by
// the node overlay, never on a production path.
void fillDisc(std::vector<float>& color, int W, int rowBegin, int rowEnd,
              float cx, float cy, float r, const float col[3]) {
  if (r <= 0.0f) return;
  const int minX = std::max(0, static_cast<int>(std::floor(cx - r)));
  const int maxX = std::min(W - 1, static_cast<int>(std::ceil(cx + r)));
  const int minY = std::max(rowBegin, static_cast<int>(std::floor(cy - r)));
  const int maxY = std::min(rowEnd - 1, static_cast<int>(std::ceil(cy + r)));
  const float r2 = r * r;
  for (int y = minY; y <= maxY; ++y) {
    const float dy = static_cast<float>(y) - cy;
    for (int x = minX; x <= maxX; ++x) {
      const float dx = static_cast<float>(x) - cx;
      if (dx * dx + dy * dy > r2) continue;
      const std::size_t idx = (static_cast<std::size_t>(y) * W + x) * 4;
      compositeOver(&color[idx], col, 1.0f);
    }
  }
}

// Stamp a hairline segment by walking discs along it (row-clipped like the
// triangle fills, so the TBB row tiling stays deterministic). DEBUG only.
void drawThinSegment(std::vector<float>& color, int W, int rowBegin, int rowEnd,
                     const Vec2& a, const Vec2& b, float r,
                     const float col[3]) {
  const float dx = b.x - a.x, dy = b.y - a.y;
  const float len = std::sqrt(dx * dx + dy * dy);
  const int steps = std::max(1, static_cast<int>(std::ceil(len * 2.0f)));
  for (int i = 0; i <= steps; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(steps);
    fillDisc(color, W, rowBegin, rowEnd, a.x + dx * t, a.y + dy * t, r, col);
  }
}

// One backbone node of the --stroke-node-dots overlay.
struct NodeDot {
  float x = 0.0f, y = 0.0f;
  bool endpoint = false;
};

// One drawn polyline of the overlay: the raw node-to-node segments in a
// per-chain palette color, so consecutive runs (a split contour) read as a
// color change instead of one continuous black line.
struct DebugPoly {
  std::vector<Vec2> pts;
  float col[3] = {0.0f, 0.0f, 0.0f};
};

// Palette for the overlay polylines: saturated hues that stay legible under
// the RED (end) / GREEN (interior) node dots drawn on top.
constexpr float kNodePalette[8][3] = {
    {0.00f, 0.20f, 1.00f},  // blue
    {1.00f, 0.45f, 0.00f},  // orange
    {0.60f, 0.00f, 0.85f},  // purple
    {0.00f, 0.60f, 0.65f},  // teal
    {1.00f, 0.00f, 0.75f},  // magenta
    {0.45f, 0.30f, 0.05f},  // brown
    {0.15f, 0.45f, 0.95f},  // steel
    {0.50f, 0.50f, 0.00f},  // olive
};

// Rasterize one ribbon strip (2*N border vertices, pairs per backbone vertex) as
// a triangle strip, restricted to rows [rowBegin,rowEnd). Two triangles per quad
// between consecutive backbone vertices. `alphas`, when non-null, holds one
// opacity per backbone vertex (strip pair) and switches the quad to the
// alpha-interpolating fill; null keeps the constant-opacity legacy fill.
// `colors`, when non-null, holds one ink color per backbone vertex and switches
// the quad to the color+alpha-interpolating fill (the depth-fog gradient); it
// composes with `alphas` (constant `opacity` is used where `alphas` is null).
void rasterizeStrip(std::vector<float>& color, int W, int rowBegin, int rowEnd,
                    const StyledStrip& ss) {
  const Strip& strip = ss.strip;
  const float* col = ss.color;
  const float opacity = ss.opacity;
  const float* alphas = ss.alphas.empty() ? nullptr : ss.alphas.data();
  const std::array<float, 3>* colors =
      ss.colors.empty() ? nullptr : ss.colors.data();
  const std::size_t pairs = strip.size() / 2;
  for (std::size_t k = 0; k + 1 < pairs; ++k) {
    const Vec2& l0 = strip[2 * k];
    const Vec2& r0 = strip[2 * k + 1];
    const Vec2& l1 = strip[2 * (k + 1)];
    const Vec2& r1 = strip[2 * (k + 1) + 1];
    if (colors) {
      const float* c0 = colors[k].data();
      const float* c1 = colors[k + 1].data();
      const float a0 = alphas ? alphas[k] : opacity;
      const float a1 = alphas ? alphas[k + 1] : opacity;
      fillTriangleColorAlpha(color, W, rowBegin, rowEnd, l0, r0, l1, c0, c0, c1,
                             a0, a0, a1, ss.clips.data(), ss.nClips);
      fillTriangleColorAlpha(color, W, rowBegin, rowEnd, r0, r1, l1, c0, c1, c1,
                             a0, a1, a1, ss.clips.data(), ss.nClips);
    } else if (alphas) {
      const float a0 = alphas[k], a1 = alphas[k + 1];
      fillTriangleAlpha(color, W, rowBegin, rowEnd, l0, r0, l1, col, a0, a0,
                        a1, ss.clips.data(), ss.nClips);
      fillTriangleAlpha(color, W, rowBegin, rowEnd, r0, r1, l1, col, a0, a1,
                        a1, ss.clips.data(), ss.nClips);
    } else {
      fillTriangle(color, W, rowBegin, rowEnd, l0, r0, l1, col, opacity,
                   ss.clips.data(), ss.nClips);
      fillTriangle(color, W, rowBegin, rowEnd, r0, r1, l1, col, opacity,
                   ss.clips.data(), ss.nClips);
    }
  }
  // Round cap/join arc fans: constant alpha/color per triangle (resolved at
  // rep-build time from the fan's backbone vertex). Empty unless
  // --stroke-cap/--stroke-join round.
  for (std::size_t i = 0; i < ss.fanAlpha.size(); ++i) {
    const float a = ss.fanAlpha[i];
    fillTriangleAlpha(color, W, rowBegin, rowEnd, ss.fanPts[3 * i],
                      ss.fanPts[3 * i + 1], ss.fanPts[3 * i + 2],
                      ss.fanColor[i].data(), a, a, a, ss.clips.data(),
                      ss.nClips);
  }
}

// ---------------------------------------------------------------------------
// Freestyle STROKE layer (parametric curve + per-vertex attribute). Sits between
// the visibility-tagged 2D polyline (projectChainSubSpans -> vector<Pt2>) and the
// ribbon rasterizer, mirroring Freestyle Stroke / StrokeVertex / StrokeAttribute
// (Stroke.h) + Operators::createStroke + Stroke::Resample. It makes per-vertex
// stylization (variable width / color / taper / noise) expressible. The DEFAULT
// path writes one constant attribute per chain, so the rasterized result is
// byte-identical until a real shader runs.

// Per-vertex stylization payload (Freestyle StrokeAttribute, Stroke.h:44-302).
// leftThick/rightThick are the +normal / -normal half-widths (Freestyle
// _thickness[1]=L, [0]=R); a symmetric stroke has leftThick==rightThick==halfThick.
struct StrokeAttribute {
  float leftThick = 0.0f, rightThick = 0.0f;
  float color[3] = {0.0f, 0.0f, 0.0f};
  float alpha = 1.0f;
};

// One backbone vertex = 2D geometry + curvilinear abscissa + attribute (Freestyle
// StrokeVertex, Stroke.h:310-458). p/vz/surfA/visible mirror Pt2; `ca` is the 2D
// arc length from the stroke start and `u = ca/length2d` in [0,1] is the shader
// parameter. `visible` stays on the vertex (not in attr) to match the run-split.
// `surfA` is the surface alpha multiplier sampled under the vertex; it stays
// OUTSIDE attr so the constant color/alpha shaders cannot clobber it -- the
// effective ink opacity is attr.alpha * surfA at rep-build time.
struct StrokeVertex {
  Vec2 p;
  float vz = 0.0f;
  float surfA = 1.0f;
  bool visible = true;
  float u = 0.0f, ca = 0.0f;
  StrokeAttribute attr;
};

// A chained, parametric stroke (Freestyle Stroke, Stroke.h:483-858).
// `precedence` is the overlap paint order carried to the strips (the mesh
// source passes naturePrecedence(nature); a screen-source chain passes its
// class precedence directly).
struct Stroke {
  std::vector<StrokeVertex> verts;
  float length2d = 0.0f;
  int chainIdx = 0;
  int precedence = 0;
};

// Stylization shader contract (Freestyle StrokeShader, StrokeShader.h:50-77): a
// shader iterates the stroke vertices and writes each one's attribute, optionally
// as a function of v.u()/v.ca. Concrete width/color/taper/noise shaders plug in
// here later; the two constant shaders below just reproduce the resolved per-section
// default attribute, exercising the read/write contract without changing output.
struct StrokeShader {
  virtual ~StrokeShader() = default;
  virtual int shade(Stroke& s) const = 0;
};

// Constant per-side half-widths (Freestyle ConstantThicknessShader,
// BasicStrokeShaders.cpp:40-58; extended to an asymmetric left/right pair
// for the outside stroke alignment).
struct ConstantThicknessShader : StrokeShader {
  float leftThick, rightThick;
  ConstantThicknessShader(float l, float r) : leftThick(l), rightThick(r) {}
  int shade(Stroke& s) const override {
    for (StrokeVertex& v : s.verts) {
      v.attr.leftThick = leftThick;
      v.attr.rightThick = rightThick;
    }
    return 0;
  }
};

// Constant color + alpha (Freestyle ConstantColorShader, BasicStrokeShaders.cpp:204-212).
struct ConstantColorShader : StrokeShader {
  float color[3];
  float alpha;
  ConstantColorShader(const float c[3], float a)
      : color{c[0], c[1], c[2]}, alpha(a) {}
  int shade(Stroke& s) const override {
    for (StrokeVertex& v : s.verts) {
      v.attr.color[0] = color[0];
      v.attr.color[1] = color[1];
      v.attr.color[2] = color[2];
      v.attr.alpha = alpha;
    }
    return 0;
  }
};

// Depth fog as a per-vertex stroke shader: fade each vertex by the SAME OpenGL
// linear fog the surface post-process uses (postprocess/fog.hpp), keyed on the
// vertex plane eye-z `vz` (the crack tracer already carried it end-to-end from
// the owner pixel's viewZ AOV -- the same depth fog reads for 3D surfaces). This
// is what makes edge lines recede into the fog like the geometry under them:
// applyFog runs on frame.color BEFORE the stroke pass, so without this shader the
// ink would be painted at full strength over the already-fogged surface. Mirrors
// applyFog's two background modes EXACTLY --
//   * opaque background: mix the ink color toward fog.color by (1 - f); opacity
//     unchanged (the color melts into the fog-colored background).
//   * transparent background: fade the ink opacity by f; color unchanged (the
//     coverage drops so the straight-alpha output can be re-composited later).
// Runs after the color shader (which stamps the resolved ink color/alpha) so it
// modulates the final attribute. f=1 near (unfogged), f=0 far (full fog).
struct FogShader : StrokeShader {
  Fog fog;
  bool transparentBackground;
  FogShader(const Fog& f, bool transparent)
      : fog(f), transparentBackground(transparent) {}
  int shade(Stroke& s) const override {
    for (StrokeVertex& v : s.verts) {
      const float f = fogFactor(fog, v.vz);
      if (transparentBackground) {
        v.attr.alpha *= f;  // fade coverage; keep ink color
      } else {
        const float g = 1.0f - f;
        v.attr.color[0] = fog.color.x * g + v.attr.color[0] * f;
        v.attr.color[1] = fog.color.y * g + v.attr.color[1] * f;
        v.attr.color[2] = fog.color.z * g + v.attr.color[2] * f;
      }
    }
    return 0;
  }
};

// Taper the width toward both stroke ends as a function of the curvilinear abscissa
// u -- a "spindle"/calligraphic look (Freestyle tip handling, the IncreasingThickness
// family). width *= `endScale` at the very ends (u=0,1), ramping (smoothstep) up to
// full by `tipFrac` of u in from each end. A real f(u) stylization that exercises
// the parametric substrate (not byte-identical: this is the demo effect).
struct TaperShader : StrokeShader {
  float tipFrac;
  float endScale;
  TaperShader(float frac, float end) : tipFrac(frac), endScale(end) {}
  int shade(Stroke& s) const override {
    for (StrokeVertex& v : s.verts) {
      const float e = std::min(v.u, 1.0f - v.u);  // distance to nearest end in u
      float k = (tipFrac > 0.0f) ? std::min(1.0f, e / tipFrac) : 1.0f;
      k = k * k * (3.0f - 2.0f * k);  // smoothstep
      const float scale = endScale + (1.0f - endScale) * k;
      v.attr.leftThick *= scale;
      v.attr.rightThick *= scale;
    }
    return 0;
  }
};

// Re-centering for the outside stroke alignment: blend the asymmetric
// left/right half-widths back to their symmetric MEAN (smoothstepped over
// `taperPx` of arc length; the mean is preserved, so the total width stays
// the resolved width) around two kinds of anchors:
//  * FLAGGED ENDS (StrokeChainInput::taperStart/taperEnd): the ribbon
//    arrives centered where it meets other lines -- an offset butt end
//    otherwise sticks its full width out sideways past the meeting line.
//  * INTERIOR FOLDS: where the backbone doubles back on itself within a
//    stroke-width window -- a contour hairpinning around a narrow
//    background wedge or a few-px notch excursion where two surfaces
//    almost touch. The one-sided band paints the excursion's full outer
//    width to one side, a spur poking far out of the meeting lines (the
//    legacy centered ribbon spread the same excursion +-half, mostly
//    swallowed by the neighboring bands). Folding is detected by
//    STRAIGHTNESS -- the chord/arc ratio over a +-taperPx/2 window: a
//    reversal scores ~0, a hairpin cos(turn/2), while a box corner (90
//    degrees) scores ~0.71 and a smooth curve ~1, so crisp corners keep
//    their offset miter. A direction-based test is NOT robust here: at a
//    thin notch the window's net vectors cancel to noise.
struct AlignRecenterShader : StrokeShader {
  bool atStart, atEnd;
  float taperPx;
  AlignRecenterShader(bool s, bool e, float px)
      : atStart(s), atEnd(e), taperPx(px) {}
  int shade(Stroke& s) const override {
    const std::size_t n = s.verts.size();
    if (n < 2 || taperPx <= 0.0f) return 0;
    // Distance from each vertex to its nearest re-center anchor.
    std::vector<float> dist(n, taperPx);
    if (atStart)
      for (std::size_t i = 0; i < n; ++i)
        dist[i] = std::min(dist[i], s.verts[i].ca);
    if (atEnd)
      for (std::size_t i = 0; i < n; ++i)
        dist[i] = std::min(dist[i], s.length2d - s.verts[i].ca);
    // Interior folds: chord/arc straightness over the +-h window.
    const float h = 0.5f * taperPx;
    const float kMinStraight = 0.55f;  // fold = turn beyond ~113 degrees
    std::vector<float> anchors;
    std::size_t a = 0, b = 0;
    for (std::size_t i = 1; i + 1 < n; ++i) {
      const float ca = s.verts[i].ca;
      while (a + 1 < i && s.verts[a + 1].ca <= ca - h) ++a;
      if (b < i + 1) b = i + 1;
      while (b + 1 < n && s.verts[b].ca < ca + h) ++b;
      const float arc = s.verts[b].ca - s.verts[a].ca;
      if (arc <= kZero) continue;
      const float chord = norm2(s.verts[b].p - s.verts[a].p);
      if (chord < kMinStraight * arc) anchors.push_back(ca);
    }
    for (std::size_t i = 0; i < n && !anchors.empty(); ++i)
      for (const float ca : anchors)
        dist[i] = std::min(dist[i], std::fabs(s.verts[i].ca - ca));
    for (std::size_t i = 0; i < n; ++i) {
      float k = std::max(0.0f, dist[i] / taperPx);
      k = k * k * (3.0f - 2.0f * k);  // smoothstep
      StrokeAttribute& at = s.verts[i].attr;
      const float mid = 0.5f * (at.leftThick + at.rightThick);
      at.leftThick = mid + (at.leftThick - mid) * k;
      at.rightThick = mid + (at.rightThick - mid) * k;
    }
    return 0;
  }
};

// GEOMETRY shader: Freestyle's ANISOTROPIC CURVATURE FLOW (a faithful port of
// Smoother in AdvancedStrokeShaders.cpp:190-354). Converges the backbone toward a
// curve of constant curvature (NOT a straight line -- so shape is preserved better
// than Laplacian), driven by two terms whose strength is gated by a Perona-Malik
// edge-stopping function exp(-x^2/sigma^2):
//   * motionNormal      = factorCurvature * curvature * es(curvature, anisoNormal)
//   * motionCurvature   = factorCurvatureDiff * sum es(dCurv, anisoCurvature)*dCurv
// each applied along the per-vertex normal; plus an optional Laplacian point term
// (factorPoint / anisoPoint). The aniso* sigmas are the CORNER-PROTECTION knobs:
// at a high-curvature vertex (a real angular feature, e.g. a ribbon box edge) the
// edge-stopping factor -> 0 so it is barely moved, while gentle tessellation
// jaggedness (low curvature) is smoothed away. Endpoints are fixed (interior loop
// only); closed curves diffuse the seam. carricature blends original->smoothed
// (1 = full, >1 exaggerates). Runs before the width/color shaders; visibility was
// resolved upstream, so moving the drawn line is safe. Recomputes the curvilinear
// abscissa afterward so later f(u) shaders stay consistent.
struct AnisoSmoothingShader : StrokeShader {
  int nbIter;
  float factorPoint, factorCurvature, factorCurvatureDiff;
  float anisoPoint, anisoNormal, anisoCurvature;
  float carricature;
  AnisoSmoothingShader(int it, float fP, float fC, float fCD, float aP, float aN,
                       float aC, float carr)
      : nbIter(it),
        factorPoint(fP),
        factorCurvature(fC),
        factorCurvatureDiff(fCD),
        anisoPoint(aP),
        anisoNormal(aN),
        anisoCurvature(aC),
        carricature(carr) {}

  int shade(Stroke& s) const override {
    const int n = static_cast<int>(s.verts.size());
    if (n < 3) return 0;
    std::vector<Vec2> X(n), orig(n), normal(n);
    std::vector<float> curv(n, 0.0f);
    for (int i = 0; i < n; ++i) {
      X[i] = s.verts[i].p;
      orig[i] = X[i];
    }
    const bool closed = norm2(X[0] - X[n - 1]) < kZero;
    const bool safeTest = (n > 4);  // Smoother::Smoother

    auto es = [](float x, float sigma) -> float {
      return sigma == 0.0f ? 1.0f : std::exp(-(x * x) / (sigma * sigma));
    };
    auto usafe = [](const Vec2& d) -> Vec2 {
      const float l = norm2(d);
      return l > kZero ? Vec2{d.x / l, d.y / l} : Vec2{0.0f, 0.0f};
    };

    auto computeCurvature = [&]() {  // Smoother::computeCurvature
      for (int i = 1; i < n - 1; ++i) {
        Vec2 BA = X[i - 1] - X[i], BC = X[i + 1] - X[i];
        const float lba = norm2(BA), lbc = norm2(BC);
        BA = usafe(BA);
        BC = usafe(BC);
        const Vec2 nc = BA + BC, dCB = BC - BA;
        normal[i] = usafe(Vec2{-dCB.y, dCB.x});
        curv[i] = dot2(nc, normal[i]);
        if (lba + lbc > kZero) curv[i] /= (0.5f * lba + lbc);
      }
      curv[0] = curv[1];
      curv[n - 1] = curv[n - 2];
      normal[0] = usafe(Vec2{-(X[1] - X[0]).y, (X[1] - X[0]).x});
      normal[n - 1] =
          usafe(Vec2{-(X[n - 1] - X[n - 2]).y, (X[n - 1] - X[n - 2]).x});
      if (closed) {  // seam: diffuse from vertex[0]'s wrap-around neighbours
        normal[n - 1] = normal[0];
        curv[n - 1] = curv[0];
      }
    };

    auto iteration = [&]() {  // Smoother::iteration
      computeCurvature();
      for (int i = 1; i < n - 1; ++i) {
        const float mN = factorCurvature * curv[i] * es(curv[i], anisoNormal);
        const float dC1 = curv[i] - curv[i - 1], dC2 = curv[i] - curv[i + 1];
        const float mC = (es(dC1, anisoCurvature) * dC1 +
                          es(dC2, anisoCurvature) * dC2) *
                         factorCurvatureDiff;
        if (safeTest) X[i] = X[i] + normal[i] * (mN + mC);
        const Vec2 v1 = X[i - 1] - X[i], v2 = X[i + 1] - X[i];
        const float d1 = norm2(v1), d2 = norm2(v2);
        X[i] = X[i] + v1 * (factorPoint * es(d2, anisoPoint)) +
               v2 * (factorPoint * es(d1, anisoPoint));
      }
      if (closed) {
        const float mN = factorCurvature * curv[0] * es(curv[0], anisoNormal);
        const float dC1 = curv[0] - curv[n - 2], dC2 = curv[0] - curv[1];
        const float mC = (es(dC1, anisoCurvature) * dC1 +
                          es(dC2, anisoCurvature) * dC2) *
                         factorCurvatureDiff;
        X[0] = X[0] + normal[0] * (mN + mC);
        X[n - 1] = X[0];
      }
    };

    for (int it = 0; it < nbIter; ++it) iteration();

    for (int i = 0; i < n; ++i)  // Smoother::copyVertices (carricature blend)
      s.verts[i].p = orig[i] + (X[i] - orig[i]) * carricature;

    // Positions moved -> recompute curvilinear abscissa / u (vz left as the original
    // scene depth; 2D smoothing does not change a vertex's view-z).
    float ca = 0.0f;
    for (int i = 0; i < n; ++i) {
      if (i > 0) ca += norm2(s.verts[i].p - s.verts[static_cast<std::size_t>(i) - 1].p);
      s.verts[static_cast<std::size_t>(i)].ca = ca;
    }
    s.length2d = ca;
    if (ca > 0.0f)
      for (StrokeVertex& v : s.verts) v.u = v.ca / ca;
    return 0;
  }
};

// Build a parametric Stroke from a visibility-tagged projected polyline, stamping
// the resolved per-chain default attribute into every vertex (Freestyle
// Operators::createStroke, Operators.cpp:1082-1155). Accumulates 2D arc length to
// set `ca` and the normalized abscissa `u`.
Stroke buildStroke(const std::vector<Pt2>& proj, const StrokeAttribute& def,
                   int chainIdx, int precedence) {
  Stroke s;
  s.chainIdx = chainIdx;
  s.precedence = precedence;
  s.verts.reserve(proj.size());
  float ca = 0.0f;
  for (std::size_t i = 0; i < proj.size(); ++i) {
    if (i > 0) ca += norm2(proj[i].p - proj[i - 1].p);
    StrokeVertex v;
    v.p = proj[i].p;
    v.vz = proj[i].vz;
    v.surfA = proj[i].surfA;
    v.visible = proj[i].visible;
    v.ca = ca;
    v.attr = def;
    s.verts.push_back(v);
  }
  s.length2d = ca;
  if (ca > 0.0f)
    for (StrokeVertex& v : s.verts) v.u = v.ca / ca;
  return s;
}

// Interpolate a StrokeVertex at parameter t in [0,1] (Freestyle StrokeVertex(A,B,t),
// Stroke.cpp:358-364): geometry + abscissa + ALL attributes are LERPed, but
// `visible` is COPIED from the segment START (never lerped) so a visible<->hidden
// boundary is realized only at a true node -- keeping the run-split identical to
// resampleChain.
StrokeVertex lerpStrokeVertex(const StrokeVertex& a, const StrokeVertex& b,
                              float t) {
  StrokeVertex q;
  q.p = a.p + (b.p - a.p) * t;
  q.vz = a.vz + (b.vz - a.vz) * t;
  q.surfA = a.surfA + (b.surfA - a.surfA) * t;
  q.u = a.u + (b.u - a.u) * t;
  q.ca = a.ca + (b.ca - a.ca) * t;
  q.visible = a.visible;
  q.attr.leftThick = a.attr.leftThick + (b.attr.leftThick - a.attr.leftThick) * t;
  q.attr.rightThick =
      a.attr.rightThick + (b.attr.rightThick - a.attr.rightThick) * t;
  for (int c = 0; c < 3; ++c)
    q.attr.color[c] = a.attr.color[c] + (b.attr.color[c] - a.attr.color[c]) * t;
  q.attr.alpha = a.attr.alpha + (b.attr.alpha - a.attr.alpha) * t;
  return q;
}

// Arc-length resample the stroke backbone every `stepPx` pixels, attribute-
// preserving (Freestyle Stroke::Resample, Stroke.cpp:636-691). Uses the SAME
// stepping arithmetic as resampleChain so the densified positions are bit-
// identical; inserted vertices interpolate geometry + attributes via
// lerpStrokeVertex.
void resampleStroke(Stroke& s, float stepPx) {
  if (s.verts.size() < 2 || stepPx <= 0.0f) return;
  std::vector<StrokeVertex> out;
  out.reserve(s.verts.size() * 2);
  out.push_back(s.verts.front());
  for (std::size_t i = 1; i < s.verts.size(); ++i) {
    const StrokeVertex& a = s.verts[i - 1];
    const StrokeVertex& b = s.verts[i];
    const Vec2 d = b.p - a.p;
    const float len = norm2(d);
    if (len <= kZero) {
      out.push_back(b);
      continue;
    }
    const int n = static_cast<int>(std::floor(len / stepPx));
    for (int k = 1; k <= n; ++k) {
      const float t = (stepPx * static_cast<float>(k)) / len;
      if (t >= 1.0f) break;
      out.push_back(lerpStrokeVertex(a, b, t));
    }
    out.push_back(b);
  }
  s.verts.swap(out);
}

// Split a (shaded) Stroke into renderable ribbon strips: one StyledStrip per maximal
// run of consecutive VISIBLE vertices (Freestyle StrokeRep::create, StrokeRep.cpp:
// 837-867), each built from the per-vertex left/right thickness via the array
// buildStrip. Color/opacity are taken per run from its first vertex's attribute
// (constant per chain today; a future color shader makes them per-vertex in the
// rasterizer). The EFFECTIVE opacity of a vertex is attr.alpha * surfA (style
// opacity times the sampled surface alpha); when it varies along the run the
// per-vertex values ride StyledStrip::alphas and the rasterizer interpolates,
// otherwise the constant path is kept (bit-identical opaque output).
// precedence keys the nature for the overlap sort. roundCap/roundJoin
// (--stroke-cap/--stroke-join round) switch the strip builder and append the
// cap/join arc fans; both off keeps the legacy butt/miter path byte-identical.
// capStart/capEnd suppress the round cap at the STROKE's first/last vertex
// (junction-tapered ends stay butts); interior hidden-run boundaries always
// cap as before.
void buildStrokeReps(const Stroke& s, bool roundCap, bool roundJoin,
                     bool capStart, bool capEnd, const ClipDisc* chainClips,
                     int nChainClips, std::vector<StyledStrip>& out) {
  const int precedence = s.precedence;
  const std::size_t minRun = 2;
  std::size_t runFirst = 0, vIdx = 0;  // stroke-vertex span of the current run
  std::vector<Vec2> pos;
  std::vector<float> lw, rw, av;
  std::vector<std::array<float, 3>> cv;  // per-vertex ink color (fog gradient)
  float col[3] = {0.0f, 0.0f, 0.0f}, opacity = 1.0f;
  float depthMin = 0.0f;  // min view-z over the current run
  auto flush = [&]() {
    if (pos.size() >= minRun) {
      StyledStrip ss;
      // pairSrc maps strip pairs back to backbone vertices: identity for the
      // legacy miter builder, with duplicated corner entries under round
      // joins. fanSrc does the same per arc-fan triangle.
      std::vector<std::size_t> pairSrc, fanSrc;
      if (roundJoin) {
        ss.strip = buildStripRound(pos, lw, rw, pairSrc, ss.fanPts, fanSrc);
      } else {
        ss.strip = buildStrip(pos, lw, rw);
        pairSrc.resize(pos.size());
        for (std::size_t i = 0; i < pairSrc.size(); ++i) pairSrc[i] = i;
      }
      if (roundCap)
        appendCapFans(pos, lw, rw,
                      capStart || runFirst != 0,
                      capEnd || vIdx != s.verts.size(),
                      ss.fanPts, fanSrc);
      ss.color[0] = col[0];
      ss.color[1] = col[1];
      ss.color[2] = col[2];
      ss.opacity = opacity;
      bool uniform = true;
      for (float a : av)
        if (a != av.front()) {
          uniform = false;
          break;
        }
      if (uniform) {
        ss.opacity = av.front();  // constant path (== legacy when surfA == 1)
      } else {
        // Per-vertex gradient path, gathered per strip PAIR (identity for
        // the miter builder -> exact legacy array).
        ss.alphas.reserve(pairSrc.size());
        for (std::size_t src : pairSrc) ss.alphas.push_back(av[src]);
      }
      // Per-vertex ink color varies only when the depth-fog shader ran on an
      // opaque background; a uniform run keeps the constant `color` (byte-
      // identical legacy path, and the case where fog fades alpha not color).
      bool colUniform = true;
      for (const std::array<float, 3>& c : cv)
        if (c != cv.front()) {
          colUniform = false;
          break;
        }
      if (!colUniform) {  // per-pair fog color gradient
        ss.colors.reserve(pairSrc.size());
        for (std::size_t src : pairSrc) ss.colors.push_back(cv[src]);
      }
      // Arc fan attributes: the constant alpha/color of each fan triangle's
      // backbone vertex (matches whatever the body would paint there).
      ss.fanAlpha.reserve(fanSrc.size());
      ss.fanColor.reserve(fanSrc.size());
      for (std::size_t src : fanSrc) {
        ss.fanAlpha.push_back(av[src]);
        ss.fanColor.push_back(cv[src]);
      }
      ss.precedence = precedence;
      ss.depthKey = depthMin;
      for (int c = 0; c < nChainClips; ++c) ss.clips[c] = chainClips[c];
      ss.nClips = nChainClips;
      out.push_back(std::move(ss));
    }
    pos.clear();
    lw.clear();
    rw.clear();
    av.clear();
    cv.clear();
  };
  for (std::size_t i = 0; i < s.verts.size(); ++i) {
    const StrokeVertex& v = s.verts[i];
    if (!v.visible) {
      vIdx = i;  // the run (if any) ended before this hidden vertex
      flush();
      continue;
    }
    if (pos.empty()) {  // run start: capture color/opacity, seed the depth min
      runFirst = i;
      col[0] = v.attr.color[0];
      col[1] = v.attr.color[1];
      col[2] = v.attr.color[2];
      opacity = v.attr.alpha;
      depthMin = v.vz;
    } else if (v.vz < depthMin) {
      depthMin = v.vz;
    }
    pos.push_back(v.p);
    lw.push_back(v.attr.leftThick);
    rw.push_back(v.attr.rightThick);
    av.push_back(v.attr.alpha * v.surfA);
    cv.push_back({v.attr.color[0], v.attr.color[1], v.attr.color[2]});
  }
  vIdx = s.verts.size();
  flush();
}

}  // namespace

// Shared draw stage (stroke_render.hpp): resolve one chain's ribbon style by
// style slot + section group. Table reads are identical to the retired
// applyStrokeEdges-local resolveStyle lambda keyed on nature (the mesh source
// passes natureStyleSlot(nature)); the per-nature master gates stayed at the
// source.
bool resolveStrokeStyle(const Scene& scene, const StrokeEdgeOptions& se,
                        float ssScale, int styleSlot, std::uint16_t group,
                        float& outHalf, float outColor[3], float& outOpacity) {
  if (scene.groupEdgeStyle.empty()) {
    // No section table: use the single global stroke style.
    outHalf = std::max(0.5f, 0.5f * static_cast<float>(se.thickness) * ssScale);
    outColor[0] = se.color[0];
    outColor[1] = se.color[1];
    outColor[2] = se.color[2];
    outOpacity = se.opacity;
    return true;
  }
  const std::vector<EdgeStyle>& table = scene.groupEdgeStyle;
  const EdgeStyle& es = (group < table.size()) ? table[group]
                                               : se.defaultStyle;
  const EdgeClassStyle& cs = es.cls[styleSlot];
  // A section that disables this slot's class inks nothing for it.
  if (!cs.enabled) return false;
  // width is the per-class FULL band width (FINAL px); half it and scale to
  // hi-res px by the supersample factor (>= 0.5 hi-res px).
  outHalf = std::max(0.5f, 0.5f * cs.width * ssScale);
  outColor[0] = cs.color[0];
  outColor[1] = cs.color[1];
  outColor[2] = cs.color[2];
  outOpacity = cs.opacity;
  return true;
}

// Shared draw stage (stroke_render.hpp): stylize + rasterize source-produced
// chains. This is the applyStrokeEdges back half moved verbatim -- per chain
// buildStroke -> resampleStroke -> shader stack -> buildStrokeReps, then the
// depth/precedence stable sort and the TBB row-tiled deterministic rasterize.
void renderStrokeChains(FrameResult& frame, const Scene& scene,
                        const RenderOptions& opt,
                        const std::vector<StrokeChainInput>& chains) {
  const StrokeEdgeOptions& se = opt.strokeEdges;
  const int W = frame.width, H = frame.height;
  if (W <= 0 || H <= 0) return;

  // Supersample-aware stroke geometry: thickness/resample are FINAL-px values,
  // scaled to hi-res px (see the mesh source's ssScale note).
  const float ssScale = static_cast<float>(std::max(1, opt.supersample));
  const float stepPx =
      std::max(1.0f, static_cast<float>(se.resampleStepPx) * ssScale);

  std::vector<StyledStrip> strips;
  std::vector<NodeDot> nodeDots;    // --stroke-node-dots overlay (else empty)
  std::vector<DebugPoly> debugPolys;
  for (std::size_t ci = 0; ci < chains.size(); ++ci) {
    const StrokeChainInput& in = chains[ci];
    float halfThick = 0.0f, col[3] = {0.0f, 0.0f, 0.0f}, opacity = 1.0f;
    if (!resolveStrokeStyle(scene, se, ssScale, in.styleSlot, in.group,
                            halfThick, col, opacity))
      continue;
    // DEBUG overlay (--stroke-node-dots): draw the chain as its raw
    // node-to-node polyline in a palette color instead of the styled ribbon,
    // and mark every node. Nodes are taken BEFORE the arc-length resample,
    // so they are the authored vertices, not the dense resampled ones. The
    // ribbon, its shaders and the junction taper/clip are all skipped --
    // this mode inspects the SOURCE geometry, not its stylization.
    if (se.debugNodeDots) {
      DebugPoly poly;
      for (int c = 0; c < 3; ++c) poly.col[c] = kNodePalette[ci % 8][c];
      poly.pts.reserve(in.pts.size());
      for (std::size_t k = 0; k < in.pts.size(); ++k) {
        poly.pts.push_back({in.pts[k].x, in.pts[k].y});
        nodeDots.push_back({in.pts[k].x, in.pts[k].y,
                            k == 0 || k + 1 == in.pts.size()});
      }
      debugPolys.push_back(std::move(poly));
      continue;
    }
    // VERIFICATION (--edges-only): ink every line solid, ignoring per-section
    // style opacity and the per-vertex surface alpha, so faint / alpha-
    // following lines stay clearly visible for annotating missing edges.
    if (se.edgesOnly) opacity = 1.0f;

    // Outside stroke alignment: shift the resolved width to the chain's outer
    // side, keeping the total footprint at 2*halfThick. The inner pad (up to
    // 0.5 FINAL px) covers the sub-pixel halo left where Chaikin/RDP pull the
    // backbone off the crack line, and keeps the round-cap fan radius above
    // appendArcFan's r <= kZero early-out. A hairline (halfThick == pad)
    // degenerates back to the symmetric ribbon.
    float leftHalf = halfThick, rightHalf = halfThick;
    if (in.outsideSide != 0) {
      const float pad = std::min(halfThick, 0.5f * ssScale);
      const float outer = 2.0f * halfThick - pad;
      leftHalf = in.outsideSide > 0 ? outer : pad;
      rightHalf = in.outsideSide > 0 ? pad : outer;
    }

    // Wrap the source points into the internal visibility-tagged polyline.
    std::vector<Pt2> proj;
    proj.reserve(in.pts.size());
    for (const StrokePoint& sp : in.pts) {
      Pt2 q;
      q.p = {sp.x, sp.y};
      q.vz = sp.vz;
      q.surfA = se.edgesOnly ? 1.0f : sp.alpha;
      q.visible = sp.visible;
      proj.push_back(q);
    }
    // Clipped junction ends: extend the RASTER backbone a hair past the
    // vector endpoint, so smoothing deviation of the met line cannot open
    // a sub-px seam; the end clip culls any overshoot. The vector data
    // (the node overlay, future exports) keeps the exact on-line endpoint.
    const float padExt = (0.5f + se.screenSimplifyPx) * ssScale;
    if (in.clipStart.enabled && proj.size() >= 2) {
      const Vec2 d = proj[0].p - proj[1].p;
      const float l = norm2(d);
      if (l > kZero) {
        Pt2 q = proj.front();
        q.p = q.p + d * (padExt / l);
        proj.insert(proj.begin(), q);
      }
    }
    if (in.clipEnd.enabled && proj.size() >= 2) {
      const Vec2 d = proj[proj.size() - 1].p - proj[proj.size() - 2].p;
      const float l = norm2(d);
      if (l > kZero) {
        Pt2 q = proj.back();
        q.p = q.p + d * (padExt / l);
        proj.push_back(q);
      }
    }

    // STROKE LAYER: wrap the visibility-tagged polyline as a parametric Stroke
    // (buildStroke), arc-length resample it (resampleStroke), then emit one
    // ribbon per maximal VISIBLE run.
    StrokeAttribute defAttr;
    defAttr.leftThick = leftHalf;
    defAttr.rightThick = rightHalf;
    defAttr.color[0] = col[0];
    defAttr.color[1] = col[1];
    defAttr.color[2] = col[2];
    defAttr.alpha = opacity;
    Stroke stroke =
        buildStroke(proj, defAttr, static_cast<int>(ci), in.precedence);
    resampleStroke(stroke, stepPx);

    // STYLIZATION: run the per-vertex stroke shaders (Freestyle
    // StrokeShader::shade). The constant width/color shaders reproduce the
    // resolved per-section default attribute, so output is byte-identical; a
    // real calligraphic/depth-cue/taper/noise shader plugs in here to make
    // L != R or color vary along u.
    std::vector<std::unique_ptr<StrokeShader>> shaders;
    if (se.smooth)  // Freestyle anisotropic curvature-flow smoothing
      shaders.push_back(std::make_unique<AnisoSmoothingShader>(
          /*nbIter=*/200, /*factorPoint=*/0.0f, /*factorCurvature=*/0.4f,
          /*factorCurvatureDiff=*/0.3f, /*anisoPoint=*/0.0f,
          /*anisoNormal=*/0.08f, /*anisoCurvature=*/0.08f,
          /*carricature=*/1.0f));
    shaders.push_back(
        std::make_unique<ConstantThicknessShader>(leftHalf, rightHalf));
    shaders.push_back(std::make_unique<ConstantColorShader>(col, opacity));
    if (se.taper)  // demo f(u) shader: taper width toward stroke ends
      shaders.push_back(std::make_unique<TaperShader>(0.5f, 0.15f));
    // Outside alignment: re-center the offset ribbon over one stroke-width
    // (2 * halfThick hi-res px) around flagged junction ends and interior
    // folds.
    if (in.outsideSide != 0)
      shaders.push_back(std::make_unique<AlignRecenterShader>(
          in.taperStart, in.taperEnd, 2.0f * halfThick));
    // Depth fog LAST (after the color shader stamps the ink color/alpha), so
    // distant edge lines recede into the fog exactly like the 3D surface under
    // them (the surface was fogged in the pipeline before this pass). Gated on
    // fog.enabled so the no-fog render stays byte-identical; skipped in the
    // --edges-only verification mode, which forces every line fully opaque to
    // keep faint/distant edges visible for annotation.
    if (scene.fog.enabled && !se.edgesOnly)
      shaders.push_back(std::make_unique<FogShader>(
          scene.fog, opt.transparentBackground));
    for (const std::unique_ptr<StrokeShader>& sh : shaders) sh->shade(stroke);

    // Build the variable-width ribbon strips (one per maximal visible run).
    // A junction-tapered or clipped end draws no round cap (appendCapFans);
    // the end clips ride every strip of the chain and cull ink per pixel at
    // rasterization time.
    ClipDisc chainClips[2];
    int nChainClips = 0;
    for (const StrokeEndClip* ec : {&in.clipStart, &in.clipEnd}) {
      if (!ec->enabled || ec->radius <= 0.0f) continue;
      chainClips[nChainClips].px = ec->px;
      chainClips[nChainClips].py = ec->py;
      chainClips[nChainClips].nx = ec->nx;
      chainClips[nChainClips].ny = ec->ny;
      chainClips[nChainClips].r2 = ec->radius * ec->radius;
      ++nChainClips;
    }
    buildStrokeReps(
        stroke, se.roundCap, se.roundJoin,
        !(in.outsideSide != 0 && (in.taperStart || in.clipStart.enabled)),
        !(in.outsideSide != 0 && (in.taperEnd || in.clipEnd.enabled)),
        chainClips, nChainClips, strips);
  }

  if (strips.empty() && debugPolys.empty()) return;

  // Stable-sort strips for compositing. compositeOver paints in array order, so
  // the LAST strip is on top (painter's algorithm). Primary key = DEPTH: sort
  // FARTHER first (descending depthKey = view-z) so the NEARER stroke is last
  // == on top (Freestyle Operators::sort + pyZBP1D, resolved to umbreon's
  // `over` blend direction). Tie-break by class PRECEDENCE. Stable so full ties
  // keep build (chain) order -> deterministic; depthKey is computed
  // single-threaded, so the result is independent of TBB scheduling.
  std::stable_sort(strips.begin(), strips.end(),
                   [](const StyledStrip& a, const StyledStrip& b) {
                     if (a.depthKey != b.depthKey) return a.depthKey > b.depthKey;
                     return a.precedence < b.precedence;
                   });

  // Composite all strips over frame.color, row-tiled with TBB. Each tile
  // rasterizes EVERY strip in PRECEDENCE order but only the rows in its range,
  // so the result is independent of tile boundaries / thread scheduling
  // (deterministic).
  // DEBUG node overlay (--stroke-node-dots): hairline polylines first, then
  // interior nodes, then END nodes on top -- so a vertex shared by two runs
  // reads as an end, and the two runs' palette colors show the split.
  // Radii are FINAL px scaled to hi-res so the marks survive the box
  // downsample.
  const float rLine = 0.5f * ssScale;
  const float rInterior = 1.0f * ssScale, rEnd = 1.6f * ssScale;
  static constexpr float kInteriorCol[3] = {0.0f, 0.85f, 0.0f};  // green
  static constexpr float kEndCol[3] = {1.0f, 0.0f, 0.0f};        // red

  tbb::parallel_for(
      tbb::blocked_range<int>(0, H),
      [&](const tbb::blocked_range<int>& rows) {
        const int rb = rows.begin(), re = rows.end();
        for (const StyledStrip& ss : strips)
          rasterizeStrip(frame.color, W, rb, re, ss);
        for (const DebugPoly& dp : debugPolys)
          for (std::size_t k = 0; k + 1 < dp.pts.size(); ++k)
            drawThinSegment(frame.color, W, rb, re, dp.pts[k], dp.pts[k + 1],
                            rLine, dp.col);
        for (const NodeDot& nd : nodeDots)
          if (!nd.endpoint)
            fillDisc(frame.color, W, rb, re, nd.x, nd.y, rInterior,
                     kInteriorCol);
        for (const NodeDot& nd : nodeDots)
          if (nd.endpoint)
            fillDisc(frame.color, W, rb, re, nd.x, nd.y, rEnd, kEndCol);
      });
}

}  // namespace umbreon
