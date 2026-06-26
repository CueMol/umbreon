#include "render/stroke_edges.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
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
  int precedence = 0;     // nature tie-break key (lower paints first)
  float depthKey = 0.0f;  // min view-z over the run (FARTHER = larger); primary sort
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

// Constant symmetric-width overload (compat for buildRibbonStrips + its unit tests):
// fills L==R==halfThick and calls the array form, which reduces to the old
// constant-width expressions exactly.
Strip buildStrip(const std::vector<Vec2>& bb, float halfThick) {
  return buildStrip(bb, std::vector<float>(bb.size(), halfThick),
                    std::vector<float>(bb.size(), halfThick));
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
// StrokeVertex, Stroke.h:310-458). p/vz/visible mirror Pt2; `ca` is the 2D arc
// length from the stroke start and `u = ca/length2d` in [0,1] is the shader
// parameter. `visible` stays on the vertex (not in attr) to match the run-split.
struct StrokeVertex {
  Vec2 p;
  float vz = 0.0f;
  bool visible = true;
  float u = 0.0f, ca = 0.0f;
  StrokeAttribute attr;
};

// A chained, parametric stroke (Freestyle Stroke, Stroke.h:483-858).
struct Stroke {
  std::vector<StrokeVertex> verts;
  float length2d = 0.0f;
  int chainIdx = 0;
  EdgeNature nature = EdgeNature::Silhouette;
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

// Constant symmetric half-width (Freestyle ConstantThicknessShader,
// BasicStrokeShaders.cpp:40-58).
struct ConstantThicknessShader : StrokeShader {
  float halfThick;
  explicit ConstantThicknessShader(float h) : halfThick(h) {}
  int shade(Stroke& s) const override {
    for (StrokeVertex& v : s.verts) {
      v.attr.leftThick = halfThick;
      v.attr.rightThick = halfThick;
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
                   int chainIdx, EdgeNature nat) {
  Stroke s;
  s.chainIdx = chainIdx;
  s.nature = nat;
  s.verts.reserve(proj.size());
  float ca = 0.0f;
  for (std::size_t i = 0; i < proj.size(); ++i) {
    if (i > 0) ca += norm2(proj[i].p - proj[i - 1].p);
    StrokeVertex v;
    v.p = proj[i].p;
    v.vz = proj[i].vz;
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
// rasterizer). precedence keys the nature for the overlap sort.
void buildStrokeReps(const Stroke& s, std::vector<StyledStrip>& out) {
  const int precedence = naturePrecedence(s.nature);
  const std::size_t minRun = 2;
  std::vector<Vec2> pos;
  std::vector<float> lw, rw;
  float col[3] = {0.0f, 0.0f, 0.0f}, opacity = 1.0f;
  float depthMin = 0.0f;  // min view-z over the current run
  auto flush = [&]() {
    if (pos.size() >= minRun) {
      StyledStrip ss;
      ss.strip = buildStrip(pos, lw, rw);
      ss.color[0] = col[0];
      ss.color[1] = col[1];
      ss.color[2] = col[2];
      ss.opacity = opacity;
      ss.precedence = precedence;
      ss.depthKey = depthMin;
      out.push_back(std::move(ss));
    }
    pos.clear();
    lw.clear();
    rw.clear();
  };
  for (const StrokeVertex& v : s.verts) {
    if (!v.visible) {
      flush();
      continue;
    }
    if (pos.empty()) {  // run start: capture color/opacity, seed the depth min
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
  }
  flush();
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

    // Per-SEGMENT exclude sets + nature for the QI ray and the crossing pass:
    // the narrow two-incident-face set (segFaces, fallback), the EXPANDED 1-ring
    // exclude set (segExclude, preferred by the QI), and the nature.
    ch.segFaces.assign(ch.segs.size(), std::array<int, 2>{-1, -1});
    ch.segExclude.assign(ch.segs.size(), {});
    ch.segNature.assign(ch.segs.size(), EdgeNature::Silhouette);
    for (std::size_t k = 0; k < ch.segs.size(); ++k) {
      const FeatureSeg& s = segs[static_cast<std::size_t>(ch.segs[k])];
      ch.segFaces[k] = {s.face0, s.face1};
      ch.segExclude[k] = s.excludeFaces;
      ch.segNature[k] = s.nature;
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

namespace {

// Maximum 3D length a QI segment may span before it is subdivided into interior
// samples (Freestyle samples the FEdge center; a long segment whose MIDDLE dips
// behind a body would leak if tested at its center alone). World units, scaled
// off the segment so the count stays small.
constexpr int kQiMaxInteriorSamples = 3;

// QI interior sampling for a crossing-sub-span (subSpanHidden): one ray per this
// many hi-res px of the sub-span's screen-space length, hidden iff ANY sample is
// occluded. A sub-span that crosses no 2D feature edge is a single piece that can
// still pass FULLY behind a SMOOTH (featureless) occluder -- e.g. a helix back-rim
// behind the tube's front wall -- with no crossing to split it; a lone midpoint
// ray grazes past such an occluder and leaks the hidden line. Real partial-
// occlusion boundaries coincide with the occluder's silhouette, itself a drawn
// feature edge that already split the span, so ANY-hidden never over-hides a span
// that genuinely straddles a visible/hidden boundary.
constexpr float kQiSubSpanSampleStepPx = 4.0f;  // hi-res px between QI samples
constexpr int kQiSubSpanMaxSamples = 64;        // cost cap for a long sub-span

// QI occlusion of one 3D point P with the given per-segment exclude faces. Casts
// a ray P->eye (persp) or P->far-camera-ward (ortho); true == an un-excluded
// solid surface lies strictly between P and the eye (qi > 0 => hidden).
//
// For ORTHO the eye is at infinity, so the ray travels along the reverse view axis
// to a far camera-ward target. That target's distance MUST stay physical (scaled
// to the camera distance), NOT a fixed huge constant: occluded() trims its near
// end by eps*rayLength as a self-leave guard, so a 1e4 reach turned that into a
// ~1 world-unit dead zone that swallowed a genuine nearby occluder -- a back fold
// only ~0.5 in front of its own grazing silhouette read as VISIBLE and leaked onto
// the front surface (persp never hit this: its ray length is the real ~camera
// distance). Push to twice the camera distance so the ray still clears every
// occluder between P and the eye while keeping the near-trim a sane ~0.04.
bool qiOccludedAt(const Vec3& P, const ScreenProj& sp,
                  const OcclusionQuery& occluded, const int* faces, int nFaces) {
  Vec3 target = sp.pos;
  if (sp.ortho) {
    const float reach = 2.0f * length(sp.pos - P);
    target = P + (sp.dir * -1.0f) * reach;
  }
  return occluded(P, target, faces, nFaces);
}

// Build segment k's QI exclude-face set, in preference order: the EXPANDED 1-ring
// set (segExclude); else the segment's two incident faces (segFaces); else the
// union of the endpoints' incident-face hubs (incidentFaces, used by unit tests).
std::vector<int> buildSegmentExclude(const EdgeChain& chain, std::size_t k,
                                     std::size_t nSeg, std::size_t nPts) {
  std::vector<int> buf;
  buf.reserve(32);
  auto pushFace = [&](int f) {
    if (f < 0) return;
    for (int g : buf)
      if (g == f) return;
    buf.push_back(f);
  };
  const bool haveExpanded = chain.segExclude.size() == nSeg;
  const bool havePerSeg = chain.segFaces.size() == nSeg;
  const bool haveVertFaces = chain.incidentFaces.size() == nPts;
  if (haveExpanded && !chain.segExclude[k].empty()) {
    for (int f : chain.segExclude[k]) pushFace(f);
  } else if (havePerSeg) {
    pushFace(chain.segFaces[k][0]);
    pushFace(chain.segFaces[k][1]);
  } else if (haveVertFaces) {
    for (int f : chain.incidentFaces[k]) pushFace(f);
    for (int f : chain.incidentFaces[k + 1]) pushFace(f);
  }
  return buf;
}

}  // namespace

std::vector<char> computeChainVisibility(const EdgeChain& chain,
                                         const ScreenProj& sp,
                                         const OcclusionQuery& occluded) {
  const std::size_t nPts = chain.pts.size();
  std::vector<char> visible(nPts, 1);
  if (!occluded || nPts < 2) return visible;  // no live BVH: all visible

  const std::size_t nSeg = nPts - 1;  // backbone has pts.size()-1 segments

  // (A) Per-SEGMENT QI: cast from the segment center (and interior samples for
  // long segments -- Freestyle FEdge::center3d) with that segment's OWN incident
  // faces excluded. A segment is hidden iff ANY interior sample is occluded by a
  // non-excluded surface.
  std::vector<char> segVisible(nSeg, 1);
  for (std::size_t k = 0; k < nSeg; ++k) {
    const Vec3& A = chain.pts[k];
    const Vec3& B = chain.pts[k + 1];
    const std::vector<int> buf = buildSegmentExclude(chain, k, nSeg, nPts);
    const int nFaces = static_cast<int>(buf.size());
    const int* faces = nFaces > 0 ? buf.data() : nullptr;

    // Number of interior samples scaled by the segment's screen footprint is not
    // available here cheaply; subdivide by a fixed small count so a mid-segment
    // dip behind a body is caught. Samples at the center and at 1/4, 3/4.
    bool hidden = false;
    const int samples = kQiMaxInteriorSamples;
    for (int sIdx = 0; sIdx < samples && !hidden; ++sIdx) {
      // t in (0,1), centered: 0.5 for 1 sample; {0.25,0.5,0.75} for 3.
      const float t = (static_cast<float>(sIdx) + 0.5f) /
                      static_cast<float>(samples);
      const Vec3 Pm = {A.x + (B.x - A.x) * t, A.y + (B.y - A.y) * t,
                       A.z + (B.z - A.z) * t};
      if (qiOccludedAt(Pm, sp, occluded, faces, nFaces)) hidden = true;
    }
    segVisible[k] = hidden ? 0 : 1;
  }

  // Derive per-backbone-vertex QI visibility: vertex k is visible iff its
  // adjacent segments (k-1 and k) are visible. Endpoints depend on their single
  // adjacent segment (the closed-loop seam wraps).
  for (std::size_t i = 0; i < nPts; ++i) {
    bool v = true;
    if (i > 0) v = v && segVisible[i - 1] != 0;
    if (i < nSeg) v = v && segVisible[i] != 0;
    if (chain.closed) {
      // Seam vertices border both the first and last segment.
      if (i == 0) v = v && segVisible[nSeg - 1] != 0;
      if (i == nPts - 1) v = v && segVisible[0] != 0;
    }
    visible[i] = v ? 1 : 0;
  }
  return visible;
}

namespace {

// A backbone SUB-SPAN of a chain: backbone segment `seg`, parameter range [a,b] in
// [0,1]. The 2D image-space crossings (Freestyle TVertices, computeEdgeCrossings)
// split each segment at their parameters; the sub-spans are the units whose
// visibility is constant, so an occluded line terminates EXACTLY at a crossing
// (the occluder's silhouette) rather than at a per-sample ray-flip boundary.
struct SubSpan {
  int seg = 0;
  float a = 0.0f, b = 1.0f;
};

// Split one chain (nSeg backbone segments) into sub-spans at its 2D crossings.
// `segCross[k]` is the (unsorted) list of crossing parameters on segment k; each
// in (0,1) becomes a split point. A segment with no crossing yields one full
// sub-span [0,1]. Sub-spans are returned in backbone order (segment-major).
std::vector<SubSpan> splitChainAtCrossings(
    std::size_t nSeg, const std::vector<std::vector<float>>& segCross) {
  std::vector<SubSpan> spans;
  for (std::size_t k = 0; k < nSeg; ++k) {
    std::vector<float> ts;
    if (k < segCross.size()) {
      for (float t : segCross[k])
        if (t > 0.0f && t < 1.0f) ts.push_back(t);
      std::sort(ts.begin(), ts.end());
    }
    float a = 0.0f;
    for (float t : ts) {
      if (t - a > kZero) spans.push_back({static_cast<int>(k), a, t});
      a = t;
    }
    spans.push_back({static_cast<int>(k), a, 1.0f});
  }
  return spans;
}

// Freestyle CUSP detection (ViewMapBuilder::computeCusps). Along a SMOOTH
// SILHOUETTE the orientation o = normalize(edgeTangent x surfaceNormal) . viewDir
// flips sign where the contour folds from front-facing to back-facing. Freestyle
// splits the ViewEdge at those points (Nature::CUSP) so the hidden back-fold is a
// ViewEdge of its OWN and is voted hidden -- WITHOUT a cusp split a fold that hides
// without crossing any drawn line shares one ViewEdge with its two visible limbs,
// the limbs out-vote it and the occluded back silhouette leaks over the front.
// Returns one flag per backbone vertex (size nPts); a set flag marks a cusp vertex
// (a ViewEdge boundary). The deadband 0.1 is Freestyle's literal hysteresis.
std::vector<char> computeCuspVerts(const EdgeChain& chain, const FeatureMesh& fm,
                                   const Mesh& mesh, const ScreenProj& sp) {
  const std::size_t nPts = chain.pts.size();
  std::vector<char> cusp(nPts, 0);
  const std::size_t nSeg = nPts >= 1 ? nPts - 1 : 0;
  if (nSeg < 2 || chain.segs.size() != nSeg) return cusp;

  // Average shading normal of the FeatureSeg's incident face0 triangle (the mesh is
  // de-indexed: normals[3*tri + {0,1,2}]). The smooth silhouette was extracted from
  // these shading normals, so they define the same n.v==0 contour.
  auto faceNormal = [&](int segIdx) -> Vec3 {
    if (segIdx < 0 || static_cast<std::size_t>(segIdx) >= fm.segs.size())
      return {0.0f, 0.0f, 0.0f};
    const int f = fm.segs[static_cast<std::size_t>(segIdx)].face0;
    if (f < 0) return {0.0f, 0.0f, 0.0f};
    const std::size_t b = static_cast<std::size_t>(f) * 3;
    if (b + 2 >= mesh.normals.size()) return {0.0f, 0.0f, 0.0f};
    return {mesh.normals[b].x + mesh.normals[b + 1].x + mesh.normals[b + 2].x,
            mesh.normals[b].y + mesh.normals[b + 1].y + mesh.normals[b + 2].y,
            mesh.normals[b].z + mesh.normals[b + 1].z + mesh.normals[b + 2].z};
  };
  auto orient = [&](std::size_t k) -> float {
    const Vec3& A = chain.pts[k];
    const Vec3& B = chain.pts[k + 1];
    Vec3 AB = {B.x - A.x, B.y - A.y, B.z - A.z};
    const float abl = std::sqrt(AB.x * AB.x + AB.y * AB.y + AB.z * AB.z);
    if (abl <= kZero) return 0.0f;
    AB = {AB.x / abl, AB.y / abl, AB.z / abl};
    const Vec3 N = faceNormal(chain.segs[k]);
    Vec3 cP = {AB.y * N.z - AB.z * N.y, AB.z * N.x - AB.x * N.z,
               AB.x * N.y - AB.y * N.x};
    const float cl = std::sqrt(cP.x * cP.x + cP.y * cP.y + cP.z * cP.z);
    if (cl <= kZero) return 0.0f;
    cP = {cP.x / cl, cP.y / cl, cP.z / cl};
    Vec3 vv;
    if (sp.ortho) {
      vv = sp.dir;  // eye->scene view axis (constant for ortho)
    } else {
      const Vec3 m = {0.5f * (A.x + B.x), 0.5f * (A.y + B.y), 0.5f * (A.z + B.z)};
      vv = {m.x - sp.pos.x, m.y - sp.pos.y, m.z - sp.pos.z};
      const float vl = std::sqrt(vv.x * vv.x + vv.y * vv.y + vv.z * vv.z);
      if (vl <= kZero) return 0.0f;
      vv = {vv.x / vl, vv.y / vl, vv.z / vl};
    }
    return cP.x * vv.x + cP.y * vv.y + cP.z * vv.z;
  };

  bool haveState = false, positive = true;
  for (std::size_t k = 0; k < nSeg; ++k) {
    const float o = orient(k);
    if (o == 0.0f) continue;  // degenerate (no normal/tangent): hold state
    if (!haveState) {
      positive = (o > 0.0f);
      haveState = true;
      continue;
    }
    if (positive && o < -0.1f) {
      positive = false;
      cusp[k] = 1;  // vertex k (between seg k-1 and seg k) is a cusp
    } else if (!positive && o > 0.1f) {
      positive = true;
      cusp[k] = 1;
    }
  }
  return cusp;
}

// VIEW-EDGE Quantitative-Invisibility (Freestyle ComputeRayCastingVisibility): a
// single visibility value is decided per VIEW EDGE -- a maximal run of sub-spans
// NOT separated by a 2D crossing (Freestyle TVertex) or a CUSP -- by MAJORITY over
// QI rays sampled along the whole run, then applied uniformly to every sub-span.
//
// Why per-ViewEdge, not per-sub-span: Freestyle does NOT ray-cast each fragment
// independently. It samples several points along one ViewEdge, takes the MAJORITY
// QI, and assigns it to the entire ViewEdge -- because the true visibility is
// uniform between two TVertices, so the multiple samples only DENOISE one
// uniform-truth signal. umbreon's old per-sub-span single test treated every
// fragment as independently variable, so one unreliable grazing ray flipped one
// fragment and punched a HOLE in a continuous object outline. The majority makes a
// minority of spurious self-occlusion samples lose, so the outline stays solid;
// a genuinely occluded run still votes hidden as a whole because it is bounded by
// crossings at its emergence points (where the occluder's silhouette crosses it)
// and so forms its OWN ViewEdge. (A fold that hides WITHOUT any 2D crossing needs
// a Freestyle CUSP split to bound it; that is a separate step -- computeCusps.)
//
// ViewEdge boundaries: splitChainAtCrossings splits a segment at each crossing, so
// a boundary between consecutive sub-spans is a crossing iff the earlier ends
// before its segment end (b<1) or the later starts after its segment start (a>0);
// a clean seg->seg transition (b==1, a==0) stays in the SAME ViewEdge UNLESS the
// shared backbone vertex is a CUSP (`cuspVert`, computeCuspVerts). Returns one flag
// per sub-span. Empty `occluded` (no live BVH) => all visible.
std::vector<char> subSpanHidden(const EdgeChain& chain,
                                const std::vector<SubSpan>& spans,
                                const ScreenProj& sp,
                                const OcclusionQuery& occluded,
                                const std::vector<char>& cuspVert) {
  std::vector<char> hidden(spans.size(), 0);
  const std::size_t nPts = chain.pts.size();
  if (!occluded || nPts < 2 || spans.empty()) return hidden;
  const std::size_t nSeg = nPts - 1;

  // Pool one sub-span's QI samples (length-weighted along its screen extent) into a
  // running occluded/total tally -- Freestyle samples ~one ray per tessellation
  // edge, i.e. proportional to length.
  auto tallySpan = [&](const SubSpan& s, int& occ, int& tot) {
    if (s.b - s.a <= kZero) return;  // degenerate sub-span: no vote
    const std::size_t k = static_cast<std::size_t>(s.seg);
    const Vec3& A = chain.pts[k];
    const Vec3& B = chain.pts[k + 1];
    const std::vector<int> buf = buildSegmentExclude(chain, k, nSeg, nPts);
    const int nF = static_cast<int>(buf.size());
    const int* fp = nF > 0 ? buf.data() : nullptr;
    float ax, ay, avz, bx, by, bvz;
    const bool aok = worldToScreen(sp, A, ax, ay, avz);
    const bool bok = worldToScreen(sp, B, bx, by, bvz);
    int samples = 1;
    if (aok && bok) {
      const float dx = (bx - ax) * (s.b - s.a), dy = (by - ay) * (s.b - s.a);
      const float spanPx = std::sqrt(dx * dx + dy * dy);
      samples = static_cast<int>(spanPx / kQiSubSpanSampleStepPx) + 1;
      if (samples < 1) samples = 1;
      if (samples > kQiSubSpanMaxSamples) samples = kQiSubSpanMaxSamples;
    }
    for (int sIdx = 0; sIdx < samples; ++sIdx) {
      const float frac =
          (static_cast<float>(sIdx) + 0.5f) / static_cast<float>(samples);
      const float tt = s.a + (s.b - s.a) * frac;
      const Vec3 P = {A.x + (B.x - A.x) * tt, A.y + (B.y - A.y) * tt,
                      A.z + (B.z - A.z) * tt};
      if (qiOccludedAt(P, sp, occluded, fp, nF)) ++occ;
      ++tot;
    }
  };

  // Group sub-spans into ViewEdges: a new group starts after each crossing OR cusp.
  auto isCusp = [&](std::size_t vert) -> bool {
    return vert < cuspVert.size() && cuspVert[vert] != 0;
  };
  std::vector<std::size_t> gstart;
  gstart.push_back(0);
  for (std::size_t i = 0; i + 1 < spans.size(); ++i) {
    const bool crossing =
        (spans[i].b < 1.0f - kZero) || (spans[i + 1].a > kZero);
    // A clean seg->seg transition still breaks the ViewEdge at a cusp vertex (the
    // vertex ending sub-span i, i.e. spans[i].seg + 1).
    const bool cuspBreak = (spans[i].b >= 1.0f - kZero) &&
                           isCusp(static_cast<std::size_t>(spans[i].seg) + 1);
    if (crossing || cuspBreak) gstart.push_back(i + 1);
  }
  const std::size_t nG = gstart.size();
  auto groupEnd = [&](std::size_t g) -> std::size_t {
    return (g + 1 < nG) ? gstart[g + 1] - 1 : spans.size() - 1;
  };
  auto assignRange = [&](std::size_t a, std::size_t b, char h) {
    for (std::size_t i = a; i <= b; ++i) hidden[i] = h;
  };

  // A CLOSED loop whose seam (last span end <-> first span start) is not a crossing
  // makes the first and last groups one ViewEdge; vote them together so the seam is
  // not an artificial split.
  bool mergeSeam = false;
  if (chain.closed && nG >= 2) {
    const bool seamCrossing =
        (spans.back().b < 1.0f - kZero) || (spans.front().a > kZero);
    mergeSeam = !seamCrossing && !isCusp(0);  // seam vertex is index 0
  }

  std::size_t gBegin = 0, gStop = nG;
  if (mergeSeam) {
    int occ = 0, tot = 0;
    const std::size_t firstEnd = groupEnd(0);
    const std::size_t lastBeg = gstart[nG - 1];
    for (std::size_t i = 0; i <= firstEnd; ++i) tallySpan(spans[i], occ, tot);
    for (std::size_t i = lastBeg; i < spans.size(); ++i)
      tallySpan(spans[i], occ, tot);
    const char h = (tot > 0 && occ * 2 > tot) ? 1 : 0;
    assignRange(0, firstEnd, h);
    assignRange(lastBeg, spans.size() - 1, h);
    gBegin = 1;       // first group already voted (merged with last)
    gStop = nG - 1;   // last group already voted
  }
  for (std::size_t g = gBegin; g < gStop; ++g) {
    const std::size_t a = gstart[g], b = groupEnd(g);
    int occ = 0, tot = 0;
    for (std::size_t i = a; i <= b; ++i) tallySpan(spans[i], occ, tot);
    const char h = (tot > 0 && occ * 2 > tot) ? 1 : 0;  // majority occluded
    assignRange(a, b, h);
  }
  return hidden;
}

// Project a chain's sub-spans to the screen as a visibility-tagged Pt2 polyline.
// Each NODE (backbone vertex or crossing point) is emitted EXACTLY ONCE -- the
// start of each sub-span carrying that sub-span's visibility, plus the final end
// node. resampleChain then assigns each sub-span the visibility of its start node,
// so a visible->hidden transition ends the visible run at the crossing's node and
// a hidden->visible transition resumes the run at the crossing's node. Emitting
// each node once (rather than doubling sub-span endpoints) is essential: a
// coincident pair would give buildStrip a zero-length segment -> a degenerate
// (NaN-normal) strip quad that drops out as a gap, dashing the line. A vertex
// behind the eye breaks the run (emitted hidden at the last good position).
std::vector<Pt2> projectChainSubSpans(const EdgeChain& chain,
                                      const std::vector<SubSpan>& spans,
                                      const std::vector<char>& hidden,
                                      const ScreenProj& sp) {
  std::vector<Pt2> proj;
  const std::size_t nPts = chain.pts.size();
  if (nPts < 2) return proj;
  // Project every backbone vertex once; sub-span endpoints interpolate these.
  struct PV {
    float x = 0.0f, y = 0.0f, vz = 0.0f;
    bool ok = false;
  };
  std::vector<PV> pv(nPts);
  for (std::size_t i = 0; i < nPts; ++i)
    pv[i].ok = worldToScreen(sp, chain.pts[i], pv[i].x, pv[i].y, pv[i].vz);

  proj.reserve(spans.size() * 2);
  Vec2 prev{0.0f, 0.0f};
  bool havePrev = false;
  auto emit = [&](int seg, float t, bool vis) {
    const PV& a = pv[static_cast<std::size_t>(seg)];
    const PV& b = pv[static_cast<std::size_t>(seg) + 1];
    Pt2 q;
    if (a.ok && b.ok) {
      q.p = {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
      q.vz = a.vz + (b.vz - a.vz) * t;
      q.visible = vis;
      prev = q.p;
      havePrev = true;
    } else {
      // Behind the eye: break the run without drawing toward an invalid point.
      q.p = havePrev ? prev : Vec2{0.0f, 0.0f};
      q.visible = false;
    }
    proj.push_back(q);
  };

  for (std::size_t i = 0; i < spans.size(); ++i)
    emit(spans[i].seg, spans[i].a, hidden[i] == 0);
  emit(spans.back().seg, spans.back().b, hidden.back() == 0);
  return proj;
}

}  // namespace

namespace {

// One backbone segment projected to screen for the crossing pass: 2D endpoints,
// their linear view-z, the owning chain/segment, the per-endpoint projection-OK
// flag, and the nature (for silhouette_binary_rule).
struct ProjSeg {
  Vec2 a, b;
  float za = 0.0f, zb = 0.0f;
  int chainIdx = 0, segIdx = 0;
  bool aOk = false, bOk = false;
  bool silhouetteOrBorder = false;  // SILHOUETTE or BORDER nature
};

inline float lerpf(float x, float y, float t) { return x + (y - x) * t; }

}  // namespace

std::vector<EdgeCrossing> computeEdgeCrossings(
    const std::vector<EdgeChain>& chains, const ScreenProj& sp, float zTol) {
  std::vector<EdgeCrossing> out;

  // (B.1) Project every backbone segment of every chain to 2D once.
  std::vector<ProjSeg> segs;
  for (std::size_t ci = 0; ci < chains.size(); ++ci) {
    const EdgeChain& ch = chains[ci];
    if (ch.pts.size() < 2) continue;
    const std::size_t nSeg = ch.pts.size() - 1;
    const bool haveNat = ch.segNature.size() == nSeg;
    for (std::size_t k = 0; k < nSeg; ++k) {
      ProjSeg ps;
      ps.chainIdx = static_cast<int>(ci);
      ps.segIdx = static_cast<int>(k);
      float ax, ay, av, bx, by, bv;
      ps.aOk = worldToScreen(sp, ch.pts[k], ax, ay, av);
      ps.bOk = worldToScreen(sp, ch.pts[k + 1], bx, by, bv);
      ps.a = {ax, ay};
      ps.b = {bx, by};
      ps.za = av;
      ps.zb = bv;
      const EdgeNature nat = haveNat ? ch.segNature[k] : EdgeNature::Silhouette;
      ps.silhouetteOrBorder =
          (nat == EdgeNature::Silhouette) || (nat == EdgeNature::Border);
      segs.push_back(ps);
    }
  }

  // (B.2) Pairwise crossings (brute force O(N^2); correctness first). A uniform
  // screen-grid bucketing is the Freestyle-sweep-line-equivalent speedup.
  const std::size_t N = segs.size();
  for (std::size_t i = 0; i < N; ++i) {
    const ProjSeg& s1 = segs[i];
    if (!s1.aOk || !s1.bOk) continue;  // partially behind the eye: no crossing
    for (std::size_t j = i + 1; j < N; ++j) {
      const ProjSeg& s2 = segs[j];
      if (!s2.aOk || !s2.bOk) continue;

      // silhouette_binary_rule: at least one SILHOUETTE/BORDER edge.
      if (!s1.silhouetteOrBorder && !s2.silhouetteOrBorder) continue;

      // Skip chain-adjacent / shared-node pairs within the same chain (a chain's
      // consecutive segments meet by construction; they must not self-hide). Two
      // segments of the SAME chain that share a backbone vertex are adjacent.
      if (s1.chainIdx == s2.chainIdx) {
        const int d = s1.segIdx - s2.segIdx;
        if (d == 1 || d == -1) continue;  // consecutive: shared node
      }

      // 2D intersection of the infinite lines; reject if outside either segment.
      Vec2 X;
      if (!intersect2dLine2dLine(s1.a, s1.b, s2.a, s2.b, X)) continue;
      // Parameter of X along each segment (project onto the segment direction).
      auto paramOf = [](const Vec2& a, const Vec2& b, const Vec2& x) -> float {
        const Vec2 d = b - a;
        const float l2 = dot2(d, d);
        if (l2 <= kZero) return -1.0f;
        return dot2(x - a, d) / l2;
      };
      const float t1 = paramOf(s1.a, s1.b, X);
      const float t2 = paramOf(s2.a, s2.b, X);
      if (t1 <= 0.0f || t1 >= 1.0f || t2 <= 0.0f || t2 >= 1.0f) continue;

      // Depth order at the crossing pixel: interpolate each segment's view-z.
      const float z1 = lerpf(s1.za, s1.zb, t1);
      const float z2 = lerpf(s2.za, s2.zb, t2);
      if (std::fabs(z1 - z2) <= zTol) continue;  // true junction: hide neither

      // Hide the FARTHER (larger view-z) segment at its crossing parameter.
      if (z1 > z2)
        out.push_back({s1.chainIdx, s1.segIdx, t1});
      else
        out.push_back({s2.chainIdx, s2.segIdx, t2});
    }
  }
  return out;
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

  // Stage B (image-space crossings) runs ONCE across ALL chains, BEFORE per-chain
  // visibility is finalized (Freestyle ComputeIntersections precedes
  // ComputeEdgesVisibility). zTol is a small view-z slack: a near-equal-depth
  // crossing is a true junction and hides neither side. Derived off the mean
  // triangle edge so it scales with the model; floored so a flat scene still has
  // a usable tolerance.
  const float zTol = std::max(1.0e-4f, 0.02f * fm.meanEdge);
  const std::vector<EdgeCrossing> crossings = computeEdgeCrossings(chains, sp, zTol);
  // Group crossings by chain -> per backbone-segment list of crossing params.
  std::vector<std::vector<std::vector<float>>> chainSegCross(chains.size());
  for (std::size_t ci = 0; ci < chains.size(); ++ci) {
    const std::size_t nSeg =
        chains[ci].pts.size() >= 2 ? chains[ci].pts.size() - 1 : 0;
    chainSegCross[ci].assign(nSeg, {});
  }
  for (const EdgeCrossing& c : crossings) {
    if (c.chainIdx < 0 || static_cast<std::size_t>(c.chainIdx) >= chains.size())
      continue;
    std::vector<std::vector<float>>& segList = chainSegCross[c.chainIdx];
    if (c.segIdx < 0 || static_cast<std::size_t>(c.segIdx) >= segList.size())
      continue;
    segList[c.segIdx].push_back(c.t);
  }

  // Build all ribbon strips for the frame up front; rasterize them tiled.
  std::vector<StyledStrip> strips;
  for (std::size_t ci = 0; ci < chains.size(); ++ci) {
    const EdgeChain& ch = chains[ci];
    if (ch.segs.empty()) continue;
    // The chain is single-nature (chaining never crosses natures); gate/style on
    // the first segment's nature and group.
    const FeatureSeg& s0 = fm.segs[static_cast<std::size_t>(ch.segs[0])];
    const EdgeNature nat = s0.nature;
    float halfThick = globalHalf, col[3] = {0.0f, 0.0f, 0.0f}, opacity = 1.0f;
    if (!resolveStyle(nat, s0.group, halfThick, col, opacity)) continue;

    // Freestyle-faithful visibility (split-at-crossing, then per-sub-span QI):
    // the 2D image-space crossings (TVertices, computeEdgeCrossings) split this
    // chain into sub-spans; each is labelled visible/hidden by a midpoint QI ray
    // cast (subSpanHidden) excluding its own 1-ring of faces. The visible run then
    // ends/starts EXACTLY at a crossing (the occluder's silhouette) instead of at
    // the camBias-lifted ray-flip boundary, which overshot a grazing silhouette
    // (a back tube's line was drawn past the front tube's outline). A segment with
    // no crossing is one sub-span, recovering the coarse per-segment occlusion for
    // edges that pass behind a body without a feature-edge crossing.
    const std::size_t nSegChain = ch.pts.size() - 1;
    const std::vector<SubSpan> spans =
        splitChainAtCrossings(nSegChain, chainSegCross[ci]);
    // Freestyle cusp split (smooth silhouettes only): bounds a hidden back-fold
    // that crosses no drawn line into its own ViewEdge so the majority vote hides
    // it instead of being out-voted by its visible limbs.
    const std::vector<char> cuspVert =
        (nat == EdgeNature::Silhouette)
            ? computeCuspVerts(ch, fm, scene.mesh, sp)
            : std::vector<char>();
    const std::vector<char> spanHide =
        subSpanHidden(ch, spans, sp, occluded, cuspVert);
    const std::vector<Pt2> proj = projectChainSubSpans(ch, spans, spanHide, sp);
    // STROKE LAYER: wrap the visibility-tagged polyline as a parametric Stroke
    // (buildStroke), arc-length resample it (resampleStroke), then emit one ribbon
    // per maximal VISIBLE run. Stage-a keeps the SCALAR buildStrip (constant
    // halfThick), so the rasterized result is byte-identical; the per-vertex
    // StrokeAttribute is the substrate for later width/color shaders.
    StrokeAttribute defAttr;
    defAttr.leftThick = halfThick;
    defAttr.rightThick = halfThick;
    defAttr.color[0] = col[0];
    defAttr.color[1] = col[1];
    defAttr.color[2] = col[2];
    defAttr.alpha = opacity;
    Stroke stroke = buildStroke(proj, defAttr, static_cast<int>(ci), nat);
    resampleStroke(stroke, stepPx);

    // STYLIZATION: run the per-vertex stroke shaders (Freestyle StrokeShader::shade).
    // The constant width/color shaders reproduce the resolved per-section default
    // attribute, so output is byte-identical; a real calligraphic/depth-cue/taper/
    // noise shader plugs in here to make L != R or color vary along u.
    std::vector<std::unique_ptr<StrokeShader>> shaders;
    if (se.smooth)  // Freestyle anisotropic curvature-flow smoothing (corner-aware)
      shaders.push_back(std::make_unique<AnisoSmoothingShader>(
          /*nbIter=*/200, /*factorPoint=*/0.0f, /*factorCurvature=*/0.4f,
          /*factorCurvatureDiff=*/0.3f, /*anisoPoint=*/0.0f, /*anisoNormal=*/0.08f,
          /*anisoCurvature=*/0.08f, /*carricature=*/1.0f));
    shaders.push_back(std::make_unique<ConstantThicknessShader>(halfThick));
    shaders.push_back(std::make_unique<ConstantColorShader>(col, opacity));
    if (se.taper)  // demo f(u) shader: taper width toward stroke ends
      shaders.push_back(std::make_unique<TaperShader>(0.5f, 0.15f));
    for (const std::unique_ptr<StrokeShader>& sh : shaders) sh->shade(stroke);

    // Build the variable-width ribbon strips (one per maximal visible run). With
    // the constant default attribute (leftThick==rightThick==halfThick) this is
    // byte-identical to the old scalar buildStrip path.
    buildStrokeReps(stroke, strips);
  }

  if (strips.empty()) return;

  // Stable-sort strips for compositing. compositeOver paints in array order, so the
  // LAST strip is on top (painter's algorithm). Primary key = DEPTH: sort FARTHER
  // first (descending depthKey = view-z) so the NEARER stroke is last == on top
  // (Freestyle Operators::sort + pyZBP1D, resolved to umbreon's `over` blend
  // direction). Tie-break by nature PRECEDENCE (Crease < Border < Silhouette).
  // Stable so full ties keep build (chain) order -> deterministic; depthKey is
  // computed single-threaded, so the result is independent of TBB scheduling.
  std::stable_sort(strips.begin(), strips.end(),
                   [](const StyledStrip& a, const StyledStrip& b) {
                     if (a.depthKey != b.depthKey) return a.depthKey > b.depthKey;
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
