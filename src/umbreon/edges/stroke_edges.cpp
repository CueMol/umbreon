#include "edges/stroke_edges.hpp"

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

#include "edges/screen_vector_edges.hpp"
#include "edges/stroke_render.hpp"

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
// view-z (carried for future depth use) and a visibility flag.
struct Pt2 {
  Vec2 p;
  float vz = 0.0f;
  bool visible = true;
};

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
  const int precedence = s.precedence;
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
  for (std::size_t ci = 0; ci < chains.size(); ++ci) {
    const StrokeChainInput& in = chains[ci];
    float halfThick = 0.0f, col[3] = {0.0f, 0.0f, 0.0f}, opacity = 1.0f;
    if (!resolveStrokeStyle(scene, se, ssScale, in.styleSlot, in.group,
                            halfThick, col, opacity))
      continue;

    // Wrap the source points into the internal visibility-tagged polyline.
    std::vector<Pt2> proj;
    proj.reserve(in.pts.size());
    for (const StrokePoint& sp : in.pts) {
      Pt2 q;
      q.p = {sp.x, sp.y};
      q.vz = sp.vz;
      q.visible = sp.visible;
      proj.push_back(q);
    }

    // STROKE LAYER: wrap the visibility-tagged polyline as a parametric Stroke
    // (buildStroke), arc-length resample it (resampleStroke), then emit one
    // ribbon per maximal VISIBLE run.
    StrokeAttribute defAttr;
    defAttr.leftThick = halfThick;
    defAttr.rightThick = halfThick;
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
    shaders.push_back(std::make_unique<ConstantThicknessShader>(halfThick));
    shaders.push_back(std::make_unique<ConstantColorShader>(col, opacity));
    if (se.taper)  // demo f(u) shader: taper width toward stroke ends
      shaders.push_back(std::make_unique<TaperShader>(0.5f, 0.15f));
    for (const std::unique_ptr<StrokeShader>& sh : shaders) sh->shade(stroke);

    // Build the variable-width ribbon strips (one per maximal visible run).
    buildStrokeReps(stroke, strips);
  }

  if (strips.empty()) return;

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
  tbb::parallel_for(
      tbb::blocked_range<int>(0, H),
      [&](const tbb::blocked_range<int>& rows) {
        const int rb = rows.begin(), re = rows.end();
        for (const StyledStrip& ss : strips)
          rasterizeStrip(frame.color, W, rb, re, ss.strip, ss.color,
                         ss.opacity);
      });
}

// STEP 4: variable-width ribbon strokes. Extract topology-tagged feature edges,
// chain them per nature into continuous polylines, mark each backbone vertex
// visible/hidden by ray-cast QI against the live Embree BVH (via `occluded`),
// project + arc-length resample, hand the visibility-tagged 2D polylines to the
// shared draw stage (stroke_render.hpp:renderStrokeChains) which builds a
// miter-joined offset RIBBON per maximal visible run and hard-fills the
// triangle strips composited over frame.color in LINEAR space at hi-res (the
// box downsample antialiases). Rasterization is row-tiled with TBB and is
// deterministic. The default (edges off) path never reaches here, so the
// no-edge render stays byte-identical.
void applyStrokeEdges(FrameResult& frame, const Scene& scene,
                      const RenderOptions& opt, const OcclusionQuery& occluded,
                      const OcclusionQuery& occludedRaw) {
  (void)occluded; (void)occludedRaw;
  const StrokeEdgeOptions& se = opt.strokeEdges;
  if (!se.enable) return;
  applyScreenVectorEdges(frame, scene, opt);
}

}  // namespace umbreon
