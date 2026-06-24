// Analytic object-space silhouette-edge generation. See silhouette_edges.hpp.
#include "render/silhouette_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "scene.hpp"

namespace umbreon {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// A raw silhouette segment (pre-clip): the two endpoints plus the source
// primitive's transparency group. Collected first so the union-boundary clip can
// split each one against the other primitives before it becomes edge geometry.
struct RawSeg {
  Vec3 a, b;
  uint16_t group;
  // Source primitive (so the union clip never clips a segment against its OWN
  // primitive: a sphere ring is a chord polygon whose chord midpoints dip just
  // inside the source sphere, and a perspective cylinder side line is a chord
  // that can dip inside its own cylinder). -1 = none.
  int srcSphere = -1;
  int srcCyl = -1;
};

// Build one emitted edge cylinder. Mirrors how the baked POV edge_line
// cylinders render: open (capless, chained into ROUND_LINEAR_CURVE), flat
// outline material (ambient 1 / diffuse 0 => raw flat color), no AO / shadow.
// The source primitive's transparency group is carried over so per-section
// alpha / styling applies to its edges too. fromEdgeMacro is left false: that
// flag is reserved for the baked POV macro edges (the screen-space pass filters
// on it), and these analytic edges are a distinct producer.
Cylinder makeEdge(const Vec3& a, const Vec3& b, const SilEdgeOptions& opt,
                  uint16_t group) {
  Cylinder c;
  c.p0 = a;
  c.p1 = b;
  c.radius = opt.width > 0.0f ? opt.width : 0.0f;  // guard a negative CLI width
  c.color = Vec4{opt.color[0], opt.color[1], opt.color[2], 1.0f};
  c.material = Material::flatOutline();
  c.group = group;
  c.opacity1 = -1.0f;  // uniform opacity along the segment
  c.open = true;       // round/chained edge (matches baked POV outlines)
  c.fromEdgeMacro = false;
  return c;
}

// Emit the analytic silhouette ring of a sphere as a CLOSED loop of `segments`
// raw segments. The ring is the n.v == 0 contour:
//   ortho: the great circle in the plane through the center perpendicular to the
//          view direction (radius = sphere radius).
//   persp: the horizon (tangent) circle as seen from the camera; every point is
//          exactly on the sphere surface.
// Each ring vertex is offset OUTWARD along its surface normal by opt.raise.
void emitSphereRing(const Sphere& s, const Camera& cam,
                    const SilEdgeOptions& opt, std::vector<RawSeg>& out) {
  const Vec3 O = s.center;
  const float r = s.radius;
  if (r <= 0.0f) return;

  Vec3 axis;  // ring plane normal
  Vec3 Q;     // ring center
  float rho;  // ring radius

  if (cam.orthographic) {
    axis = normalize(cam.direction);
    Q = O;
    rho = r;
  } else {
    const Vec3 d = O - cam.position;
    const float L = length(d);
    // Camera on or inside the sphere: no silhouette horizon circle exists.
    if (L <= r * 1.0001f) return;
    axis = d * (1.0f / L);
    Q = O - axis * (r * r / L);
    const float disc = L * L - r * r;
    rho = r * std::sqrt(disc > 0.0f ? disc : 0.0f) / L;
  }
  if (rho <= 0.0f) return;

  const Frame fr = frameFromNormal(axis);  // fr.t, fr.b span the ring plane
  const int N = opt.segments < 3 ? 3 : opt.segments;

  std::vector<Vec3> ring(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i) {
    const float theta = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(N);
    const Vec3 dir = fr.t * std::cos(theta) + fr.b * std::sin(theta);
    Vec3 P = Q + dir * rho;
    // Outward surface normal at P, then raise the contour off the surface.
    const Vec3 nrm = normalize(P - O);
    P = P + nrm * opt.raise;
    ring[static_cast<std::size_t>(i)] = P;
  }

  for (int i = 0; i < N; ++i)
    out.push_back(RawSeg{ring[static_cast<std::size_t>(i)],
                         ring[static_cast<std::size_t>((i + 1) % N)], s.group});
}

// Emit a cap-rim circle (a ring of raw segments) of radius `r` centered at
// `center`, lying in the plane perpendicular to `axis`. Used for end-on
// cylinders whose silhouette degenerates to the visible cap circle. The raise
// is applied radially outward in the cap plane.
void emitCapCircle(const Vec3& center, const Vec3& axis, float r,
                   const SilEdgeOptions& opt, uint16_t group,
                   std::vector<RawSeg>& out) {
  if (r <= 0.0f) return;
  const Frame fr = frameFromNormal(normalize(axis));
  const int N = opt.segments < 3 ? 3 : opt.segments;
  const float rr = r + opt.raise;
  std::vector<Vec3> ring(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i) {
    const float theta = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(N);
    const Vec3 dir = fr.t * std::cos(theta) + fr.b * std::sin(theta);
    ring[static_cast<std::size_t>(i)] = center + dir * rr;
  }
  for (int i = 0; i < N; ++i)
    out.push_back(RawSeg{ring[static_cast<std::size_t>(i)],
                         ring[static_cast<std::size_t>((i + 1) % N)], group});
}

// The two silhouette contact directions (unit surface normals at the two grazing
// generators) on the cross-section circle of radius r at axis point P, for a
// cylinder with unit axis u viewed by `cam`. The tangents from the eye to that
// circle touch at angle alpha = acos(r / Lp) either side of the in-plane eye
// direction, where Lp is the perpendicular eye-to-axis distance. Orthographic
// (eye at infinity) collapses to alpha = 90deg, i.e. exactly +/- the
// axis-and-view perpendicular, recovering the simple side generators. Returns
// false when degenerate (end-on, or the eye is within the cylinder radius of the
// axis), so the caller emits the cap rim instead.
bool cylinderContactDirs(const Vec3& P, const Vec3& u, float r, const Camera& cam,
                         Vec3& dPlus, Vec3& dMinus) {
  constexpr float kEps = 1.0e-4f;
  if (cam.orthographic) {
    const Vec3 w = normalize(cam.direction) * -1.0f;  // toward the camera
    Vec3 eT = cross(u, w);
    const float l = length(eT);
    if (l < kEps) return false;  // end-on: axis ~parallel to the view
    eT = eT * (1.0f / l);
    dPlus = eT;
    dMinus = eT * -1.0f;
    return true;
  }
  // Perspective: project the eye onto the cross-section plane (perpendicular to
  // u); the silhouette contacts are the tangents from that projected eye.
  const Vec3 cp = cam.position - P;
  const Vec3 cpPerp = cp - u * dot(u, cp);
  const float Lp = length(cpPerp);
  if (Lp <= r * 1.0001f) return false;  // eye on/inside the cylinder radius
  const Vec3 eR = cpPerp * (1.0f / Lp);  // in-plane, toward the eye
  Vec3 eT = cross(u, eR);
  const float lt = length(eT);
  if (lt < kEps) return false;
  eT = eT * (1.0f / lt);
  const float cosA = r / Lp;  // in [0,1); -> 0 (alpha -> 90deg) as Lp -> inf
  const float sinA = std::sqrt(std::fmax(0.0f, 1.0f - cosA * cosA));
  dPlus = eR * cosA + eT * sinA;   // contact tilted toward the eye (exact)
  dMinus = eR * cosA - eT * sinA;
  return true;
}

// Emit the two side silhouette generators of a cylinder, connecting the per-
// endpoint grazing-contact points (exact for both orthographic and perspective
// projection). Near end-on the contacts degenerate and the outline is the cap
// circle, so emit the nearer cap rim instead.
void emitCylinderEdges(const Cylinder& cyl, const Camera& cam,
                       const SilEdgeOptions& opt, std::vector<RawSeg>& out) {
  const Vec3 A = cyl.p0;
  const Vec3 B = cyl.p1;
  const float r = cyl.radius;
  if (r <= 0.0f) return;

  const Vec3 ab = B - A;
  const float axisLen = length(ab);
  if (axisLen <= 0.0f) return;  // degenerate zero-length cylinder
  const Vec3 u = ab * (1.0f / axisLen);

  Vec3 dAp, dAm, dBp, dBm;
  if (!cylinderContactDirs(A, u, r, cam, dAp, dAm) ||
      !cylinderContactDirs(B, u, r, cam, dBp, dBm)) {
    // End-on: the outline is the cap circle, not side lines. Emit the cap rim of
    // the cap nearer the camera (whose rim is the visible silhouette).
    const Vec3 dir = normalize(cam.direction);
    Vec3 nearCap;
    if (cam.orthographic)
      nearCap = (dot(A, dir) < dot(B, dir)) ? A : B;  // smaller view-depth = nearer
    else
      nearCap = (length(cam.position - A) < length(cam.position - B)) ? A : B;
    emitCapCircle(nearCap, u, r, opt, cyl.group, out);
    return;
  }

  const float off = r + opt.raise;
  // The +contact and -contact directions are computed with the SAME axis u, so
  // line1 stays on one consistent side and line2 on the other.
  out.push_back(RawSeg{A + dAp * off, B + dBp * off, cyl.group});
  out.push_back(RawSeg{A + dAm * off, B + dBm * off, cyl.group});
}

// --- union-boundary clip --------------------------------------------------
// A silhouette point belongs to the molecular UNION boundary only where it is
// not strictly inside another primitive's solid; the inside parts are where a
// connecting primitive's surface takes over. Clipping there makes a bond's
// silhouette terminate at the atom it enters (and vice versa) instead of the two
// per-primitive contours crossing and leaving a coincident-depth "junction
// notch". `eps` insets the test by ~half the edge width so a tangent contact
// (the legitimate join point, exactly on both surfaces) is kept.

bool insideSphere(const Vec3& X, const Sphere& s, float eps) {
  return length(X - s.center) < s.radius - eps;
}

bool insideCylinder(const Vec3& X, const Cylinder& c, float eps) {
  const Vec3 ab = c.p1 - c.p0;
  const float L = length(ab);
  if (L <= 0.0f) return false;
  const Vec3 u = ab * (1.0f / L);
  const float t = dot(X - c.p0, u);
  if (t < 0.0f || t > L) return false;  // beyond the flat caps
  const Vec3 perp = (X - c.p0) - u * t;
  return length(perp) < c.radius - eps;
}

bool insideAnySolid(const Vec3& X, const Scene& scene, std::size_t nSph,
                    std::size_t nCyl, float eps, int skipSphere, int skipCyl) {
  for (std::size_t i = 0; i < nSph; ++i)
    if (static_cast<int>(i) != skipSphere &&
        insideSphere(X, scene.spheres[i], eps))
      return true;
  for (std::size_t i = 0; i < nCyl; ++i)
    if (static_cast<int>(i) != skipCyl &&
        insideCylinder(X, scene.cylinders[i], eps))
      return true;
  return false;
}

// Sample `seg` and emit edge cylinders for the maximal runs that stay OUTSIDE
// every (original) solid. Sampling spacing ~= edge width: finer spacing yields a
// cleaner intersection (the "tessellation -> infinity" limit).
void clipAndEmit(const RawSeg& seg, const SilEdgeOptions& opt, const Scene& scene,
                 std::size_t nSph, std::size_t nCyl, std::vector<Cylinder>& out) {
  const Vec3 d = seg.b - seg.a;
  const float len = length(d);
  if (len <= 0.0f) return;
  // Negative eps => the "inside" test extends a margin (~half the edge width)
  // OUTSIDE each solid, so a silhouette generator that runs tangent to a
  // same-radius connecting primitive (its endpoint sits exactly on that surface)
  // is trimmed too, removing the tangent "stub" at a same-size sphere/cylinder
  // join. The connecting primitive's own ring/side line continues the outline.
  const float eps = -(opt.width > 0.0f ? opt.width : 0.01f) * 0.5f;
  const float step = opt.width > 0.0f ? opt.width : len;
  const int K = std::max(2, static_cast<int>(std::ceil(len / step)));

  auto pointAt = [&](int i) { return seg.a + d * (static_cast<float>(i) / static_cast<float>(K)); };
  auto emitRun = [&](int s, int e) {
    if (e > s) out.push_back(makeEdge(pointAt(s), pointAt(e), opt, seg.group));
  };

  int runStart = -1;
  for (int i = 0; i <= K; ++i) {
    const bool inside = insideAnySolid(pointAt(i), scene, nSph, nCyl, eps,
                                       seg.srcSphere, seg.srcCyl);
    if (!inside && runStart < 0) runStart = i;
    if (inside && runStart >= 0) {
      emitRun(runStart, i - 1);  // last outside sample closes the run
      runStart = -1;
    }
  }
  if (runStart >= 0) emitRun(runStart, K);  // run reached the far endpoint
}

}  // namespace

void generateSilhouetteEdges(Scene& scene, const SilEdgeOptions& opt) {
  if (!opt.enable) return;  // byte-identical default: no edges appended

  // SNAPSHOT the original primitive counts so the edges we append below are not
  // themselves silhouetted (and the clip tests only against the originals).
  const std::size_t sphereCount = scene.spheres.size();
  const std::size_t cylinderCount = scene.cylinders.size();

  // 1) Collect the raw (unclipped) silhouette segments.
  std::vector<RawSeg> raw;
  const int seg = opt.segments < 3 ? 3 : opt.segments;
  raw.reserve(sphereCount * static_cast<std::size_t>(seg) + cylinderCount * 2);
  for (std::size_t i = 0; i < sphereCount; ++i) {
    const std::size_t before = raw.size();
    emitSphereRing(scene.spheres[i], scene.camera, opt, raw);
    for (std::size_t k = before; k < raw.size(); ++k)
      raw[k].srcSphere = static_cast<int>(i);
  }
  for (std::size_t i = 0; i < cylinderCount; ++i) {
    const std::size_t before = raw.size();
    emitCylinderEdges(scene.cylinders[i], scene.camera, opt, raw);
    for (std::size_t k = before; k < raw.size(); ++k)
      raw[k].srcCyl = static_cast<int>(i);
  }

  // 2) Convert to edge cylinders, clipping each against the union of the
  //    original solids (or emitting verbatim when the clip is disabled).
  std::vector<Cylinder> edges;
  edges.reserve(raw.size());
  if (opt.clip) {
    for (const RawSeg& s : raw)
      clipAndEmit(s, opt, scene, sphereCount, cylinderCount, edges);
  } else {
    for (const RawSeg& s : raw) edges.push_back(makeEdge(s.a, s.b, opt, s.group));
  }

  scene.cylinders.insert(scene.cylinders.end(), edges.begin(), edges.end());
}

}  // namespace umbreon
