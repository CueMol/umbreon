// Analytic silhouette extraction for the analytic primitives. See
// render/analytic_silhouette.hpp. The emitters here were MOVED verbatim from the
// anonymous namespace of render/object_space_edges.cpp; the float expressions are
// preserved exactly so `circumscribe == false` reproduces the former inline
// obj-edges silhouette geometry bit-for-bit.
#include "render/analytic_silhouette.hpp"

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "scene.hpp"

namespace umbreon {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// Emit the analytic silhouette ring of a sphere as a CLOSED loop of `N` vertices.
// The ring is the n.v == 0 contour:
//   ortho: the great circle in the plane through the center perpendicular to the
//          view direction (radius = sphere radius).
//   persp: the horizon (tangent) circle as seen from the camera; every point is
//          exactly on the sphere surface.
// Each ring vertex is offset OUTWARD along its surface normal by `raise`. When
// `circumscribe` is set the ring radius is scaled by 1/cos(pi/N) so the chord
// polygon's apothem equals the contour radius (vertices just outside, chord
// midpoints on the surface). Returns false when the silhouette is degenerate.
bool emitSphereRingLoop(const Sphere& s, const Camera& cam, int N, float raise,
                        bool circumscribe, AnalyticLoop& out) {
  const Vec3 O = s.center;
  const float r = s.radius;
  if (r <= 0.0f) return false;

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
    if (L <= r * 1.0001f) return false;
    axis = d * (1.0f / L);
    Q = O - axis * (r * r / L);
    const float disc = L * L - r * r;
    rho = r * std::sqrt(disc > 0.0f ? disc : 0.0f) / L;
  }
  if (rho <= 0.0f) return false;

  const Frame fr = frameFromNormal(axis);  // fr.t, fr.b span the ring plane
  // circumscribe == false leaves rhoUse == rho (bit-identical to obj-edges).
  const float rhoUse =
      circumscribe ? rho / std::cos(kPi / static_cast<float>(N)) : rho;

  out.pts.resize(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i) {
    const float theta = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(N);
    const Vec3 dir = fr.t * std::cos(theta) + fr.b * std::sin(theta);
    Vec3 P = Q + dir * rhoUse;
    // Outward surface normal at P, then raise the contour off the surface.
    const Vec3 nrm = normalize(P - O);
    P = P + nrm * raise;
    out.pts[static_cast<std::size_t>(i)] = P;
  }
  out.closed = true;
  return true;
}

// Emit a cap-rim circle (a CLOSED loop of `N` vertices) of radius `r` centered at
// `center`, lying in the plane perpendicular to `axis`. Used for end-on cylinders
// whose silhouette degenerates to the visible cap circle. The raise is applied
// radially outward in the cap plane; `circumscribe` scales the rim like the sphere
// ring.
void emitCapCircleLoop(const Vec3& center, const Vec3& axis, float r, int N,
                       float raise, bool circumscribe, AnalyticLoop& out) {
  if (r <= 0.0f) return;
  const Frame fr = frameFromNormal(normalize(axis));
  const float rBase = circumscribe ? r / std::cos(kPi / static_cast<float>(N)) : r;
  const float rr = rBase + raise;
  out.pts.resize(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i) {
    const float theta = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(N);
    const Vec3 dir = fr.t * std::cos(theta) + fr.b * std::sin(theta);
    out.pts[static_cast<std::size_t>(i)] = center + dir * rr;
  }
  out.closed = true;
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

// Emit the silhouette loops of a cylinder: the two side generators (one open
// 2-point loop each, exact for both ortho and perspective) connecting the per-
// endpoint grazing contacts, or -- when end-on -- the nearer cap rim circle.
void emitCylinderLoops(const Cylinder& cyl, const Camera& cam, int N, float raise,
                       bool circumscribe, int srcCyl,
                       std::vector<AnalyticLoop>& out) {
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
    AnalyticLoop cap;
    emitCapCircleLoop(nearCap, u, r, N, raise, circumscribe, cap);
    cap.group = cyl.group;
    cap.srcCyl = srcCyl;
    out.push_back(std::move(cap));
    return;
  }

  const float off = r + raise;
  // The +contact and -contact directions are computed with the SAME axis u, so
  // line1 stays on one consistent side and line2 on the other. Each side
  // generator is its own open 2-point loop (it never chains with anything else).
  AnalyticLoop l1;
  l1.pts = {A + dAp * off, B + dBp * off};
  l1.group = cyl.group;
  l1.srcCyl = srcCyl;
  out.push_back(std::move(l1));
  AnalyticLoop l2;
  l2.pts = {A + dAm * off, B + dBm * off};
  l2.group = cyl.group;
  l2.srcCyl = srcCyl;
  out.push_back(std::move(l2));
}

}  // namespace

void emitAnalyticSilhouettes(const Scene& scene, const Camera& cam, int N,
                             float raise, bool circumscribe,
                             std::vector<AnalyticLoop>& out) {
  const int n = N < 3 ? 3 : N;
  // Spheres first, then cylinders, in primitive-index order -- the order the
  // former inline obj-edges collection produced (preserves byte-identity).
  for (std::size_t i = 0; i < scene.spheres.size(); ++i) {
    const Sphere& s = scene.spheres[i];
    if (s.fromEdgeMacro) continue;  // baked outline: an edge, not a surface
    AnalyticLoop loop;
    if (emitSphereRingLoop(s, cam, n, raise, circumscribe, loop)) {
      loop.group = s.group;
      loop.srcSphere = static_cast<int>(i);
      out.push_back(std::move(loop));
    }
  }
  for (std::size_t i = 0; i < scene.cylinders.size(); ++i) {
    const Cylinder& c = scene.cylinders[i];
    if (c.fromEdgeMacro) continue;
    emitCylinderLoops(c, cam, n, raise, circumscribe, static_cast<int>(i), out);
  }
}

int appendAnalyticFeatureSegs(const Scene& scene, const Camera& cam, int N,
                              float raise, int nodeBase,
                              std::vector<FeatureSeg>& segs) {
  std::vector<AnalyticLoop> loops;
  // The stroke QI path always circumscribes (so a sphere ring chord never dips
  // inside the source and self-hides the contour into dashes).
  emitAnalyticSilhouettes(scene, cam, N, raise, /*circumscribe=*/true, loops);

  int next = nodeBase;
  for (const AnalyticLoop& loop : loops) {
    const int n = static_cast<int>(loop.pts.size());
    if (n < 2) continue;  // need at least one segment
    const int base = next;
    next += n;  // each loop gets its own contiguous id block
    const int last = loop.closed ? n : n - 1;  // # segments (wrap if closed)
    for (int i = 0; i < last; ++i) {
      const int j = (i + 1) % n;
      FeatureSeg fs;
      fs.v0 = base + i;
      fs.v1 = base + j;
      fs.p0 = loop.pts[static_cast<std::size_t>(i)];
      fs.p1 = loop.pts[static_cast<std::size_t>(j)];
      fs.nature = EdgeNature::Silhouette;
      fs.group = loop.group;
      fs.face0 = -1;
      fs.face1 = -1;
      // excludeFaces stays empty and nrm stays zero (struct defaults): an analytic
      // silhouette point self-rejects via the grazing/coplanar QI filter, and a
      // convex primitive contour has no image-space cusp.
      segs.push_back(std::move(fs));
    }
  }
  return next;
}

}  // namespace umbreon
