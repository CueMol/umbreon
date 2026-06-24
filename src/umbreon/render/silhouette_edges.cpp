// Analytic object-space silhouette-edge generation. See silhouette_edges.hpp.
#include "render/silhouette_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
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
// not inside another primitive's solid; the inside parts are where a connecting
// primitive's surface takes over. Clipping there makes a bond's silhouette
// terminate at the atom it enters (and vice versa) instead of the two
// per-primitive contours crossing and leaving a "junction notch".
//
// The inset is STRICT (eps = 0: drop only points genuinely inside another
// solid). It is deliberately NOT an outward margin. An earlier version inset the
// test OUTWARD by ~half the edge width to trim the tiny tangent/overshoot stub a
// contour leaves in a junction's concave wedge; but in a densely packed molecule
// nearly every primitive's surface runs within that margin of some neighbor, so
// the outward band chopped the continuous outline into dashes. Strict clip keeps
// the outline continuous; a sub-pixel stub may remain at a sharp concave corner
// (the analytic per-primitive contours genuinely cross there), which is the same
// residue a finite mesh tessellation leaves and is far less objectionable than a
// broken line. True 3D occlusion of unrelated geometry is left to the ray tracer.

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

// True where `X` is inside any ORIGINAL solid other than the segment's source
// (the source is skipped so a contour is never clipped by its own primitive: a
// sphere ring is a chord polygon dipping just inside its sphere, and a cylinder
// side line can dip inside its own cylinder under perspective).
// `ringRs` >= 0 marks the segment as a SPHERE ring of that radius; a cylinder
// THINNER than the ring's sphere then does NOT clip it (pass -1 for cylinder side
// lines, which clip against everything). Rationale: when a sphere protrudes
// beyond a bond (rs > rc, the ball-and-stick case) the sphere's whole silhouette
// circle is on the union boundary -- the thin bond is buried inside it -- so the
// atom circle must stay CONTINUOUS (as in the CueMol OpenGL edge reference). Only
// when the bond is as thick as the atom (rs ~= rc, the licorice/stick case) does
// the bond surface take over: there the ring IS clipped, so no "ball bump" breaks
// the smooth tube. The ray tracer still occludes whatever the bond truly hides.
bool insideAnySolid(const Vec3& X, const Scene& scene, std::size_t nSph,
                    std::size_t nCyl, float eps, int skipSphere, int skipCyl,
                    float ringRs) {
  for (std::size_t i = 0; i < nSph; ++i)
    if (static_cast<int>(i) != skipSphere &&
        insideSphere(X, scene.spheres[i], eps))
      return true;
  for (std::size_t i = 0; i < nCyl; ++i) {
    if (static_cast<int>(i) == skipCyl) continue;
    // A protruding sphere's ring is not clipped by a thinner bond (1e-4 tol so an
    // exactly-equal licorice radius still counts as "as thick" and clips).
    if (ringRs > 0.0f && scene.cylinders[i].radius < ringRs - 1.0e-4f) continue;
    if (insideCylinder(X, scene.cylinders[i], eps)) return true;
  }
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
  const float eps = 0.0f;  // strict: clip only the parts genuinely inside a solid
  const float step = opt.width > 0.0f ? opt.width : len;
  const int K = std::max(2, static_cast<int>(std::ceil(len / step)));

  auto pointAt = [&](int i) { return seg.a + d * (static_cast<float>(i) / static_cast<float>(K)); };
  auto emitRun = [&](int s, int e) {
    if (e > s) out.push_back(makeEdge(pointAt(s), pointAt(e), opt, seg.group));
  };

  // A sphere ring carries its sphere's radius so a thinner bond cannot clip it.
  const float ringRs =
      seg.srcSphere >= 0 ? scene.spheres[static_cast<std::size_t>(seg.srcSphere)].radius : -1.0f;
  int runStart = -1;
  for (int i = 0; i <= K; ++i) {
    const bool inside = insideAnySolid(pointAt(i), scene, nSph, nCyl, eps,
                                       seg.srcSphere, seg.srcCyl, ringRs);
    if (!inside && runStart < 0) runStart = i;
    if (inside && runStart >= 0) {
      emitRun(runStart, i - 1);  // last outside sample closes the run
      runStart = -1;
    }
  }
  if (runStart >= 0) emitRun(runStart, K);  // run reached the far endpoint
}

// --- triangle-mesh edges --------------------------------------------------
// "Toward the viewer" unit vector at P (orthographic: constant -view direction;
// perspective: direction to the eye).
Vec3 viewerDirAt(const Vec3& P, const Camera& cam) {
  const Vec3 toCam = normalize(cam.direction * -1.0f);
  if (cam.orthographic) return toCam;
  const Vec3 d = cam.position - P;
  const float l = length(d);
  return l > 0.0f ? d * (1.0f / l) : toCam;
}

// Smooth silhouette (Freestyle WXFaceLayer::BuildSmoothEdge), face-normal crease
// and open-border edges of the de-indexed triangle mesh. Segments are raised
// outward along the local interpolated normal by opt.raise and pushed as RawSegs
// to be emitted VERBATIM (the ray tracer resolves their visibility).
void emitMeshEdges(const Mesh& mesh, const Camera& cam, const SilEdgeOptions& opt,
                   std::vector<RawSeg>& out) {
  const std::size_t nCorner = mesh.positions.size();
  const std::size_t nTri = nCorner / 3;
  if (nTri == 0) return;
  const bool haveNrm = mesh.normals.size() == nCorner;

  // 1) Weld the de-indexed corners into shared vertices (quantized-position
  //    hash), accumulating an averaged vertex normal. Welding is required so an
  //    interior edge sees BOTH its faces (crease/border) and so the smooth
  //    silhouette's per-edge zero-crossings coincide across adjacent faces.
  struct VKey {
    int x, y, z;
    bool operator==(const VKey& o) const { return x == o.x && y == o.y && z == o.z; }
  };
  struct VKeyHash {
    std::size_t operator()(const VKey& k) const {
      return (static_cast<std::size_t>(k.x) * 73856093u) ^
             (static_cast<std::size_t>(k.y) * 19349663u) ^
             (static_cast<std::size_t>(k.z) * 83492791u);
    }
  };
  const float kQuant = 1.0e4f;  // weld within ~1e-4 world units
  auto keyOf = [&](const Vec3& p) {
    return VKey{static_cast<int>(std::lround(p.x * kQuant)),
                static_cast<int>(std::lround(p.y * kQuant)),
                static_cast<int>(std::lround(p.z * kQuant))};
  };
  std::unordered_map<VKey, int, VKeyHash> vmap;
  vmap.reserve(nCorner);
  std::vector<Vec3> vpos, vacc;
  std::vector<int> corner2v(nCorner);
  for (std::size_t c = 0; c < nCorner; ++c) {
    const VKey k = keyOf(mesh.positions[c]);
    auto it = vmap.find(k);
    if (it == vmap.end()) {
      const int vi = static_cast<int>(vpos.size());
      vmap.emplace(k, vi);
      vpos.push_back(mesh.positions[c]);
      vacc.push_back(haveNrm ? mesh.normals[c] : Vec3{0.0f, 0.0f, 0.0f});
      corner2v[c] = vi;
    } else {
      corner2v[c] = it->second;
      if (haveNrm)
        vacc[static_cast<std::size_t>(it->second)] =
            vacc[static_cast<std::size_t>(it->second)] + mesh.normals[c];
    }
  }
  const std::size_t nV = vpos.size();

  // 2) Face welded indices and geometric face normals.
  std::vector<int> fa(nTri), fb(nTri), fc(nTri);
  std::vector<Vec3> fNg(nTri);
  for (std::size_t f = 0; f < nTri; ++f) {
    fa[f] = corner2v[3 * f];
    fb[f] = corner2v[3 * f + 1];
    fc[f] = corner2v[3 * f + 2];
    fNg[f] = normalize(cross(vpos[static_cast<std::size_t>(fb[f])] - vpos[static_cast<std::size_t>(fa[f])],
                             vpos[static_cast<std::size_t>(fc[f])] - vpos[static_cast<std::size_t>(fa[f])]));
  }
  // Per-vertex normal: the averaged smooth normal, or a borrowed incident face
  // normal where the mesh carried none (a hard-faceted mesh degrades to the
  // face-normal silhouette).
  std::vector<Vec3> vN(nV, Vec3{0.0f, 0.0f, 0.0f});
  for (std::size_t v = 0; v < nV; ++v) {
    const float l = length(vacc[v]);
    if (haveNrm && l > 1.0e-6f) vN[v] = vacc[v] * (1.0f / l);
  }
  for (std::size_t f = 0; f < nTri; ++f)
    for (int e = 0; e < 3; ++e) {
      const std::size_t v = static_cast<std::size_t>(e == 0 ? fa[f] : e == 1 ? fb[f] : fc[f]);
      if (length(vN[v]) < 0.5f) vN[v] = fNg[f];
    }

  std::vector<float> dotp(nV);
  for (std::size_t v = 0; v < nV; ++v) dotp[v] = dot(vN[v], viewerDirAt(vpos[v], cam));

  auto push = [&](const Vec3& A, const Vec3& nA, const Vec3& B, const Vec3& nB, uint16_t grp) {
    out.push_back(RawSeg{A + nA * opt.raise, B + nB * opt.raise, grp});
  };

  // 3) SMOOTH SILHOUETTE: per face, connect the two n.v==0 zero-crossings.
  if (opt.meshSilhouette) {
    for (std::size_t f = 0; f < nTri; ++f) {
      const int idx[3] = {fa[f], fb[f], fc[f]};
      const float d[3] = {dotp[static_cast<std::size_t>(idx[0])],
                          dotp[static_cast<std::size_t>(idx[1])],
                          dotp[static_cast<std::size_t>(idx[2])]};
      Vec3 cp[2], cn[2];
      int nc = 0;
      for (int e = 0; e < 3 && nc < 2; ++e) {
        const std::size_t i = static_cast<std::size_t>(idx[e]);
        const std::size_t j = static_cast<std::size_t>(idx[(e + 1) % 3]);
        const float di = d[e], dj = d[(e + 1) % 3];
        if (di * dj < 0.0f) {
          const float t = di / (di - dj);
          cp[nc] = vpos[i] + (vpos[j] - vpos[i]) * t;
          cn[nc] = normalize(vN[i] + (vN[j] - vN[i]) * t);
          ++nc;
        }
      }
      if (nc == 2) push(cp[0], cn[0], cp[1], cn[1], mesh.groupForTri(f));
    }
  }

  // 4) CREASE (face-normal dihedral) + BORDER (single-face edge) on mesh edges.
  //    Both are GEOMETRICALLY GATED (no color) so they fire only on genuine
  //    features, leaving the smooth silhouette as the primary outline.
  if (opt.meshCrease || opt.meshBorder) {
    const float creaseCos = std::cos(opt.creaseAngleDeg * kPi / 180.0f);
    // Smooth-facet veto threshold: a crease whose two faces both lie within this
    // angle of the edge's interpolated vertex normals is tessellation facetting.
    const bool smoothVeto = opt.meshCreaseSmoothVetoDeg > 0.0f;
    const float smoothCos = std::cos(opt.meshCreaseSmoothVetoDeg * kPi / 180.0f);
    // Border coplanar-continuation veto threshold (cos of the max chain bend).
    const bool borderVeto = opt.meshBorderCoplanarVetoDeg > 0.0f;
    const float borderColinCos = std::cos(opt.meshBorderCoplanarVetoDeg * kPi / 180.0f);

    struct EAdj {
      int f1 = -1, f2 = -1;
    };
    std::unordered_map<std::uint64_t, EAdj> emap;
    emap.reserve(nTri * 3);
    auto ekey = [](int a, int b) {
      const std::uint32_t lo = static_cast<std::uint32_t>(a < b ? a : b);
      const std::uint32_t hi = static_cast<std::uint32_t>(a < b ? b : a);
      return (static_cast<std::uint64_t>(hi) << 32) | lo;
    };
    for (std::size_t f = 0; f < nTri; ++f) {
      const int v[3] = {fa[f], fb[f], fc[f]};
      for (int e = 0; e < 3; ++e) {
        EAdj& adj = emap[ekey(v[e], v[(e + 1) % 3])];
        if (adj.f1 < 0)
          adj.f1 = static_cast<int>(f);
        else if (adj.f2 < 0)
          adj.f2 = static_cast<int>(f);
      }
    }

    // The third (apex) vertex of face f, i.e. the one not on edge (a,b).
    auto apexOf = [&](int f, std::size_t a, std::size_t b) -> Vec3 {
      const std::size_t x = static_cast<std::size_t>(fa[f]);
      const std::size_t y = static_cast<std::size_t>(fb[f]);
      const std::size_t z = static_cast<std::size_t>(fc[f]);
      if (x != a && x != b) return vpos[x];
      if (y != a && y != b) return vpos[y];
      return vpos[z];
    };

    // For the border coplanar-continuation veto: collect, per vertex, the OTHER
    // endpoint of each incident border edge (so a border edge can ask whether it
    // continues smoothly through each of its endpoints).
    std::unordered_map<int, std::vector<int>> borderAdj;
    if (opt.meshBorder && borderVeto) {
      borderAdj.reserve(nV);
      for (const auto& kv : emap) {
        if (kv.second.f2 >= 0) continue;  // interior, not a border edge
        const int a = static_cast<int>(kv.first & 0xffffffffu);
        const int b = static_cast<int>(kv.first >> 32);
        borderAdj[a].push_back(b);
        borderAdj[b].push_back(a);
      }
    }
    // A border edge (a-b) "continues smoothly" through endpoint p if some other
    // border edge p-q leaves p near-collinear with b->p (the chain barely bends).
    // A seam running along the smooth body satisfies this at BOTH ends; a true
    // terminus (cap rim corner, strand end) bends sharply or dead-ends.
    auto borderContinues = [&](std::size_t p, std::size_t other) -> bool {
      const auto it = borderAdj.find(static_cast<int>(p));
      if (it == borderAdj.end()) return false;
      const Vec3 in = normalize(vpos[p] - vpos[other]);  // direction into p
      for (const int q : it->second) {
        if (static_cast<std::size_t>(q) == other) continue;  // the edge itself
        const Vec3 outDir = normalize(vpos[static_cast<std::size_t>(q)] - vpos[p]);
        if (dot(in, outDir) >= borderColinCos) return true;  // bends < threshold
      }
      return false;
    };

    for (const auto& kv : emap) {
      const std::size_t a = static_cast<std::size_t>(kv.first & 0xffffffffu);
      const std::size_t b = static_cast<std::size_t>(kv.first >> 32);
      const EAdj& adj = kv.second;

      if (adj.f2 < 0) {
        // BORDER: a single-face open edge. Suppress it when it is an internal
        // strip seam (smooth border chain through BOTH endpoints); keep true
        // termini.
        if (!opt.meshBorder) continue;
        if (borderVeto && borderContinues(a, b) && borderContinues(b, a)) continue;
        push(vpos[a], vN[a], vpos[b], vN[b],
             mesh.groupForTri(static_cast<std::size_t>(adj.f1)));
        continue;
      }

      // CREASE: an interior fold. Gate by angle, smooth-facet veto and convexity.
      if (!opt.meshCrease) continue;
      const std::size_t f1 = static_cast<std::size_t>(adj.f1);
      const std::size_t f2 = static_cast<std::size_t>(adj.f2);
      if (dot(fNg[f1], fNg[f2]) >= creaseCos) continue;  // too shallow to be a fold

      // Smooth-facet veto: if BOTH face normals stay within smoothVetoDeg of the
      // edge's interpolated vertex normals at BOTH endpoints, the surface is
      // smooth-shaded across the edge -> the dihedral is tessellation, not a
      // crease (helix-barrel hatching, ribbon-face facet seams). Skip it.
      if (smoothVeto) {
        const float agree =
            std::fmin(std::fmin(dot(vN[a], fNg[f1]), dot(vN[a], fNg[f2])),
                      std::fmin(dot(vN[b], fNg[f1]), dot(vN[b], fNg[f2])));
        if (agree >= smoothCos) continue;  // smooth facet, not a real fold
      }

      // Convexity: a real outline fold is a CONVEX ridge. The two apex vertices of
      // a convex ridge sit on the inner (below) side of the average outward
      // normal's plane through the edge midpoint; a concave valley has them above.
      // Concave creases are the SS-junction strip-step valleys (CueMol MESHXX
      // no-edge faces), so drop them when meshCreaseConvexOnly is set.
      if (opt.meshCreaseConvexOnly) {
        const Vec3 nAvg = normalize(fNg[f1] + fNg[f2]);
        const Vec3 edgeMid = (vpos[a] + vpos[b]) * 0.5f;
        const Vec3 apexMid =
            (apexOf(adj.f1, a, b) + apexOf(adj.f2, a, b)) * 0.5f;
        if (dot(nAvg, apexMid - edgeMid) > 0.0f) continue;  // concave valley
      }

      push(vpos[a], vN[a], vpos[b], vN[b],
           mesh.groupForTri(static_cast<std::size_t>(adj.f1)));
    }
  }
}

}  // namespace

void generateSilhouetteEdges(Scene& scene, const SilEdgeOptions& opt) {
  if (!opt.enable) return;  // byte-identical default: no edges appended

  // SNAPSHOT the original primitive counts so the edges we append below are not
  // themselves silhouetted (and the clip tests only against the originals).
  const std::size_t sphereCount = scene.spheres.size();
  const std::size_t cylinderCount = scene.cylinders.size();

  // 1) Collect the raw (unclipped) ANALYTIC silhouette segments from spheres and
  //    cylinders. Baked POV outline primitives (fromEdgeMacro) are skipped: they
  //    are themselves edges, not surfaces to silhouette.
  std::vector<RawSeg> raw;
  const int seg = opt.segments < 3 ? 3 : opt.segments;
  raw.reserve(sphereCount * static_cast<std::size_t>(seg) + cylinderCount * 2);
  for (std::size_t i = 0; i < sphereCount; ++i) {
    if (scene.spheres[i].fromEdgeMacro) continue;
    const std::size_t before = raw.size();
    emitSphereRing(scene.spheres[i], scene.camera, opt, raw);
    for (std::size_t k = before; k < raw.size(); ++k)
      raw[k].srcSphere = static_cast<int>(i);
  }
  for (std::size_t i = 0; i < cylinderCount; ++i) {
    if (scene.cylinders[i].fromEdgeMacro) continue;
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

  // 3) Triangle-mesh edges (silhouette/crease/border). These are emitted VERBATIM
  //    -- their visibility is the ray tracer's job, and they must not be clipped
  //    against the analytic solids (which are unrelated to the mesh surface).
  std::vector<RawSeg> meshRaw;
  emitMeshEdges(scene.mesh, scene.camera, opt, meshRaw);
  for (const RawSeg& s : meshRaw) edges.push_back(makeEdge(s.a, s.b, opt, s.group));

  scene.cylinders.insert(scene.cylinders.end(), edges.begin(), edges.end());
}

}  // namespace umbreon
