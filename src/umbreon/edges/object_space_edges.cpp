// Analytic object-space silhouette-edge generation. See object_space_edges.hpp.
#include "edges/object_space_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "edges/analytic_silhouette.hpp"
#include "edges/edge_mesh_bvh.hpp"
#include "edges/mesh_feature_edges.hpp"
#include "scene.hpp"

namespace umbreon {
namespace {

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
Cylinder makeEdge(const Vec3& a, const Vec3& b, const ObjectSpaceEdgeOptions& opt,
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

// Flatten one analytic silhouette loop into the RawSeg chord sequence the clip
// consumes: a CLOSED loop of N vertices -> N chords (wrapping); an OPEN loop of N
// vertices -> N-1 chords. Carries the loop's group + source provenance onto every
// chord. Reproduces the order/values the former inline emitSphereRing /
// emitCylinderEdges produced (byte-identical --obj-edges).
void flattenLoop(const AnalyticLoop& loop, std::vector<RawSeg>& raw) {
  const std::size_t n = loop.pts.size();
  if (n < 2) return;
  const std::size_t last = loop.closed ? n : n - 1;
  for (std::size_t i = 0; i < last; ++i) {
    const std::size_t j = (i + 1) % n;
    raw.push_back(RawSeg{loop.pts[i], loop.pts[j], loop.group, loop.srcSphere,
                         loop.srcCyl});
  }
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
void clipAndEmit(const RawSeg& seg, const ObjectSpaceEdgeOptions& opt, const Scene& scene,
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

}  // namespace

void generateObjectSpaceEdges(Scene& scene, const ObjectSpaceEdgeOptions& opt) {
  if (!opt.enable) return;  // byte-identical default: no edges appended

  // SNAPSHOT the original primitive counts so the edges we append below are not
  // themselves silhouetted (and the clip tests only against the originals).
  const std::size_t sphereCount = scene.spheres.size();
  const std::size_t cylinderCount = scene.cylinders.size();

  // 1) Collect the raw (unclipped) ANALYTIC silhouette segments from spheres and
  //    cylinders via the shared analytic-silhouette core (no circumscription, so
  //    the chords match the former inline emitters bit-for-bit). Baked POV outline
  //    primitives (fromEdgeMacro) are skipped inside the core. Spheres are emitted
  //    before cylinders, preserving the previous RawSeg order.
  const int seg = opt.segments < 3 ? 3 : opt.segments;
  std::vector<AnalyticLoop> loops;
  emitAnalyticSilhouettes(scene, scene.camera, seg, opt.raise,
                          /*circumscribe=*/false, loops);
  std::vector<RawSeg> raw;
  raw.reserve(sphereCount * static_cast<std::size_t>(seg) + cylinderCount * 2);
  for (const AnalyticLoop& loop : loops) flattenLoop(loop, raw);

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

  // 3) Triangle-mesh edges (silhouette/crease/border). Detection is factored to
  //    the shared extractor (render/mesh_feature_edges.cpp); each topology-tagged
  //    FeatureSeg is wrapped into a cylinder VERBATIM here (its endpoints already
  //    carry the silhouette lift/cam bias, so the output is byte-identical to the
  //    former inline emitMeshEdges). These are NOT clipped against the analytic
  //    solids (which are unrelated to the mesh surface).
  ExtractParams ep;
  ep.raise = opt.raise;
  ep.width = opt.width;
  ep.silhouette = opt.meshSilhouette;
  ep.crease = opt.meshCrease;
  ep.border = opt.meshBorder;
  ep.meshHardEdgeDeg = opt.meshHardEdgeDeg;
  ep.creaseAngleDeg = opt.creaseAngleDeg;
  ep.meshCreaseSmoothVetoDeg = opt.meshCreaseSmoothVetoDeg;
  ep.meshCreaseConvexOnly = opt.meshCreaseConvexOnly;
  ep.meshBorderCoplanarVetoDeg = opt.meshBorderCoplanarVetoDeg;
  ep.meshCreaseMaxDegree = opt.meshCreaseMaxDegree;
  const FeatureMesh fm = extractMeshFeatureEdges(scene.mesh, scene.camera, ep);
  if (!opt.visibilityClip) {
    // Legacy path: emit each feature segment verbatim (ray tracer occludes the
    // cylinders). Byte-identical default.
    for (const FeatureSeg& s : fm.segs)
      edges.push_back(makeEdge(s.p0, s.p1, opt, s.group));
  } else {
    // Object-space hidden-line: clip each edge to its visible spans against a
    // throwaway mesh BVH (CueMol RendIntData_AABBTree ported to Embree). The
    // exclude set is the edge's own incident faces (+ any 1-ring the extractor
    // populated), so a grazing silhouette is not self-hidden.
    const detail::EdgeBVH bvh = detail::buildEdgeMeshBVH(scene.mesh);
    // Subdivision spacing ~ edgeWidth/2 (CueMol calcSilhIntrsec divw); fall back
    // to the segment length when width is unset so a degenerate width still emits.
    const float step = opt.width > 0.0f ? 0.5f * opt.width : 0.0f;
    std::vector<int> excl;
    for (const FeatureSeg& s : fm.segs) {
      excl.clear();
      if (s.face0 >= 0) excl.push_back(s.face0);
      if (s.face1 >= 0) excl.push_back(s.face1);
      for (int f : s.excludeFaces)
        if (f >= 0) excl.push_back(f);
      const int* ep2 = excl.empty() ? nullptr : excl.data();
      const auto spans = detail::clipSegmentToVisibleSpans(
          bvh, scene.camera, s.p0, s.p1, ep2, static_cast<int>(excl.size()), step);
      for (const auto& sp : spans)
        edges.push_back(makeEdge(sp.first, sp.second, opt, s.group));
    }
  }

  scene.cylinders.insert(scene.cylinders.end(), edges.begin(), edges.end());
}

}  // namespace umbreon
