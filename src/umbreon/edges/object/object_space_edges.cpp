// Analytic object-space silhouette-edge generation. See object_space_edges.hpp.
#include "edges/object/object_space_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "edges/object/analytic_silhouette.hpp"
#include "edges/object/mesh_feature_edges.hpp"
#include "scene.hpp"

namespace umbreon {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// The edge style one section resolves to: its Scene::groupObjectEdgeStyle
// entry, or the global ObjectSpaceEdgeOptions when the table has none.
struct Style {
  bool analyticSil = true;  // sphere / cylinder contours
  bool meshSil = true;
  bool crease = false;
  bool border = true;
  bool outline = false;
  Vec3 color{0.0f, 0.0f, 0.0f};
  float width = 0.03f;  // line radius
  float raise = 0.0f;
};

Style resolveStyle(const Scene& scene, const ObjectSpaceEdgeOptions& opt,
                   std::uint16_t group) {
  Style st;
  if (group < scene.groupObjectEdgeStyle.size()) {
    const ObjectEdgeStyle& s = scene.groupObjectEdgeStyle[group];
    st.analyticSil = s.silhouette;
    st.meshSil = s.silhouette;
    st.crease = s.crease;
    st.border = s.border;
    st.outline = s.mode == SilhouetteMode::Outline;
    st.color = Vec3{s.color[0], s.color[1], s.color[2]};
    st.width = s.width;
    st.raise = s.raise;
  } else {
    st.analyticSil = true;
    st.meshSil = opt.meshSilhouette;
    st.crease = opt.meshCrease;
    st.border = opt.meshBorder;
    st.outline = false;
    st.color = Vec3{opt.color[0], opt.color[1], opt.color[2]};
    st.width = opt.width;
    st.raise = opt.raise;
  }
  if (st.width < 0.0f) st.width = 0.0f;  // guard a negative CLI width
  return st;
}

// A raw silhouette segment (pre-clip): the two endpoints plus the source
// primitive's section. Collected first so the union-boundary clip can split
// each one against the other primitives before it becomes edge geometry.
struct RawSeg {
  Vec3 a, b;
  uint16_t group;
  // Source primitive (so the union clip never clips a segment against its OWN
  // primitive: a perspective cylinder side line is a chord that can dip inside
  // its own cylinder). -1 = none.
  int srcSphere = -1;
  int srcCyl = -1;
};

// Build one emitted (linear) edge line. Open (capless, chained into
// ROUND_LINEAR_CURVE) and tagged objEdge, which the renderer draws in its exact
// color, never shadows and hides from secondary rays.
Cylinder makeEdge(const Vec3& a, const Vec3& b, const Style& st,
                  uint16_t group) {
  Cylinder c;
  c.p0 = a;
  c.p1 = b;
  c.radius = st.width;
  c.color = Vec4{st.color.x, st.color.y, st.color.z, 1.0f};
  c.material = Material::flatOutline();
  c.group = group;
  c.opacity1 = -1.0f;
  c.open = true;
  c.fromEdgeMacro = false;
  c.objEdge = st.outline ? kObjEdgeOutline : kObjEdgeFull;
  return c;
}

// --- union-boundary clip --------------------------------------------------
// A silhouette point belongs to the section's union boundary only where it is
// not inside another primitive's solid of the SAME section; the inside parts
// are where a connecting primitive's surface takes over. Clipping there makes
// a bond's silhouette terminate at the atom it enters (and vice versa) instead
// of the two per-primitive contours crossing and leaving a "junction notch".
// Solids of OTHER sections never clip: a neighbor section may be translucent
// or cut by the slab, and whatever it really hides is left to the tracer.
// Line-derived cylinders (noEdge) and generated edges never clip either.
//
// The inset is STRICT (eps = 0: drop only points genuinely inside another
// solid). It is deliberately NOT an outward margin: in a densely packed
// molecule nearly every primitive's surface runs within such a margin of some
// neighbor, and the outward band chopped the continuous outline into dashes.

// Extra growth of the clip solids, so a contour point on an equal-radius
// neighbour's surface (a licorice ring around its own bond) counts as inside
// it despite rounding.
constexpr float kClipEps = 1.0e-4f;

bool insideSphere(const Vec3& X, const Sphere& s, float grow) {
  return length(X - s.center) < s.radius + grow;
}

bool insideCylinder(const Vec3& X, const Cylinder& c, float grow) {
  const Vec3 ab = c.p1 - c.p0;
  const float L = length(ab);
  if (L <= 0.0f) return false;
  const Vec3 u = ab * (1.0f / L);
  const float t = dot(X - c.p0, u);
  if (t < 0.0f || t > L) return false;  // beyond the flat caps
  const Vec3 perp = (X - c.p0) - u * t;
  return length(perp) < c.radius + grow;
}

// True where `X` is inside any ORIGINAL solid of section `group` other than the
// contour's source, each grown by `grow`. A sphere ring is clipped by every
// bond, thinner ones too: where a bond leaves the atom, the ring runs inside
// the bond, and POV-Ray (the reference) draws no ring there.
bool insideAnySolid(const Vec3& X, const Scene& scene, std::size_t nSph,
                    std::size_t nCyl, uint16_t group, int skipSphere,
                    int skipCyl, float grow) {
  for (std::size_t i = 0; i < nSph; ++i) {
    const Sphere& s = scene.spheres[i];
    if (static_cast<int>(i) == skipSphere || s.group != group || s.fromEdgeMacro)
      continue;
    if (insideSphere(X, s, grow)) return true;
  }
  for (std::size_t i = 0; i < nCyl; ++i) {
    const Cylinder& c = scene.cylinders[i];
    if (static_cast<int>(i) == skipCyl || c.group != group || c.noEdge ||
        c.fromEdgeMacro || c.objEdge != 0)
      continue;
    if (insideCylinder(X, c, grow)) return true;
  }
  return false;
}

// Sample `seg` and emit edge lines for the maximal runs that stay OUTSIDE every
// clip solid. Sampling spacing ~= line radius.
void clipAndEmit(const RawSeg& seg, const Style& st, const Scene& scene,
                 std::size_t nSph, std::size_t nCyl, bool clip,
                 std::vector<Cylinder>& out) {
  const Vec3 d = seg.b - seg.a;
  const float len = length(d);
  if (len <= 0.0f) return;
  if (!clip) {
    out.push_back(makeEdge(seg.a, seg.b, st, seg.group));
    return;
  }
  const float step = st.width > 0.0f ? st.width : len;
  const int K = std::max(2, static_cast<int>(std::ceil(len / step)));
  // The clip follows POV-Ray: whether a contour point is inside a solid is
  // decided at the surface point it was raised from (a cylinder side line is
  // raised radially off its axis), and a kept run is extended past each end
  // by the line radius, leaving the tracer to hide what goes behind the
  // neighbour. POV-Ray likewise draws a whole contour edge when one of its
  // ends is visible.
  const Cylinder* src =
      seg.srcCyl >= 0 ? &scene.cylinders[static_cast<std::size_t>(seg.srcCyl)]
                      : nullptr;
  auto surfacePoint = [&](const Vec3& X) {
    if (!src) return X;
    const Vec3 ax = src->p1 - src->p0;
    const float al = dot(ax, ax);
    if (al <= 0.0f) return X;
    const Vec3 onAxis = src->p0 + ax * (dot(X - src->p0, ax) / al);
    const Vec3 radial = X - onAxis;
    const float rl = length(radial);
    if (rl <= src->radius || rl <= 0.0f) return X;
    return onAxis + radial * (src->radius / rl);
  };
  auto insideAt = [&](float t) {
    return insideAnySolid(surfacePoint(seg.a + d * t), scene, nSph, nCyl,
                          seg.group, seg.srcSphere, seg.srcCyl, kClipEps);
  };
  const float ext = st.width / len;
  // The parameter where the line crosses a solid surface between an outside
  // sample tOut and an inside sample tIn, so a run ends at the surface rather
  // than up to one sample short of it (a visible gap at a joint).
  auto boundary = [&](float tOut, float tIn) {
    for (int it = 0; it < 12; ++it) {
      const float tm = 0.5f * (tOut + tIn);
      (insideAt(tm) ? tIn : tOut) = tm;
    }
    return tOut;
  };
  auto paramOf = [&](int i) {
    return static_cast<float>(i) / static_cast<float>(K);
  };
  auto emitRun = [&](float t0, float t1) {
    t0 = std::max(0.0f, t0);
    t1 = std::min(1.0f, t1);
    if (t1 > t0)
      out.push_back(makeEdge(seg.a + d * t0, seg.a + d * t1, st, seg.group));
  };
  float runStart = -1.0f;
  for (int i = 0; i <= K; ++i) {
    const bool inside = insideAt(paramOf(i));
    if (!inside && runStart < 0.0f)
      runStart = i == 0 ? 0.0f : boundary(paramOf(i), paramOf(i - 1)) - ext;
    if (inside && runStart >= 0.0f) {
      emitRun(runStart, boundary(paramOf(i - 1), paramOf(i)) + ext);
      runStart = -1.0f;
    }
  }
  if (runStart >= 0.0f) emitRun(runStart, 1.0f);  // run reached the far end
}

// --- circles as Bezier arcs -----------------------------------------------
// A sphere horizon ring / cylinder cap rim is drawn as the exact circle: cubic
// Bezier arcs of at most 45 degrees (radial error ~4e-6 of the radius), which
// the renderer sweeps as round tubes that join without overlap.

void emitArc(const AnalyticLoop& loop, float th0, float th1, const Style& st,
             std::vector<EdgeArc>& out) {
  const float span = th1 - th0;
  if (span <= 0.0f) return;
  const int n = std::max(
      1, static_cast<int>(std::ceil(span / (kPi / 4.0f) - 1.0e-4f)));
  const float h = span / static_cast<float>(n);
  const float k = (4.0f / 3.0f) * std::tan(h / 4.0f) * loop.cr;
  auto pointAt = [&](float th) {
    return loop.cc + (loop.ct * std::cos(th) + loop.cb * std::sin(th)) * loop.cr;
  };
  auto tangentAt = [&](float th) {
    return loop.ct * -std::sin(th) + loop.cb * std::cos(th);
  };
  for (int i = 0; i < n; ++i) {
    const float a0 = th0 + h * static_cast<float>(i);
    const float a1 = a0 + h;
    EdgeArc arc;
    arc.cp[0] = pointAt(a0);
    arc.cp[3] = pointAt(a1);
    arc.cp[1] = arc.cp[0] + tangentAt(a0) * k;
    arc.cp[2] = arc.cp[3] - tangentAt(a1) * k;
    arc.radius = st.width;
    arc.color = Vec4{st.color.x, st.color.y, st.color.z, 1.0f};
    arc.group = loop.group;
    arc.kind = st.outline ? kObjEdgeOutline : kObjEdgeFull;
    out.push_back(arc);
  }
}

// Emit the parts of a circle loop that lie outside every clip solid.
void clipAndEmitCircle(const AnalyticLoop& loop, const Style& st,
                       const Scene& scene, std::size_t nSph, std::size_t nCyl,
                       bool clip, std::vector<EdgeArc>& out,
                       std::vector<Cylinder>& caps) {
  if (loop.cr <= 0.0f) return;
  if (!clip) {
    emitArc(loop, 0.0f, 2.0f * kPi, st, out);
    return;
  }
  const float circ = 2.0f * kPi * loop.cr;
  const float step = st.width > 0.0f ? st.width : circ / 64.0f;
  const int K = std::max(16, static_cast<int>(std::ceil(circ / step)));
  auto angleOf = [&](int i) {
    return 2.0f * kPi * static_cast<float>(i) / static_cast<float>(K);
  };
  // As for the straight contours (clipAndEmit): the inside test is made at
  // the surface point (the raise scales the ring about its sphere center or
  // cap center), and a kept arc ends at the boundary with a round cap.
  Vec3 sc = loop.cc;
  float toSurface = loop.cr > st.raise ? (loop.cr - st.raise) / loop.cr : 1.0f;
  if (loop.srcSphere >= 0) {
    const Sphere& sp = scene.spheres[static_cast<std::size_t>(loop.srcSphere)];
    sc = sp.center;
    toSurface = sp.radius / (sp.radius + st.raise);
  }
  auto ringInside = [&](float th) {
    const Vec3 X =
        loop.cc + (loop.ct * std::cos(th) + loop.cb * std::sin(th)) * loop.cr;
    return insideAnySolid(sc + (X - sc) * toSurface, scene, nSph, nCyl,
                          loop.group, loop.srcSphere, loop.srcCyl, kClipEps);
  };
  // No extension past the boundary for an arc: its end gets a round cap
  // there instead (below), which reaches as far as the straight contours'
  // extension and keeps the end round.
  const float ext = 0.0f;
  std::vector<char> inside(static_cast<std::size_t>(K), 0);
  int firstInside = -1;
  for (int i = 0; i < K; ++i) {
    const float th = angleOf(i);
    if (ringInside(th)) {
      inside[static_cast<std::size_t>(i)] = 1;
      if (firstInside < 0) firstInside = i;
    }
  }
  if (firstInside < 0) {  // nothing clipped: the whole circle
    emitArc(loop, 0.0f, 2.0f * kPi, st, out);
    return;
  }
  // The angle where the ring crosses a solid surface between an outside
  // sample angle aOut and an inside one aIn, so an arc ends at the surface
  // rather than up to one sample short of it (a visible gap at a joint).
  auto boundary = [&](float aOut, float aIn) {
    for (int it = 0; it < 12; ++it) {
      const float am = 0.5f * (aOut + aIn);
      (ringInside(am) ? aIn : aOut) = am;
    }
    return aOut;
  };
  // Walk once around starting at an inside sample, so every outside run is
  // contiguous in the walk (a run may wrap past sample 0; the unwrapped angle
  // keeps growing past 2*pi, which the arc math accepts).
  float runStart = -1.0f;
  for (int j = 1; j <= K; ++j) {
    const int i = firstInside + j;
    const bool in = inside[static_cast<std::size_t>(i % K)] != 0;
    if (!in && runStart < 0.0f)
      runStart = boundary(angleOf(i), angleOf(i - 1)) - ext;
    if (in && runStart >= 0.0f) {
      const float end =
          std::min(boundary(angleOf(i - 1), angleOf(i)) + ext,
                   runStart + 2.0f * kPi);
      emitArc(loop, runStart, end, st, out);
      // A round Bezier tube has flat ends; where a clipped arc meets the next
      // contour at an angle that leaves a notch. A very short round line at
      // each end gives it the round end the straight contours have.
      for (const float th : {runStart, end}) {
        const Vec3 P = loop.cc +
                       (loop.ct * std::cos(th) + loop.cb * std::sin(th)) * loop.cr;
        const Vec3 T = loop.ct * -std::sin(th) + loop.cb * std::cos(th);
        caps.push_back(makeEdge(P, P + T * (1.0e-3f * loop.cr), st, loop.group));
      }
      runStart = -1.0f;
    }
  }
}

}  // namespace

void generateObjectSpaceEdges(Scene& scene, const ObjectSpaceEdgeOptions& opt,
                              const RenderProgress* progress) {
  if (!opt.enable) return;  // byte-identical default: no edges appended
  const auto cancelled = [progress] {
    return progress && progress->cancelRequested();
  };

  // SNAPSHOT the original primitive counts so the edges we append below are not
  // themselves silhouetted (and the clip tests only against the originals).
  const std::size_t sphereCount = scene.spheres.size();
  const std::size_t cylinderCount = scene.cylinders.size();

  // Resolved style per section, computed lazily by group id.
  std::vector<Style> styles;
  std::vector<char> resolved;
  auto styleOf = [&](std::uint16_t g) -> const Style& {
    if (g >= styles.size()) {
      styles.resize(static_cast<std::size_t>(g) + 1);
      resolved.resize(static_cast<std::size_t>(g) + 1, 0);
    }
    if (!resolved[g]) {
      styles[g] = resolveStyle(scene, opt, g);
      resolved[g] = 1;
    }
    return styles[g];
  };

  // 1) Analytic sphere / cylinder contours. Circles (sphere rings, end-on cap
  //    rims) become Bezier arcs; cylinder side lines stay linear. (The ring
  //    tessellation passed here only shapes the unused chord polygon.)
  std::vector<AnalyticLoop> loops;
  emitAnalyticSilhouettes(
      scene, scene.camera, 48,
      [&](std::uint16_t g) { return styleOf(g).raise; },
      /*circumscribe=*/false, loops);
  std::vector<Cylinder> edges;
  std::vector<EdgeArc> arcs;
  for (const AnalyticLoop& loop : loops) {
    if (cancelled()) return;  // nothing appended yet; render bails next
    const Style& st = styleOf(loop.group);
    if (!st.analyticSil) continue;
    if (loop.circle) {
      clipAndEmitCircle(loop, st, scene, sphereCount, cylinderCount, opt.clip,
                        arcs, edges);
      continue;
    }
    const std::size_t n = loop.pts.size();
    if (n < 2) continue;
    const std::size_t last = loop.closed ? n : n - 1;
    for (std::size_t i = 0; i < last; ++i) {
      const RawSeg seg{loop.pts[i], loop.pts[(i + 1) % n], loop.group,
                       loop.srcSphere, loop.srcCyl};
      clipAndEmit(seg, st, scene, sphereCount, cylinderCount, opt.clip, edges);
    }
  }
  if (cancelled()) return;

  // 2) Triangle-mesh edges (silhouette / crease / border), extracted inside
  //    each section with that section's raise and width. A nature is
  //    extracted when any section wants it and filtered per section below.
  //    The tracer occludes the resulting lines; nothing is pre-clipped.
  const Mesh& mesh = scene.mesh;
  ExtractParams ep;
  ep.raise = opt.raise;
  ep.width = opt.width;
  ep.silhouette = false;
  ep.crease = false;
  ep.border = false;
  ep.meshHardEdgeDeg = opt.meshHardEdgeDeg;
  ep.creaseAngleDeg = opt.creaseAngleDeg;
  ep.meshCreaseSmoothVetoDeg = opt.meshCreaseSmoothVetoDeg;
  ep.meshCreaseConvexOnly = opt.meshCreaseConvexOnly;
  ep.meshBorderCoplanarVetoDeg = opt.meshBorderCoplanarVetoDeg;
  ep.meshCreaseMaxDegree = opt.meshCreaseMaxDegree;
  ep.weldPerGroup = true;
  // Mesh silhouettes on mesh edges (face-normal straddle), as the POV-Ray
  // exporter draws them. The smooth n.v == 0 contour sits inside the faceted
  // outline wherever the vertex normals turn before the faces do, and the
  // tracer then hides the line under its own surface (a tapering, broken line).
  ep.geomSilhouette = true;
  // A line on a mesh edge does not sink into its own surface, so it needs no
  // eye-ward pull; POV-Ray uses none either, and the pull made the visible
  // line wider than POV-Ray's.
  ep.silhouetteCamBias = false;
  if (mesh.triangleCount() > 0) {
    std::uint16_t maxGroup = 0;
    for (std::size_t t = 0; t < mesh.triangleCount(); ++t)
      maxGroup = std::max(maxGroup, mesh.groupForTri(t));
    const std::size_t ng = static_cast<std::size_t>(maxGroup) + 1;
    ep.groupRaise.resize(ng);
    ep.groupWidth.resize(ng);
    std::vector<std::uint8_t> outline(ng, 0);
    bool anyOutline = false;
    for (std::size_t g = 0; g < ng; ++g) {
      const Style& st = styleOf(static_cast<std::uint16_t>(g));
      ep.groupRaise[g] = st.raise;
      ep.groupWidth[g] = st.width;
      ep.silhouette = ep.silhouette || st.meshSil;
      ep.crease = ep.crease || st.crease;
      ep.border = ep.border || st.border;
      if (st.outline && st.meshSil) {
        outline[g] = 1;
        anyOutline = true;
      }
    }
    if (anyOutline) ep.groupOutline = std::move(outline);
  }
  if (ep.silhouette || ep.crease || ep.border || !ep.groupOutline.empty()) {
    const FeatureMesh fm = extractMeshFeatureEdges(mesh, scene.camera, ep);
    for (const FeatureSeg& s : fm.segs) {
      const Style& st = styleOf(s.group);
      const bool keep = s.nature == EdgeNature::Silhouette ? st.meshSil
                        : s.nature == EdgeNature::Border   ? st.border
                                                           : st.crease;
      if (keep) edges.push_back(makeEdge(s.p0, s.p1, st, s.group));
    }
  }
  if (cancelled()) return;

  scene.cylinders.insert(scene.cylinders.end(), edges.begin(), edges.end());
  scene.edgeArcs.insert(scene.edgeArcs.end(), arcs.begin(), arcs.end());
}

}  // namespace umbreon
