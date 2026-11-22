#include "render/curve_build.hpp"

#include <array>
#include <cstring>
#include <map>
#include <utility>
#include <vector>

#include <embree4/rtcore.h>

namespace umbreon {
namespace detail {

// --- cylinders: two POV cap semantics, two Embree geometries ---------------
// POV emits two kinds of cylinders, distinguished by the parser's `open` flag:
//
//  (1) `open` (capless) silhouette EDGES -> ROUND_LINEAR_CURVE, chained.
//      POV draws silhouette edges as a union of short `open` cylinders that
//      share endpoints. Rendering each as an independent ROUND_LINEAR_CURVE
//      capsule adds a hemispherical cap at every shared vertex, so two caps
//      stack at each joint; through a transparent (faded) edge the extra cap
//      layers multiply the transmittance and show as dark beads at the joints.
//      We therefore stitch the segments into connected polylines and tag each
//      with RTC_CURVE_FLAG_NEIGHBOR_*, so Embree drops the internal caps and a
//      joint becomes a single shared swept-sphere -- POV's seamless union.
//
//  (2) CAPPED (CLOSED) bonds/wireframes -> CONE_LINEAR_CURVE, unchained.
//      POV stick bonds are plain cylinder{p0,p1,r} with FLAT disk caps at the
//      exact endpoints. Consecutive bonds OVERLAP but do not share endpoints
//      and carry different colors, so they cannot chain. A ROUND cap would
//      poke ~radius past its endpoint into the overlap and (with no neighbor
//      link) never get clipped -- a hemispherical bulge that protrudes through
//      an overlapping transparent surface (the red arc artifact). A
//      CONE_LINEAR_CURVE segment ("capped cone, discontinuous at edge
//      boundaries") has a flat cap of zero axial thickness at each endpoint,
//      so it hides inside the overlap exactly as POV's disk caps do.
//
// The two geometries have independent primID spaces (Embree restarts primID at
// 0 per geometry), so each gets its own primID-indexed side-tables and its own
// GeomKind; shadeHit dispatches on the kind.
//
// Object-space edge lines (Cylinder::objEdge) are open too, but go into their
// own chained ROUND_LINEAR_CURVE geometry (GeomKind::EdgeLine) with the edge
// ray mask, so secondary rays can skip them (see kEdgeGeomMask).

namespace {

// One emitted segment of a chained curve: the source cylinder and the bake
// instance it belongs to, in primID order.
struct ChainSeg {
  int cyl;
  std::size_t inst;
};

// Stitch the open cylinders `idx` (indices into scene.cylinders) into
// connected polylines per bake instance and append their ROUND_LINEAR_CURVE
// buffers. Vertices are shared within a chain (chainLen+1 vertices per chain),
// so segment j of a chain uses vertices [base+j, base+j+1] and its neighbor
// flags reference base+j-1 / base+j+2, which stay inside the chain (LEFT only
// when j>0, RIGHT only when j<n-1). `segs` receives the primID-ordered source.
void buildChains(const Scene& scene, const std::vector<int>& idx,
                 const std::vector<Vec3>& bakeOffsets, std::vector<float>& vbuf,
                 std::vector<unsigned int>& ibuf,
                 std::vector<unsigned char>& fbuf, std::vector<ChainSeg>& segs) {
  const std::size_t no = idx.size();
  const float eps = 1.0e-4f;  // endpoint match tolerance (<< radius ~0.03)
  auto keyOf = [&](const Vec3& p) {
    return std::array<long long, 3>{std::llround(p.x / eps),
                                    std::llround(p.y / eps),
                                    std::llround(p.z / eps)};
  };
  for (std::size_t inst = 0; inst < bakeOffsets.size(); ++inst) {
    const Vec3& off = bakeOffsets[inst];
    // Baked endpoints for this instance (local index 0..no-1 over idx).
    std::vector<Vec3> a0(no), a1(no);
    for (std::size_t i = 0; i < no; ++i) {
      const Cylinder& c = scene.cylinders[idx[i]];
      a0[i] = {c.p0.x + off.x, c.p0.y + off.y, c.p0.z + off.z};
      a1[i] = {c.p1.x + off.x, c.p1.y + off.y, c.p1.z + off.z};
    }
    // Vertex incidence: key -> list of (segment, end) with end 0=p0, 1=p1.
    std::map<std::array<long long, 3>, std::vector<std::pair<int, int>>> inc;
    for (int i = 0; i < static_cast<int>(no); ++i) {
      inc[keyOf(a0[i])].push_back({i, 0});
      inc[keyOf(a1[i])].push_back({i, 1});
    }
    auto compatible = [&](int s, int t) {
      const Cylinder& cs = scene.cylinders[idx[s]];
      const Cylinder& ct = scene.cylinders[idx[t]];
      return cs.group == ct.group && cs.objEdge == ct.objEdge &&
             (cs.opacity1 < 0.0f) == (ct.opacity1 < 0.0f);
    };
    // next[s]: the segment that continues from s.p1 (its p0 meets s.p1 at a
    // degree-2 vertex, same direction, compatible attributes). prev derived.
    std::vector<int> next(no, -1), prev(no, -1);
    for (int s = 0; s < static_cast<int>(no); ++s) {
      const auto& ends = inc[keyOf(a1[s])];
      if (ends.size() != 2) continue;
      int t = -1;
      for (const auto& e : ends)
        if (e.first != s) t = (e.second == 0) ? e.first : -1;
      if (t >= 0 && t != s && compatible(s, t)) next[s] = t;
    }
    for (int s = 0; s < static_cast<int>(no); ++s)
      if (next[s] >= 0) prev[next[s]] = s;

    std::vector<char> visited(no, 0);
    auto emitChain = [&](int start) {
      std::vector<int> chain;
      for (int s = start; s >= 0 && !visited[s]; s = next[s]) {
        visited[s] = 1;
        chain.push_back(s);
      }
      const unsigned int base = static_cast<unsigned int>(vbuf.size() / 4);
      auto pushV = [&](const Vec3& p, float r) {
        vbuf.push_back(p.x);
        vbuf.push_back(p.y);
        vbuf.push_back(p.z);
        vbuf.push_back(r);
      };
      pushV(a0[chain[0]], scene.cylinders[idx[chain[0]]].radius);
      for (int cs : chain) pushV(a1[cs], scene.cylinders[idx[cs]].radius);
      const int n = static_cast<int>(chain.size());
      for (int j = 0; j < n; ++j) {
        ibuf.push_back(base + static_cast<unsigned int>(j));
        unsigned char fl = 0;
        if (j > 0) fl |= RTC_CURVE_FLAG_NEIGHBOR_LEFT;
        if (j < n - 1) fl |= RTC_CURVE_FLAG_NEIGHBOR_RIGHT;
        fbuf.push_back(fl);
        segs.push_back(ChainSeg{idx[chain[j]], inst});
      }
    };
    // Open chains first (start = no predecessor), then any leftover cycles.
    for (int s = 0; s < static_cast<int>(no); ++s)
      if (prev[s] < 0 && !visited[s]) emitChain(s);
    for (int s = 0; s < static_cast<int>(no); ++s)
      if (!visited[s]) emitChain(s);
  }
}

// Attach a chained ROUND_LINEAR_CURVE geometry built by buildChains.
unsigned int attachChains(RTCDevice device, RTCScene rscene,
                          const std::vector<float>& vbuf,
                          const std::vector<unsigned int>& ibuf,
                          const std::vector<unsigned char>& fbuf,
                          unsigned int mask, RTCGeometry& gOut) {
  const std::size_t nSeg = ibuf.size();
  const std::size_t nVert = vbuf.size() / 4;
  RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE);
  auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float),
      nVert));
  std::memcpy(vb, vbuf.data(), vbuf.size() * sizeof(float));
  auto* ib = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(unsigned int),
      nSeg));
  std::memcpy(ib, ibuf.data(), ibuf.size() * sizeof(unsigned int));
  auto* fb = static_cast<unsigned char*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_FLAGS, 0, RTC_FORMAT_UCHAR, sizeof(unsigned char),
      nSeg));
  std::memcpy(fb, fbuf.data(), fbuf.size() * sizeof(unsigned char));
  rtcSetGeometryMask(g, mask);
  rtcCommitGeometry(g);
  gOut = g;
  return rtcAttachGeometry(rscene, g);
}

}  // namespace

void buildCylinderGeometry(RTCDevice device, RTCScene rscene, const Scene& scene,
                           const std::vector<Vec3>& bakeOffsets, BuiltScene& out,
                           bool buildEdgeTables, bool forceAxisTables) {
  if (scene.cylinders.empty()) return;

  // Per-segment axis side tables are only needed when an anisotropic
  // principled cylinder exists (the principled tangent frame) or the NPR
  // hatch asks for the same frame; POV scenes with neither allocate nothing.
  bool buildAxisTables = forceAxisTables;
  if (!buildAxisTables)
    for (const Cylinder& c : scene.cylinders)
      if (c.material.model == ShadingModel::Principled &&
          c.material.pbr.anisotropy != 0.0f) {
        buildAxisTables = true;
        break;
      }

  // Partition source cylinders by cap semantics BEFORE chaining so the chain
  // builder only ever sees `open` edges (capped bonds must not be chained);
  // object-space edge lines get their own geometry.
  std::vector<int> openIdx, capIdx, edgeIdx;
  openIdx.reserve(scene.cylinders.size());
  capIdx.reserve(scene.cylinders.size());
  for (int i = 0; i < static_cast<int>(scene.cylinders.size()); ++i) {
    const Cylinder& c = scene.cylinders[i];
    if (c.objEdge != 0) edgeIdx.push_back(i);
    else if (c.open) openIdx.push_back(i);
    else capIdx.push_back(i);
  }

  // ----- (1) OPEN edges: ROUND_LINEAR_CURVE chained (seam fix) -------------
  if (!openIdx.empty()) {
    const std::size_t segsN = openIdx.size() * bakeOffsets.size();
    std::vector<float> vbuf;          // 4 floats per vertex (x, y, z, radius)
    std::vector<unsigned int> ibuf;   // start-vertex index per segment
    std::vector<unsigned char> fbuf;  // RTCCurveFlags per segment
    std::vector<ChainSeg> segs;
    vbuf.reserve((segsN + openIdx.size()) * 4);
    ibuf.reserve(segsN);
    fbuf.reserve(segsN);
    segs.reserve(segsN);
    buildChains(scene, openIdx, bakeOffsets, vbuf, ibuf, fbuf, segs);
    out.cylColor.reserve(segs.size());
    out.cylMat.reserve(segs.size());
    out.cylGroup.reserve(segs.size());
    out.cylOpacity1.reserve(segs.size());
    if (buildEdgeTables) out.cylMatIndex.reserve(segs.size());
    for (const ChainSeg& cs : segs) {
      const Cylinder& c = scene.cylinders[cs.cyl];
      // Edge pass: raw per-segment material index in primID (= push) order.
      if (buildEdgeTables)
        out.cylMatIndex.push_back(static_cast<uint32_t>(out.cylMatIndex.size()));
      out.cylColor.push_back(c.color);
      out.cylMat.push_back(c.material);
      out.cylGroup.push_back(c.group);
      out.cylOpacity1.push_back(c.opacity1);
      out.cylFromEdge.push_back(c.fromEdgeMacro ? 1 : 0);
      if (buildAxisTables) {
        const Vec3& off = bakeOffsets[cs.inst];
        const Vec3 a0{c.p0.x + off.x, c.p0.y + off.y, c.p0.z + off.z};
        const Vec3 a1{c.p1.x + off.x, c.p1.y + off.y, c.p1.z + off.z};
        out.cylAxis.push_back(
            safeNormalize(Vec3{a1.x - a0.x, a1.y - a0.y, a1.z - a0.z}));
      }
    }
    if (buildEdgeTables) out.cylMatCount = static_cast<uint32_t>(ibuf.size());
    RTCGeometry g = nullptr;
    const unsigned int id =
        attachChains(device, rscene, vbuf, ibuf, fbuf, 0xFFFFFFFFu, g);
    if (id >= out.records.size()) out.records.resize(id + 1);
    out.records[id] = {GeomKind::Cylinder, g};
    rtcReleaseGeometry(g);
  }

  // ----- (1b) object-space edge lines: chained, own geometry + mask --------
  if (!edgeIdx.empty()) {
    std::vector<float> vbuf;
    std::vector<unsigned int> ibuf;
    std::vector<unsigned char> fbuf;
    std::vector<ChainSeg> segs;
    buildChains(scene, edgeIdx, bakeOffsets, vbuf, ibuf, fbuf, segs);
    out.edgeLineColor.reserve(segs.size());
    out.edgeLineGroup.reserve(segs.size());
    out.edgeLineKind.reserve(segs.size());
    out.edgeLineA.reserve(segs.size());
    out.edgeLineB.reserve(segs.size());
    for (const ChainSeg& cs : segs) {
      const Cylinder& c = scene.cylinders[cs.cyl];
      out.edgeLineColor.push_back(c.color);
      out.edgeLineGroup.push_back(c.group);
      out.edgeLineKind.push_back(c.objEdge);
      const Vec3& off = bakeOffsets[cs.inst];
      out.edgeLineA.push_back(Vec3{c.p0.x + off.x, c.p0.y + off.y, c.p0.z + off.z});
      out.edgeLineB.push_back(Vec3{c.p1.x + off.x, c.p1.y + off.y, c.p1.z + off.z});
    }
    RTCGeometry g = nullptr;
    const unsigned int id =
        attachChains(device, rscene, vbuf, ibuf, fbuf, kEdgeGeomMask, g);
    if (id >= out.records.size()) out.records.resize(id + 1);
    out.records[id] = {GeomKind::EdgeLine, g};
    rtcReleaseGeometry(g);
  }

  // ----- (2) CAPPED bonds: CONE_LINEAR_CURVE, one segment per primID -------
  // Independent (unchained) segments: each control-point pair gets a flat disk
  // cap at p0 and p1. No flags buffer is needed -- a cone is discontinuous at
  // edge boundaries, and the flat caps occupy zero axial thickness so they
  // hide inside the overlap of consecutive bonds regardless of neighbors.
  if (!capIdx.empty()) {
    const std::size_t segs = capIdx.size() * bakeOffsets.size();
    out.cylCapColor.reserve(segs);
    out.cylCapMat.reserve(segs);
    out.cylCapGroup.reserve(segs);
    out.cylCapOpacity1.reserve(segs);
    if (buildEdgeTables) {
      out.cylCapMatIndex.reserve(segs);
      out.cylCapMatCount = static_cast<uint32_t>(segs);
    }

    std::vector<float> vbuf;         // 4 floats per vertex (x, y, z, radius)
    std::vector<unsigned int> ibuf;  // start-vertex index per segment
    vbuf.reserve(segs * 2 * 4);
    ibuf.reserve(segs);

    for (const Vec3& off : bakeOffsets) {
      for (int idx : capIdx) {
        const Cylinder& c = scene.cylinders[idx];
        const unsigned int base = static_cast<unsigned int>(vbuf.size() / 4);
        vbuf.push_back(c.p0.x + off.x);
        vbuf.push_back(c.p0.y + off.y);
        vbuf.push_back(c.p0.z + off.z);
        vbuf.push_back(c.radius);
        vbuf.push_back(c.p1.x + off.x);
        vbuf.push_back(c.p1.y + off.y);
        vbuf.push_back(c.p1.z + off.z);
        vbuf.push_back(c.radius);
        ibuf.push_back(base);  // segment uses vertices [base, base+1]
        if (buildEdgeTables)
          out.cylCapMatIndex.push_back(
              static_cast<uint32_t>(out.cylCapMatIndex.size()));
        out.cylCapColor.push_back(c.color);
        out.cylCapMat.push_back(c.material);
        out.cylCapGroup.push_back(c.group);
        out.cylCapOpacity1.push_back(c.opacity1);
        out.cylCapFromEdge.push_back(c.fromEdgeMacro ? 1 : 0);
        if (buildAxisTables)
          out.cylCapAxis.push_back(safeNormalize(
              Vec3{c.p1.x - c.p0.x, c.p1.y - c.p0.y, c.p1.z - c.p0.z}));
      }
    }

    const std::size_t nSeg = ibuf.size();
    const std::size_t nVert = vbuf.size() / 4;
    RTCGeometry g =
        rtcNewGeometry(device, RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE);
    auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float),
        nVert));
    std::memcpy(vb, vbuf.data(), vbuf.size() * sizeof(float));
    auto* ib = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(unsigned int),
        nSeg));
    std::memcpy(ib, ibuf.data(), ibuf.size() * sizeof(unsigned int));
    rtcCommitGeometry(g);
    unsigned int id = rtcAttachGeometry(rscene, g);
    if (id >= out.records.size()) out.records.resize(id + 1);
    out.records[id] = {GeomKind::CylinderCapped, g};
    rtcReleaseGeometry(g);
  }
}

// Object-space edge arcs (Scene::edgeArcs): one ROUND_BEZIER_CURVE segment per
// arc (4 control points of its own), in a geometry with the edge ray mask. A
// round cubic curve is a tube swept along the curve with no end spheres, so
// tangent-continuous arcs of one circle join without overlap.
void buildEdgeArcGeometry(RTCDevice device, RTCScene rscene, const Scene& scene,
                          const std::vector<Vec3>& bakeOffsets,
                          BuiltScene& out) {
  if (scene.edgeArcs.empty()) return;
  const std::size_t segs = scene.edgeArcs.size() * bakeOffsets.size();
  std::vector<float> vbuf;
  std::vector<unsigned int> ibuf;
  vbuf.reserve(segs * 16);
  ibuf.reserve(segs);
  out.edgeArcColor.reserve(segs);
  out.edgeArcGroup.reserve(segs);
  out.edgeArcKind.reserve(segs);
  for (const Vec3& off : bakeOffsets) {
    for (const EdgeArc& a : scene.edgeArcs) {
      ibuf.push_back(static_cast<unsigned int>(vbuf.size() / 4));
      std::array<Vec3, 4> cp;
      for (int k = 0; k < 4; ++k)
        cp[static_cast<std::size_t>(k)] =
            Vec3{a.cp[k].x + off.x, a.cp[k].y + off.y, a.cp[k].z + off.z};
      out.edgeArcCP.push_back(cp);
      for (const Vec3& p : a.cp) {
        vbuf.push_back(p.x + off.x);
        vbuf.push_back(p.y + off.y);
        vbuf.push_back(p.z + off.z);
        vbuf.push_back(a.radius);
      }
      out.edgeArcColor.push_back(a.color);
      out.edgeArcGroup.push_back(a.group);
      out.edgeArcKind.push_back(a.kind);
    }
  }
  RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);
  auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float),
      vbuf.size() / 4));
  std::memcpy(vb, vbuf.data(), vbuf.size() * sizeof(float));
  auto* ib = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(unsigned int),
      ibuf.size()));
  std::memcpy(ib, ibuf.data(), ibuf.size() * sizeof(unsigned int));
  rtcSetGeometryMask(g, kEdgeGeomMask);
  rtcCommitGeometry(g);
  const unsigned int id = rtcAttachGeometry(rscene, g);
  if (id >= out.records.size()) out.records.resize(id + 1);
  out.records[id] = {GeomKind::EdgeArc, g};
  rtcReleaseGeometry(g);
}

}  // namespace detail
}  // namespace umbreon
