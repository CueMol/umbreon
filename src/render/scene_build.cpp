#include "render/scene_build.hpp"

#include <array>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <embree4/rtcore.h>

namespace umbreon {
namespace detail {

BuiltScene buildEmbreeScene(RTCDevice device, const Scene& scene) {
  RTCScene rscene = rtcNewScene(device);
  rtcSetSceneFlags(rscene, RTC_SCENE_FLAG_ROBUST);
  // Static offline scene: committed once, then traversed by every primary ray
  // (and every future AO/shadow ray). A one-time HIGH-quality BVH (spatial
  // splits) amortizes over the whole frame, so HIGH is the right default vs
  // Embree's MEDIUM, as OSPRay builds its static scenes. Pure traversal-speed
  // win; it cannot change which primitive a ray hits.
  rtcSetSceneBuildQuality(rscene, RTC_BUILD_QUALITY_HIGH);

  const Mesh& m = scene.mesh;

  // Instance offsets are baked into the geometry (the .pov scenes have none;
  // the legacy grid path replicates each primitive per offset).
  std::vector<Vec3> bakeOffsets = scene.instanceOffsets;
  if (bakeOffsets.empty()) bakeOffsets.push_back(Vec3{0.0f, 0.0f, 0.0f});

  // Map geomID -> kind/handle so the shader can interpolate or read flat color.
  std::vector<GeomRecord> records;

  // --- triangle mesh (de-indexed, replicated per instance offset) ---
  if (m.vertexCount() >= 3) {
    const std::size_t baseV = m.positions.size();
    const std::size_t copies = bakeOffsets.size();
    const std::size_t nV = baseV * copies;
    const std::size_t nT = nV / 3;

    RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    auto* pos = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), nV));
    auto* idx = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned int),
        nT));

    // Two vertex-attribute slots: 0 = shading normal (FLOAT3), 1 = pigment
    // color RGBA (FLOAT4). Interpolated with the hit barycentrics.
    rtcSetGeometryVertexAttributeCount(g, 2);
    auto* nrm = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
        3 * sizeof(float), nV));
    auto* col = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT4,
        4 * sizeof(float), nV));

    for (std::size_t c = 0; c < copies; ++c) {
      const Vec3 off = bakeOffsets[c];
      for (std::size_t v = 0; v < baseV; ++v) {
        const std::size_t d = c * baseV + v;
        const Vec3 p = m.positions[v] + off;
        const Vec3 n = m.normals[v];
        const Vec4 cc = m.colors[v];
        pos[d * 3 + 0] = p.x;
        pos[d * 3 + 1] = p.y;
        pos[d * 3 + 2] = p.z;
        nrm[d * 3 + 0] = n.x;
        nrm[d * 3 + 1] = n.y;
        nrm[d * 3 + 2] = n.z;
        col[d * 4 + 0] = cc.x;
        col[d * 4 + 1] = cc.y;
        col[d * 4 + 2] = cc.z;
        col[d * 4 + 3] = cc.w;
      }
    }
    for (std::size_t t = 0; t < nT; ++t) {
      idx[t * 3 + 0] = static_cast<unsigned int>(3 * t + 0);
      idx[t * 3 + 1] = static_cast<unsigned int>(3 * t + 1);
      idx[t * 3 + 2] = static_cast<unsigned int>(3 * t + 2);
    }

    rtcCommitGeometry(g);
    unsigned int id = rtcAttachGeometry(rscene, g);
    if (id >= records.size()) records.resize(id + 1);
    records[id] = {GeomKind::Mesh, g};
    rtcReleaseGeometry(g);
  }

  // --- spheres (CueMol balls / silhouette joints) ---
  std::vector<Vec4> sphereColor;     // indexed by primID
  std::vector<Material> sphereMat;   // parallel to sphereColor (primID order)
  std::vector<uint16_t> sphereGroup; // parallel: transparency group (section)
  if (!scene.spheres.empty()) {
    const std::size_t n = scene.spheres.size() * bakeOffsets.size();
    RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SPHERE_POINT);
    auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float), n));
    sphereColor.reserve(n);
    sphereMat.reserve(n);
    sphereGroup.reserve(n);
    std::size_t k = 0;
    for (const Vec3& off : bakeOffsets) {
      for (const Sphere& s : scene.spheres) {
        vb[k * 4 + 0] = s.center.x + off.x;
        vb[k * 4 + 1] = s.center.y + off.y;
        vb[k * 4 + 2] = s.center.z + off.z;
        vb[k * 4 + 3] = s.radius;
        sphereColor.push_back(s.color);
        sphereMat.push_back(s.material);
        sphereGroup.push_back(s.group);
        ++k;
      }
    }
    rtcCommitGeometry(g);
    unsigned int id = rtcAttachGeometry(rscene, g);
    if (id >= records.size()) records.resize(id + 1);
    records[id] = {GeomKind::Sphere, g};
    rtcReleaseGeometry(g);
  }

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
  std::vector<Vec4> cylColor;      // ROUND edges: indexed by primID (per segment)
  std::vector<Material> cylMat;    // parallel to cylColor (primID order)
  std::vector<uint16_t> cylGroup;  // parallel: transparency group (section)
  std::vector<float> cylOpacity1;  // parallel: p1 opacity (< 0 = uniform)
  std::vector<Vec4> cylCapColor;      // CONE bonds: indexed by primID (per segment)
  std::vector<Material> cylCapMat;    // parallel to cylCapColor (primID order)
  std::vector<uint16_t> cylCapGroup;  // parallel: transparency group (section)
  std::vector<float> cylCapOpacity1;  // parallel: p1 opacity (< 0 = uniform)
  if (!scene.cylinders.empty()) {
    // Partition source cylinders by cap semantics BEFORE chaining so the chain
    // builder only ever sees `open` edges (capped bonds must not be chained).
    std::vector<int> openIdx, capIdx;
    openIdx.reserve(scene.cylinders.size());
    capIdx.reserve(scene.cylinders.size());
    for (int i = 0; i < static_cast<int>(scene.cylinders.size()); ++i) {
      if (scene.cylinders[i].open) openIdx.push_back(i);
      else capIdx.push_back(i);
    }

    // ----- (1) OPEN edges: ROUND_LINEAR_CURVE chained (seam fix, unchanged) ---
    if (!openIdx.empty()) {
      const std::size_t no = openIdx.size();
      const std::size_t segs = no * bakeOffsets.size();
      cylColor.reserve(segs);
      cylMat.reserve(segs);
      cylGroup.reserve(segs);
      cylOpacity1.reserve(segs);

      // Curve buffers, built in chain order. Vertices are shared within a chain
      // (chainLen+1 vertices per chain), so segment j of a chain uses vertices
      // [base+j, base+j+1] and its neighbor flags reference base+j-1 / base+j+2,
      // which stay inside the chain (LEFT only when j>0, RIGHT only when j<n-1).
      std::vector<float> vbuf;          // 4 floats per vertex (x, y, z, radius)
      std::vector<unsigned int> ibuf;   // start-vertex index per segment
      std::vector<unsigned char> fbuf;  // RTCCurveFlags per segment
      vbuf.reserve((segs + no) * 4);
      ibuf.reserve(segs);
      fbuf.reserve(segs);

      const float eps = 1.0e-4f;  // endpoint match tolerance (<< radius ~0.03)
      auto keyOf = [&](const Vec3& p) {
        return std::array<long long, 3>{std::llround(p.x / eps),
                                        std::llround(p.y / eps),
                                        std::llround(p.z / eps)};
      };

      for (const Vec3& off : bakeOffsets) {
        // Baked endpoints for this instance (local index 0..no-1 over openIdx).
        std::vector<Vec3> a0(no), a1(no);
        for (std::size_t i = 0; i < no; ++i) {
          const Cylinder& c = scene.cylinders[openIdx[i]];
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
          const Cylinder& cs = scene.cylinders[openIdx[s]];
          const Cylinder& ct = scene.cylinders[openIdx[t]];
          return cs.group == ct.group &&
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
          pushV(a0[chain[0]], scene.cylinders[openIdx[chain[0]]].radius);
          for (int cs : chain)
            pushV(a1[cs], scene.cylinders[openIdx[cs]].radius);
          const int n = static_cast<int>(chain.size());
          for (int j = 0; j < n; ++j) {
            ibuf.push_back(base + static_cast<unsigned int>(j));
            unsigned char fl = 0;
            if (j > 0) fl |= RTC_CURVE_FLAG_NEIGHBOR_LEFT;
            if (j < n - 1) fl |= RTC_CURVE_FLAG_NEIGHBOR_RIGHT;
            fbuf.push_back(fl);
            const Cylinder& c = scene.cylinders[openIdx[chain[j]]];
            cylColor.push_back(c.color);
            cylMat.push_back(c.material);
            cylGroup.push_back(c.group);
            cylOpacity1.push_back(c.opacity1);
          }
        };
        // Open chains first (start = no predecessor), then any leftover cycles.
        for (int s = 0; s < static_cast<int>(no); ++s)
          if (prev[s] < 0 && !visited[s]) emitChain(s);
        for (int s = 0; s < static_cast<int>(no); ++s)
          if (!visited[s]) emitChain(s);
      }

      const std::size_t nSeg = ibuf.size();
      const std::size_t nVert = vbuf.size() / 4;
      RTCGeometry g =
          rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE);
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
      rtcCommitGeometry(g);
      unsigned int id = rtcAttachGeometry(rscene, g);
      if (id >= records.size()) records.resize(id + 1);
      records[id] = {GeomKind::Cylinder, g};
      rtcReleaseGeometry(g);
    }

    // ----- (2) CAPPED bonds: CONE_LINEAR_CURVE, one segment per primID -------
    // Independent (unchained) segments: each control-point pair gets a flat disk
    // cap at p0 and p1. No flags buffer is needed -- a cone is discontinuous at
    // edge boundaries, and the flat caps occupy zero axial thickness so they
    // hide inside the overlap of consecutive bonds regardless of neighbors.
    if (!capIdx.empty()) {
      const std::size_t segs = capIdx.size() * bakeOffsets.size();
      cylCapColor.reserve(segs);
      cylCapMat.reserve(segs);
      cylCapGroup.reserve(segs);
      cylCapOpacity1.reserve(segs);

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
          cylCapColor.push_back(c.color);
          cylCapMat.push_back(c.material);
          cylCapGroup.push_back(c.group);
          cylCapOpacity1.push_back(c.opacity1);
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
      if (id >= records.size()) records.resize(id + 1);
      records[id] = {GeomKind::CylinderCapped, g};
      rtcReleaseGeometry(g);
    }
  }

  rtcCommitScene(rscene);
  if (RTCError err = rtcGetDeviceError(device); err != RTC_ERROR_NONE) {
    rtcReleaseScene(rscene);
    throw std::runtime_error(std::string("embree scene build failed: ") +
                             rtcErrorString(err));
  }

  BuiltScene out;
  out.scene = rscene;
  out.records = std::move(records);
  out.sphereColor = std::move(sphereColor);
  out.sphereMat = std::move(sphereMat);
  out.sphereGroup = std::move(sphereGroup);
  out.cylColor = std::move(cylColor);
  out.cylMat = std::move(cylMat);
  out.cylGroup = std::move(cylGroup);
  out.cylOpacity1 = std::move(cylOpacity1);
  out.cylCapColor = std::move(cylCapColor);
  out.cylCapMat = std::move(cylCapMat);
  out.cylCapGroup = std::move(cylCapGroup);
  out.cylCapOpacity1 = std::move(cylCapOpacity1);
  return out;
}

}  // namespace detail
}  // namespace umbreon
