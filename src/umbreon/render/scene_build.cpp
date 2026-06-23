#include "render/scene_build.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <embree4/rtcore.h>

#include "render/curve_build.hpp"

namespace umbreon {
namespace detail {
namespace {

// --- triangle mesh (de-indexed, replicated per instance offset) ---
void buildTriangleMesh(RTCDevice device, RTCScene rscene, const Mesh& m,
                       const std::vector<Vec3>& bakeOffsets, BuiltScene& out) {
  if (m.vertexCount() < 3) return;

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
  if (id >= out.records.size()) out.records.resize(id + 1);
  out.records[id] = {GeomKind::Mesh, g};
  rtcReleaseGeometry(g);
}

// --- spheres (CueMol balls / silhouette joints) ---
void buildSpheres(RTCDevice device, RTCScene rscene, const Scene& scene,
                  const std::vector<Vec3>& bakeOffsets, BuiltScene& out,
                  bool buildEdgeTables) {
  if (scene.spheres.empty()) return;

  const std::size_t n = scene.spheres.size() * bakeOffsets.size();
  RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SPHERE_POINT);
  auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float), n));
  out.sphereColor.reserve(n);
  out.sphereMat.reserve(n);
  out.sphereGroup.reserve(n);
  // Edge pass: raw per-primitive material index (phase-1, no dedup) so a pixel
  // can recover a global materialId. Only built when edges are enabled.
  if (buildEdgeTables) out.sphereMatIndex.reserve(n);
  std::size_t k = 0;
  for (const Vec3& off : bakeOffsets) {
    for (const Sphere& s : scene.spheres) {
      vb[k * 4 + 0] = s.center.x + off.x;
      vb[k * 4 + 1] = s.center.y + off.y;
      vb[k * 4 + 2] = s.center.z + off.z;
      vb[k * 4 + 3] = s.radius;
      out.sphereColor.push_back(s.color);
      out.sphereMat.push_back(s.material);
      out.sphereGroup.push_back(s.group);
      if (buildEdgeTables)
        out.sphereMatIndex.push_back(static_cast<uint32_t>(k));
      ++k;
    }
  }
  if (buildEdgeTables) out.sphereMatCount = static_cast<uint32_t>(n);
  rtcCommitGeometry(g);
  unsigned int id = rtcAttachGeometry(rscene, g);
  if (id >= out.records.size()) out.records.resize(id + 1);
  out.records[id] = {GeomKind::Sphere, g};
  rtcReleaseGeometry(g);
}

}  // namespace

BuiltScene buildEmbreeScene(RTCDevice device, const Scene& scene,
                            bool buildEdgeTables) {
  BuiltScene out;
  RTCScene rscene = rtcNewScene(device);
  out.scene = rscene;
  rtcSetSceneFlags(rscene, RTC_SCENE_FLAG_ROBUST);
  // Static offline scene: committed once, then traversed by every primary ray
  // (and every future AO/shadow ray). A one-time HIGH-quality BVH (spatial
  // splits) amortizes over the whole frame, so HIGH is the right default vs
  // Embree's MEDIUM, as OSPRay builds its static scenes. Pure traversal-speed
  // win; it cannot change which primitive a ray hits.
  rtcSetSceneBuildQuality(rscene, RTC_BUILD_QUALITY_HIGH);

  // Instance offsets are baked into the geometry (the .pov scenes have none;
  // the legacy grid path replicates each primitive per offset).
  std::vector<Vec3> bakeOffsets = scene.instanceOffsets;
  if (bakeOffsets.empty()) bakeOffsets.push_back(Vec3{0.0f, 0.0f, 0.0f});

  // Edge pass: mesh materialId is the per-triangle index directly (triMaterialId
  // is uint8, so the mesh block never exceeds 256). meshMatCount is the base
  // offset above which the sphere/cylinder global ids start.
  if (buildEdgeTables)
    out.meshMatCount = static_cast<uint32_t>(scene.mesh.materials.size());

  // Each builder attaches its geometry and appends its geomID record (and, for
  // the flat outline primitives, the primID side tables the shader reads back).
  buildTriangleMesh(device, rscene, scene.mesh, bakeOffsets, out);
  buildSpheres(device, rscene, scene, bakeOffsets, out, buildEdgeTables);
  buildCylinderGeometry(device, rscene, scene, bakeOffsets, out, buildEdgeTables);

  rtcCommitScene(rscene);
  if (RTCError err = rtcGetDeviceError(device); err != RTC_ERROR_NONE) {
    rtcReleaseScene(rscene);
    throw std::runtime_error(std::string("embree scene build failed: ") +
                             rtcErrorString(err));
  }
  return out;
}

}  // namespace detail
}  // namespace umbreon
