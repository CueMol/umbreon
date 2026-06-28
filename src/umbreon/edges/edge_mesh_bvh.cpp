// Object-space edge visibility BVH. See edge_mesh_bvh.hpp.
#include "edges/edge_mesh_bvh.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "shading/ray_walk.hpp"

namespace umbreon {
namespace detail {

EdgeBVH::EdgeBVH(EdgeBVH&& o) noexcept
    : device(o.device),
      scene(o.scene),
      triCount(o.triCount),
      boundsDiag(o.boundsDiag) {
  o.device = nullptr;
  o.scene = nullptr;
  o.triCount = 0;
  o.boundsDiag = 0.0f;
}

EdgeBVH& EdgeBVH::operator=(EdgeBVH&& o) noexcept {
  if (this != &o) {
    if (scene) rtcReleaseScene(scene);
    if (device) rtcReleaseDevice(device);
    device = o.device;
    scene = o.scene;
    triCount = o.triCount;
    boundsDiag = o.boundsDiag;
    o.device = nullptr;
    o.scene = nullptr;
    o.triCount = 0;
    o.boundsDiag = 0.0f;
  }
  return *this;
}

EdgeBVH::~EdgeBVH() {
  if (scene) rtcReleaseScene(scene);
  if (device) rtcReleaseDevice(device);
}

EdgeBVH buildEdgeMeshBVH(const Mesh& m) {
  EdgeBVH bvh;
  const std::size_t baseV = m.vertexCount();
  const std::size_t baseT = m.triangleCount();
  if (baseV < 3 || baseT == 0) return bvh;  // invalid: queries report visible

  bvh.device = rtcNewDevice(nullptr);
  bvh.scene = rtcNewScene(bvh.device);
  // ROBUST mirrors the render scene (scene_build.cpp): edge visibility rays graze
  // silhouette triangles, so robust traversal avoids dropped near-tangent hits.
  rtcSetSceneFlags(bvh.scene, RTC_SCENE_FLAG_ROBUST);

  RTCGeometry g = rtcNewGeometry(bvh.device, RTC_GEOMETRY_TYPE_TRIANGLE);
  auto* pos = static_cast<float*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), baseV));
  auto* idx = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
      g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned int),
      baseT));

  Aabb bb;
  for (std::size_t v = 0; v < baseV; ++v) {
    const Vec3& p = m.positions[v];
    pos[v * 3 + 0] = p.x;
    pos[v * 3 + 1] = p.y;
    pos[v * 3 + 2] = p.z;
    bb.extend(p);
  }
  // primID == triangle id: corner -> vertex via cornerVertex (trivial 3t+k for a
  // de-indexed mesh), matching FeatureSeg::face0/face1.
  for (std::size_t t = 0; t < baseT; ++t) {
    for (int k = 0; k < 3; ++k)
      idx[t * 3 + static_cast<std::size_t>(k)] = static_cast<unsigned int>(
          m.cornerVertex(t * 3 + static_cast<std::size_t>(k)));
  }

  rtcCommitGeometry(g);
  rtcAttachGeometry(bvh.scene, g);
  rtcReleaseGeometry(g);
  rtcCommitScene(bvh.scene);

  bvh.triCount = static_cast<unsigned int>(baseT);
  bvh.boundsDiag = bb.valid() ? bb.diagonal() : 0.0f;
  return bvh;
}

bool isSegmentClear(const EdgeBVH& bvh, const Vec3& P, const Vec3& Q,
                    const int* excludeFaces, int nExclude) {
  if (!bvh.valid()) return true;  // no occluder
  const Vec3 d = Q - P;
  const float L = length(d);
  if (L <= 0.0f) return true;
  const Vec3 dir = d * (1.0f / L);
  // Trim both ends so the surface P lies on and the target Q are not counted; a
  // relative epsilon adapts to scene scale (the float hit precision ~ t*2^-23).
  const float eps = L * 1.0e-4f;
  const float tnear = eps;
  const float tfar = L - eps;
  if (tfar <= tnear) return true;
  auto hits = collectHitsAlongRay(
      bvh.scene, P, dir, tnear, tfar, [&](unsigned int primID) {
        for (int i = 0; i < nExclude; ++i)
          if (excludeFaces[i] == static_cast<int>(primID)) return true;
        return false;
      });
  return hits.empty();
}

bool isPointVisibleToViewer(const EdgeBVH& bvh, const Vec3& P, const Camera& cam,
                            const int* excludeFaces, int nExclude) {
  if (!bvh.valid()) return true;
  // "Toward the viewer" target Q (mirrors viewerDirAt in mesh_feature_edges.cpp):
  // the eye for perspective, a far camera-ward point for orthographic.
  const Vec3 toCam = normalize(cam.direction * -1.0f);
  Vec3 Q;
  if (cam.orthographic) {
    Q = P + toCam * (2.0f * bvh.boundsDiag + 1.0f);
  } else {
    const Vec3 toEye = cam.position - P;
    Q = length(toEye) > 0.0f ? cam.position
                             : P + toCam * (2.0f * bvh.boundsDiag + 1.0f);
  }
  return isSegmentClear(bvh, P, Q, excludeFaces, nExclude);
}

std::vector<std::pair<Vec3, Vec3>> clipSegmentToVisibleSpans(
    const EdgeBVH& bvh, const Camera& cam, const Vec3& a, const Vec3& b,
    const int* excludeFaces, int nExclude, float step) {
  std::vector<std::pair<Vec3, Vec3>> spans;
  const Vec3 d = b - a;
  const float len = length(d);
  if (len <= 0.0f) return spans;
  // No occluder mesh => the whole segment is visible (verbatim emit fallback).
  if (!bvh.valid()) {
    spans.emplace_back(a, b);
    return spans;
  }
  const float s = step > 0.0f ? step : len;
  const int K = std::max(2, static_cast<int>(std::ceil(len / s)));
  auto pointAt = [&](int i) {
    return a + d * (static_cast<float>(i) / static_cast<float>(K));
  };
  int runStart = -1;
  auto emitRun = [&](int s0, int e0) {
    if (e0 > s0) spans.emplace_back(pointAt(s0), pointAt(e0));
  };
  for (int i = 0; i <= K; ++i) {
    const bool vis =
        isPointVisibleToViewer(bvh, pointAt(i), cam, excludeFaces, nExclude);
    if (vis && runStart < 0) runStart = i;
    if (!vis && runStart >= 0) {
      emitRun(runStart, i - 1);  // last visible sample closes the run
      runStart = -1;
    }
  }
  if (runStart >= 0) emitRun(runStart, K);
  return spans;
}

}  // namespace detail
}  // namespace umbreon
