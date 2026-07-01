// pt1 Phase 0 smoke test: register a single triangle with Embree 4 and verify
// rtcIntersect1 hit/miss semantics (geomID sentinel init, RTCIntersectArguments
// API). Locks the Embree 4 query pattern the pt1 integrator builds on.
#include <embree4/rtcore.h>

#include <cmath>
#include <limits>

#include "test_util.hpp"

namespace {

RTCRayHit makeRay(float ox, float oy, float oz, float dx, float dy, float dz) {
  RTCRayHit rh;
  rh.ray.org_x = ox;
  rh.ray.org_y = oy;
  rh.ray.org_z = oz;
  rh.ray.dir_x = dx;
  rh.ray.dir_y = dy;
  rh.ray.dir_z = dz;
  rh.ray.tnear = 0.0f;
  rh.ray.tfar = std::numeric_limits<float>::infinity();
  rh.ray.mask = 0xFFFFFFFFu;
  rh.ray.flags = 0;
  rh.ray.time = 0.0f;
  // Miss detection relies on this sentinel staying untouched on a miss.
  rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
  return rh;
}

}  // namespace

int main() {
  umbreon::test::Suite suite("pt1_embree_smoke");

  RTCDevice device = rtcNewDevice(nullptr);
  suite.check("device created", device != nullptr);

  RTCScene scene = rtcNewScene(device);
  rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST);

  // Unit triangle in the z=0 plane: (0,0,0), (1,0,0), (0,1,0).
  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  float* verts = static_cast<float*>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), 3));
  verts[0] = 0.0f; verts[1] = 0.0f; verts[2] = 0.0f;
  verts[3] = 1.0f; verts[4] = 0.0f; verts[5] = 0.0f;
  verts[6] = 0.0f; verts[7] = 1.0f; verts[8] = 0.0f;
  uint32_t* idx = static_cast<uint32_t*>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(uint32_t), 1));
  idx[0] = 0;
  idx[1] = 1;
  idx[2] = 2;
  rtcCommitGeometry(geom);
  const unsigned triGeomID = rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
  rtcCommitScene(scene);

  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);

  // Hit: from above the triangle interior, straight down.
  RTCRayHit hit = makeRay(0.25f, 0.25f, 1.0f, 0.0f, 0.0f, -1.0f);
  rtcIntersect1(scene, &hit, &iargs);
  suite.check("interior ray hits", hit.hit.geomID != RTC_INVALID_GEOMETRY_ID);
  suite.check("hit geomID matches attached geometry",
              hit.hit.geomID == triGeomID);
  suite.check("hit primID is 0", hit.hit.primID == 0u);
  suite.check("hit distance is 1", std::fabs(hit.ray.tfar - 1.0f) < 1e-5f);

  // Miss: same direction but offset outside the triangle.
  RTCRayHit miss = makeRay(2.0f, 2.0f, 1.0f, 0.0f, 0.0f, -1.0f);
  rtcIntersect1(scene, &miss, &iargs);
  suite.check("exterior ray misses (geomID sentinel preserved)",
              miss.hit.geomID == RTC_INVALID_GEOMETRY_ID);

  rtcReleaseScene(scene);
  rtcReleaseDevice(device);
  return suite.report();
}
