#include "render/embree_renderer.hpp"

#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace umbreon {
namespace {

// Per-geometry kind, recorded against the geomID so the shader knows how to
// color a hit (smooth-shaded mesh vs flat-colored outline primitive).
enum class GeomKind { Mesh, Sphere, Cylinder };

struct GeomRecord {
  GeomKind kind = GeomKind::Mesh;
  RTCGeometry geom = nullptr;  // borrowed handle, for rtcInterpolate on mesh
};

Vec3 faceForward(Vec3 n, Vec3 rayDir) {
  // POV/CueMol normals should point toward the viewer; flip if back-facing.
  return (dot(n, rayDir) > 0.0f) ? Vec3{-n.x, -n.y, -n.z} : n;
}

}  // namespace

FrameResult EmbreeRenderer::render(const Scene& scene, const RenderOptions& opt) {
  RTCDevice device = rtcNewDevice(nullptr);
  if (!device) throw std::runtime_error("rtcNewDevice failed");

  RTCScene rscene = rtcNewScene(device);
  rtcSetSceneFlags(rscene, RTC_SCENE_FLAG_ROBUST);

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

  // --- spheres (CueMol balls / silhouette joints), flat-colored ---
  std::vector<Vec4> sphereColor;  // indexed by primID
  if (!scene.spheres.empty()) {
    const std::size_t n = scene.spheres.size() * bakeOffsets.size();
    RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SPHERE_POINT);
    auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float), n));
    sphereColor.reserve(n);
    std::size_t k = 0;
    for (const Vec3& off : bakeOffsets) {
      for (const Sphere& s : scene.spheres) {
        vb[k * 4 + 0] = s.center.x + off.x;
        vb[k * 4 + 1] = s.center.y + off.y;
        vb[k * 4 + 2] = s.center.z + off.z;
        vb[k * 4 + 3] = s.radius;
        sphereColor.push_back(s.color);
        ++k;
      }
    }
    rtcCommitGeometry(g);
    unsigned int id = rtcAttachGeometry(rscene, g);
    if (id >= records.size()) records.resize(id + 1);
    records[id] = {GeomKind::Sphere, g};
    rtcReleaseGeometry(g);
  }

  // --- cylinders (CueMol sticks / silhouette edges) as round linear curves ---
  std::vector<Vec4> cylColor;  // indexed by primID (one per segment)
  if (!scene.cylinders.empty()) {
    const std::size_t segs = scene.cylinders.size() * bakeOffsets.size();
    const std::size_t nV = segs * 2;
    RTCGeometry g =
        rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE);
    auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float), nV));
    auto* ib = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(unsigned int),
        segs));
    cylColor.reserve(segs);
    std::size_t k = 0;
    for (const Vec3& off : bakeOffsets) {
      for (const Cylinder& c : scene.cylinders) {
        vb[(2 * k + 0) * 4 + 0] = c.p0.x + off.x;
        vb[(2 * k + 0) * 4 + 1] = c.p0.y + off.y;
        vb[(2 * k + 0) * 4 + 2] = c.p0.z + off.z;
        vb[(2 * k + 0) * 4 + 3] = c.radius;
        vb[(2 * k + 1) * 4 + 0] = c.p1.x + off.x;
        vb[(2 * k + 1) * 4 + 1] = c.p1.y + off.y;
        vb[(2 * k + 1) * 4 + 2] = c.p1.z + off.z;
        vb[(2 * k + 1) * 4 + 3] = c.radius;
        ib[k] = static_cast<unsigned int>(2 * k);
        cylColor.push_back(c.color);
        ++k;
      }
    }
    rtcCommitGeometry(g);
    unsigned int id = rtcAttachGeometry(rscene, g);
    if (id >= records.size()) records.resize(id + 1);
    records[id] = {GeomKind::Cylinder, g};
    rtcReleaseGeometry(g);
  }

  rtcCommitScene(rscene);

  // --- camera basis (POV orthographic framing) ---
  const Camera& cam = scene.camera;
  const int W = opt.width, H = opt.height;
  const float aspect = static_cast<float>(W) / static_cast<float>(H);

  const Vec3 dir = normalize(cam.direction);
  const Vec3 right = normalize(cross(dir, cam.up));
  const Vec3 trueUp = normalize(cross(right, dir));

  const float halfH = cam.height * 0.5f;
  const float halfW = halfH * aspect;
  // Perspective fallback half-extents at unit distance from the image plane.
  const float persHalfH = std::tan(radians(cam.fovy) * 0.5f);
  const float persHalfW = persHalfH * aspect;

  // --- POV-native lights: direction the light travels -> direction to light. ---
  struct Light {
    Vec3 L;       // unit direction from surface toward the light
    Vec3 color;   // light color * intensity
  };
  std::vector<Light> lights;
  lights.reserve(scene.lights.size());
  for (const DistantLight& dl : scene.lights) {
    Light l;
    l.L = normalize(Vec3{-dl.direction.x, -dl.direction.y, -dl.direction.z});
    l.color = Vec3{dl.color.x * dl.intensity, dl.color.y * dl.intensity,
                   dl.color.z * dl.intensity};
    lights.push_back(l);
  }
  // POV ambient radiance: ambient_light defaults to <1,1,1>; the mesh ambient
  // term is material.ambient * pigment, applied below via ambK.
  const Vec3 ambLight = scene.ambientColor;  // expected <1,1,1> on the embree path

  FrameResult res;
  res.width = W;
  res.height = H;
  res.color.assign(static_cast<std::size_t>(W) * H * 4, 0.0f);
  res.depth.assign(static_cast<std::size_t>(W) * H, 0.0f);
  res.effectiveTriangles = scene.effectiveTriangles();

  const Vec3 bg = {scene.background.x, scene.background.y, scene.background.z};
  const float ambK = m.material.ambient;
  const float difK = m.material.diffuse;

  auto t0 = std::chrono::high_resolution_clock::now();

  // Parallelize over image rows with TBB (CueMol's unified CPU parallel
  // primitive). Each ray is independent and rtcIntersect1 on a committed scene
  // is thread-safe; pixels write to disjoint framebuffer indices.
  tbb::parallel_for(tbb::blocked_range<int>(0, H),
                    [&](const tbb::blocked_range<int>& rows) {
  for (int py = rows.begin(); py != rows.end(); ++py) {
    // Top-left origin: row 0 maps to v = +1 (top), last row to v = -1 (bottom).
    const float v = 1.0f - 2.0f * (static_cast<float>(py) + 0.5f) /
                               static_cast<float>(H);
    for (int px = 0; px < W; ++px) {
      const float u = 2.0f * (static_cast<float>(px) + 0.5f) /
                          static_cast<float>(W) - 1.0f;

      Vec3 org, rd;
      if (cam.orthographic) {
        org = cam.position + right * (u * halfW) + trueUp * (v * halfH);
        rd = dir;
      } else {
        org = cam.position;
        rd = normalize(dir + right * (u * persHalfW) + trueUp * (v * persHalfH));
      }

      RTCRayHit rh;
      rh.ray.org_x = org.x;
      rh.ray.org_y = org.y;
      rh.ray.org_z = org.z;
      rh.ray.dir_x = rd.x;
      rh.ray.dir_y = rd.y;
      rh.ray.dir_z = rd.z;
      rh.ray.tnear = 0.0f;
      rh.ray.tfar = std::numeric_limits<float>::infinity();
      rh.ray.mask = 0xFFFFFFFFu;
      rh.ray.flags = 0;
      rh.ray.time = 0.0f;
      rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

      RTCIntersectArguments iargs;
      rtcInitIntersectArguments(&iargs);
      rtcIntersect1(rscene, &rh, &iargs);

      const std::size_t pix = (static_cast<std::size_t>(py) * W + px);
      Vec3 out;
      float depth = 0.0f;

      if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
          rh.hit.geomID >= records.size()) {
        out = bg;
        depth = 0.0f;
      } else {
        depth = rh.ray.tfar;
        const GeomRecord& rec = records[rh.hit.geomID];
        if (rec.kind == GeomKind::Mesh) {
          // Interpolate the shading normal and pigment color (slots 0/1).
          float nbuf[3] = {0, 0, 0};
          float cbuf[4] = {0, 0, 0, 1};
          rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                          RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
          rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                          RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
          Vec3 N = normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
          N = faceForward(N, rd);
          const Vec3 C = {cbuf[0], cbuf[1], cbuf[2]};

          // POV local illumination: ambient + sum of diffuse lobes.
          Vec3 lit = {ambK * ambLight.x, ambK * ambLight.y, ambK * ambLight.z};
          for (const Light& l : lights) {
            const float ndl = dot(N, l.L);
            if (ndl > 0.0f) {
              const float k = difK * ndl;
              lit.x += k * l.color.x;
              lit.y += k * l.color.y;
              lit.z += k * l.color.z;
            }
          }
          out = {C.x * lit.x, C.y * lit.y, C.z * lit.z};
        } else {
          // Outline primitives: flat primitive color (ambient 1, diffuse 0).
          const Vec4& fc = (rec.kind == GeomKind::Sphere)
                               ? sphereColor[rh.hit.primID]
                               : cylColor[rh.hit.primID];
          out = {fc.x, fc.y, fc.z};
        }
      }

      res.color[pix * 4 + 0] = out.x;
      res.color[pix * 4 + 1] = out.y;
      res.color[pix * 4 + 2] = out.z;
      res.color[pix * 4 + 3] = 1.0f;
      res.depth[pix] = depth;
    }
  }
  });

  auto t1 = std::chrono::high_resolution_clock::now();
  res.renderSeconds = std::chrono::duration<double>(t1 - t0).count();

  rtcReleaseScene(rscene);
  rtcReleaseDevice(device);
  return res;
}

}  // namespace umbreon
