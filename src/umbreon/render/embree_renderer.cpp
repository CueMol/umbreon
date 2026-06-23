#include "render/embree_renderer.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "render/hit_shader.hpp"
#include "render/scene_build.hpp"
#include "render/transparency.hpp"

namespace umbreon {

// The Embree scene construction (scene_build.hpp), the per-hit shader
// (hit_shader.hpp) and the transparency integrator (transparency.hpp) live in
// the detail namespace; pull it in so the driver below uses them unqualified.
using namespace detail;

FrameResult EmbreeRenderer::render(const Scene& scene, const RenderOptions& opt) {
  RTCDevice device = rtcNewDevice(nullptr);
  if (!device) throw std::runtime_error("rtcNewDevice failed");
  rtcSetDeviceErrorFunction(device, embreeErrorCallback, nullptr);

  // Build all Embree geometry (cold, once per frame). On a build error this
  // releases the partial scene and throws; release the device too before
  // propagating so render() leaks neither handle.
  BuiltScene built;
  try {
    built = buildEmbreeScene(device, scene, opt.edges.enable);
  } catch (...) {
    rtcReleaseDevice(device);
    throw;
  }

  const Mesh& m = scene.mesh;

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
  std::vector<Light> lights;
  lights.reserve(scene.lights.size());
  for (const DistantLight& dl : scene.lights) {
    Light l;
    l.L = normalize(Vec3{-dl.direction.x, -dl.direction.y, -dl.direction.z});
    l.color = Vec3{dl.color.x * dl.intensity, dl.color.y * dl.intensity,
                   dl.color.z * dl.intensity};
    l.highlight = dl.castsHighlight;
    l.radius = radians(opt.lightRadius);  // soft-shadow angular radius (0 = hard)
    lights.push_back(l);
  }
  // POV ambient radiance: ambient_light defaults to <1,1,1>; the mesh ambient
  // term is material.ambient * pigment, applied below via ambK.
  const Vec3 ambLight = scene.ambientColor;  // expected <1,1,1> on the embree path
  const Vec3 bg = {scene.background.x, scene.background.y, scene.background.z};

  FrameResult res;
  res.width = W;
  res.height = H;
  res.color.assign(static_cast<std::size_t>(W) * H * 4, 0.0f);
  res.depth.assign(static_cast<std::size_t>(W) * H, 0.0f);
  // Screen-space edge AOVs: allocated ONLY when edges are enabled. With edges
  // off these stay empty, so no extra memory is touched and the output path is
  // byte-identical to the no-edge render.
  if (opt.edges.enable) {
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    res.viewZ.assign(npix, 0.0f);
    res.normal.assign(npix * 3, 0.0f);
    res.objectId.assign(npix, 0xFFFFFFFFu);
    res.materialId.assign(npix, 0xFFFFFFFFu);
  }
  res.effectiveTriangles = scene.effectiveTriangles();

  // Everything the per-hit shader reads, gathered once.
  const ShadeContext sc{built, m, lights, ambLight, bg, opt};

  // Veil lookup: groups rendered as additive single-layer (group alpha). Every
  // other transparency uses front-to-back "over" (fragment alpha). Empty =>
  // all transparency is "over".
  std::vector<uint8_t> isVeil;
  for (uint16_t g : scene.veilGroups) {
    if (g >= isVeil.size()) isVeil.resize(static_cast<std::size_t>(g) + 1, 0);
    isVeil[g] = 1;
  }

  auto t0 = std::chrono::high_resolution_clock::now();

  // Count primary rays that exhaust the transparent-layer cap so truncation of
  // far transmission is observable (warned once below) rather than silent.
  std::atomic<long long> cappedRays{0};

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

      const PixelResult pr =
          integratePixel(sc, isVeil, org, rd, static_cast<uint32_t>(px),
                         static_cast<uint32_t>(py), dir, cappedRays);

      const std::size_t pix = (static_cast<std::size_t>(py) * W + px);
      res.color[pix * 4 + 0] = pr.r;
      res.color[pix * 4 + 1] = pr.g;
      res.color[pix * 4 + 2] = pr.b;
      res.color[pix * 4 + 3] = pr.a;
      res.depth[pix] = pr.depth;
      // Edge G-buffer store: gated so the default path writes nothing extra.
      if (opt.edges.enable) {
        res.viewZ[pix] = pr.viewZ;
        res.normal[pix * 3 + 0] = pr.worldNormal.x;
        res.normal[pix * 3 + 1] = pr.worldNormal.y;
        res.normal[pix * 3 + 2] = pr.worldNormal.z;
        res.objectId[pix] = pr.objectId;
        res.materialId[pix] = pr.materialId;
      }
    }
  }
  });

  if (const long long capped = cappedRays.load(std::memory_order_relaxed);
      capped > 0) {
    std::fprintf(stderr,
                 "warning: %lld ray(s) reached the transparent-layer cap (%d); "
                 "transmission past that depth was dropped -- raise "
                 "maxTransparentLayers if it is visible\n",
                 capped, opt.maxTransparentLayers);
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  res.renderSeconds = std::chrono::duration<double>(t1 - t0).count();

  rtcReleaseScene(built.scene);
  rtcReleaseDevice(device);
  return res;
}

}  // namespace umbreon
