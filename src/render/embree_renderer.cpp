#include "render/embree_renderer.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "render/scene_build.hpp"
#include "render/shading.hpp"

namespace umbreon {

// The Embree scene construction (scene_build.hpp) and the POV local shader plus
// AO/shadow estimators (shading.hpp) live in the detail namespace; pull them in
// so the renderer below calls them unqualified.
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
    built = buildEmbreeScene(device, scene);
  } catch (...) {
    rtcReleaseDevice(device);
    throw;
  }

  // Alias the built scene and its primID side tables under their original names
  // so the hot shadeHit / pixel-loop code below is unchanged by the split.
  RTCScene rscene = built.scene;
  const std::vector<GeomRecord>& records = built.records;
  const std::vector<Vec4>& sphereColor = built.sphereColor;
  const std::vector<Material>& sphereMat = built.sphereMat;
  const std::vector<uint16_t>& sphereGroup = built.sphereGroup;
  const std::vector<Vec4>& cylColor = built.cylColor;
  const std::vector<Material>& cylMat = built.cylMat;
  const std::vector<uint16_t>& cylGroup = built.cylGroup;
  const std::vector<float>& cylOpacity1 = built.cylOpacity1;
  const std::vector<Vec4>& cylCapColor = built.cylCapColor;
  const std::vector<Material>& cylCapMat = built.cylCapMat;
  const std::vector<uint16_t>& cylCapGroup = built.cylCapGroup;
  const std::vector<float>& cylCapOpacity1 = built.cylCapOpacity1;

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

  FrameResult res;
  res.width = W;
  res.height = H;
  res.color.assign(static_cast<std::size_t>(W) * H * 4, 0.0f);
  res.depth.assign(static_cast<std::size_t>(W) * H, 0.0f);
  res.effectiveTriangles = scene.effectiveTriangles();

  const Vec3 bg = {scene.background.x, scene.background.y, scene.background.z};

  // Shade a single hit and report its opacity and transparency group (CueMol
  // section). Used by the front-to-back walk below.
  struct HitShade {
    Vec3 color{0.0f, 0.0f, 0.0f};
    float opacity = 1.0f;
    int group = 0;
  };
  auto shadeHit = [&](const RTCRayHit& rh, const Vec3& rd, const Vec3& org,
                      uint32_t px, uint32_t py) -> HitShade {
    HitShade hs;
    const GeomRecord& rec = records[rh.hit.geomID];
    const Vec3 V = normalize(Vec3{-rd.x, -rd.y, -rd.z});
    if (rec.kind == GeomKind::Mesh) {
      // Interpolate the shading normal and pigment color (slots 0/1).
      float nbuf[3] = {0, 0, 0};
      float cbuf[4] = {0, 0, 0, 1};
      rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                      RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
      rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                      RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
      Vec3 N = faceForward(normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]}), rd);
      const Vec3 C = {cbuf[0], cbuf[1], cbuf[2]};
      const Material& triMat = m.materialForTri(rh.hit.primID);
      // Hit point and GEOMETRIC normal (rh.hit.Ng, not the interpolated shading
      // normal N): shared by the AO and shadow secondary rays. Offsetting the
      // secondary-ray origin along Ng avoids self-hits on concave meshes.
      const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                   org.z + rd.z * rh.ray.tfar};
      const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
      // Self-intersection epsilon for the secondary (AO/shadow) rays: the SAME
      // scale-adaptive value the transparency walk uses, from the PRIMARY ray
      // length rh.ray.tfar. A far camera makes the hit-point error ~ tfar*2^-23,
      // so a fixed / t=1 epsilon is far too small and the surface shadows itself
      // (shadow acne).
      const float secEps = selfIntersectEps(P, rd, rh.ray.tfar);
      // AO darkens ONLY the ambient term; gated off by default (aoSamples == 0)
      // so the flag-less render stays bit-exact (aoFactor == 1 -> x*1 == x).
      float aoFactor = 1.0f;
      if (opt.aoSamples > 0) {
        const float rawAO = computeAO(rscene, P, Ng, N, secEps, opt.aoSamples,
                                      opt.aoDistance, px, py, W);
        aoFactor = 1.0f - opt.aoIntensity * (1.0f - rawAO);
      }
      hs.color = shadeLocal(triMat, C, N, V, lights, ambLight, bg,
                            opt.specularScale, aoFactor, P, Ng, secEps, rscene,
                            opt.shadows, opt.shadowSamples, px, py);
      hs.opacity = cbuf[3];
      hs.group = m.groupForTri(rh.hit.primID);
    } else {
      // Outline / VdW primitives: shade with the per-primitive material.
      // The Embree geometric normal Ng is valid for SPHERE_POINT and for both
      // linear-curve modes (ROUND swept-sphere and CONE capped-cone): in round
      // mode and on the cone surface Ng is the non-normalized geometric surface
      // normal POV shades against (flat-curve tangent semantics do not apply).
      // Each cylinder geometry has its own primID space and side-tables; select
      // the table set by kind (Sphere / Cylinder=round edge / capped=cone bond).
      const bool isSphere = (rec.kind == GeomKind::Sphere);
      const bool isCapped = (rec.kind == GeomKind::CylinderCapped);
      const Vec4& fc = isSphere ? sphereColor[rh.hit.primID]
                       : isCapped ? cylCapColor[rh.hit.primID]
                                  : cylColor[rh.hit.primID];
      const Material& pm = isSphere ? sphereMat[rh.hit.primID]
                           : isCapped ? cylCapMat[rh.hit.primID]
                                      : cylMat[rh.hit.primID];
      Vec3 N = faceForward(
          normalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z}), rd);
      const Vec3 C = {fc.x, fc.y, fc.z};
      // Outline / VdW primitives are flat silhouette geometry: never AO-darkened
      // and never shadowed (aoFactor 1, shadowsOn false). This is the gate.
      const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                   org.z + rd.z * rh.ray.tfar};
      const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
      const float secEps = selfIntersectEps(P, rd, rh.ray.tfar);
      hs.color = shadeLocal(pm, C, N, V, lights, ambLight, bg,
                            opt.specularScale, 1.0f, P, Ng, secEps, rscene, false,
                            1, px, py);
      hs.opacity = fc.w;
      hs.group = isSphere ? sphereGroup[rh.hit.primID]
                 : isCapped ? cylCapGroup[rh.hit.primID]
                            : cylGroup[rh.hit.primID];
      // edge_line2 gradient: opacity varies p0 (fc.w) -> p1 (opacity1) along the
      // segment. For both linear-curve modes rh.hit.u is the axial curve fraction
      // in [0,1], so a linear lerp reproduces POV's "gradient z" transmit fade
      // (uniform when opacity1 < 0). Opaque outlines and uniform bonds
      // (opacity1 < 0) are unaffected.
      if (!isSphere) {
        const float op1 =
            isCapped ? cylCapOpacity1[rh.hit.primID] : cylOpacity1[rh.hit.primID];
        if (op1 >= 0.0f) {
          float u = rh.hit.u;
          if (u < 0.0f) u = 0.0f;
          if (u > 1.0f) u = 1.0f;
          hs.opacity = fc.w * (1.0f - u) + op1 * u;
        }
      }
    }
    return hs;
  };

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

      const std::size_t pix = (static_cast<std::size_t>(py) * W + px);

      // Single-pass transparency with two coexisting models, selected per hit
      // by the hit's group:
      //   - VEIL groups (group alpha): additive single-layer, frontmost-per-
      //     group, order-independent -- exactly CueMol's blendpng.
      //   - everything else (fragment alpha = intrinsic per-color opacity):
      //     front-to-back "over", every surface composited (no dedup) -- POV
      //     native transmit; group alpha (if any) already multiplied in.
      // Combine: fragments composite over the opaque floor, then the veils are
      // laid additively on top. Reduces to pure "over" (no veils, e.g. scene5),
      // pure additive (all veils, the group-alpha path), or opaque (unchanged).
      Vec3 Cv{0.0f, 0.0f, 0.0f};  // additive (veil) premultiplied color
      float sumBeta = 0.0f;       // sum of veil weights
      Vec3 Cf{0.0f, 0.0f, 0.0f};  // over (fragment) premultiplied color
      float Af = 0.0f;            // over accumulated coverage
      float nearDepth = 0.0f;
      Vec3 base = bg;
      float baseCov = opt.transparentBackground ? 0.0f : 1.0f;

      constexpr int kMaxSeen = 64;  // distinct veil groups per ray
      int seen[kMaxSeen];
      int nseen = 0;

      const float kOpaque = 1.0f - 1e-4f;  // opacity at/above this == opaque
      float tnear = 0.0f;
      const int maxIters = opt.transparency ? (opt.maxTransparentLayers + 1) : 1;

      for (int iter = 0; iter < maxIters; ++iter) {
        RTCRayHit rh;
        rh.ray.org_x = org.x;
        rh.ray.org_y = org.y;
        rh.ray.org_z = org.z;
        rh.ray.dir_x = rd.x;
        rh.ray.dir_y = rd.y;
        rh.ray.dir_z = rd.z;
        rh.ray.tnear = tnear;
        rh.ray.tfar = std::numeric_limits<float>::infinity();
        rh.ray.mask = 0xFFFFFFFFu;
        rh.ray.flags = 0;
        rh.ray.time = 0.0f;
        rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        RTCIntersectArguments iargs;
        rtcInitIntersectArguments(&iargs);
        rtcIntersect1(rscene, &rh, &iargs);

        if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
            rh.hit.geomID >= records.size()) {
          break;  // ray escaped: base stays the background
        }
        if (nearDepth == 0.0f) nearDepth = rh.ray.tfar;

        const HitShade hs = shadeHit(rh, rd, org, static_cast<uint32_t>(px),
                                     static_cast<uint32_t>(py));

        if (!opt.transparency || hs.opacity >= kOpaque) {
          base = hs.color;  // nearest opaque surface = the floor
          baseCov = 1.0f;
          break;
        }

        const bool veil =
            hs.group >= 0 && static_cast<std::size_t>(hs.group) < isVeil.size() &&
            isVeil[hs.group] != 0;
        if (veil) {
          // additive: only the frontmost surface of each veil group
          bool dup = false;
          for (int sidx = 0; sidx < nseen; ++sidx)
            if (seen[sidx] == hs.group) { dup = true; break; }
          if (!dup) {
            if (nseen < kMaxSeen) seen[nseen++] = hs.group;
            const float a = hs.opacity;
            Cv.x += a * hs.color.x;
            Cv.y += a * hs.color.y;
            Cv.z += a * hs.color.z;
            sumBeta += a;
          }
        } else {
          // over: every fragment composited front-to-back (no dedup)
          const float w = (1.0f - Af) * hs.opacity;
          Cf.x += w * hs.color.x;
          Cf.y += w * hs.color.y;
          Cf.z += w * hs.color.z;
          Af += w;
        }
        if (sumBeta >= kOpaque || Af >= kOpaque) break;  // fully saturated
        // Advance just past this surface to find the next one. The step is a
        // self-intersection epsilon: it must clear only floating-point jitter,
        // never a distinct surface sitting just behind this one. A hardcoded
        // step is wrong because the hit-point precision degrades with scale --
        // the camera is ~200 units from the molecule, so the hit coordinates
        // carry an absolute error ~ t * 2^-23. Use a scale-adaptive epsilon, as
        // OSPRay does (modules/cpu/common/DifferentialGeometry.ih, calcEpsilon):
        // max(|hit point|, |dir| * t) scaled by a small float-ULP factor. This
        // adapts to any camera distance / scene scale with no magic constant.
        const Vec3 hitP{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                        org.z + rd.z * rh.ray.tfar};
        const float dirMax = std::fmax(
            std::fabs(rd.x), std::fmax(std::fabs(rd.y), std::fabs(rd.z)));
        const float epsScale =
            std::fmax(std::fmax(std::fabs(hitP.x), std::fabs(hitP.y)),
                      std::fmax(std::fabs(hitP.z), dirMax * rh.ray.tfar));
        constexpr float kUlpEps = 0x1.0fp-21f;  // ~5.05e-7 (~4 ULP), OSPRay ulpEpsilon
        tnear = rh.ray.tfar + epsScale * kUlpEps;
        // Reaching the final allowed iteration after compositing a transparent
        // layer means more surfaces may lie behind that we will not trace:
        // record the truncation for the post-loop warning.
        if (iter == maxIters - 1)
          cappedRays.fetch_add(1, std::memory_order_relaxed);
      }

      // Fragments over the opaque floor; veils laid additively on top. The base
      // (floor / background) contributes COLOR only where it is covered
      // (baseCov), so an opaque background (default) leaves opaque scenes
      // byte-unchanged, while a transparent background (baseCov 0) yields a
      // premultiplied result with alpha = accumulated coverage.
      const float floorW = (1.0f - Af) * baseCov;
      const Vec3 baseEff{Cf.x + floorW * base.x,
                         Cf.y + floorW * base.y,
                         Cf.z + floorW * base.z};
      const float covEff = Af + floorW;
      float vw = 1.0f - sumBeta;
      if (vw < 0.0f) vw = 0.0f;
      float outA = sumBeta + vw * covEff;
      outA = std::fmin(1.0f, std::fmax(0.0f, outA));

      res.color[pix * 4 + 0] = Cv.x + vw * baseEff.x;
      res.color[pix * 4 + 1] = Cv.y + vw * baseEff.y;
      res.color[pix * 4 + 2] = Cv.z + vw * baseEff.z;
      res.color[pix * 4 + 3] = outA;
      res.depth[pix] = nearDepth;
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

  rtcReleaseScene(rscene);
  rtcReleaseDevice(device);
  return res;
}

}  // namespace umbreon
