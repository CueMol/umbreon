#include "render/embree_renderer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "ao/env_dome.hpp"
#include "shading/hit_shader.hpp"
#include "experimental/irradiance_cache/irradiance_cache.hpp"
#include "experimental/pt1/pt1_integrator.hpp"
#include "render/scene_build.hpp"
#include "shading/secondary_rays.hpp"
#include "shading/transparency.hpp"

namespace umbreon {

// The Embree scene construction (scene_build.hpp), the per-hit shader
// (hit_shader.hpp) and the transparency integrator (transparency.hpp) live in
// the detail namespace; pull it in so the driver below uses them unqualified.
using namespace detail;

namespace {

// Ray query context for the self-face-excluding occlusion test. Derives from
// RTCRayQueryContext (first member, as Embree requires) and carries the set of
// mesh (geomID, primID) hits to REJECT -- the feature edge's own incident faces.
// Embree passes a pointer to this through to the argument filter below.
struct ExcludeFaceContext {
  RTCRayQueryContext base;        // MUST be first (Embree reinterpret_casts it)
  unsigned int meshGeomID;        // the mesh geometry id (RTC_INVALID if none)
  unsigned int baseTriCount;      // de-indexed base tri count (for baked copies)
  const int* faces;               // excluded base triangle ids
  int nFaces;                     // count
  // Freestyle TANGENTIAL-occluder rejection (ViewMapBuilder.cpp: fabs(u*normal) >
  // 0.0001): a hit whose face is grazed nearly edge-on by the QI ray -- |dir . n|
  // <= this cosine -- is NOT a real occluder (it is the silhouette's own grazing
  // surface or a tessellation neighbour the ray skims), so the filter rejects it.
  // This is the SCALE-INVARIANT self-occlusion discriminator that replaces the old
  // distance dead-zone. <= 0 disables it (shadows/AO, which never set it).
  float grazeCosEps;              // reject hit if |dir . normalize(Ng)| <= this
  // Freestyle COINCIDENT-plane skip (GeomUtils::intersectRayPlane == COINCIDENT):
  // a hit whose face PLANE passes through the silhouette point (the QI ray origin)
  // is that point's OWN surface, not a real occluder. The perpendicular distance
  // from the origin to the hit plane is tHit * |dir . normalize(Ng)| (the origin
  // is on the ray, the hit is on its own plane), so a hit with that distance <=
  // this epsilon is coincident (skip). A genuine front occluder a fold-gap away
  // has a large perpendicular distance (count). World units; <= 0 disables.
  float coplanarEps;
};

// Argument intersection filter: reject (do not count) a hit that lands on one of
// the excluded incident faces of the mesh. With rtcOccluded1 + this filter a
// rejected hit leaves the ray un-occluded, so the ray is occluded only by a
// NON-excluded primitive -- exactly Freestyle's "count occluders, skip the
// edge's own/adjacent faces" (ViewMapBuilder.cpp:2152-2195). N==1 here.
void excludeFaceFilter(const RTCFilterFunctionNArguments* args) {
  const auto* ctx = reinterpret_cast<const ExcludeFaceContext*>(args->context);
  if (ctx == nullptr) return;
  const bool doExclude = ctx->nFaces > 0;
  const bool doGraze = ctx->grazeCosEps > 0.0f;
  const bool doCoplanar = ctx->coplanarEps > 0.0f;
  if (!doExclude && !doGraze && !doCoplanar) return;  // nothing to filter
  int* valid = args->valid;
  RTCHitN* hit = args->hit;
  RTCRayN* ray = args->ray;
  for (unsigned int i = 0; i < args->N; ++i) {
    if (valid[i] == 0) continue;  // already invalid
    const unsigned int geomID = RTCHitN_geomID(hit, args->N, i);
    // (1) Self/adjacent FACE exclusion (the edge's own incident mesh triangles).
    if (doExclude && geomID == ctx->meshGeomID) {
      unsigned int primID = RTCHitN_primID(hit, args->N, i);
      if (ctx->baseTriCount > 0) primID %= ctx->baseTriCount;  // undo baked copies
      bool rejected = false;
      for (int k = 0; k < ctx->nFaces; ++k) {
        if (static_cast<unsigned int>(ctx->faces[k]) == primID) {
          valid[i] = 0;  // reject this self-face hit (not an occluder)
          rejected = true;
          break;
        }
      }
      if (rejected) continue;
    }
    // (2) TANGENTIAL (grazing) + (3) COINCIDENT-plane rejection. A face the QI ray
    // hits nearly edge-on (grazing) OR whose plane passes through the silhouette
    // point (coincident) is the silhouette's OWN surface, not a real occluder.
    if (doGraze || doCoplanar) {
      const float nx = RTCHitN_Ng_x(hit, args->N, i);
      const float ny = RTCHitN_Ng_y(hit, args->N, i);
      const float nz = RTCHitN_Ng_z(hit, args->N, i);
      const float dx = RTCRayN_dir_x(ray, args->N, i);
      const float dy = RTCRayN_dir_y(ray, args->N, i);
      const float dz = RTCRayN_dir_z(ray, args->N, i);
      const float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
      const float dlen = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (nlen > 0.0f && dlen > 0.0f) {
        // |dir . n| / (|dir||n|): the cosine; and tHit*that-unnormalized-by-|n| is
        // the perpendicular distance from the ray origin to the hit's plane.
        const float dDotN = nx * dx + ny * dy + nz * dz;
        const float c = std::fabs(dDotN) / (nlen * dlen);
        if (doGraze && c <= ctx->grazeCosEps) {
          valid[i] = 0;  // (2) grazed face: not an occluder
          continue;
        }
        if (doCoplanar) {
          // perp dist(origin, plane) = tHit * |dir . n| / |n| (dir already unit).
          const float tHit = RTCRayN_tfar(ray, args->N, i);
          const float perp = tHit * std::fabs(dDotN) / nlen;
          if (perp <= ctx->coplanarEps) valid[i] = 0;  // (3) coincident self-surface
        }
      }
    }
  }
}

// True if the built scene contains a triangle mesh (the GI / AO gate: indirect
// is gathered for mesh hits only, never for flat outline primitives).
bool meshPresent(const detail::BuiltScene& built) {
  for (const detail::GeomRecord& r : built.records)
    if (r.kind == detail::GeomKind::Mesh) return true;
  return false;
}

}  // namespace

EmbreeRenderer::~EmbreeRenderer() { releaseEmbree(); }

void EmbreeRenderer::releaseEmbree() {
  if (scene_) {
    rtcReleaseScene(scene_);
    scene_ = nullptr;
  }
  if (device_) {
    rtcReleaseDevice(device_);
    device_ = nullptr;
  }
}

bool EmbreeRenderer::occluded(const Vec3& p, const Vec3& q,
                              const int* excludeFaces, int nExclude, float eps,
                              float grazeCosEps, float coplanarEps) const {
  if (!scene_) return false;  // no live BVH (edges/visibility not built)
  const Vec3 d = q - p;
  const float len = length(d);
  if (len <= 0.0f) return false;
  const Vec3 dir = {d.x / len, d.y / len, d.z / len};
  // Trim both ends to the OPEN interval (0, raylength): a tiny absolute+relative
  // epsilon clears the surface the ray leaves and excludes the target itself.
  // Self-occlusion is handled by the FACE exclusion + TANGENTIAL rejection in the
  // filter (Freestyle), so this is only a small numerical guard -- it must NOT be
  // sized to swallow a nearby occluder (the ortho QI ray therefore uses a physical,
  // not fixed-huge, length so eps*len stays small; see qiOccludedAt).
  const float tnear = std::max(eps * len, eps);
  const float tfar = len * (1.0f - eps);
  if (tnear >= tfar) return false;

  // Plain any-hit test only when NO filter stage is needed (no self-faces to
  // exclude AND no tangential AND no coincident-plane rejection -- e.g. a non-QI
  // caller like shadows/AO).
  if ((excludeFaces == nullptr || nExclude == 0 ||
       meshGeomID_ == static_cast<unsigned int>(-1)) &&
      grazeCosEps <= 0.0f && coplanarEps <= 0.0f)
    return detail::occluded(scene_, p, dir, tnear, tfar);

  // Filtered any-hit test: the argument filter rejects (a) hits on the edge's own
  // incident mesh faces and (b) faces grazed nearly edge-on (Freestyle self-face
  // exclusion + tangential-occluder rejection), so neither is counted as an
  // occluder.
  RTCRay r;
  r.org_x = p.x;
  r.org_y = p.y;
  r.org_z = p.z;
  r.dir_x = dir.x;
  r.dir_y = dir.y;
  r.dir_z = dir.z;
  r.tnear = tnear;
  r.tfar = tfar;
  r.mask = 0xFFFFFFFFu;
  r.flags = 0;
  r.time = 0.0f;

  ExcludeFaceContext ctx;
  rtcInitRayQueryContext(&ctx.base);
  ctx.meshGeomID = meshGeomID_;
  ctx.baseTriCount = meshBaseTriCount_;
  ctx.faces = excludeFaces;
  ctx.nFaces = (excludeFaces != nullptr &&
                meshGeomID_ != static_cast<unsigned int>(-1))
                   ? nExclude
                   : 0;
  ctx.grazeCosEps = grazeCosEps;
  ctx.coplanarEps = coplanarEps;

  RTCOccludedArguments oargs;
  rtcInitOccludedArguments(&oargs);
  oargs.flags = RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER;
  oargs.filter = excludeFaceFilter;
  oargs.context = &ctx.base;

  rtcOccluded1(scene_, &r, &oargs);
  return r.tfar < 0.0f;  // a NON-excluded occluder was found
}

FrameResult EmbreeRenderer::render(const Scene& scene, const RenderOptions& opt) {
  // A renderer instance renders one scene; if reused, drop the previous BVH
  // before building the new one (keeps the lifetime invariant simple).
  releaseEmbree();

  RTCDevice device = rtcNewDevice(nullptr);
  if (!device) throw std::runtime_error("rtcNewDevice failed");
  rtcSetDeviceErrorFunction(device, embreeErrorCallback, nullptr);

  // Build all Embree geometry (cold, once per frame). On a build error this
  // releases the partial scene and throws; release the device too before
  // propagating so render() leaks neither handle.
  BuiltScene built;
  const auto tBvh0 = std::chrono::high_resolution_clock::now();
  try {
    built = buildEmbreeScene(device, scene, opt.strokeEdges.enable);
  } catch (...) {
    rtcReleaseDevice(device);
    throw;
  }
  const auto tBvh1 = std::chrono::high_resolution_clock::now();

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
  // Environment dome fill (opt.envLights > 0): a hemisphere of distant diffuse-
  // only lights around the camera-forward axis, meant to be used together with
  // AO. Appended to `lights` here; the estimator lives in ao/env_dome.hpp.
  buildEnvDomeLights(lights, opt, dir, right, trueUp);
  // POV ambient radiance: ambient_light defaults to <1,1,1>; the mesh ambient
  // term is material.ambient * pigment, applied below via ambK.
  const Vec3 ambLight = scene.ambientColor;  // expected <1,1,1> on the embree path
  const Vec3 bg = {scene.background.x, scene.background.y, scene.background.z};

  FrameResult res;
  res.width = W;
  res.height = H;
  res.color.assign(static_cast<std::size_t>(W) * H * 4, 0.0f);
  res.depth.assign(static_cast<std::size_t>(W) * H, 0.0f);
  // Edge G-buffer AOVs: allocated ONLY when edges are enabled. With edges off
  // these stay empty, so no extra memory is touched and the output path is
  // byte-identical to the no-edge render. The plane eye-z (`viewZ`) is also the
  // depth that linear fog needs, so it is additionally allocated when fog is on.
  {
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    if (opt.strokeEdges.enable || scene.fog.enabled)
      res.viewZ.assign(npix, 0.0f);
    if (opt.strokeEdges.enable) {
      res.objectId.assign(npix, 0xFFFFFFFFu);
      res.materialId.assign(npix, 0xFFFFFFFFu);
    }
    // normal doubles as the OIDN guide / cache spatial key, so it is allocated
    // for the edge pass OR the AO AOV dump. The remaining AO AOVs (albedo, the
    // contact/shape split, bent normal, mean occluder distance) ride aoWriteAov.
    if (opt.strokeEdges.enable || opt.aoWriteAov)
      res.normal.assign(npix * 3, 0.0f);
    // Albedo is the denoiser's demodulation guide as well as an AO AOV, so it is
    // allocated whenever the GI denoise pass will demodulate by it (not only on an
    // explicit AOV dump).
    const bool wantAlbedoGuide =
        opt.aoWriteAov ||
        (opt.gi && opt.denoiser != 0 && opt.denoiseDemodulateAlbedo);
    if (wantAlbedoGuide) res.albedo.assign(npix * 3, 0.0f);
    if (opt.aoWriteAov) {
      res.bentNormal.assign(npix * 3, 0.0f);
      res.contactAo.assign(npix, 1.0f);
      res.shapeAo.assign(npix, 1.0f);
      res.avgHitDist.assign(npix, 0.0f);
    }
    // GI cache AOVs: world-space first-hit position (cache seed key), the
    // interpolated indirect (debug E_cached) and the record-density debug viz.
    // The normal AOV is also needed as the cache seed normal, so allocate it for
    // the GI path too (harmless: written from the same N, no color change).
    if (opt.gi) {
      if (res.normal.empty()) res.normal.assign(npix * 3, 0.0f);
      res.position.assign(npix * 3, 0.0f);
      res.indirect.assign(npix * 3, 0.0f);
      res.giRecordViz.assign(npix * 3, 0.0f);
      res.giOcclusion.assign(npix, 0.0f);
    }
  }
  res.effectiveTriangles = scene.effectiveTriangles();

  // Resolve the bent-normal ambient gradient axis once: the camera true-up
  // (view-stable soft-box, the default) or an explicit world axis. Only read
  // when aoBentNormal is on; harmless otherwise.
  const Vec3 aoUpAxis =
      opt.aoUseCameraUp
          ? trueUp
          : safeNormalize(Vec3{opt.aoUp[0], opt.aoUp[1], opt.aoUp[2]}, trueUp);

  // Everything the per-hit shader reads, gathered once.
  const ShadeContext sc{built, m, lights, ambLight, bg, opt, aoUpAxis};

  // Veil lookup: groups rendered as additive single-layer (group alpha). Every
  // other transparency uses front-to-back "over" (fragment alpha). Empty =>
  // all transparency is "over".
  std::vector<uint8_t> isVeil;
  for (uint16_t g : scene.veilGroups) {
    if (g >= isVeil.size()) isVeil.resize(static_cast<std::size_t>(g) + 1, 0);
    isVeil[g] = 1;
  }

  // Per-pixel GI cache seed side-channels (component id / Embree geomID of the
  // first hit). Kept local to render() (not in FrameResult): the cache build
  // reads them right below. Allocated only when GI is on.
  std::vector<int> giGroup;
  std::vector<uint32_t> giGeom;
  std::vector<float> giRefl;  // per-pixel mat.diffuse * pigment (GI composite)
  if (opt.gi) {
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    giGroup.assign(npix, -1);
    giGeom.assign(npix, 0xFFFFFFFFu);
    giRefl.assign(npix * 3, 0.0f);
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
      // Plane eye-z store: written whenever allocated (edges or fog).
      if (!res.viewZ.empty()) res.viewZ[pix] = pr.viewZ;
      // Normal: written whenever allocated (edge pass or AO AOV dump).
      if (!res.normal.empty()) {
        res.normal[pix * 3 + 0] = pr.worldNormal.x;
        res.normal[pix * 3 + 1] = pr.worldNormal.y;
        res.normal[pix * 3 + 2] = pr.worldNormal.z;
      }
      // Remaining edge G-buffer: gated so the default path writes nothing extra.
      if (opt.strokeEdges.enable) {
        res.objectId[pix] = pr.objectId;
        res.materialId[pix] = pr.materialId;
      }
      // GI cache seed: world-space first-hit position + per-pixel component /
      // geomID side-channels read by the cache build below.
      if (opt.gi) {
        res.position[pix * 3 + 0] = pr.worldPos.x;
        res.position[pix * 3 + 1] = pr.worldPos.y;
        res.position[pix * 3 + 2] = pr.worldPos.z;
        giGroup[pix] = pr.firstGroup;
        giGeom[pix] = pr.firstGeomID;
        giRefl[pix * 3 + 0] = pr.giReflectance.x;
        giRefl[pix * 3 + 1] = pr.giReflectance.y;
        giRefl[pix * 3 + 2] = pr.giReflectance.z;
      }
      // Albedo guide (denoiser demodulation or AO AOV dump). Written whenever the
      // buffer was allocated above (wantAlbedoGuide).
      if (!res.albedo.empty()) {
        res.albedo[pix * 3 + 0] = pr.albedo.x;
        res.albedo[pix * 3 + 1] = pr.albedo.y;
        res.albedo[pix * 3 + 2] = pr.albedo.z;
      }
      // AO AOVs: contact/shape + bent normal + mean occluder distance.
      if (opt.aoWriteAov) {
        res.bentNormal[pix * 3 + 0] = pr.bentNormal.x;
        res.bentNormal[pix * 3 + 1] = pr.bentNormal.y;
        res.bentNormal[pix * 3 + 2] = pr.bentNormal.z;
        res.contactAo[pix] = pr.contactAo;
        res.shapeAo[pix] = pr.shapeAo;
        res.avgHitDist[pix] = pr.avgHitDist;
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
  res.pt1Timing.bvhBuild = std::chrono::duration<double>(tBvh1 - tBvh0).count();
  res.pt1Timing.primary = res.renderSeconds;

  // --- diffuse GI post-pass on the live BVH, at the (supersampled) hi-res
  // W*H. Two alternative indirect integrators share the seam: the surface
  // irradiance cache (default) and the pt1 per-pixel path-traced gather
  // (opt.giIntegrator == 1; experimental/pt1/). Both add giIntensity *
  // giReflectance * E to res.color and fill the same `indirect` AOV, on top of
  // the unchanged direct shading from the pixel loop above.
  if (opt.gi && meshPresent(built) && opt.giIntegrator == 1) {
    // Env-dome lights are DIRECT distant lights; the pt1 gather also collects
    // the sky through its miss term, so the combination counts the sky twice.
    if (opt.envLights > 0)
      std::fprintf(stderr,
                   "warning: --integrator pt1 with env-dome lights (--env-light"
                   " %d) double-counts the sky energy; use the gather sky "
                   "(--sky/--sky-radiance) instead\n",
                   opt.envLights);
    const Aabb b = scene.mesh.bounds();
    const float diag = b.valid() ? b.diagonal() : 1.0f;
    detail::IrradianceCacheParams gp;
    gp.scene = built.scene;
    gp.built = &built;
    gp.mesh = &m;
    gp.lights = &lights;
    gp.ambLight = ambLight;
    gp.envUp = aoUpAxis;
    // Gather sky: uniform (sky == ground == --sky-radiance; the all-white
    // default matches the cache's environment exactly) or gradient (zenith =
    // --sky-radiance, ground = --ao-ground). Radiance = this tint * ambLight *
    // --gi-env-intensity, evaluated ONLY at gather misses (see pt1_design.md).
    {
      const Vec3 skyR{opt.pt1SkyRadiance[0], opt.pt1SkyRadiance[1],
                      opt.pt1SkyRadiance[2]};
      gp.skyColor = skyR;
      gp.groundColor = (opt.pt1SkyMode == 1)
                           ? Vec3{opt.aoGroundColor[0], opt.aoGroundColor[1],
                                  opt.aoGroundColor[2]}
                           : skyR;
    }
    gp.envIntensity = opt.giEnvIntensity;
    gp.samples = std::max(1, opt.pt1Spp);
    gp.bounces = 1;
    // Unlike the cache (which truncates at 0.1 * diag for concavity contrast),
    // pt1 is the ground-truth reference: gather to infinity unless the user
    // caps it explicitly.
    gp.maxDistance = (opt.giMaxDistance > 0.0f)
                         ? opt.giMaxDistance
                         : std::numeric_limits<float>::infinity();
    gp.spacing = diag * 0.007f;  // unused by the per-pixel gather; keep valid
    gp.shadows = opt.shadows;
    gp.shadowSamples = opt.shadowSamples;

    const int spp = std::max(1, opt.pt1Spp);
    const std::size_t npix = static_cast<std::size_t>(W) * H;
    // Gather-eligible mask: first hit is a mesh (same gate as the cache; CSG
    // outline primitives receive no indirect, matching the cache path).
    std::vector<uint8_t> eligible(npix, 0);
    for (std::size_t pix = 0; pix < npix; ++pix) {
      const uint32_t g = giGeom[pix];
      if (g != 0xFFFFFFFFu && built.records[g].kind == GeomKind::Mesh)
        eligible[pix] = 1;
    }

    std::vector<float> Ebuf, occBuf;
    const auto tg0 = std::chrono::high_resolution_clock::now();
    detail::gatherPt1Grid(gp, W, H, res.position.data(), res.normal.data(),
                          /*geomNormal=*/nullptr, eligible.data(), spp,
                          opt.pt1Seed, diag, Ebuf, occBuf);
    const auto tg1 = std::chrono::high_resolution_clock::now();
    res.pt1Timing.gather = std::chrono::duration<double>(tg1 - tg0).count();

    // Denoise the indirect irradiance ONLY (pre-composite; direct light and
    // albedo are noise-free): OIDN on E with the reflectance/normal guides.
    if (opt.pt1Denoise) {
      const auto td0 = std::chrono::high_resolution_clock::now();
      detail::denoisePt1E(W, H, Ebuf, giRefl.data(), res.normal.data(),
                          res.position.data(), opt);
      const auto td1 = std::chrono::high_resolution_clock::now();
      res.pt1Timing.denoise = std::chrono::duration<double>(td1 - td0).count();
    }

    // [E] composite -- same form as the cache path: L += giIntensity *
    // (mat.diffuse * pigment) * E. The constant ambient was already dropped in
    // the shade (gi path); occlusion lives inside E, counted exactly once.
    const float gI = opt.giIntensity;
    tbb::parallel_for(tbb::blocked_range<int>(0, H),
                      [&](const tbb::blocked_range<int>& rows) {
      for (int py = rows.begin(); py != rows.end(); ++py) {
        for (int px = 0; px < W; ++px) {
          const std::size_t pix = static_cast<std::size_t>(py) * W + px;
          if (!eligible[pix]) continue;
          const Vec3 ind{gI * giRefl[pix * 3 + 0] * Ebuf[pix * 3 + 0],
                         gI * giRefl[pix * 3 + 1] * Ebuf[pix * 3 + 1],
                         gI * giRefl[pix * 3 + 2] * Ebuf[pix * 3 + 2]};
          res.color[pix * 4 + 0] += ind.x;
          res.color[pix * 4 + 1] += ind.y;
          res.color[pix * 4 + 2] += ind.z;
          res.indirect[pix * 3 + 0] = ind.x;
          res.indirect[pix * 3 + 1] = ind.y;
          res.indirect[pix * 3 + 2] = ind.z;
          res.giOcclusion[pix] = occBuf[pix];
        }
      }
    });

    std::fprintf(stderr,
                 "pt1: full-res gather %dx%d spp=%d -> %.3fs (denoise %.3fs)\n",
                 W, H, spp, res.pt1Timing.gather, res.pt1Timing.denoise);
  } else if (opt.gi && meshPresent(built)) {
    // Surface irradiance cache: [B] placement + [C] gather/fill + neighbor
    // clamp, then [D] interpolation into the `indirect` (E_cached) and
    // `giRecordViz` (record-density) AOVs and the [E] color composite.
    const Aabb b = scene.mesh.bounds();
    const float diag = b.valid() ? b.diagonal() : 1.0f;
    detail::IrradianceCacheParams gp;
    gp.scene = built.scene;
    gp.built = &built;
    gp.mesh = &m;
    gp.lights = &lights;
    gp.ambLight = ambLight;
    gp.envUp = aoUpAxis;
    gp.skyColor = Vec3{opt.aoSkyColor[0], opt.aoSkyColor[1], opt.aoSkyColor[2]};
    gp.groundColor =
        Vec3{opt.aoGroundColor[0], opt.aoGroundColor[1], opt.aoGroundColor[2]};
    // Environment (sky/ground miss) radiance = scene.ambientColor (gp.ambLight)
    // times the user gi-env-intensity multiplier. scene.ambientColor IS the
    // ambient light energy the GI gathers occlusion-aware; GI is meaningful only
    // when that ambient carries real energy, so the harness moves a fraction of
    // the lighting energy into it when GI is on (the POV radiosity _amb_frac
    // balance) -- otherwise an all-direct lighting starves GI to a near no-op.
    gp.envIntensity = opt.giEnvIntensity;
    gp.samples = std::max(1, opt.giSamples);
    gp.bounces = std::max(1, opt.giBounces);
    gp.gradients = opt.giGradients;
    gp.outlierReject = opt.giOutlierReject;
    // Auto gather distance: a fraction of the scene diagonal. Full-diagonal
    // gather lets distant surfaces fill the hemisphere uniformly, washing out the
    // concavity contrast (every point sees ~the same far geometry); a contact-
    // scale fraction keeps near occluders dominant so pockets read darker, while
    // still collecting the local one-bounce indirect. Tune with --gi-max-dist.
    gp.maxDistance = (opt.giMaxDistance > 0.0f) ? opt.giMaxDistance : diag * 0.1f;
    // Auto record spacing = a small fraction of the scene diagonal. k0 = 0.007
    // (~140 records across the diagonal) is a balanced default: fine enough that
    // the interpolated cache resolves surface concavities without the blocky look
    // of the coarser k0 = 0.01, at ~2x the record count. Override with
    // --gi-spacing; the adaptive step will later refine tight (small-R_i) regions.
    gp.spacing = (opt.giRecordSpacing > 0.0f) ? opt.giRecordSpacing : diag * 0.007f;
    if (gp.spacing <= 0.0f) gp.spacing = 1.0f;
    gp.accuracy = opt.giAccuracy;
    gp.normalReject = opt.giNormalReject;
    gp.componentReject = opt.giComponentReject;
    gp.shadows = opt.shadows;
    gp.shadowSamples = opt.shadowSamples;

    detail::IrradianceCache cache = detail::buildIrradianceCache(
        gp, W, H, res.position.data(), res.normal.data(), giGroup.data(),
        giGeom.data(), opt.giSeedPerVertex);

    // Record-radius heatmap normalization. A record's harmonic-mean radius R_i
    // is small in a concavity (nearby occluders dominate) and large on an open
    // surface; the two extremes span orders of magnitude (Rmin .. maxDistance),
    // so map log(R_i) linearly into [0,1]. Dark = small radius = tight/concave
    // record, bright = large radius = open surface.
    const float densRmin = std::fmax(0.25f * gp.spacing, 1.0e-6f);
    const float densLogLo = std::log(densRmin);
    const float densLogSpan =
        std::fmax(std::log(std::fmax(gp.maxDistance, densRmin)) - densLogLo, 1.0e-6f);

    // [D] per-pixel interpolation into the debug AOVs (parallel, read-only).
    tbb::parallel_for(tbb::blocked_range<int>(0, H),
                      [&](const tbb::blocked_range<int>& rows) {
      for (int py = rows.begin(); py != rows.end(); ++py) {
        for (int px = 0; px < W; ++px) {
          const std::size_t pix = static_cast<std::size_t>(py) * W + px;
          const uint32_t g = giGeom[pix];
          if (g == 0xFFFFFFFFu) continue;
          if (built.records[g].kind != GeomKind::Mesh) continue;
          const Vec3 X{res.position[pix * 3 + 0], res.position[pix * 3 + 1],
                       res.position[pix * 3 + 2]};
          const Vec3 Nx{res.normal[pix * 3 + 0], res.normal[pix * 3 + 1],
                        res.normal[pix * 3 + 2]};
          Vec3 E{0.0f, 0.0f, 0.0f};
          float occ = 0.0f;
          float rad = gp.maxDistance;
          detail::interpolateIrradiance(cache, gp, X, Nx, giGroup[pix], E, &occ,
                                        &rad);
          // [E] A-route composite: L += giIntensity * (mat.diffuse * pigment) *
          // E_cached. The constant ambient was already dropped from res.color in
          // the shade (gi path), and no AO multiplies here -- occlusion lives
          // inside E_cached, so it is counted exactly once. indirect AOV stores
          // this added term (for denoise demodulation / debug).
          const float gI = opt.giIntensity;
          const Vec3 ind{gI * giRefl[pix * 3 + 0] * E.x,
                         gI * giRefl[pix * 3 + 1] * E.y,
                         gI * giRefl[pix * 3 + 2] * E.z};
          res.color[pix * 4 + 0] += ind.x;
          res.color[pix * 4 + 1] += ind.y;
          res.color[pix * 4 + 2] += ind.z;
          res.indirect[pix * 3 + 0] = ind.x;
          res.indirect[pix * 3 + 1] = ind.y;
          res.indirect[pix * 3 + 2] = ind.z;
          res.giOcclusion[pix] = occ;  // AO-like concavity map (env-independent)
          // Trust-radius heatmap: the interpolated (smooth) record radius R_i.
          // R_i is the harmonic-mean distance to surrounding geometry, so it is
          // SMALL where the surface folds in on itself (a record there only
          // covers a tight area => that is where adaptive seeding would add more
          // records) and LARGE on open surfaces. log-mapped: dark = small R_i =
          // tight concavity / contact, bright = large R_i = open.
          if (rad > 0.0f) {
            float h = (std::log(rad) - densLogLo) / densLogSpan;
            if (h < 0.0f) h = 0.0f;
            if (h > 1.0f) h = 1.0f;
            res.giRecordViz[pix * 3 + 0] = h;
            res.giRecordViz[pix * 3 + 1] = h;
            res.giRecordViz[pix * 3 + 2] = h;
          }
        }
      }
    });

    std::fprintf(stderr,
                 "gi: irradiance cache built -- %zu records (spacing %.4g, "
                 "samples %d, maxDist %.4g)\n",
                 cache.records.size(), gp.spacing, gp.samples, gp.maxDistance);
  }

  // Record the mesh geometry identity so occluded() can map an excluded
  // mesh-triangle id to an Embree (geomID, primID) hit for Freestyle self-face
  // exclusion. The mesh is attached first (scene_build.cpp), so it is geomID 0
  // when present; find it by kind to stay robust if the build order changes or
  // the scene has no mesh.
  meshGeomID_ = static_cast<unsigned int>(-1);  // RTC_INVALID_GEOMETRY_ID
  for (std::size_t g = 0; g < built.records.size(); ++g)
    if (built.records[g].kind == GeomKind::Mesh) {
      meshGeomID_ = static_cast<unsigned int>(g);
      break;
    }
  meshBaseTriCount_ = static_cast<unsigned int>(scene.mesh.triangleCount());

  // Keep the device + committed scene ALIVE so the edge pass (run after this
  // returns, before the box downsample) can ray-cast against the live BVH via
  // occluded(). They are released in the destructor / on the next render().
  device_ = device;
  scene_ = built.scene;
  return res;
}

}  // namespace umbreon
