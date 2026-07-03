// Bit-exactness harness for the group-alpha multipass reuse
// (RenderOptions::blendReuse). Every scenario renders the SAME scene+options
// twice in this process -- blendReuse=0 (naive reference) and blendReuse=1
// (reuse) -- and requires every framebuffer/AOV to match byte-for-byte.
// A determinism canary (same scene+options rendered twice with the same flag)
// runs first: it establishes that the whole pipeline, including the OIDN
// denoiser, is deterministic in-process, so any A/B mismatch later is
// attributable to the reuse path and not to the environment.
//
// Phase 1 note: while the reuse path is not implemented yet, both runs take
// the naive multipass and every comparison passes trivially; the value is the
// harness itself plus the canary. The scenario matrix below is extended in
// later phases (raytracing reuse, pt1 reuse).
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

#include "render/blend_reuse.hpp"   // BlendProbeHolder / BlendReuseContext
#include "render/pipeline.hpp"      // renderFrame (capture-mode touch tests)
#include "shading/blend_probe.hpp"  // RayProbe / probeSegment unit test

namespace {

using umbreon::Vec3;
using umbreon::Vec4;

// --- scene builders ---------------------------------------------------------

// Append a flat quad spanning [x0,x1]x[y0,y1] at depth z, facing +Z, with a
// uniform color (w = fragment opacity) and a transparency group tag.
void addQuad(umbreon::Mesh& m, Vec4 color, float x0, float y0, float x1,
             float y1, float z, std::uint16_t g) {
  const Vec3 c[6] = {{x0, y0, z}, {x1, y0, z}, {x1, y1, z},
                     {x0, y0, z}, {x1, y1, z}, {x0, y1, z}};
  const Vec3 n{0, 0, 1};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(c[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  m.triGroupId.push_back(g);
  m.triGroupId.push_back(g);
}

umbreon::Camera makeOrthoCam() {
  umbreon::Camera c;
  c.position = {0, 0, 10};
  c.direction = {0, 0, -1};
  c.up = {0, 1, 0};
  c.orthographic = true;
  c.height = 4.0f;
  return c;
}

umbreon::DistantLight makeKeyLight() {
  umbreon::DistantLight l;
  l.direction = {-0.4f, -0.3f, -1.0f};
  l.color = {1, 1, 1};
  l.intensity = 0.7f;
  return l;
}

// Base scene: an opaque backdrop quad (group 0) plus per-test blend geometry.
// Mixed primitive kinds (mesh triangles, spheres, capped cylinders) so the
// probe/copy paths see every geometry type.
umbreon::Scene makeBaseScene() {
  umbreon::Scene sc;
  sc.mesh.material.ambient = 0.2f;
  sc.mesh.material.diffuse = 0.8f;
  addQuad(sc.mesh, {0.8f, 0.2f, 0.2f, 1.0f}, -2, -2, 2, 2, 0.0f, 0);
  sc.camera = makeOrthoCam();
  sc.lights.push_back(makeKeyLight());
  sc.background = {1, 1, 1};
  sc.ambientColor = {1, 1, 1};
  return sc;
}

void addBlendSphere(umbreon::Scene& sc, Vec3 center, float r, Vec4 color,
                    std::uint16_t g) {
  umbreon::Sphere s;
  s.center = center;
  s.radius = r;
  s.color = color;
  s.material.ambient = 0.2f;
  s.material.diffuse = 0.8f;
  s.group = g;
  sc.spheres.push_back(s);
}

void addBlendCylinder(umbreon::Scene& sc, Vec3 p0, Vec3 p1, float r,
                      Vec4 color, std::uint16_t g) {
  umbreon::Cylinder c;
  c.p0 = p0;
  c.p1 = p1;
  c.radius = r;
  c.color = color;
  c.material.ambient = 0.2f;
  c.material.diffuse = 0.8f;
  c.group = g;
  c.open = false;
  sc.cylinders.push_back(c);
}

// --- frame comparison -------------------------------------------------------

template <class T>
bool sameVec(const std::vector<T>& a, const std::vector<T>& b) {
  if (a.size() != b.size()) return false;
  if (a.empty()) return true;
  return std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0;
}

// Byte-compare every image output of two frames. renderSeconds / pt1Timing
// are timing metadata and intentionally excluded. materialId is skipped for
// scenes with sphere/cylinder primitives: the sphere/cyl materialId AOV
// encodes raw per-pass primitive indices, which primID renumbering across
// pass subsets legitimately changes (debug-only channel; documented in the
// reuse plan).
bool sameFrame(const umbreon::FrameResult& a, const umbreon::FrameResult& b,
               bool compareMaterialId, std::string* why) {
  auto fail = [&](const char* field) {
    if (why) *why = field;
    return false;
  };
  if (a.width != b.width || a.height != b.height) return fail("dims");
  if (!sameVec(a.color, b.color)) return fail("color");
  if (!sameVec(a.depth, b.depth)) return fail("depth");
  if (!sameVec(a.viewZ, b.viewZ)) return fail("viewZ");
  if (!sameVec(a.normal, b.normal)) return fail("normal");
  if (!sameVec(a.albedo, b.albedo)) return fail("albedo");
  if (!sameVec(a.objectId, b.objectId)) return fail("objectId");
  if (compareMaterialId && !sameVec(a.materialId, b.materialId))
    return fail("materialId");
  if (!sameVec(a.surfAlpha, b.surfAlpha)) return fail("surfAlpha");
  if (!sameVec(a.bentNormal, b.bentNormal)) return fail("bentNormal");
  if (!sameVec(a.contactAo, b.contactAo)) return fail("contactAo");
  if (!sameVec(a.shapeAo, b.shapeAo)) return fail("shapeAo");
  if (!sameVec(a.avgHitDist, b.avgHitDist)) return fail("avgHitDist");
  if (!sameVec(a.position, b.position)) return fail("position");
  if (!sameVec(a.indirect, b.indirect)) return fail("indirect");
  if (!sameVec(a.giRecordViz, b.giRecordViz)) return fail("giRecordViz");
  if (!sameVec(a.giOcclusion, b.giOcclusion)) return fail("giOcclusion");
  return true;
}

// Render with blendReuse=0 (naive) and =1 (reuse) and compare bit-for-bit.
void checkAB(umbreon::test::Suite& s, const std::string& name,
             const umbreon::Scene& sc, umbreon::RenderOptions opt,
             bool compareMaterialId = true) {
  opt.blendReuse = 0;
  const umbreon::FrameResult ref = umbreon::render(sc, opt);
  opt.blendReuse = 1;
  const umbreon::FrameResult got = umbreon::render(sc, opt);
  std::string why;
  const bool ok = sameFrame(ref, got, compareMaterialId, &why);
  s.check(name + (ok ? "" : " [differs: " + why + "]"), ok);
}

}  // namespace

int main() {
  umbreon::test::Suite s("blend_reuse");

  umbreon::RenderOptions base;
  base.width = 32;
  base.height = 32;

  // --- determinism canary: identical scene+options twice, SAME flag. Locks
  // in-process determinism of the full pipeline (incl. OIDN when pt1 is on)
  // before any A/B comparison is trusted.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    {
      umbreon::RenderOptions o = base;
      const umbreon::FrameResult f1 = umbreon::render(sc, o);
      const umbreon::FrameResult f2 = umbreon::render(sc, o);
      std::string why;
      s.check("canary: rt multipass deterministic",
              sameFrame(f1, f2, true, &why));
    }
    {
      umbreon::RenderOptions o = base;
      o.gi = true;
      o.giIntegrator = 1;  // pt1
      o.pt1Spp = 2;
      o.pt1HalfRes = false;
      o.pt1Denoise = true;  // OIDN (or a-trous fallback) determinism
      const umbreon::FrameResult f1 = umbreon::render(sc, o);
      const umbreon::FrameResult f2 = umbreon::render(sc, o);
      std::string why;
      s.check("canary: pt1+denoise deterministic",
              sameFrame(f1, f2, true, &why));
    }
  }

  // --- A/B scenarios (reuse off vs on). One blend group unless noted. ---

  // rt plain: one mesh blend group in front of the opaque backdrop.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.4f});
    checkAB(s, "rt plain, 1 mesh group", sc, base);
  }

  // rt with fog (exercises the viewZ AOV copy).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    sc.fog.enabled = true;
    sc.fog.color = sc.background;
    sc.fog.start = 8.0f;
    sc.fog.end = 12.0f;
    checkAB(s, "rt + fog", sc, base);
  }

  // rt with hard shadows; the blend sphere shadows the backdrop, so pixels
  // OUTSIDE the sphere footprint depend on the group via shadow rays only.
  {
    umbreon::Scene sc = makeBaseScene();
    addBlendSphere(sc, {0.0f, 0.0f, 1.2f}, 0.6f, {0.2f, 0.8f, 0.3f, 1.0f}, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.shadows = true;
    checkAB(s, "rt + hard shadows (sphere group)", sc, o,
            /*compareMaterialId=*/false);
  }

  // rt with mixed primitive kinds in two groups + a cylinder.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1.8f, -1.8f, -0.2f, 0.2f,
            1.0f, 1);
    addBlendSphere(sc, {1.0f, 1.0f, 1.0f}, 0.5f, {0.9f, 0.8f, 0.1f, 1.0f}, 2);
    addBlendCylinder(sc, {0.5f, -1.5f, 1.0f}, {1.5f, -0.5f, 1.0f}, 0.2f,
                     {0.5f, 0.2f, 0.8f, 1.0f}, 2);
    sc.groupBlend.push_back({1, 0.3f});
    sc.groupBlend.push_back({2, 0.4f});
    checkAB(s, "rt 2 groups, mesh+sphere+cylinder", sc, base,
            /*compareMaterialId=*/false);
  }

  // rt overlapping groups (both cover the center).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    addQuad(sc.mesh, {0.9f, 0.6f, 0.1f, 1.0f}, -0.5f, -0.5f, 1.5f, 1.5f,
            1.5f, 2);
    sc.groupBlend.push_back({1, 0.5f});
    sc.groupBlend.push_back({2, 0.3f});
    checkAB(s, "rt overlapping groups", sc, base);
  }

  // rt empty blend group (group id with no primitives).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.4f});
    sc.groupBlend.push_back({5, 0.2f});  // no geometry carries group 5
    checkAB(s, "rt empty blend group", sc, base);
  }

  // rt weight sum > 1 (bg weight clamps to 0).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    addQuad(sc.mesh, {0.9f, 0.6f, 0.1f, 1.0f}, 0, 0, 2, 2, 1.5f, 2);
    sc.groupBlend.push_back({1, 0.7f});
    sc.groupBlend.push_back({2, 0.7f});
    checkAB(s, "rt weight sum > 1", sc, base);
  }

  // rt transparent background + supersample 2 (dirty mask at hi-res).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.transparentBackground = true;
    o.supersample = 2;
    checkAB(s, "rt transparent bg + ss2", sc, o);
  }

  // rt fragment alpha inside a blend group (the "over" walk within a pass).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 0.5f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    checkAB(s, "rt fragment alpha in blend group", sc, base);
  }

  // rt stroke edges (screen-source vectorizer over the assembled AOVs). The
  // blend quad overlaps the backdrop edge region so silhouette chains cross
  // the (future) clean/dirty boundary.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 2.2f, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.strokeEdges.enable = true;
    auto& cs = o.strokeEdges.defaultStyle
                   .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
    cs.enabled = true;
    cs.width = 2.0f;
    checkAB(s, "rt stroke edges across blend boundary", sc, o);
  }

  // rt + soft shadows (multi-sample cone shadow rays).
  {
    umbreon::Scene sc = makeBaseScene();
    addBlendSphere(sc, {0.0f, 0.0f, 1.2f}, 0.6f, {0.2f, 0.8f, 0.3f, 1.0f}, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.shadows = true;
    o.shadowSamples = 4;
    o.lightRadius = 3.0f;
    checkAB(s, "rt + soft shadows", sc, o, /*compareMaterialId=*/false);
  }

  // rt + legacy AO (binary occluded() rays).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.aoSamples = 4;
    o.aoDistance = 3.0f;
    checkAB(s, "rt + AO legacy", sc, o);
  }

  // rt + enhanced AO (multi-scale + falloff + bent normal): exercises the
  // intersectNearest probe path.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.aoSamples = 4;
    o.aoDistance = 3.0f;
    o.aoFalloffPower = 1.0f;
    o.aoMultiScale = true;
    o.aoBentNormal = true;
    o.aoWriteAov = true;
    checkAB(s, "rt + AO enhanced (intersectNearest)", sc, o);
  }

  // rt + env-dome lights (many shadowed distant lights).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.envLights = 8;
    checkAB(s, "rt + env dome lights", sc, o);
  }

  // rt edges-only verification mode (surface blanked, only strokes drawn).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 2.2f, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.strokeEdges.enable = true;
    o.strokeEdges.edgesOnly = true;
    auto& cs = o.strokeEdges.defaultStyle
                   .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
    cs.enabled = true;
    cs.width = 2.0f;
    checkAB(s, "rt edges-only", sc, o);
  }

  // rt edges with a per-section style override on the blend group.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 2.2f, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.strokeEdges.enable = true;
    auto& cs = o.strokeEdges.defaultStyle
                   .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
    cs.enabled = true;
    cs.width = 2.0f;
    sc.groupEdgeStyle.assign(2, o.strokeEdges.defaultStyle);
    sc.groupEdgeStyle[1]
        .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)]
        .color[0] = 1.0f;  // red silhouette for the blend section
    checkAB(s, "rt edges, per-section style", sc, o);
  }

  // rt + final-color a-trous denoiser (global post stage on assembled color).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.denoiser = 1;  // a-trous
    checkAB(s, "rt + atrous final denoise", sc, o);
  }

  // rt verify mode (blendReuse=2): reuse output must equal naive AND the
  // internal full-frame check must pass (it prints to stderr; here we assert
  // the frame equality like the other scenarios).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    addBlendSphere(sc, {1.0f, 1.0f, 1.0f}, 0.5f, {0.9f, 0.8f, 0.1f, 1.0f}, 2);
    sc.groupBlend.push_back({1, 0.4f});
    sc.groupBlend.push_back({2, 0.3f});
    umbreon::RenderOptions o = base;
    o.shadows = true;
    o.blendReuse = 0;
    const umbreon::FrameResult ref = umbreon::render(sc, o);
    o.blendReuse = 2;  // verify
    const umbreon::FrameResult got = umbreon::render(sc, o);
    std::string why;
    s.check("rt verify mode bit-exact",
            sameFrame(ref, got, /*compareMaterialId=*/false, &why));
  }

  // pt1 full-res, denoise on.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.gi = true;
    o.giIntegrator = 1;
    o.pt1Spp = 2;
    o.pt1HalfRes = false;
    checkAB(s, "pt1 full-res + denoise", sc, o);
  }

  // pt1 half-res (private G-buffer + joint bilateral upsample), odd size.
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0.2f, 0.4f, 0.9f, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::RenderOptions o = base;
    o.width = 33;
    o.height = 31;
    o.gi = true;
    o.giIntegrator = 1;
    o.pt1Spp = 2;
    o.pt1HalfRes = true;
    checkAB(s, "pt1 half-res, odd dims", sc, o);
  }

  // --- probeSegment unit test: one blend group = the standard blend quad at
  // z=1 covering [-1,1]^2 (group 1).
  {
    umbreon::Scene sc = makeBaseScene();
    addQuad(sc.mesh, {0, 0, 1, 1.0f}, -1, -1, 1, 1, 1.0f, 1);
    sc.groupBlend.push_back({1, 0.5f});
    umbreon::detail::BlendProbeHolder probes =
        umbreon::detail::buildBlendProbes(sc);
    const float inf = std::numeric_limits<float>::infinity();
    auto touchOf = [&](Vec3 O, Vec3 dir, float tnear, float tfar) {
      std::uint32_t t = 0;
      umbreon::detail::RayProbe rp{&probes.scenes, &t};
      umbreon::detail::probeSegment(&rp, O, dir, tnear, tfar);
      return t;
    };
    // Straight through the quad center: touch bit 0 set.
    s.check("probeSegment: hit sets bit",
            touchOf({0, 0, 10}, {0, 0, -1}, 0.0f, inf) == 1u);
    // Segment ends BEFORE the quad (t up to z=2): no touch.
    s.check("probeSegment: short segment misses",
            touchOf({0, 0, 10}, {0, 0, -1}, 0.0f, 7.0f) == 0u);
    // Segment starts BEHIND the quad: no touch.
    s.check("probeSegment: tnear past quad misses",
            touchOf({0, 0, 10}, {0, 0, -1}, 10.0f, inf) == 0u);
    // Laterally outside the quad: no touch.
    s.check("probeSegment: lateral miss",
            touchOf({1.8f, 1.8f, 10}, {0, 0, -1}, 0.0f, inf) == 0u);
    // nullptr probe is a no-op (does not crash, touches nothing).
    umbreon::detail::probeSegment(nullptr, {0, 0, 10}, {0, 0, -1}, 0.0f, inf);
    s.check("probeSegment: nullptr no-op", true);
  }

  // --- capture-mode touch masks, verified directly against known geometry.
  // Blend group 1 = quad at z=1 covering the [0,2]x[0,2] (top-right) quadrant;
  // blend group 2 = a sphere whose only effect on some pixels is its SHADOW.
  // The background pass renders WITHOUT the blend groups but probes them, so
  // touch must flag: (a) pixels whose primary ray crosses the quad/sphere,
  // (b) pixels whose shadow ray toward the light crosses the sphere, and stay
  // clear elsewhere.
  {
    // Full scene carries the blend spec (for probe construction). The group-1
    // quad starts at 0.5 so the shadow-only verification point (0.2, 0.0)
    // stays OUTSIDE its footprint.
    umbreon::Scene full = makeBaseScene();
    addQuad(full.mesh, {0, 0, 1, 1.0f}, 0.5f, 0.5f, 2, 2, 1.0f, 1);
    umbreon::Sphere sp;
    sp.center = {1.0f, 0.6f, 2.0f};
    sp.radius = 0.4f;
    sp.color = {0.2f, 0.8f, 0.3f, 1.0f};
    sp.group = 2;
    full.spheres.push_back(sp);
    full.groupBlend.push_back({1, 0.3f});
    full.groupBlend.push_back({2, 0.3f});
    // ...the background scene is the SAME opaque backdrop without them.
    umbreon::Scene bg = makeBaseScene();

    umbreon::RenderOptions o;
    o.width = 32;
    o.height = 32;
    o.shadows = true;

    umbreon::detail::BlendProbeHolder probes =
        umbreon::detail::buildBlendProbes(full);
    umbreon::detail::BlendReuseContext ctx;
    ctx.mode = umbreon::detail::BlendReuseContext::Mode::Capture;
    ctx.probe = &probes.scenes;
    (void)umbreon::renderFrame(bg, o, &ctx);

    s.check("touch: dims recorded",
            ctx.hiW == 32 && ctx.hiH == 32 &&
                ctx.touch.size() == std::size_t{32 * 32});
    // World->pixel mapping for the 32x32 ortho camera framing [-2,2]:
    // x = (px+0.5)/8 - 2, y = 2 - (py+0.5)/8.
    auto touchAt = [&](float x, float y) {
      const int px = static_cast<int>((x + 2.0f) * 8.0f);
      const int py = static_cast<int>((2.0f - y) * 8.0f);
      return ctx.touch[static_cast<std::size_t>(py) * 32 + px];
    };
    // Deep inside the group-1 quad: primary rays cross it -> bit 0.
    s.check("touch: quad footprint dirty (group 1)",
            (touchAt(1.0f, 1.0f) & 1u) != 0u);
    // Far corner: no ray goes near either group.
    s.check("touch: far corner clean", touchAt(-1.8f, -1.8f) == 0u);
    // Under the sphere footprint: primary rays cross it -> bit 1.
    s.check("touch: sphere footprint dirty (group 2)",
            (touchAt(1.0f, 0.6f) & 2u) != 0u);
    // Shadow-only region: the key light travels (-0.4,-0.3,-1)/|.|, so the
    // sphere at (1.0,0.6,2) shadows the backdrop around (0.2,0.0) -- outside
    // the sphere's footprint and outside the quad. Those pixels' PRIMARY rays
    // miss both groups; only their shadow ray toward the light crosses the
    // sphere. This is the probe-coverage case for occluded() plumbing.
    const std::uint32_t shadowTouch = touchAt(0.2f, 0.0f);
    s.check("touch: shadow-only region dirty (group 2)",
            (shadowTouch & 2u) != 0u);
    s.check("touch: shadow-only region NOT dirty for group 1",
            (shadowTouch & 1u) == 0u);
  }

  return s.report();
}
