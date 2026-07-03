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
#include <string>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

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

  return s.report();
}
