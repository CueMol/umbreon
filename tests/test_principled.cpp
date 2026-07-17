// Principled BSDF subset (Material::model == ShadingModel::Principled),
// direct-shading stage (S1). Locks the invariants the design rests on:
//
//  1. Bitwise diffuse parity: a principled material with metallic = 0 and
//     specular = 0 (f0max == 0, so the GGX lobe AND the fake environment
//     term are skipped entirely) renders bitwise-equal to the POV material
//     with specular = 0, phong = 0, brilliance = 1, reflection = 0 -- in
//     shadows-off, hard-shadow and pt2-GI configurations. This is the
//     energy-continuity anchor between the two models.
//  2. POV scenes are untouched (structural: shadeLocal early-returns for
//     principled only; enforced globally by refactor_check).
//  3. Determinism: run-to-run and thread-count bit-exactness of the
//     principled paths (single-direction and area-light sampling loops are
//     pure per-pixel functions).
//  4. Model rules: fill lights get no specular (roughness must not matter
//     under a fill light); GGX highlight peak decreases with roughness;
//     the fake Fresnel*background environment term keeps metals visible in
//     every mode (basic / AO / pt1) until pt2's traced pass takes over.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <tbb/global_control.h>

#include "test_util.hpp"
#include "umbreon.hpp"

#include "shading/principled.hpp"

namespace {

umbreon::Mesh makeFloor(umbreon::Vec4 color) {
  using umbreon::Vec3;
  umbreon::Mesh m;
  const Vec3 p00{-4, 0, -4}, p10{4, 0, -4}, p11{4, 0, 4}, p01{-4, 0, 4};
  const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
  const Vec3 n{0, 1, 0};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  return m;
}

// Floor + one sphere, one distant light. Both carry `mat`.
umbreon::Scene makeScene(const umbreon::Material& mat,
                         bool fillLight = false) {
  umbreon::Scene sc;
  sc.mesh = makeFloor(umbreon::Vec4{0.7f, 0.7f, 0.7f, 1.0f});
  sc.mesh.material = mat;
  sc.camera.position = {0, 3, 8};
  sc.camera.direction = umbreon::normalize(umbreon::Vec3{0, -0.35f, -1});
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = false;
  sc.camera.fovy = 40.0f;
  umbreon::DistantLight l;
  l.direction = umbreon::normalize(umbreon::Vec3{-0.4f, -1.0f, -0.3f});
  l.color = {1, 1, 1};
  l.intensity = 0.8f;
  l.castsHighlight = !fillLight;
  sc.lights.push_back(l);
  umbreon::Sphere sp;
  sp.center = {0, 1, 0};
  sp.radius = 1.0f;
  sp.color = {0.8f, 0.3f, 0.3f, 1.0f};
  sp.material = mat;
  sc.spheres.push_back(sp);
  sc.background = {0.9f, 0.9f, 0.95f};
  sc.ambientColor = {0.5f, 0.5f, 0.5f};
  return sc;
}

umbreon::Material povDiffuseOnly() {
  umbreon::Material m;
  m.ambient = 0.2f;
  m.diffuse = 0.8f;
  m.specular = 0.0f;
  m.phong = 0.0f;
  m.brilliance = 1.0f;
  m.reflection = 0.0f;
  return m;
}

umbreon::Material principled(float metallic, float roughness,
                             float specular) {
  umbreon::Material m = povDiffuseOnly();
  m.model = umbreon::ShadingModel::Principled;
  m.pbr.metallic = metallic;
  m.pbr.roughness = roughness;
  m.pbr.specular = specular;
  return m;
}

umbreon::Material principledAniso(float anisotropy, float rotation) {
  umbreon::Material m = principled(1.0f, 0.35f, 0.5f);
  m.pbr.anisotropy = anisotropy;
  m.pbr.anisotropyRotation = rotation;
  return m;
}

// Adds a capped principled cylinder lying above the floor (axis along x).
void addBond(umbreon::Scene& sc, const umbreon::Material& mat) {
  umbreon::Cylinder cy;
  cy.p0 = {-2.0f, 1.4f, 1.0f};
  cy.p1 = {2.0f, 1.4f, 1.0f};
  cy.radius = 0.5f;
  cy.color = {0.85f, 0.7f, 0.3f, 1.0f};
  cy.material = mat;
  cy.open = false;
  sc.cylinders.push_back(cy);
}

bool allFinite(const std::vector<float>& v) {
  for (float x : v)
    if (!std::isfinite(x)) return false;
  return true;
}

umbreon::RenderOptions makeOpts() {
  umbreon::RenderOptions o;
  o.width = 48;
  o.height = 36;
  return o;
}

bool bitEqual(const std::vector<float>& a, const std::vector<float>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i] != b[i]) return false;
  return true;
}

float meanLum(const umbreon::FrameResult& f) {
  double s = 0.0;
  const std::size_t n = f.color.size() / 4;
  for (std::size_t p = 0; p < n; ++p)
    s += 0.2126 * f.color[p * 4] + 0.7152 * f.color[p * 4 + 1] +
         0.0722 * f.color[p * 4 + 2];
  return static_cast<float>(s / static_cast<double>(n));
}

// Two-material variant: floor and sphere carry different materials.
umbreon::Scene makeScene2(const umbreon::Material& floorMat,
                          const umbreon::Material& sphereMat) {
  umbreon::Scene sc = makeScene(floorMat);
  sc.spheres[0].material = sphereMat;
  return sc;
}

umbreon::RenderOptions makePt2Opts() {
  umbreon::RenderOptions o = makeOpts();
  o.gi = true;
  o.giIntegrator = 2;
  o.pt1Spp = 4;
  o.pt1GatherDiv = 1;
  o.pt1Denoise = false;
  o.pt1EdgePatch = false;
  return o;
}

}  // namespace

int main() {
  umbreon::test::Suite s("principled");

  // 1) Bitwise diffuse parity: metallic=0 + specular=0 principled ==
  //    diffuse-only POV, in (a) shadows off, (b) hard shadows, (c) pt2 GI.
  {
    const umbreon::Scene pov = makeScene(povDiffuseOnly());
    const umbreon::Scene pbr = makeScene(principled(0.0f, 0.5f, 0.0f));

    umbreon::RenderOptions a = makeOpts();
    s.check("diffuse parity, shadows off (bitwise)",
            bitEqual(umbreon::render(pov, a).color,
                     umbreon::render(pbr, a).color));

    umbreon::RenderOptions b = makeOpts();
    b.shadows = true;  // lightRadius 0 -> hard single-ray path
    s.check("diffuse parity, hard shadows (bitwise)",
            bitEqual(umbreon::render(pov, b).color,
                     umbreon::render(pbr, b).color));

    umbreon::RenderOptions c = makeOpts();
    c.gi = true;
    c.giIntegrator = 2;
    c.pt1Spp = 4;
    c.pt1GatherDiv = 1;
    c.pt1Denoise = false;
    c.pt1EdgePatch = false;
    s.check("diffuse parity, pt2 GI (bitwise: kd seam is model-neutral)",
            bitEqual(umbreon::render(pov, c).color,
                     umbreon::render(pbr, c).color));
  }

  // 2) Determinism of the principled paths: run-to-run and thread count,
  //    exercising both the single-direction path (hard light) and the
  //    area-light per-sample loop (angularRadius > 0 under pt2).
  {
    const umbreon::Scene sc = makeScene(principled(0.6f, 0.25f, 0.5f));
    umbreon::RenderOptions o = makeOpts();
    o.shadows = true;
    const umbreon::FrameResult f1 = umbreon::render(sc, o);
    const umbreon::FrameResult f2 = umbreon::render(sc, o);
    s.check("principled render is run-to-run bit-exact",
            bitEqual(f1.color, f2.color));
    umbreon::FrameResult t1;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      t1 = umbreon::render(sc, o);
    }
    s.check("principled render: 1 thread == N threads (bitwise)",
            bitEqual(t1.color, f1.color));

    umbreon::Scene area = makeScene(principled(0.6f, 0.25f, 0.5f));
    area.lights[0].angularRadius = 0.15f;
    umbreon::RenderOptions oa = makeOpts();
    oa.shadows = true;
    oa.shadowSamples = 4;
    oa.gi = true;
    oa.giIntegrator = 2;  // per-light angularRadius is honored under pt2
    oa.pt1Spp = 4;
    oa.pt1GatherDiv = 1;
    oa.pt1Denoise = false;
    oa.pt1EdgePatch = false;
    const umbreon::FrameResult a1 = umbreon::render(area, oa);
    const umbreon::FrameResult a2 = umbreon::render(area, oa);
    s.check("area-light principled path is run-to-run bit-exact",
            bitEqual(a1.color, a2.color));
    umbreon::FrameResult at;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      at = umbreon::render(area, oa);
    }
    s.check("area-light principled path: 1 thread == N threads (bitwise)",
            bitEqual(at.color, a1.color));
    // The area path must actually differ from the hard-light render (the
    // non-separable estimator is exercised, not silently skipped).
    umbreon::Scene hard = makeScene(principled(0.6f, 0.25f, 0.5f));
    const umbreon::FrameResult h1 = umbreon::render(hard, oa);
    s.check("area light changes the principled render",
            !bitEqual(a1.color, h1.color));
  }

  // 3) Fill-light rule: with only a fill light, roughness is never read
  //    (no specular lobe evaluated; the fake environment term does not
  //    depend on roughness either) -> images bitwise-equal.
  {
    const umbreon::Scene r1 = makeScene(principled(0.8f, 0.1f, 0.5f), true);
    const umbreon::Scene r2 = makeScene(principled(0.8f, 0.7f, 0.5f), true);
    umbreon::RenderOptions o = makeOpts();
    s.check("fill light: roughness does not enter (bitwise)",
            bitEqual(umbreon::render(r1, o).color,
                     umbreon::render(r2, o).color));
  }

  // 4) GGX estimator sanity at the helper level (a render-level peak
  //    comparison is discretization-fragile at test resolutions: a sharp
  //    lobe's peak pixel may simply not be sampled), plus the render-level
  //    fact that roughness changes the image at all.
  {
    using umbreon::detail::ggxD;
    using umbreon::detail::ggxLambda;
    using umbreon::detail::schlickF;
    // Peak NDF value decreases with alpha; D integrates to <= 1 in cosine
    // measure so a wider lobe must be lower at its peak.
    const bool dMonotone = ggxD(0.01f * 0.01f, 1.0f) > ggxD(0.1f * 0.1f, 1.0f) &&
                           ggxD(0.1f * 0.1f, 1.0f) > ggxD(0.5f * 0.5f, 1.0f);
    s.check("ggxD peak decreases with alpha", dMonotone);
    // Smith Lambda grows with alpha and with grazing angle (more masking).
    const bool lamMonotone =
        ggxLambda(0.25f, 0.3f) > ggxLambda(0.04f, 0.3f) &&
        ggxLambda(0.25f, 0.1f) > ggxLambda(0.25f, 0.9f);
    s.check("ggxLambda grows with alpha and grazing", lamMonotone);
    // Schlick endpoints: F(1) = F0, F(0) = 1.
    const umbreon::Vec3 F0{0.04f, 0.04f, 0.04f};
    const umbreon::Vec3 fHead = schlickF(F0, 1.0f);
    const umbreon::Vec3 fGraze = schlickF(F0, 0.0f);
    s.check("schlickF endpoints (F(1)=F0, F(0)=1)",
            fHead.x == 0.04f && fGraze.x == 1.0f);

    umbreon::RenderOptions o = makeOpts();
    const umbreon::Scene r1 = makeScene(principled(1.0f, 0.15f, 0.5f));
    const umbreon::Scene r2 = makeScene(principled(1.0f, 0.6f, 0.5f));
    s.check("roughness changes the principled render",
            !bitEqual(umbreon::render(r1, o).color,
                      umbreon::render(r2, o).color));
  }

  // 5) Mode commonality: a metallic=1 surface stays visibly non-black in
  //    basic raytracing, AO mode and pt1 (the fake Fresnel*background term
  //    stands in for the traced reflection outside pt2).
  {
    const umbreon::Scene sc = makeScene(principled(1.0f, 0.3f, 0.5f));
    umbreon::RenderOptions basic = makeOpts();
    s.check("metal visible in basic mode", meanLum(umbreon::render(sc, basic)) > 0.05f);

    umbreon::RenderOptions ao = makeOpts();
    ao.aoSamples = 8;
    ao.aoDistance = 10.0f;
    const umbreon::FrameResult fao = umbreon::render(sc, ao);
    s.check("metal visible in AO mode", meanLum(fao) > 0.05f);
    const umbreon::FrameResult fao2 = umbreon::render(sc, ao);
    s.check("AO-mode principled render is bit-exact", bitEqual(fao.color, fao2.color));

    umbreon::RenderOptions p1 = makeOpts();
    p1.gi = true;
    p1.giIntegrator = 1;
    p1.pt1Spp = 4;
    p1.pt1GatherDiv = 1;
    p1.pt1Denoise = false;
    p1.pt1EdgePatch = false;
    s.check("metal visible in pt1 mode", meanLum(umbreon::render(sc, p1)) > 0.05f);
  }

  // 6) giIntensity = 0 with the traced passes disabled: pt1 and pt2 share
  //    the pixel loop, so the principled direct render (incl. the fake env
  //    term) must agree bitwise across integrators. (With pt2Reflect ON the
  //    two legitimately differ: pt2 replaces the fake env with traced
  //    reflections regardless of giIntensity -- that swap is test 11's
  //    subject.)
  {
    const umbreon::Scene sc = makeScene(principled(0.5f, 0.3f, 0.5f));
    umbreon::RenderOptions o1 = makeOpts();
    umbreon::RenderOptions o2 = makeOpts();
    o1.gi = true;
    o1.giIntegrator = 1;
    o2.gi = true;
    o2.giIntegrator = 2;
    o2.pt2Reflect = false;  // keep the fake env term on both sides
    o1.giIntensity = 0.0f;
    o2.giIntensity = 0.0f;
    for (umbreon::RenderOptions* o : {&o1, &o2}) {
      o->pt1Spp = 4;
      o->pt1GatherDiv = 1;
      o->pt1Denoise = false;
      o->pt1EdgePatch = false;
    }
    s.check("giIntensity=0, pt2Reflect off: pt1 == pt2 direct (bitwise)",
            bitEqual(umbreon::render(sc, o1).color,
                     umbreon::render(sc, o2).color));
  }

  // ---- S2: traced specular (pt2 mirror + glossy with per-pixel F0) ----

  // 7) Mirror degeneracy: roughness 0.03 (alpha 9e-4) and roughness 0 both
  //    snap to the mirror pass (alpha < kPt2GlossyAlphaMin) AND clamp to the
  //    same direct-lobe alpha (kGgxDirectAlphaMin = 1e-3), so the renders
  //    must be bitwise equal end to end (same F0, both pixels mirror-owned,
  //    same direct highlight).
  {
    const umbreon::Scene a = makeScene(principled(1.0f, 0.03f, 0.5f));
    const umbreon::Scene b = makeScene(principled(1.0f, 0.0f, 0.5f));
    umbreon::RenderOptions o = makePt2Opts();
    s.check("alpha below the mirror cut == roughness 0 (bitwise)",
            bitEqual(umbreon::render(a, o).color,
                     umbreon::render(b, o).color));
  }

  // 8) Metallic color fidelity (furnace-ish): a lightless metal sphere
  //    under a uniform white sky reflects ~F0 at normal incidence -- the
  //    center pixel's channel ratios must match F0 = baseColor within 20%
  //    (colored metal reflection, the point of the metallic axis).
  {
    umbreon::Scene sc;
    sc.camera.position = {0, 0, 8};
    sc.camera.direction = {0, 0, -1};
    sc.camera.up = {0, 1, 0};
    sc.camera.orthographic = false;
    sc.camera.fovy = 30.0f;
    umbreon::Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.5f;
    sp.color = {0.9f, 0.5f, 0.2f, 1.0f};
    sp.material = principled(1.0f, 0.4f, 0.5f);
    sc.spheres.push_back(sp);
    sc.background = {1, 1, 1};
    sc.ambientColor = {1, 1, 1};
    umbreon::RenderOptions o = makePt2Opts();
    const umbreon::FrameResult f = umbreon::render(sc, o);
    const std::size_t cx = o.width / 2, cy = o.height / 2;
    const std::size_t pix = cy * o.width + cx;
    const float r = f.color[pix * 4 + 0], g = f.color[pix * 4 + 1],
                b = f.color[pix * 4 + 2];
    const bool ratios =
        std::fabs(r / g - 0.9f / 0.5f) <= 0.2f * (0.9f / 0.5f) &&
        std::fabs(g / b - 0.5f / 0.2f) <= 0.2f * (0.5f / 0.2f);
    // Magnitude: near-normal incidence reflects ~F0 (single-scatter loss
    // and the slight Fresnel rise keep it within a broad band).
    const bool magnitude = r > 0.5f * 0.9f && r < 1.2f * 0.9f;
    s.check("metal center pixel ~ F0 = baseColor (ratios within 20%)",
            ratios && magnitude);
  }

  // 9) Gates: a matte principled material (no specular lobe) never enters
  //    the traced passes -- toggling them is a bit-exact no-op.
  {
    const umbreon::Scene sc = makeScene(principled(0.0f, 0.5f, 0.0f));
    umbreon::RenderOptions a = makePt2Opts();
    umbreon::RenderOptions b = makePt2Opts();
    b.pt2Reflect = false;
    s.check("matte principled: --pt2-reflect is a bit-exact no-op",
            bitEqual(umbreon::render(sc, a).color,
                     umbreon::render(sc, b).color));
    umbreon::RenderOptions c = makePt2Opts();
    c.pt2Glossy = false;
    s.check("matte principled: --pt2-glossy is a bit-exact no-op",
            bitEqual(umbreon::render(sc, a).color,
                     umbreon::render(sc, c).color));
  }

  // 10) reflF0 neutrality end-to-end: swapping a matte POV sphere for the
  //     bitwise-equivalent matte principled sphere must leave a scene with
  //     POV reflective/glossy pixels byte-identical -- the neutral (1,1,1)
  //     F0 reproduces every k*L composite exactly even though the buffer
  //     switches the passes onto their Fresnel code path.
  {
    umbreon::Material povFloor = povDiffuseOnly();
    povFloor.reflection = 0.3f;
    povFloor.specular = 0.4f;
    povFloor.roughness = 0.05f;
    const umbreon::Scene a = makeScene2(povFloor, povDiffuseOnly());
    const umbreon::Scene b = makeScene2(povFloor, principled(0.0f, 0.5f, 0.0f));
    umbreon::RenderOptions o = makePt2Opts();
    s.check("neutral F0: POV reflective pixels bitwise with principled nearby",
            bitEqual(umbreon::render(a, o).color,
                     umbreon::render(b, o).color));
  }

  // 11) gi off: the traced passes never run, so the integrator id cannot
  //     matter for a principled metal (fake env term is shared). Under pt2
  //     GI, toggling pt2Reflect swaps fake env <-> traced reflection and
  //     the images must actually differ (the swap happens).
  {
    const umbreon::Scene sc = makeScene(principled(1.0f, 0.3f, 0.5f));
    umbreon::RenderOptions o1 = makeOpts();
    umbreon::RenderOptions o2 = makeOpts();
    o1.gi = false;
    o2.gi = false;
    o1.giIntegrator = 1;
    o2.giIntegrator = 2;
    s.check("gi off: principled metal identical across integrators",
            bitEqual(umbreon::render(sc, o1).color,
                     umbreon::render(sc, o2).color));

    umbreon::RenderOptions p2on = makePt2Opts();
    umbreon::RenderOptions p2off = makePt2Opts();
    p2off.pt2Reflect = false;
    s.check("pt2Reflect swaps fake env for traced reflection (differs)",
            !bitEqual(umbreon::render(sc, p2on).color,
                      umbreon::render(sc, p2off).color));
  }

  // 12) Determinism of the traced principled paths (glossy metal floor +
  //     dielectric sphere exercises mirror-free glossy pixels + F0 fold).
  {
    const umbreon::Scene sc = makeScene2(principled(1.0f, 0.35f, 0.5f),
                                         principled(0.0f, 0.2f, 0.8f));
    umbreon::RenderOptions o = makePt2Opts();
    const umbreon::FrameResult f1 = umbreon::render(sc, o);
    const umbreon::FrameResult f2 = umbreon::render(sc, o);
    s.check("traced principled specular is run-to-run bit-exact",
            bitEqual(f1.color, f2.color));
    umbreon::FrameResult t1;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      t1 = umbreon::render(sc, o);
    }
    s.check("traced principled specular: 1 thread == N threads (bitwise)",
            bitEqual(t1.color, f1.color));
  }

  // ---- S3: anisotropy (sphere / cylinder frames) ----

  // 13) Mesh anisotropy is inert (no per-vertex tangents in v1): setting
  //     anisotropy on a MESH material must leave the render bitwise
  //     unchanged, end to end (no tangent in the direct pass, no aniso
  //     buffers allocated -- the scan covers primitives only).
  {
    umbreon::Material meshAniso = principled(1.0f, 0.35f, 0.5f);
    meshAniso.pbr.anisotropy = 0.8f;
    const umbreon::Scene a =
        makeScene2(meshAniso, principled(0.0f, 0.5f, 0.0f));
    const umbreon::Scene b = makeScene2(principled(1.0f, 0.35f, 0.5f),
                                        principled(0.0f, 0.5f, 0.0f));
    umbreon::RenderOptions o = makePt2Opts();
    s.check("mesh anisotropy is bitwise inert (v1)",
            bitEqual(umbreon::render(a, o).color,
                     umbreon::render(b, o).color));
  }

  // 14) Primitive anisotropy is live: an anisotropic metal sphere and a
  //     capped bond change both the direct highlight (gi off) and the
  //     traced glossy lobe (pt2), and rotation reorients the lobe.
  {
    umbreon::Scene an = makeScene2(povDiffuseOnly(), principledAniso(0.8f, 0.0f));
    addBond(an, principledAniso(0.8f, 0.0f));
    umbreon::Scene iso = makeScene2(povDiffuseOnly(), principled(1.0f, 0.35f, 0.5f));
    addBond(iso, principled(1.0f, 0.35f, 0.5f));
    umbreon::RenderOptions direct = makeOpts();
    const umbreon::FrameResult fa = umbreon::render(an, direct);
    s.check("anisotropy changes the direct highlight (primitives)",
            !bitEqual(fa.color, umbreon::render(iso, direct).color));
    s.check("anisotropic direct render is finite", allFinite(fa.color));

    umbreon::Scene rot = makeScene2(povDiffuseOnly(), principledAniso(0.8f, 0.25f));
    addBond(rot, principledAniso(0.8f, 0.25f));
    s.check("anisotropyRotation reorients the lobe (differs)",
            !bitEqual(fa.color, umbreon::render(rot, direct).color));

    umbreon::RenderOptions pt2 = makePt2Opts();
    const umbreon::FrameResult ga = umbreon::render(an, pt2);
    s.check("anisotropy changes the traced glossy lobe (pt2)",
            !bitEqual(ga.color, umbreon::render(iso, pt2).color));
    s.check("anisotropic pt2 render is finite", allFinite(ga.color));
  }

  // 15) Determinism of the anisotropic paths (tangent frames and the
  //     stretched VNDF are pure per-pixel functions).
  {
    umbreon::Scene sc = makeScene2(povDiffuseOnly(), principledAniso(0.6f, 0.1f));
    addBond(sc, principledAniso(0.6f, 0.35f));
    umbreon::RenderOptions o = makePt2Opts();
    const umbreon::FrameResult f1 = umbreon::render(sc, o);
    const umbreon::FrameResult f2 = umbreon::render(sc, o);
    s.check("anisotropic render is run-to-run bit-exact",
            bitEqual(f1.color, f2.color));
    umbreon::FrameResult t1;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      t1 = umbreon::render(sc, o);
    }
    s.check("anisotropic render: 1 thread == N threads (bitwise)",
            bitEqual(t1.color, f1.color));
  }

  // 15b) Toon/unlit GI exemption (Material::toonLike): a toon material's
  //      look is self-contained -- GI neither replaces its ambient nor
  //      composites indirect onto it, so an all-toon scene renders bitwise
  //      identically with GI on and off, and an unlit ("nolighting")
  //      surface keeps its exact flat pigment under pt2.
  {
    umbreon::Material toon;  // scene4-style 3-tone finish
    toon.ambient = 0.3f;
    toon.diffuse = 0.5f;
    toon.brilliance = 0.0f;
    toon.phong = 10000.0f;
    toon.phongSize = 50.0f;
    umbreon::Scene sc = makeScene2(toon, toon);
    // White ambient: under GI toon evaluates at UNIT ambient (the scene
    // ambientColor carries the gather's energy split), so the gi-off render
    // matches bitwise exactly when the gi-off ambient is white too -- the
    // bench's non-GI convention.
    sc.ambientColor = {1.0f, 1.0f, 1.0f};
    umbreon::RenderOptions off = makeOpts();
    umbreon::RenderOptions on = makePt2Opts();
    s.check("all-toon scene: pt2 == gi-off (bitwise; ambient kept, no GI)",
            bitEqual(umbreon::render(sc, off).color,
                     umbreon::render(sc, on).color));

    umbreon::Material unlit;  // nolighting: flat pigment is the final color
    unlit.ambient = 1.0f;
    unlit.diffuse = 0.0f;
    unlit.specular = 0.0f;
    // NOTE: ambientColor deliberately left at the scene builder's non-white
    // (0.5): toon ambient evaluates at UNIT ambient regardless (the scene
    // ambient carries the GI energy split, not the toon look's base color).
    umbreon::Scene flat = makeScene2(unlit, unlit);
    const umbreon::FrameResult f = umbreon::render(flat, makePt2Opts());
    // The image-center pixel lies on the unlit sphere (pigment 0.8/0.3/0.3):
    // the material must reproduce the pigment exactly (ambient 1 * C * 1).
    const int W = 48, H = 36;
    const std::size_t pix = static_cast<std::size_t>(H / 2) * W + W / 2;
    s.check("unlit (nolighting) renders the exact flat pigment under pt2",
            f.color[pix * 4 + 0] == 0.8f && f.color[pix * 4 + 1] == 0.3f &&
                f.color[pix * 4 + 2] == 0.3f);
  }

  // 15c) ShadingModel::Toon -- the explicit NPR tag. Toon is a LABEL, not a
  //      look: it shades through the same lobes as Pov, so tagging a finish
  //      must not move a pixel. What the tag buys is INTENT -- toonLike()
  //      honours it even when the field values look perfectly physical, which
  //      the Pov heuristic (fields only) can never infer. An API caller tags;
  //      a .pov scene, having no way to spell NPR, is still sniffed.
  {
    umbreon::Material pov;  // deliberately physical-looking fields: the Pov
    pov.ambient = 0.3f;     // field heuristic does NOT call this toon
    pov.diffuse = 0.5f;
    pov.specular = 0.4f;
    pov.roughness = 0.01f;
    s.check("physical fields on Pov: not toonLike", !pov.toonLike());

    umbreon::Material tagged = pov;
    tagged.model = umbreon::ShadingModel::Toon;
    s.check("same fields tagged Toon: toonLike (the tag, not the fields)",
            tagged.toonLike());

    // Same fields -> same lobes: the tag is pixel-neutral outside GI.
    s.check("Toon renders bitwise == Pov (tag is a label, not a look)",
            bitEqual(umbreon::render(makeScene2(pov, pov), makeOpts()).color,
                     umbreon::render(makeScene2(tagged, tagged), makeOpts())
                         .color));

    // Under GI the tag is what earns the exemption: the Toon scene keeps its
    // (unit) ambient and takes no indirect, so pt2 == gi-off bitwise.
    umbreon::Scene toonScene = makeScene2(tagged, tagged);
    toonScene.ambientColor = {1.0f, 1.0f, 1.0f};
    s.check("Toon-tagged scene: pt2 == gi-off (bitwise; GI-exempt via tag)",
            bitEqual(umbreon::render(toonScene, makeOpts()).color,
                     umbreon::render(toonScene, makePt2Opts()).color));

    // Negative control: the SAME fields left on Pov are physical, so GI does
    // re-light them. Without this the check above would pass even if the tag
    // did nothing and GI were simply inert on this scene.
    umbreon::Scene povScene = makeScene2(pov, pov);
    povScene.ambientColor = {1.0f, 1.0f, 1.0f};
    s.check("untagged Pov twin IS re-lit by GI (proves the tag does the work)",
            !bitEqual(umbreon::render(povScene, makeOpts()).color,
                      umbreon::render(povScene, makePt2Opts()).color));
  }

  // 16) Supersampled E_spec denoise round trip (box-downsample -> OIDN at
  //     the output grid -> joint-bilateral upsample): deterministic
  //     run-to-run and across thread counts, and finite.
  {
    const umbreon::Scene sc = makeScene2(principled(1.0f, 0.35f, 0.5f),
                                         principled(0.0f, 0.2f, 0.8f));
    umbreon::RenderOptions o = makePt2Opts();
    o.supersample = 3;
    o.pt1Denoise = true;  // enables the E_spec round trip
    const umbreon::FrameResult f1 = umbreon::render(sc, o);
    const umbreon::FrameResult f2 = umbreon::render(sc, o);
    s.check("ss=3 E_spec denoise round trip is run-to-run bit-exact",
            bitEqual(f1.color, f2.color) && allFinite(f1.color));
    umbreon::FrameResult t1;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      t1 = umbreon::render(sc, o);
    }
    s.check("ss=3 E_spec round trip: 1 thread == N threads (bitwise)",
            bitEqual(t1.color, f1.color));
  }

  return s.report();
}
