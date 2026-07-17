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

  // 6) giIntensity = 0: pt1 and pt2 direct passes agree bitwise for a
  //    principled scene too (they share the pixel loop; the S1 fake env
  //    term is integrator-independent).
  {
    const umbreon::Scene sc = makeScene(principled(0.5f, 0.3f, 0.5f));
    umbreon::RenderOptions o1 = makeOpts();
    umbreon::RenderOptions o2 = makeOpts();
    o1.gi = true;
    o1.giIntegrator = 1;
    o2.gi = true;
    o2.giIntegrator = 2;
    o1.giIntensity = 0.0f;
    o2.giIntensity = 0.0f;
    for (umbreon::RenderOptions* o : {&o1, &o2}) {
      o->pt1Spp = 4;
      o->pt1GatherDiv = 1;
      o->pt1Denoise = false;
      o->pt1EdgePatch = false;
    }
    s.check("giIntensity=0: pt1 and pt2 principled direct bitwise equal",
            bitEqual(umbreon::render(sc, o1).color,
                     umbreon::render(sc, o2).color));
  }

  return s.report();
}
