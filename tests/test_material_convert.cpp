// POV finish -> principled conversion table (src/bench/material_convert.cpp):
// row-by-row checks of the lossy mapping, including the F_Metal-style
// metallic promotion, the exponent -> perceptual-roughness map, the mirror
// snap for highlight-less reflection, and the flagged
// reflection-with-highlight -> partial-metallic heuristic.
#include <cmath>

#include "test_util.hpp"

#include "material_convert.hpp"

namespace {

// Reference re-implementation of the exponent -> roughness map.
float expToRough(float e) {
  return std::sqrt(std::sqrt(2.0f / (e + 2.0f)));
}

bool near(float a, float b, float tol = 1e-6f) {
  return std::fabs(a - b) <= tol;
}

}  // namespace

int main() {
  umbreon::test::Suite s("material_convert");
  using umbreon::Material;
  using umbreon::ShadingModel;
  using umbreon::toPrincipledMaterial;

  // Diffuse-only finish: base fields carried, no dielectric lobe.
  {
    Material in;  // defaults: diffuse .8, specular 0, phong 0, reflection 0
    in.ambient = 0.15f;
    in.emission = 2.0f;
    const Material out = toPrincipledMaterial(in);
    s.check("diffuse-only: model switched, base fields carried",
            out.model == ShadingModel::Principled &&
                near(out.ambient, 0.15f) && near(out.diffuse, 0.8f) &&
                near(out.emission, 2.0f));
    s.check("diffuse-only: no specular lobe",
            near(out.pbr.specular, 0.0f) && near(out.pbr.metallic, 0.0f));
  }

  // Blinn highlight: specular s + roughness r -> exponent 1/r.
  {
    Material in;
    in.specular = 0.4f;
    in.roughness = 0.02f;  // exponent 50
    const Material out = toPrincipledMaterial(in);
    s.check("blinn: specular carried (clamped)", near(out.pbr.specular, 0.4f));
    s.check("blinn: roughness via alpha = sqrt(2/(e+2))",
            near(out.pbr.roughness, expToRough(50.0f), 1e-4f));
    s.check("blinn: stays dielectric", near(out.pbr.metallic, 0.0f));
  }

  // Phong highlight (no Blinn): phong_size is the exponent directly.
  {
    Material in;
    in.phong = 0.8f;
    in.phongSize = 50.0f;
    const Material out = toPrincipledMaterial(in);
    s.check("phong: amount carried", near(out.pbr.specular, 0.8f));
    s.check("phong: phongSize is the exponent",
            near(out.pbr.roughness, expToRough(50.0f), 1e-4f));
  }

  // Toon/NPR finishes are NON-PHYSICAL and stay POV: an overdriven
  // highlight amount (saturating hard pip) or brilliance 0 (flat diffuse)
  // cannot be represented by an energy-conserving BSDF.
  {
    Material toon;  // scene4's 3-tone finish
    toon.ambient = 0.3f;
    toon.diffuse = 0.5f;
    toon.brilliance = 0.0f;
    toon.phong = 10000.0f;
    toon.phongSize = 50.0f;
    s.check("toon finish detected as non-physical",
            umbreon::isNonPhysicalFinish(toon));
    const Material out = toPrincipledMaterial(toon);
    s.check("toon finish stays POV (model unchanged)",
            out.model == ShadingModel::Pov && near(out.phong, 10000.0f) &&
                near(out.brilliance, 0.0f));

    Material flat;  // 1-tone flat diffuse
    flat.brilliance = 0.0f;
    s.check("brilliance 0 alone stays POV",
            toPrincipledMaterial(flat).model == ShadingModel::Pov);

    Material overdriven;
    overdriven.specular = 4.0f;
    s.check("specular > 1 stays POV",
            toPrincipledMaterial(overdriven).model == ShadingModel::Pov);

    Material metal;  // F_MetalA-shaped: brilliance 2 is fine to convert
    metal.metallic = true;
    metal.brilliance = 2.0f;
    metal.specular = 0.8f;
    metal.roughness = 0.05f;
    s.check("F_Metal-style brilliance 2 still converts",
            toPrincipledMaterial(metal).model == ShadingModel::Principled);
  }

  // POV metallic bool (F_Metal presets set it): promoted to metallic 1.
  {
    Material in;
    in.metallic = true;
    in.specular = 0.8f;
    in.roughness = 0.05f;
    const Material out = toPrincipledMaterial(in);
    s.check("metallic finish -> pbr.metallic 1", near(out.pbr.metallic, 1.0f));
  }

  // Mirror: reflection without a highlight -> polished metal, roughness 0.
  {
    Material in;
    in.reflection = 0.8f;
    const Material out = toPrincipledMaterial(in);
    s.check("highlight-less reflection -> metallic mirror",
            near(out.pbr.metallic, 1.0f) && near(out.pbr.roughness, 0.0f));
  }

  // FLAGGED HEURISTIC: reflection + highlight -> partial metallic by the
  // reflection amount (keeps a `reflection 0.3` glossy floor visible where
  // a dielectric F0 <= 0.08 would nearly erase it).
  {
    Material in;
    in.reflection = 0.3f;
    in.specular = 0.6f;
    in.roughness = 0.02f;
    const Material out = toPrincipledMaterial(in);
    s.check("reflection+highlight -> metallic = reflection",
            near(out.pbr.metallic, 0.3f));
    s.check("reflection+highlight keeps the highlight lobe",
            near(out.pbr.specular, 0.6f) &&
                near(out.pbr.roughness, expToRough(50.0f), 1e-4f));
  }

  // Already-principled materials pass through unchanged.
  {
    Material in;
    in.model = ShadingModel::Principled;
    in.pbr.metallic = 0.7f;
    in.pbr.roughness = 0.25f;
    const Material out = toPrincipledMaterial(in);
    s.check("principled input is returned unchanged",
            near(out.pbr.metallic, 0.7f) && near(out.pbr.roughness, 0.25f));
  }

  return s.report();
}
