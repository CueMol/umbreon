// bench-side POV finish -> principled subset conversion (--material
// principled): maps every parsed Material onto the ShadingModel::Principled
// parameters so a .pov scene exercises the principled pipeline. The mapping
// is LOSSY by design -- the POV lobes (unnormalized Blinn + additive Phong,
// no Fresnel, brilliance) and the principled GGX lobe are different models;
// only diffuse-only materials (specular = 0, phong = 0, reflection = 0,
// brilliance = 1) reproduce the POV render bitwise. See
// docs/principled_design.md section 6 for the exactness classes.
#pragma once

#include "scene.hpp"

namespace umbreon {

// True for finishes whose look DEPENDS on the POV model being non-physical
// -- the toon/NPR tricks an energy-conserving BSDF cannot represent:
// an overdriven highlight amount (specular/phong > 1: the term saturates
// the channel into a hard-edged flat pip, while a physical lobe is bounded
// by Fresnel <= 1 and can never clip) and brilliance == 0 (a constant,
// N.L-independent diffuse -- Lambert cannot be flat). The converter keeps
// these on ShadingModel::Pov, so a --material principled scene renders
// physically-representable finishes as principled and toon finishes
// unchanged (mixed-model scenes are a supported, tested configuration).
bool isNonPhysicalFinish(const Material& in);

// Convert one POV-model material to its principled equivalent. Materials
// already tagged Principled and non-physical (toon) finishes are returned
// unchanged.
Material toPrincipledMaterial(const Material& in);

}  // namespace umbreon
