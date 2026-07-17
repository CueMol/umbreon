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

// Convert one POV-model material to its principled equivalent. Materials
// already tagged Principled are returned unchanged.
Material toPrincipledMaterial(const Material& in);

}  // namespace umbreon
