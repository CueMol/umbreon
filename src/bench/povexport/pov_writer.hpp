// Emit a POV-Ray scene equivalent to a rendered Scene, so POV-Ray can be
// benchmarked on the identical geometry, camera and instance grid.
#pragma once

#include <string>

#include "scene.hpp"

namespace umbreon {

struct PovWriteOptions {
  int width = 1024;
  int height = 768;
  bool radiosity = true;   // emit a radiosity setup (POV-Ray's AO mechanism)
  // rad_def.inc Rad_Settings preset index. 7 = OutdoorHQ, the high-quality
  // radiosity preset offered by CueMol2's POV-Ray rendering dialog
  // (render-pov-dlg.xul: <menuitem value="7" label="OutdoorHQ"/>, passed as
  // Declare=_radiosity=7 -> Rad_Settings(_radiosity, off, off)).
  int radiosityQuality = 7;
};

// Write a self-contained .pov file: camera, lights, a single mesh2 declaration
// and one object{} per grid instance.
void writePovScene(const std::string& path, const Scene& scene,
                   const PovWriteOptions& opt);

}  // namespace umbreon
