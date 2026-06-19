// Focused reader for the POV-Ray scene file (.pov) emitted by CueMol.
//
// This is NOT a general POV-Ray SDL interpreter. It evaluates just enough of
// the CueMol scene template to recover the *viewing setup*: the camera
// (orthographic / perspective), the two light sources produced by the
// SpecLighting / FlashLighting macros, and the background color. Geometry is
// handled separately by the mesh2 reader; this reader only reports the path of
// the .inc referenced via `#declare _scene = #include "..."`.
//
// Supported subset:
//   - #declare of scalar / vector constants (with arithmetic and a few
//     built-in functions: degrees, atan2, sqrt, vnormalize, ...)
//   - #if / #ifdef / #ifndef / #else / #end conditional evaluation
//   - #macro ... #end definitions (skipped, not expanded)
//   - camera { ... } with orthographic | perspective branches
//   - background { color rgb[...] ... }
//   - SpecLighting(...) / FlashLighting(...) macro invocations
//   - fog { ... } (distance, color, fog_type, fog_offset, fog_alt, up)
// Everything else (radiosity spheres, includes of standard headers, ...) is
// ignored.
#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "scene.hpp"

namespace umbreon {
// Resolved scalar/vector constants (name -> components), shared with the
// geometry reader so the .inc can reference values declared in the .pov.
using PovSymbols = std::map<std::string, std::vector<double>>;
}  // namespace umbreon

namespace umbreon {

// Thrown when the input cannot be interpreted as a CueMol .pov scene.
struct PovSceneError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct PovParseOptions {
  int imageWidth = 300;
  int imageHeight = 300;
  // Constants predefined on the command line (POV "Declare=name=value").
  // These seed the symbol table before the file is read, so an `#ifndef`
  // default inside the file does not override them.
  std::map<std::string, double> predefined;
};

struct PovSceneResult {
  Camera camera;
  std::vector<DistantLight> lights;
  Vec3 background{0.0f, 0.0f, 0.0f};
  Fog fog;                  // fog { ... } if present and active
  // global_settings { assumed_gamma G }. POV authors scene colors in this
  // gamma space; its net effect on the output is equivalent to raising the
  // final linear radiance to the power G before encoding (G=1.0 is a no-op).
  float assumedGamma = 1.0f;
  // All scalar/vector #declare constants resolved while reading the .pov, to
  // seed the geometry reader (the .inc uses e.g. `_34_35_sl_rise`).
  PovSymbols declares;
  std::string includePath;  // .inc referenced by `#declare _scene = #include`
  bool radiosity = false;   // true if `_radiosity` was defined
};

// Parse the scene setup from POV-Ray SDL text.
PovSceneResult readPovSceneFromString(const std::string& text,
                                      const PovParseOptions& opt);

// Parse the scene setup from a .pov file.
PovSceneResult readPovScene(const std::string& path,
                            const PovParseOptions& opt);

}  // namespace umbreon
