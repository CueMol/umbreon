// Focused reader for the POV-Ray geometry emitted by CueMol2.
//
// This is NOT a general POV-Ray SDL interpreter. It scans a .inc file for the
// geometry CueMol emits and returns it in renderer-ready form:
//   - mesh2 blocks (one or more) -> a single de-indexed triangle Mesh
//   - sphere { ... } primitives  -> spheres (balls / silhouette joints)
//   - edge_line(...) / edge_line2(...) macro calls -> cylinders (sticks /
//     silhouette edges); these expand to `cylinder { v1+raise*n1, v2+raise*n2,
//     w ... }` in CueMol's macros.
// Macro definitions, container objects and labels are skipped. The geometry
// references constants declared in the .pov (e.g. `_34_35_sl_rise`), so callers
// pass those in via `seedSymbols`.
#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "scene.hpp"

namespace umbreon {

// Thrown when the input cannot be interpreted as a CueMol geometry file.
struct Mesh2ReadError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// Geometry recovered from a CueMol .inc file.
struct SceneGeometry {
  Mesh mesh;
  std::vector<Sphere> spheres;
  std::vector<Cylinder> cylinders;
  // Transparency group names: group index -> CueMol section id (e.g. "_34_35",
  // from "#if (_show_34_35)"). Index 0 is the default group (geometry outside
  // any "#if (_show_*)" block). Lets callers map a section id to a group.
  std::vector<std::string> groupNames;
};

// Scalar/vector constants predefined by the .pov before its `#include` of the
// geometry (name -> components; 1 component is a scalar).
using SymbolTable = std::map<std::string, std::vector<double>>;

// Parse all geometry from POV-Ray SDL text.
SceneGeometry readGeometryFromString(const std::string& text,
                                     const SymbolTable& seedSymbols = {});

// Parse all geometry from the given .inc file.
SceneGeometry readGeometryFromFile(const std::string& path,
                                   const SymbolTable& seedSymbols = {});

// Convenience wrappers returning just the merged mesh (used by the tests).
Mesh readMesh2FromString(const std::string& text);
Mesh readMesh2FromFile(const std::string& path);

}  // namespace umbreon
