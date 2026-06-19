// Builds a renderable Scene from a base Mesh: camera auto-framing and the
// N x N x N instance grid used to scale the polygon count for benchmarking.
#pragma once

#include "scene.hpp"

namespace umbreon {

struct BuildOptions {
  int gridN = 1;               // produces gridN^3 instances
  float spacing = 1.15f;       // grid pitch as a multiple of the mesh size
  float cameraAzimuth = 25.0f;    // degrees, orbit around Y
  float cameraElevation = 22.0f;  // degrees above the horizon
  float fitMargin = 1.12f;     // >1 leaves a margin around the geometry
  float fovy = 35.0f;          // vertical field of view, degrees
  // Sky (ambient) + sun (distant) lighting, tuned for the path tracer so the
  // global illumination is well exposed.
  float lightIntensity = 1.5f;     // distant "sun"
  float ambientIntensity = 0.6f;   // uniform "sky"
  Vec3 background{0.04f, 0.04f, 0.06f};
};

// Construct a Scene: copies the mesh, lays out the instance grid, frames the
// camera around the whole grid and adds a key + ambient light.
Scene buildScene(const Mesh& mesh, const BuildOptions& opt);

}  // namespace umbreon
