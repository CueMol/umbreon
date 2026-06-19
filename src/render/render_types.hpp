// Renderer-agnostic options and framebuffer result shared by the umbreon
// (Embree) renderer and the bench harness. Pure C++17, no
// rendering-library dependency.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace umbreon {

struct RenderOptions {
  int width = 1024;
  int height = 768;
  // Supersampling factor: umbreon::render() renders at width*ss x height*ss and
  // box-averages down to width x height in linear space (antialiasing).
  int supersample = 1;
  int aoSamples = 16;          // scivis only: AO rays per shading point
  float aoDistance = 1.0e20f;  // scivis only: AO ray max distance
  int spp = 1;                 // pixel samples per accumulation frame
  int accumFrames = 16;        // progressive accumulation frames
  int maxPathLength = 16;      // pathtracer only: max GI bounce depth
  bool flatten = false;        // bake the instance grid into one mesh
  bool flipNormals = false;
  bool shadows = true;         // scivis: cast shadows from lights
  float specularScale = 1.0f;  // multiplies the material specular (ks / weight)
  float shininess = -1.0f;     // obj: overrides the Phong exponent (ns); <0 = auto
  std::string material = "obj";  // cartoon material: "obj" or "principled"
  float ior = 1.5f;            // principled: dielectric index of refraction
  float outlineKd = 1.0f;      // outline/wireframe matte kd; high => flat (ambient-1 look)
  std::string renderer = "pathtracer";  // "pathtracer" (GI) or "scivis" (AO)

  // --- transparency (single-pass single-layer-per-group compositing) ---
  // When on, the renderer walks hits front-to-back and additively composites
  // the frontmost surface of each transparency group over the nearest opaque
  // surface (linear space; order-independent, matching CueMol's blendpng).
  bool transparency = true;
  // When on, the background contributes 0 coverage so the output alpha equals
  // the accumulated transparent coverage (POV "_transpbg"); default = opaque bg.
  bool transparentBackground = false;
  // Cap on transparent hits walked per primary ray (groups are few in practice).
  int maxTransparentLayers = 32;
};

// Rendered frame: linear HDR color plus AOV channels, top-left pixel origin.
struct FrameResult {
  int width = 0;
  int height = 0;
  std::vector<float> color;   // width*height*4 linear HDR RGBA
  std::vector<float> albedo;  // width*height*3
  std::vector<float> normal;  // width*height*3 world-space
  std::vector<float> depth;   // width*height   ray distance from camera
  double renderSeconds = 0.0;
  std::size_t effectiveTriangles = 0;
};

}  // namespace umbreon
