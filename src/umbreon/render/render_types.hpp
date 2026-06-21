// Renderer-agnostic options and framebuffer result shared by the umbreon
// (Embree) renderer and the bench harness. Pure C++17, no
// rendering-library dependency.
#pragma once

#include <cstddef>
#include <vector>

namespace umbreon {

// Options for umbreon::render(). Every field here is honored by the renderer;
// the defaults reproduce the POV-faithful look with all secondary effects off
// (so a default-constructed RenderOptions yields plain primary-ray shading).
struct RenderOptions {
  // --- output ---
  int width = 1024;   // final image width  (pixels)
  int height = 768;   // final image height (pixels)
  // Supersampling factor: render at width*ss x height*ss and box-average down to
  // width x height in linear space (antialiasing). 1 = off.
  int supersample = 1;

  // --- ambient occlusion (mesh hits only; modulates the ambient term) ---
  // Default 0 = AO off, so flag-less output stays the bit-exact POV-matched
  // local shading. AO never darkens flat outline primitives (spheres/cylinders).
  int aoSamples = 0;           // AO rays per mesh hit; 0 = off
  float aoDistance = 1.0e20f;  // AO occluder search radius (ray tfar / world units)
  float aoIntensity = 1.0f;    // AO strength: aoFactor = 1 - aoIntensity*(1-rawAO)

  // --- shadows (per-light visibility; never applied to outline primitives) ---
  bool shadows = false;        // cast shadows from the lights; false = off
  int shadowSamples = 1;       // shadow rays per light (>1 = soft area light)
  float lightRadius = 0.0f;    // light angular radius (deg); >0 = soft shadows

  // --- shading ---
  float specularScale = 1.0f;  // multiplies each material's specular weight

  // --- transparency (single-pass front-to-back compositing) ---
  // When on, the renderer walks hits front-to-back and composites every
  // transparent fragment ("over"), with groups in Scene::veilGroups instead laid
  // additively as single-layer "veils" (CueMol blendpng). Off = opaque only.
  bool transparency = true;
  // When on, the background contributes 0 coverage so the output alpha equals the
  // accumulated transparent coverage (POV "_transpbg"); default = opaque bg.
  bool transparentBackground = false;
  // Safety ceiling on transparent hits walked per primary ray. Normal
  // termination is the opacity early-out (accumulated alpha >= ~1); this only
  // bites pathological deep stacks. The renderer warns once if a ray hits it.
  int maxTransparentLayers = 256;
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
