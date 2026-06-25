// Renderer-agnostic options and framebuffer result shared by the umbreon
// (Embree) renderer and the bench harness. Pure C++17, no
// rendering-library dependency.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace umbreon {

// --- screen-space NPR edge rendering (Warabi-style) -----------------------
// The five Warabi edge classes. A pixel can qualify for several; Stage C
// composites them in a fixed precedence order (most-specific structural edge
// wins). `Count` is the array size, not a class.
enum class EdgeClass : uint8_t {
  Silhouette = 0,    // object vs background boundary
  Disconnected = 1,  // same object, depth discontinuity (signature Warabi line)
  Object = 2,        // different object id across the boundary
  Material = 3,      // different material id across the boundary
  Crease = 4,        // ridge/valley fold on the world-normal field
  Count = 5
};

// Independent styling for one edge class. Color is linear RGB (composited in
// linear space); width is the mask dilation radius in hi-res (supersample) px.
struct EdgeClassStyle {
  bool enabled = false;
  float color[3] = {0.0f, 0.0f, 0.0f};  // linear RGB
  float opacity = 1.0f;                 // 0..1
  float width = 2.0f;                   // dilation radius, hi-res px
};

// Per CueMol section (per transparency group): a bundle of the five class
// styles. A section without an explicit override uses EdgeOptions::defaultStyle.
struct EdgeStyle {
  EdgeClassStyle cls[static_cast<int>(EdgeClass::Count)];
};

// Global screen-space edge block. The master `enable` flag gates ALL new work:
// a default-constructed EdgeOptions (enable == false) allocates no AOV and runs
// no extra pass, so the renderer output stays byte-identical to the no-edge path.
struct EdgeOptions {
  bool enable = false;  // MASTER gate; false => zero new work, byte-identical

  // detection thresholds (Mol* analogues)
  float distanceThreshold = 1.0f;  // depth-gap, LINEAR VIEW-Z world units, pixelSize-scaled
  // SCALE-INVARIANT curvature veto. The depth 2nd-difference is normalized by the
  // local 1st-difference (with a pixelSize floor), so curvatureGate is a
  // dimensionless ratio: ~1 at a genuine step (where the slope jumps), well below
  // on a smoothly curved surface (where the slope changes slowly). Independent of
  // scene scale / camera units, unlike the old raw-view-z Mol* GPU constant.
  float curvatureGate = 0.2f;      // dimensionless 2nd/1st-difference ratio (disc)
  // Crease has its OWN curvature gate. A crease (fold in the normal field) occurs
  // at near-constant depth, so its depth 2nd-difference is small and the shared
  // disc curvatureGate would over-suppress it. Default 0 = no curvature veto for
  // crease (it relies on the depth-gap veto + the fold-angle test); set positive
  // only to tame a smoothly-but-tightly curved mesh barrel.
  float creaseCurvatureGate = 0.0f;
  float creaseAngleDeg = 22.0f;    // crease: fire when dot(n) < cos(creaseAngleDeg)
  // Crease grazing-angle bias gain: at full grazing the effective crease angle is
  // (1 + creaseGrazingBias)x the base angle, suppressing rim false positives on
  // smooth curvature without affecting head-on folds.
  float creaseGrazingBias = 0.3f;
  // Disconnected-face (class 2) NORMAL-CONSISTENCY gate. The line fires only when
  // the world normals across the depth gap also DISAGREE: dot(n_center, n_sample)
  // < cos(discNormalAngleDeg). A smooth SES self-occlusion fold (normals
  // continuous across the gap) is then NOT inked; a true face-to-face step
  // (normals discontinuous) still inks. Keeps dense molecular SES readable.
  float discNormalAngleDeg = 35.0f;
  int neighborhood = 4;            // 4 (+-x,+-y) default; 8 (+diagonals) thicker/closed

  // suppression tables: group ids 1..32, bit i set => co-group => boundary suppressed
  std::array<uint32_t, 33> objectSuppress{};
  std::array<uint32_t, 33> materialSuppress{};

  // styling: a section without an override uses defaultStyle
  EdgeStyle defaultStyle;

  // later-phase calligraphic pen (parsed/stored now, ignored until phase 2)
  float penHardness = 1.0f, penRoundness = 1.0f, penSlant = 0.0f;
};

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

  // --- screen-space NPR edges --- defaulted OFF (enable == false). When off,
  // no new AOV is allocated and no edge pass runs, so output is byte-identical
  // to the no-edge path.
  EdgeOptions edges;
};

// Rendered frame: linear HDR color plus AOV channels, top-left pixel origin.
struct FrameResult {
  int width = 0;
  int height = 0;
  std::vector<float> color;   // width*height*4 linear HDR RGBA
  std::vector<float> albedo;  // width*height*3
  std::vector<float> normal;  // width*height*3 world-space
  std::vector<float> depth;   // width*height   ray distance from camera
  // Screen-space edge AOVs: sized and written ONLY when RenderOptions::edges is
  // enabled (otherwise left empty, keeping the default path byte-identical).
  std::vector<float> viewZ;          // width*height   linear view-z (edge-only)
  std::vector<std::uint32_t> objectId;    // width*height   per-pixel object id
  std::vector<std::uint32_t> materialId;  // width*height   per-pixel material id
  double renderSeconds = 0.0;
  std::size_t effectiveTriangles = 0;
};

}  // namespace umbreon
