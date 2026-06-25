// Renderer-agnostic options and framebuffer result shared by the umbreon
// (Embree) renderer and the bench harness. Pure C++17, no
// rendering-library dependency.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace umbreon {

// --- NPR edge styling types (shared) --------------------------------------
// These styling types are consumed by the Freestyle-style STROKE edge pass
// (render/stroke_edges.cpp, --edges): the stroke pipeline maps each EdgeNature
// onto an EdgeClass slot for per-section coloring (see Scene::groupEdgeStyle).
// They are independent of the OBJECT-SPACE (3D geometry) edge pass, whose
// options live in render/object_space_edges.hpp (ObjectSpaceEdgeOptions,
// --obj-edges).
//
// The five edge classes. The stroke pass uses Silhouette / Object (border) /
// Crease as its per-nature styling slots and composites in a fixed precedence
// order (most-specific structural edge wins). `Count` is the array size, not a
// class.
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
// styles. A section without an explicit override uses
// StrokeEdgeOptions::defaultStyle.
struct EdgeStyle {
  EdgeClassStyle cls[static_cast<int>(EdgeClass::Count)];
};

// --- Freestyle-style stroke edge rendering (--edges) ----------------------
// Options for the STROKE edge pipeline (render/stroke_edges.cpp), the
// Freestyle-faithful replacement for the retired per-pixel screen-space pass.
// It extracts topology-tagged feature edges (silhouette/crease/border), chains
// them into continuous polylines, computes ray-cast visibility against the live
// Embree BVH, then rasterizes variable-width ribbons composited in linear space.
// Per-section styling reuses EdgeStyle/EdgeClassStyle/Scene::groupEdgeStyle.
//
// The master `enable` flag gates ALL new work: a default-constructed
// StrokeEdgeOptions (enable == false) runs no extra pass, so the renderer output
// stays byte-identical to the no-edge path.
struct StrokeEdgeOptions {
  bool enable = false;  // MASTER gate; false => zero new work, byte-identical

  // --- which natures to extract/draw ---
  bool silhouette = true;  // smooth n.v==0 contour + hard-edge straddle
  bool crease = true;      // interior fold edges (dihedral test)
  bool border = true;      // open boundary edges (one incident face)

  // --- feature-edge extraction params (mirror ObjectSpaceEdgeOptions) ---
  // Ray-cast visibility is analytic, so no 3D lift is needed (raise == 0).
  float raise = 0.0f;                      // outward contour offset, world units
  float meshHardEdgeDeg = 40.0f;           // hard-edge straddle / cluster split
  float creaseAngleDeg = 30.0f;            // crease dihedral threshold (degrees)
  float meshCreaseSmoothVetoDeg = 35.0f;   // smooth-facet crease veto (0 = off)
  bool meshCreaseConvexOnly = true;        // keep convex creases, drop valleys
  float meshBorderCoplanarVetoDeg = 35.0f; // coplanar-continuation border veto
  int meshCreaseMaxDegree = 4;             // drop crease hubs above this degree

  // --- visibility (FREESTYLE-FAITHFUL image-space hidden-line) ---
  // Visibility is decided in two complementary image-space stages, run per chain
  // by applyStrokeEdges (NO primary z-buffer / G-buffer is read, keeping this
  // distinct from --obj-edges):
  //  (A) Quantitative Invisibility: a ray cast from each feature SEGMENT's 3D
  //      center toward the eye, counting solid surfaces, with the segment's OWN
  //      incident mesh faces excluded as self-occluders (Freestyle
  //      ViewMapBuilder self/adjacent-face skip, live via the Embree argument
  //      filter). qi>0 => hidden.
  //  (B) a 2D crossing pass over the PROJECTED feature segments: where two drawn
  //      lines cross in screen space, the farther one (larger eye-space view-z
  //      at the crossing) is hidden (Freestyle CreateTVertex).
  // A stroke point is visible iff QI says visible AND it is not inside a (B)
  // hidden notch. There are no tuning knobs: the zTol that guards a coincident-
  // depth junction is derived from the mesh mean edge length internally.

  // --- stylization ---
  // Stroke geometry in FINAL-resolution pixels; applyStrokeEdges scales these by
  // the supersample factor since it runs on the hi-res (pre-downsample) frame, so
  // a line keeps its requested final width at any --supersample.
  int thickness = 2;        // stroke FULL width, final px (phase-1 const)
  int resampleStepPx = 2;   // arc-length resample step, final px
  float color[3] = {0.0f, 0.0f, 0.0f};  // default linear RGB
  float opacity = 1.0f;                 // 0..1

  // Per-section styling: a section without an override uses defaultStyle. The
  // stroke pipeline maps each EdgeNature onto a styling slot in EdgeStyle (see
  // Scene::groupEdgeStyle).
  EdgeStyle defaultStyle;
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

  // --- Freestyle-style stroke edges (--edges) --- defaulted OFF (enable ==
  // false). When off, no edge AOV is allocated and applyStrokeEdges is never
  // invoked, so output is byte-identical to the no-edge path. This single flag
  // is the master gate for the whole --edges pipeline (G-buffer AOV capture,
  // the stroke pass, and the baked-edge removal); see StrokeEdgeOptions.
  StrokeEdgeOptions strokeEdges;
};

// Rendered frame: linear HDR color plus AOV channels, top-left pixel origin.
struct FrameResult {
  int width = 0;
  int height = 0;
  std::vector<float> color;   // width*height*4 linear HDR RGBA
  std::vector<float> albedo;  // width*height*3
  std::vector<float> normal;  // width*height*3 world-space
  std::vector<float> depth;   // width*height   ray distance from camera
  // Edge G-buffer AOVs: sized and written ONLY when RenderOptions::strokeEdges
  // is enabled (otherwise left empty, keeping the default path byte-identical).
  std::vector<float> viewZ;          // width*height   linear view-z (edge-only)
  std::vector<std::uint32_t> objectId;    // width*height   per-pixel object id
  std::vector<std::uint32_t> materialId;  // width*height   per-pixel material id
  double renderSeconds = 0.0;
  std::size_t effectiveTriangles = 0;
};

}  // namespace umbreon
