// Command-line option parsing for the prototype.
#pragma once

#include <map>
#include <string>

#include "render/render_types.hpp"

namespace umbreon {

struct Options {
  std::string input;            // input .pov scene or .inc geometry (positional)
  std::string output = "out.png";
  int width = 1024;
  int height = 768;
  int gridN = 1;                // N^3 instance grid
  float spacing = 1.15f;        // grid pitch as a multiple of mesh size
  float aoDistance = -1.0f;     // AO occluder radius; < 0 => auto from scene size
  int aoSamples = 0;            // AO rays per mesh hit; 0 = AO off
  float aoIntensity = 1.0f;     // AO strength multiplier (0..1+)
  bool shadows = false;         // cast shadows from lights
  int shadowSamples = 1;        // shadow rays per light (>1 = soft area light)
  float lightRadius = 0.0f;     // light angular radius (deg); >0 = soft shadows
  std::string emitPov;          // empty => do not emit a POV-Ray scene
  bool povRadiosity = true;     // emit a radiosity setup in the .pov
  float lightIntensity = 1.5f;  // distant "sun"
  float ambientIntensity = 0.6f;  // uniform "sky"

  // POV scene mode (input is a .pov): constants predefined like the POV-Ray
  // "Declare=name=value" command-line options. Seeded with the CueMol defaults
  // matching the reference render and overridable with --declare.
  std::map<std::string, double> declares;
  // Overall exposure gain applied to the POV-derived lights + ambient. With the
  // matte cartoon (specular off), the pi factor of the BRDF unit conversion
  // alone matches POV, so 1.0 is the joint optimum across all four reference
  // scenes (the old 1.20 only compensated for the now-removed specular).
  float povGain = 1.0f;
  // Radius multiplier for CueMol spheres/cylinders (silhouette outlines), to
  // calibrate their on-screen thickness against POV-Ray.
  float outlineScale = 1.0f;
  // Supersampling factor: render at N x the output resolution and box-average
  // down in linear space. Matches POV-Ray's adaptive antialiasing on the thin
  // silhouette lines far better than per-pixel sampling alone. The .pov path
  // defaults to 3 when not set explicitly.
  int supersample = 1;
  bool supersampleSet = false;
  // Specular control: multiplies the per-material POV finish specular weight.
  // Defaults to 1.0 (the finish highlight is rendered at full strength); pass
  // --specular-scale 0 for a matte look with no highlight.
  float specularScale = 1.0f;
  bool specularScaleSet = false;

  // TBB parallelism cap for a no-rebuild speed comparison: 0 = all cores
  // (default), 1 = serial (TBB effectively disabled), N = cap at N. Applied via
  // tbb::global_control around render() in the CLI.
  int threads = 0;

  // Track whether these were set explicitly so the .pov path can pick
  // render defaults that match the CueMol reference without clobbering the
  // user's choices.
  bool widthSet = false;
  bool heightSet = false;

  // Per-section transparency (single-layer compositing). Opacity override for a
  // CueMol section id (from "#if (_show_<id>)"). Key is the id with the "_show"
  // prefix stripped (e.g. "_34_35"); value is opacity 0..1. Repeatable: --alpha.
  std::map<std::string, float> sectionAlpha;
  // Print the input's section ids (transparency groups) and exit.
  bool listGroups = false;

  // --- screen-space NPR edges ---
  // Master switch for the screen-space edge pass (off => byte-identical default,
  // no extra AOVs allocated). --edges on|off.
  bool edges = false;
  // TEMPORARY global edge-class selector (--edge-classes <csv of
  // sil,disc,obj,mat,crease>, plus all/none): enables those classes in
  // EdgeOptions::defaultStyle so each incremental class is testable before
  // per-section --edge styling lands. Indexed by the EdgeClass enum order
  // {Silhouette, Disconnected, Object, Material, Crease}. When --edges is on but
  // --edge-classes is not given, main defaults to SILHOUETTE ONLY (edgeClassesSet
  // stays false to signal that fallback); disc/crease/obj/mat are opt-in because
  // they over-ink dense molecular SES out of the box.
  bool edgeClass[5] = {false, false, false, false, false};
  bool edgeClassesSet = false;
  // Per-section edge style override (--edge ID=spec, repeatable), mirroring
  // --alpha one-for-one. Key is the section id with the "_show" prefix stripped
  // (e.g. "_34_35"); value is the parsed EdgeStyle (which classes are enabled and
  // their per-class color/width/opacity). Resolved against geo.groupNames into
  // Scene::groupEdgeStyle in main, warn-on-miss like --alpha. A section without
  // an override keeps EdgeOptions::defaultStyle (the global --edge-classes set).
  std::map<std::string, EdgeStyle> sectionEdge;
  // Global edge detection scalars (override EdgeOptions defaults when set). These
  // feed ropt.edges.{distanceThreshold,curvatureGate,creaseAngleDeg}; per design
  // open-risk #1, curvatureGate/distanceThreshold often need per-scene tuning on
  // dense molecular scenes, so they are CLI-exposed here.
  float edgeDistanceThreshold = 1.0f;
  float edgeCurvatureGate = 0.5f;
  float edgeCreaseAngleDeg = 30.0f;
  float edgeCreaseGrazingBias = 1.0f;
  float edgeDiscNormalAngleDeg = 35.0f;
  bool edgeDistanceSet = false;
  bool edgeCurvatureSet = false;
  bool edgeCreaseAngleSet = false;
  bool edgeCreaseGrazingSet = false;
  bool edgeDiscNormalAngleSet = false;
  // Debug AOV dump prefix (--dump-aov <prefix>): when set AND edges are on, write
  // false-color objectId/materialId, normal*0.5+0.5 and normalized viewZ images
  // named "<prefix>_*.png". Empty => no dump.
  std::string dumpAovPrefix;
  // Escape hatch (--keep-baked-edges on|off, default off): when on, the baked POV
  // edge_line cylinders are NOT filtered even with --edges on, so the baked and
  // screen-space outlines can be A/B-compared side by side. No effect when
  // --edges is off (nothing is filtered in either case).
  bool keepBakedEdges = false;
  // --- analytic OBJECT-SPACE silhouette edges (spheres/cylinders) ---
  // Master switch (--obj-edges on|off, default off => byte-identical default).
  // When on, each analytic primitive's n.v==0 silhouette contour is emitted in
  // 3D as thin flat-black "open" cylinders appended to the scene before render,
  // so the ray tracer handles visibility/occlusion/AA/fog automatically. This is
  // independent of the screen-space --edges pass above.
  bool objEdges = false;
  float objEdgeWidth = 0.03f;    // edge cylinder radius (world units)
  float objEdgeRaise = 0.0f;     // outward contour offset (world units)
  int objEdgeSegments = 48;      // ring tessellation per sphere / cap circle
  float objEdgeColor[3] = {0.0f, 0.0f, 0.0f};  // linear RGB (from #RRGGBB)
  bool objEdgeClip = true;       // union-boundary clip (drop notch-causing parts
                                 // inside connecting primitives); --obj-edge-clip

  // Emit a transparent background (output alpha = accumulated coverage).
  bool transparentBackground = false;
  // Master switch for the single-layer transparency walk (off = opaque only).
  bool transparency = true;

  // Image-compare mode: print PSNR/SSIM between two PPM files and exit.
  bool compareMode = false;
  std::string compareA;
  std::string compareB;

  // Convert mode: read a PPM and write it as PNG/PPM, then exit.
  bool convertMode = false;
  std::string convertIn;
  std::string convertOut;

  bool showHelp = false;
  bool ok = true;
  std::string error;
};

Options parseCli(int argc, char** argv);
void printUsage(const char* prog);

}  // namespace umbreon
