// Command-line option parsing for the prototype.
#pragma once

#include <map>
#include <string>

#include "render/render_types.hpp"

namespace umbreon {

struct Options {
  std::string input;            // input .pov scene file (positional)
  std::string output = "out.png";
  int width = 1024;
  int height = 768;
  float aoDistance = -1.0f;     // AO occluder radius; < 0 => auto from scene size
  int aoSamples = 0;            // AO rays per mesh hit; 0 = AO off
  float aoIntensity = 1.0f;     // AO strength multiplier (0..1+)
  bool shadows = false;         // cast shadows from lights
  int shadowSamples = 1;        // shadow rays per light (>1 = soft area light)
  float lightRadius = 0.0f;     // light angular radius (deg); >0 = soft shadows

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

  // --- Freestyle-style STROKE edges (--edges) ---
  // Master switch for the stroke edge pass (off => byte-identical default, no
  // extra AOVs allocated). --edges on|off.
  bool edges = false;
  // Per-section edge style override (--edge ID=spec, repeatable), mirroring
  // --alpha one-for-one. Key is the section id with the "_show" prefix stripped
  // (e.g. "_34_35"); value is the parsed EdgeStyle (which natures are enabled and
  // their per-nature color/width/opacity). Resolved against geo.groupNames into
  // Scene::groupEdgeStyle in main, warn-on-miss like --alpha. A section without
  // an override keeps the global stroke style (StrokeEdgeOptions::defaultStyle,
  // seeded from the --stroke-* toggles below).
  std::map<std::string, EdgeStyle> sectionEdge;
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
  // Triangle-mesh (ribbon/SES/cartoon) edge classes for the object-space pass.
  // The mesh silhouette is the SMOOTH n.v==0 contour (Freestyle-style per-vertex
  // zero-crossing); crease/border lie on mesh edges. --obj-edge-mesh-* toggle
  // each class; --obj-edge-crease-deg sets the crease dihedral threshold.
  //
  // CREASE IS OFF BY DEFAULT to match CueMol: its silhouette extractor
  // (RendIntData::calcSilEdgeLines) gates the dihedral/crease test on
  // creaseLimit > 0, and the default creaseLimit is -1 (PovDisplayContext /
  // PovSceneExporter), so CueMol draws ONLY the n.v-sign-change silhouette, never
  // an adjacent-face-normal crease line. Crease ON over-inks rectangular ribbon
  // cross-sections (the beta-sheet box has 90-degree convex folds that pass every
  // geometric gate). Turn it back on with --obj-edge-mesh-crease on when a faceted
  // crease look is wanted; the gate defaults below then apply.
  bool objEdgeMeshSil = true;
  bool objEdgeMeshCrease = false;
  bool objEdgeMeshBorder = true;
  float objEdgeCreaseDeg = 75.0f;
  // Hard-edge angle (deg): a sharp ribbon cross-section (rectangular beta-sheet)
  // duplicates its box-corner vertices with normals this far apart. The mesh
  // silhouette keeps such split normals SEPARATE (no averaging, so the smooth
  // contour stops dashing on sharp ribbons) and draws the box edge by the CueMol
  // face-normal straddle test. Smooth tubes (all normals within this angle) are
  // unaffected. See ObjectSpaceEdgeOptions::meshHardEdgeDeg.
  float objEdgeHardDeg = 40.0f;
  // Geometric crease/border gates (no color). Mirror ObjectSpaceEdgeOptions defaults so
  // the out-of-the-box look reproduces the clean CueMol OpenGL outline:
  //   - smooth-facet veto (deg): drop creases that are smooth-shaded tessellation
  //     facets (helix-barrel / ribbon-face hatching); 0 disables.
  //   - convex-only: keep convex ridges, drop concave junction-step valleys.
  //   - border coplanar veto (deg): drop internal strip-seam borders (coil-tube
  //     dashes), keep true termini; 0 disables.
  float objEdgeCreaseSmoothDeg = 25.0f;
  bool objEdgeCreaseConvexOnly = true;
  float objEdgeBorderCoplanarDeg = 35.0f;
  int objEdgeCreaseMaxDeg = 4;   // drop crease-cluster (cap/terminus) hubs; 0=off

  // --- Freestyle-style STROKE edges (--edges, NEW) ---
  // --edges <on|off> now drives the stroke pipeline (StrokeEdgeOptions), the
  // replacement for the retired per-pixel screen-space pass; the `edges` flag
  // above is reused as its master gate. These stub scalar knobs and nature
  // toggles are parsed/stored now (wired to ropt.strokeEdges) but only honored as
  // later steps implement the pipeline. They mirror the --obj-edge-* block.
  float strokeThickness = 2.0f;   // stroke full width, FINAL px (--stroke-thickness; ss-scaled internally)
  float strokeResample = 2.0f;    // arc-length resample step, FINAL px (--stroke-resample; ss-scaled internally)
  float strokeCreaseDeg = 30.0f;  // crease dihedral threshold (--edge-crease-deg)
  bool strokeThicknessSet = false;
  bool strokeResampleSet = false;
  bool strokeCreaseDegSet = false;
  // Per-nature toggles (--stroke-silhouette / --stroke-crease / --stroke-border).
  // Crease defaults OFF (CueMol2 creaseLimit -1): a dihedral crease over-inks the
  // degenerate-vertex sharp edges of a rectangular ribbon sheet body.
  bool strokeSilhouette = true;
  bool strokeCrease = false;
  bool strokeBorder = true;
  // Demo stylization shader (--stroke-taper): taper width toward stroke ends.
  bool strokeTaper = false;
  // Demo geometry shader (--stroke-smooth): corner-preserving backbone smoothing.
  bool strokeSmooth = false;

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
