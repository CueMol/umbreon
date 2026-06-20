// Command-line option parsing for the prototype.
#pragma once

#include <map>
#include <string>

namespace umbreon {

struct Options {
  std::string input;            // input .pov scene or .inc geometry (positional)
  std::string output = "out.png";
  int width = 1024;
  int height = 768;
  int gridN = 1;                // N^3 instance grid
  float spacing = 1.15f;        // grid pitch as a multiple of mesh size
  bool flatten = false;
  float aoDistance = -1.0f;     // AO occluder radius; < 0 => auto from scene size
  int aoSamples = 0;            // AO rays per mesh hit; 0 = AO off
  float aoIntensity = 1.0f;     // AO strength multiplier (0..1+)
  int spp = 1;
  int accumFrames = 16;
  bool prefilterAux = false;
  bool flipNormals = false;
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
  // defaults to 2 when not set explicitly.
  int supersample = 1;
  bool supersampleSet = false;
  // Specular control: multiplies the per-material POV finish specular weight.
  // Defaults to 1.0 (the finish highlight is rendered at full strength); pass
  // --specular-scale 0 for a matte look with no highlight.
  float specularScale = 1.0f;
  bool specularScaleSet = false;

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
