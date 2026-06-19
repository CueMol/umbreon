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
  int aoSamples = 16;
  float aoDistance = -1.0f;     // < 0 => auto from scene size
  int spp = 1;
  int accumFrames = 16;
  bool denoise = true;
  bool prefilterAux = false;
  std::string renderer = "pathtracer";  // "pathtracer" (GI) or "scivis" (AO)
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
  // Cartoon specular control. SciVis's OBJ specular is a normalized
  // Blinn-Phong whose peak grows with shininess, so reproducing POV's sharp
  // highlight blows out on view-facing saturated faces. The .pov path therefore
  // defaults to matte (scale 0); pass --specular-scale 1 to restore it. The
  // shininess override (<0 keeps the roughness-derived value) is for tuning.
  float specularScale = 1.0f;
  bool specularScaleSet = false;
  float shininess = -1.0f;
  std::string material = "obj";  // cartoon material: "obj" or "principled"
  float ior = 1.5f;              // principled dielectric index of refraction
  float flatKd = 1.0f;          // outline/wireframe kd (high => flat ambient-1 look)
  int maxPathLength = 16;       // pathtracer: GI bounce depth (1 = direct only)

  // Track whether these were set explicitly so the .pov path can pick
  // render defaults that match the CueMol reference without clobbering the
  // user's choices.
  bool widthSet = false;
  bool heightSet = false;
  bool rendererSet = false;
  bool aoSamplesSet = false;
  bool denoiseSet = false;

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
