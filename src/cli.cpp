#include "cli.hpp"

#include <cstdio>
#include <cstdlib>

namespace umbreon {
namespace {

bool parseBool(const std::string& s, bool& out) {
  if (s == "on" || s == "1" || s == "true" || s == "yes") { out = true; return true; }
  if (s == "off" || s == "0" || s == "false" || s == "no") { out = false; return true; }
  return false;
}

}  // namespace

Options parseCli(int argc, char** argv) {
  Options o;
  // CueMol POV reference constants (the POV-Ray "Declare=..." command line).
  o.declares = {
      {"_stereo", 0.0},      {"_iod", 0.03},      {"_perspective", 0.0},
      {"_shadow", 0.0},      {"_light_inten", 1.3}, {"_flash_frac", 0.6},
      {"_amb_frac", 0.0},
  };
  auto fail = [&](const std::string& msg) {
    o.ok = false;
    o.error = msg;
  };

  for (int i = 1; i < argc && o.ok; ++i) {
    std::string a = argv[i];
    auto value = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        fail(std::string("missing value for ") + name);
        return std::string();
      }
      return argv[++i];
    };

    if (a == "-h" || a == "--help") {
      o.showHelp = true;
    } else if (a == "-o") {
      o.output = value("-o");
    } else if (a == "-W" || a == "--width") {
      o.width = std::atoi(value("-W").c_str());
      o.widthSet = true;
    } else if (a == "-H" || a == "--height") {
      o.height = std::atoi(value("-H").c_str());
      o.heightSet = true;
    } else if (a == "--grid") {
      o.gridN = std::atoi(value("--grid").c_str());
    } else if (a == "--spacing") {
      o.spacing = static_cast<float>(std::atof(value("--spacing").c_str()));
    } else if (a == "--flatten") {
      o.flatten = true;
    } else if (a == "--ao-distance") {
      o.aoDistance = static_cast<float>(std::atof(value("--ao-distance").c_str()));
      std::fprintf(stderr,
                   "warning: --ao-distance is not implemented yet (AO is on the "
                   "roadmap); the value is ignored\n");
    } else if (a == "--spp") {
      o.spp = std::atoi(value("--spp").c_str());
    } else if (a == "--accum") {
      o.accumFrames = std::atoi(value("--accum").c_str());
    } else if (a == "--prefilter-aux") {
      std::string v = value("--prefilter-aux");
      if (o.ok && !parseBool(v, o.prefilterAux))
        fail("--prefilter-aux expects on/off");
    } else if (a == "--pov-gain") {
      o.povGain = static_cast<float>(std::atof(value("--pov-gain").c_str()));
    } else if (a == "--outline-scale") {
      o.outlineScale =
          static_cast<float>(std::atof(value("--outline-scale").c_str()));
    } else if (a == "--supersample") {
      o.supersample = std::atoi(value("--supersample").c_str());
      o.supersampleSet = true;
    } else if (a == "--specular-scale") {
      o.specularScale =
          static_cast<float>(std::atof(value("--specular-scale").c_str()));
      o.specularScaleSet = true;
    } else if (a == "--declare") {
      // --declare name=value (predefined POV constant for the .pov path)
      std::string kv = value("--declare");
      std::size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        fail("--declare expects name=value");
      } else {
        o.declares[kv.substr(0, eq)] =
            std::atof(kv.substr(eq + 1).c_str());
      }
    } else if (a == "--alpha") {
      // --alpha ID=value : set the opacity of a CueMol section (e.g.
      // _34_35=0.5). The "_show" prefix is accepted and stripped.
      std::string kv = value("--alpha");
      std::size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        fail("--alpha expects ID=value (e.g. _34_35=0.5)");
      } else {
        std::string id = kv.substr(0, eq);
        if (id.rfind("_show", 0) == 0) id = id.substr(5);
        o.sectionAlpha[id] =
            static_cast<float>(std::atof(kv.substr(eq + 1).c_str()));
      }
    } else if (a == "--list-groups") {
      o.listGroups = true;
    } else if (a == "--transparent-bg") {
      std::string v = value("--transparent-bg");
      if (o.ok && !parseBool(v, o.transparentBackground))
        fail("--transparent-bg expects on/off");
    } else if (a == "--transparency") {
      std::string v = value("--transparency");
      if (o.ok && !parseBool(v, o.transparency))
        fail("--transparency expects on/off");
    } else if (a == "--flip-normals") {
      o.flipNormals = true;
    } else if (a == "--emit-pov") {
      o.emitPov = value("--emit-pov");
    } else if (a == "--pov-radiosity") {
      std::string v = value("--pov-radiosity");
      if (o.ok && !parseBool(v, o.povRadiosity))
        fail("--pov-radiosity expects on/off");
    } else if (a == "--compare") {
      o.compareMode = true;
      o.compareA = value("--compare");
      o.compareB = value("--compare");
    } else if (a == "--convert") {
      o.convertMode = true;
      o.convertIn = value("--convert");
      o.convertOut = value("--convert");
    } else if (a == "--light-intensity") {
      o.lightIntensity =
          static_cast<float>(std::atof(value("--light-intensity").c_str()));
    } else if (a == "--ambient") {
      o.ambientIntensity =
          static_cast<float>(std::atof(value("--ambient").c_str()));
    } else if (!a.empty() && a[0] == '-') {
      fail("unknown option: " + a);
    } else if (o.input.empty()) {
      o.input = a;
    } else {
      fail("unexpected argument: " + a);
    }
  }

  if (o.ok && !o.showHelp && !o.compareMode && !o.convertMode) {
    if (o.input.empty()) fail("no input .inc file given");
    else if (o.width <= 0 || o.height <= 0) fail("width/height must be positive");
    else if (o.gridN < 1) fail("--grid must be >= 1");
  }
  return o;
}

void printUsage(const char* prog) {
  std::printf(
      "Usage: %s <input.pov|input.inc> [options]\n"
      "  A .pov input reproduces the CueMol POV-Ray scene (camera, lights,\n"
      "  background) and renders its referenced .inc geometry; defaults match\n"
      "  the reference render (300x300, orthographic). A .inc input\n"
      "  uses the legacy auto-framed scene with the N^3 instance grid.\n"
      "  -o <path>                output image (.png or .ppm)  [out.png]\n"
      "  -W, --width <int>        image width          [1024; .pov: 300]\n"
      "  -H, --height <int>       image height         [768;  .pov: 300]\n"
      "  --declare <name=value>   predefine a POV constant (.pov path)\n"
      "  --pov-gain <float>       exposure gain for POV lights   [1.20]\n"
      "  --outline-scale <float>  radius x for spheres/cylinders [1.00]\n"
      "  --supersample <int>      render NxN and downsample  [1; .pov: 2]\n"
      "  --specular-scale <float> cartoon specular x      [.pov: 0 = matte]\n"
      "  --alpha <ID=value>       set a section's opacity (e.g. _34_35=0.5)\n"
      "  --list-groups            list the input's section ids and exit\n"
      "  --transparent-bg <on|off> transparent background output      [off]\n"
      "  --transparency <on|off>  single-layer transparency walk        [on]\n"
      "  --grid <int>             N for an N^3 instance grid    [1]\n"
      "  --spacing <float>        grid pitch (x mesh size)      [1.15]\n"
      "  --flatten                bake the instance grid into one mesh\n"
      "  --ao-distance <float>    AO ray max distance (not implemented yet)\n"
      "  --spp <int>              pixel samples per frame       [1]\n"
      "  --accum <int>            accumulation frames           [16]\n"
      "  --prefilter-aux <on|off> prefilter albedo/normal AOVs  [off]\n"
      "  --flip-normals           negate mesh normals\n"
      "  --light-intensity <f>    distant 'sun' intensity       [1.5]\n"
      "  --ambient <f>            ambient 'sky' intensity        [0.6]\n"
      "  --emit-pov <path>        also write an equivalent .pov scene\n"
      "  --pov-radiosity <on|off> radiosity in the emitted .pov       [on]\n"
      "  --compare <a> <b>        print PSNR/SSIM between two PPM files\n"
      "  --convert <in> <out>     convert a PPM to PNG/PPM\n"
      "  -h, --help               show this help\n",
      prog);
}

}  // namespace umbreon
