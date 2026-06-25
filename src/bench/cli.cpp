#include "cli.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace umbreon {
namespace {

bool parseBool(const std::string& s, bool& out) {
  if (s == "on" || s == "1" || s == "true" || s == "yes") { out = true; return true; }
  if (s == "off" || s == "0" || s == "false" || s == "no") { out = false; return true; }
  return false;
}

// Map an edge-class token to its EdgeClass index, or -1 if it is not a single
// class name. The five Warabi classes share the EdgeClass enum order
// {Silhouette, Disconnected, Object, Material, Crease}.
int edgeClassIndex(const std::string& tok) {
  if (tok == "sil") return static_cast<int>(EdgeClass::Silhouette);
  if (tok == "disc") return static_cast<int>(EdgeClass::Disconnected);
  if (tok == "obj") return static_cast<int>(EdgeClass::Object);
  if (tok == "mat") return static_cast<int>(EdgeClass::Material);
  if (tok == "crease") return static_cast<int>(EdgeClass::Crease);
  return -1;
}

// Parse "#RRGGBB" into a linear RGB triple (component/255). The edge color is a
// linear-space value (composited in linear space, matching EdgeClassStyle), so
// the hex digits are taken as the linear channel values directly -- no sRGB
// decode, which keeps the common black <#000000> exact. Returns false on a
// malformed token (wrong length / non-hex digit).
bool parseHexColor(const std::string& s, float rgb[3]) {
  if (s.size() != 7 || s[0] != '#') return false;
  auto hx = [](char c, int& v) -> bool {
    if (c >= '0' && c <= '9') { v = c - '0'; return true; }
    if (c >= 'a' && c <= 'f') { v = c - 'a' + 10; return true; }
    if (c >= 'A' && c <= 'F') { v = c - 'A' + 10; return true; }
    return false;
  };
  for (int ch = 0; ch < 3; ++ch) {
    int hi, lo;
    if (!hx(s[1 + ch * 2], hi) || !hx(s[2 + ch * 2], lo)) return false;
    rgb[ch] = static_cast<float>(hi * 16 + lo) / 255.0f;
  }
  return true;
}

// Split a string on a delimiter into non-throwing pieces (empty pieces kept so
// callers can tolerate / reject them as they wish).
std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> out;
  std::size_t pos = 0;
  while (true) {
    std::size_t d = s.find(delim, pos);
    out.push_back(s.substr(pos, d == std::string::npos ? std::string::npos
                                                       : d - pos));
    if (d == std::string::npos) break;
    pos = d + 1;
  }
  return out;
}

// Parse a per-section --edge spec into an EdgeStyle. Grammar (comma-separated
// class entries; each entry is a class token followed by optional :key=value
// style attributes):
//
//   spec   := entry ("," entry)*
//   entry  := class (":" attr)*
//   class  := sil | disc | obj | mat | crease | all | none
//   attr   := color=#RRGGBB | width=W | opacity=O
//
// "all" enables every class, "none" disables every class; either may carry style
// attributes that then apply to every class it (re)enabled in that entry. A bare
// class token enables that one class (style attributes scoped to it). Later
// entries override earlier ones for the same class. Every enabled class starts
// from a black/opacity-1/width-1 default before its attributes apply. Returns
// false on any unknown class token or malformed attribute (caller reports it).
bool parseEdgeSpec(const std::string& spec, EdgeStyle& out) {
  out = EdgeStyle{};  // all classes disabled, default style
  for (const std::string& entry : split(spec, ',')) {
    if (entry.empty()) continue;  // tolerate stray / trailing commas
    std::vector<std::string> parts = split(entry, ':');
    const std::string& cls = parts[0];
    if (cls.empty()) return false;

    // Resolve which class slots this entry targets.
    std::vector<int> targets;
    if (cls == "all") {
      for (int i = 0; i < static_cast<int>(EdgeClass::Count); ++i)
        targets.push_back(i);
    } else if (cls == "none") {
      for (int i = 0; i < static_cast<int>(EdgeClass::Count); ++i)
        out.cls[i].enabled = false;
      // "none" still accepts (and ignores) trailing attrs for uniformity.
    } else {
      int idx = edgeClassIndex(cls);
      if (idx < 0) return false;
      targets.push_back(idx);
    }

    // Enable the targeted classes (resetting each to its default style first so a
    // re-listing of a class is a clean override, not an accumulation).
    for (int t : targets) out.cls[t] = EdgeClassStyle{};
    for (int t : targets) out.cls[t].enabled = true;

    // Apply style attributes to all targeted classes.
    for (std::size_t a = 1; a < parts.size(); ++a) {
      const std::string& kv = parts[a];
      std::size_t eq = kv.find('=');
      if (eq == std::string::npos) return false;
      const std::string key = kv.substr(0, eq);
      const std::string val = kv.substr(eq + 1);
      if (key == "color") {
        float rgb[3];
        if (!parseHexColor(val, rgb)) return false;
        for (int t : targets)
          for (int c = 0; c < 3; ++c) out.cls[t].color[c] = rgb[c];
      } else if (key == "width") {
        const float w = static_cast<float>(std::atof(val.c_str()));
        for (int t : targets) out.cls[t].width = w;
      } else if (key == "opacity") {
        const float o = static_cast<float>(std::atof(val.c_str()));
        for (int t : targets) out.cls[t].opacity = o;
      } else {
        return false;
      }
    }
  }
  return true;
}

// Parse the TEMPORARY --edge-classes csv (sil,disc,obj,mat,crease, plus the
// shorthands all/none) into the EdgeClass-ordered enable flags
// {Silhouette, Disconnected, Object, Material, Crease}. Returns false on an
// unknown token (caller reports the error).
bool parseEdgeClasses(const std::string& csv, bool out[5]) {
  for (int i = 0; i < 5; ++i) out[i] = false;
  std::size_t pos = 0;
  while (pos <= csv.size()) {
    std::size_t comma = csv.find(',', pos);
    std::string tok =
        csv.substr(pos, comma == std::string::npos ? std::string::npos
                                                   : comma - pos);
    pos = (comma == std::string::npos) ? csv.size() + 1 : comma + 1;
    if (tok.empty()) continue;  // tolerate stray / trailing commas
    if (tok == "none") {
      for (int i = 0; i < 5; ++i) out[i] = false;
    } else if (tok == "all") {
      for (int i = 0; i < 5; ++i) out[i] = true;
    } else if (tok == "sil") {
      out[0] = true;
    } else if (tok == "disc") {
      out[1] = true;
    } else if (tok == "obj") {
      out[2] = true;
    } else if (tok == "mat") {
      out[3] = true;
    } else if (tok == "crease") {
      out[4] = true;
    } else {
      return false;
    }
  }
  return true;
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
    } else if (a == "--ao-distance") {
      o.aoDistance = static_cast<float>(std::atof(value("--ao-distance").c_str()));
    } else if (a == "--ao-samples") {
      o.aoSamples = std::atoi(value("--ao-samples").c_str());
    } else if (a == "--ao-intensity") {
      o.aoIntensity =
          static_cast<float>(std::atof(value("--ao-intensity").c_str()));
    } else if (a == "--shadows") {
      std::string v = value("--shadows");
      if (o.ok && !parseBool(v, o.shadows)) fail("--shadows expects on/off");
    } else if (a == "--shadow-samples") {
      o.shadowSamples = std::atoi(value("--shadow-samples").c_str());
    } else if (a == "--light-radius") {
      o.lightRadius =
          static_cast<float>(std::atof(value("--light-radius").c_str()));
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
    } else if (a == "--threads") {
      o.threads = std::atoi(value("--threads").c_str());
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
    } else if (a == "--edges") {
      std::string v = value("--edges");
      if (o.ok && !parseBool(v, o.edges)) fail("--edges expects on/off");
    } else if (a == "--edge-classes") {
      std::string v = value("--edge-classes");
      if (o.ok && !parseEdgeClasses(v, o.edgeClass))
        fail("--edge-classes expects a csv of sil,disc,obj,mat,crease "
             "(or all/none)");
      else
        o.edgeClassesSet = true;
    } else if (a == "--edge") {
      // --edge ID=spec : per-section edge style override (repeatable), mirroring
      // --alpha. Split on the first '=', strip a leading "_show", parse the spec
      // into an EdgeStyle stored under the stripped id.
      std::string kv = value("--edge");
      std::size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        fail("--edge expects ID=spec (e.g. _34_35=sil,crease:color=#000000)");
      } else {
        std::string id = kv.substr(0, eq);
        if (id.rfind("_show", 0) == 0) id = id.substr(5);
        EdgeStyle st;
        if (o.ok && !parseEdgeSpec(kv.substr(eq + 1), st))
          fail("--edge: bad spec '" + kv.substr(eq + 1) +
               "' (classes sil/disc/obj/mat/crease|all|none, attrs "
               "color=#RRGGBB:width=W:opacity=O)");
        else
          o.sectionEdge[id] = st;
      }
    } else if (a == "--edge-distance") {
      o.edgeDistanceThreshold =
          static_cast<float>(std::atof(value("--edge-distance").c_str()));
      o.edgeDistanceSet = true;
    } else if (a == "--edge-curvature") {
      o.edgeCurvatureGate =
          static_cast<float>(std::atof(value("--edge-curvature").c_str()));
      o.edgeCurvatureSet = true;
    } else if (a == "--edge-crease-angle") {
      o.edgeCreaseAngleDeg =
          static_cast<float>(std::atof(value("--edge-crease-angle").c_str()));
      o.edgeCreaseAngleSet = true;
    } else if (a == "--edge-crease-grazing") {
      o.edgeCreaseGrazingBias =
          static_cast<float>(std::atof(value("--edge-crease-grazing").c_str()));
      o.edgeCreaseGrazingSet = true;
    } else if (a == "--edge-disc-normal-angle") {
      o.edgeDiscNormalAngleDeg = static_cast<float>(
          std::atof(value("--edge-disc-normal-angle").c_str()));
      o.edgeDiscNormalAngleSet = true;
    } else if (a == "--obj-edges") {
      std::string v = value("--obj-edges");
      if (o.ok && !parseBool(v, o.objEdges)) fail("--obj-edges expects on/off");
    } else if (a == "--obj-edge-width") {
      o.objEdgeWidth =
          static_cast<float>(std::atof(value("--obj-edge-width").c_str()));
    } else if (a == "--obj-edge-raise") {
      o.objEdgeRaise =
          static_cast<float>(std::atof(value("--obj-edge-raise").c_str()));
    } else if (a == "--obj-edge-segments") {
      o.objEdgeSegments = std::atoi(value("--obj-edge-segments").c_str());
    } else if (a == "--obj-edge-color") {
      std::string v = value("--obj-edge-color");
      if (o.ok && !parseHexColor(v, o.objEdgeColor))
        fail("--obj-edge-color expects #RRGGBB");
    } else if (a == "--obj-edge-clip") {
      std::string v = value("--obj-edge-clip");
      if (o.ok && !parseBool(v, o.objEdgeClip))
        fail("--obj-edge-clip expects on/off");
    } else if (a == "--obj-edge-crease-deg") {
      o.objEdgeCreaseDeg =
          static_cast<float>(std::atof(value("--obj-edge-crease-deg").c_str()));
    } else if (a == "--obj-edge-crease-smooth-deg") {
      o.objEdgeCreaseSmoothDeg = static_cast<float>(
          std::atof(value("--obj-edge-crease-smooth-deg").c_str()));
    } else if (a == "--obj-edge-crease-convex-only") {
      std::string v = value("--obj-edge-crease-convex-only");
      if (o.ok && !parseBool(v, o.objEdgeCreaseConvexOnly))
        fail("--obj-edge-crease-convex-only expects on/off");
    } else if (a == "--obj-edge-border-coplanar-deg") {
      o.objEdgeBorderCoplanarDeg = static_cast<float>(
          std::atof(value("--obj-edge-border-coplanar-deg").c_str()));
    } else if (a == "--obj-edge-crease-max-deg") {
      o.objEdgeCreaseMaxDeg = std::atoi(value("--obj-edge-crease-max-deg").c_str());
    } else if (a == "--obj-edge-mesh-sil") {
      std::string v = value("--obj-edge-mesh-sil");
      if (o.ok && !parseBool(v, o.objEdgeMeshSil))
        fail("--obj-edge-mesh-sil expects on/off");
    } else if (a == "--obj-edge-mesh-crease") {
      std::string v = value("--obj-edge-mesh-crease");
      if (o.ok && !parseBool(v, o.objEdgeMeshCrease))
        fail("--obj-edge-mesh-crease expects on/off");
    } else if (a == "--obj-edge-mesh-border") {
      std::string v = value("--obj-edge-mesh-border");
      if (o.ok && !parseBool(v, o.objEdgeMeshBorder))
        fail("--obj-edge-mesh-border expects on/off");
    } else if (a == "--dump-aov") {
      o.dumpAovPrefix = value("--dump-aov");
    } else if (a == "--keep-baked-edges") {
      std::string v = value("--keep-baked-edges");
      if (o.ok && !parseBool(v, o.keepBakedEdges))
        fail("--keep-baked-edges expects on/off");
    } else if (a == "--transparent-bg") {
      std::string v = value("--transparent-bg");
      if (o.ok && !parseBool(v, o.transparentBackground))
        fail("--transparent-bg expects on/off");
    } else if (a == "--transparency") {
      std::string v = value("--transparency");
      if (o.ok && !parseBool(v, o.transparency))
        fail("--transparency expects on/off");
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
      "  --supersample <int>      render NxN and downsample  [1; .pov: 3]\n"
      "  --specular-scale <float> cartoon specular x      [.pov: 0 = matte]\n"
      "  --threads <int>          TBB parallelism cap (1 = serial)  [0 = all]\n"
      "  --alpha <ID=value>       set a section's opacity (e.g. _34_35=0.5)\n"
      "  --list-groups            list the input's section ids and exit\n"
      "  --edges <on|off>         screen-space NPR edge pass (silhouette)  [off]\n"
      "  --edge-classes <csv>     edge classes: sil,disc,obj,mat,crease|all|none\n"
      "                           (default with --edges on: sil only)\n"
      "  --edge <ID=spec>         per-section edge style (repeatable), e.g.\n"
      "                           _34_35=sil,crease:color=#000000:width=1.5\n"
      "  --edge-distance <float>  depth-gap threshold (view-z units)     [1.00]\n"
      "  --edge-curvature <float> curvature-veto gate (2nd/1st ratio)     [0.50]\n"
      "  --edge-crease-angle <f>  crease fold angle, degrees            [30.00]\n"
      "  --edge-crease-grazing <f> crease grazing-angle bias gain        [1.00]\n"
      "  --edge-disc-normal-angle <f> disc normal-disagree angle, deg   [35.00]\n"
      "  --obj-edges <on|off>     analytic object-space edges (sph/cyl/mesh) [off]\n"
      "  --obj-edge-width <float> object-edge cylinder radius (world)   [0.03]\n"
      "  --obj-edge-raise <float> object-edge outward offset (world)    [0.00]\n"
      "  --obj-edge-segments <int> sphere/cap ring tessellation           [48]\n"
      "  --obj-edge-color <#RRGGBB> object-edge color              [#000000]\n"
      "  --obj-edge-clip <on|off> trim object edges at primitive joins    [on]\n"
      "  --obj-edge-crease-deg <float> mesh crease dihedral threshold     [75]\n"
      "  --obj-edge-crease-smooth-deg <f> veto smooth-facet creases (0=off) [25]\n"
      "  --obj-edge-crease-convex-only <on|off> drop concave-valley creases [on]\n"
      "  --obj-edge-border-coplanar-deg <f> veto strip-seam borders (0=off) [35]\n"
      "  --obj-edge-crease-max-deg <int> drop crease-cluster cap hubs (0=off)  [4]\n"
      "  --obj-edge-mesh-sil <on|off> mesh smooth n.v==0 silhouette        [on]\n"
      "  --obj-edge-mesh-crease <on|off> mesh crease (sharp-fold) edges    [on]\n"
      "  --obj-edge-mesh-border <on|off> mesh open-boundary edges          [on]\n"
      "  --dump-aov <prefix>      with --edges on, dump G-buffer AOV images\n"
      "  --keep-baked-edges <on|off> keep baked POV edges with --edges on (A/B) [off]\n"
      "  --transparent-bg <on|off> transparent background output      [off]\n"
      "  --transparency <on|off>  single-layer transparency walk        [on]\n"
      "  --grid <int>             N for an N^3 instance grid    [1]\n"
      "  --spacing <float>        grid pitch (x mesh size)      [1.15]\n"
      "  --ao-samples <int>       ambient occlusion rays/hit  [0 = off]\n"
      "  --ao-distance <float>    AO occluder radius   [auto from scene]\n"
      "  --ao-intensity <float>   AO strength multiplier        [1.0]\n"
      "  --shadows <on|off>       cast shadows from lights           [off]\n"
      "  --shadow-samples <int>   shadow rays/light (>1 = soft)       [1]\n"
      "  --light-radius <float>   light angular radius deg (soft)     [0]\n"
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
