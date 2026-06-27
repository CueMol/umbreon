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
    } else if (a == "--obj-edge-hard-deg") {
      o.objEdgeHardDeg =
          static_cast<float>(std::atof(value("--obj-edge-hard-deg").c_str()));
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
    } else if (a == "--stroke-thickness") {
      o.strokeThickness =
          static_cast<float>(std::atof(value("--stroke-thickness").c_str()));
      o.strokeThicknessSet = true;
    } else if (a == "--stroke-resample") {
      o.strokeResample =
          static_cast<float>(std::atof(value("--stroke-resample").c_str()));
      o.strokeResampleSet = true;
    } else if (a == "--edge-crease-deg") {
      o.strokeCreaseDeg =
          static_cast<float>(std::atof(value("--edge-crease-deg").c_str()));
      o.strokeCreaseDegSet = true;
    } else if (a == "--stroke-silhouette") {
      std::string v = value("--stroke-silhouette");
      if (o.ok && !parseBool(v, o.strokeSilhouette))
        fail("--stroke-silhouette expects on/off");
    } else if (a == "--stroke-crease") {
      std::string v = value("--stroke-crease");
      if (o.ok && !parseBool(v, o.strokeCrease))
        fail("--stroke-crease expects on/off");
    } else if (a == "--stroke-border") {
      std::string v = value("--stroke-border");
      if (o.ok && !parseBool(v, o.strokeBorder))
        fail("--stroke-border expects on/off");
    } else if (a == "--stroke-taper") {
      std::string v = value("--stroke-taper");
      if (o.ok && !parseBool(v, o.strokeTaper))
        fail("--stroke-taper expects on/off");
    } else if (a == "--stroke-smooth") {
      std::string v = value("--stroke-smooth");
      if (o.ok && !parseBool(v, o.strokeSmooth))
        fail("--stroke-smooth expects on/off");
    } else if (a == "--stroke-analytic") {
      std::string v = value("--stroke-analytic");
      if (o.ok && !parseBool(v, o.strokeAnalytic))
        fail("--stroke-analytic expects on/off");
    } else if (a == "--stroke-analytic-segments") {
      o.strokeAnalyticSegments =
          std::atoi(value("--stroke-analytic-segments").c_str());
      o.strokeAnalyticSegmentsSet = true;
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
    } else if (a == "--compare") {
      o.compareMode = true;
      o.compareA = value("--compare");
      o.compareB = value("--compare");
    } else if (a == "--convert") {
      o.convertMode = true;
      o.convertIn = value("--convert");
      o.convertOut = value("--convert");
    } else if (!a.empty() && a[0] == '-') {
      fail("unknown option: " + a);
    } else if (o.input.empty()) {
      o.input = a;
    } else {
      fail("unexpected argument: " + a);
    }
  }

  if (o.ok && !o.showHelp && !o.compareMode && !o.convertMode) {
    if (o.input.empty()) fail("no input .pov file given");
    else if (o.width <= 0 || o.height <= 0) fail("width/height must be positive");
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
      "  --edges <on|off>         Freestyle stroke edge pass (sil/crease/border) [off]\n"
      "  --edge <ID=spec>         per-section edge style (repeatable), e.g.\n"
      "                           _34_35=sil,crease:color=#000000:width=1.5\n"
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
      "  --obj-edge-hard-deg <float> hard-edge angle: split sharp-ribbon normals,\n"
      "                              draw box edges by face-normal straddle   [40]\n"
      "  --obj-edge-mesh-sil <on|off> mesh smooth n.v==0 silhouette        [on]\n"
      "  --obj-edge-mesh-crease <on|off> mesh crease (sharp-fold) edges   [off]\n"
      "  --obj-edge-mesh-border <on|off> mesh open-boundary edges          [on]\n"
      "  --stroke-thickness <float> stroke full width, final px             [2]\n"
      "  --stroke-resample <float> stroke arc-length resample step, px      [2]\n"
      "  --edge-crease-deg <float> stroke crease dihedral threshold (deg)  [30]\n"
      "  --stroke-silhouette <on|off> stroke silhouette nature             [on]\n"
      "  --stroke-crease <on|off> stroke crease nature                    [off]\n"
      "  --stroke-border <on|off> stroke border nature                     [on]\n"
      "  --stroke-taper <on|off>  taper stroke width toward its ends (demo) [off]\n"
      "  --stroke-smooth <on|off> corner-preserving backbone smoothing (demo)[off]\n"
      "  --stroke-analytic <on|off> draw sphere/cylinder outlines (ball-stick) [on]\n"
      "  --stroke-analytic-segments <int> sphere ring / cap tessellation   [48]\n"
      "  --dump-aov <prefix>      with --edges on, dump G-buffer AOV images\n"
      "  --keep-baked-edges <on|off> keep baked POV edges with --edges on (A/B) [off]\n"
      "  --transparent-bg <on|off> transparent background output      [off]\n"
      "  --transparency <on|off>  single-layer transparency walk        [on]\n"
      "  --ao-samples <int>       ambient occlusion rays/hit  [0 = off]\n"
      "  --ao-distance <float>    AO occluder radius   [auto from scene]\n"
      "  --ao-intensity <float>   AO strength multiplier        [1.0]\n"
      "  --shadows <on|off>       cast shadows from lights           [off]\n"
      "  --shadow-samples <int>   shadow rays/light (>1 = soft)       [1]\n"
      "  --light-radius <float>   light angular radius deg (soft)     [0]\n"
      "  --compare <a> <b>        print PSNR/SSIM between two PPM files\n"
      "  --convert <in> <out>     convert a PPM to PNG/PPM\n"
      "  -h, --help               show this help\n",
      prog);
}

}  // namespace umbreon
