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

// Parse "x,y,z" into a float triple (used for the explicit AO gradient axis).
// Returns false on the wrong component count.
bool parseVec3(const std::string& s, float v[3]) {
  std::vector<std::string> parts = split(s, ',');
  if (parts.size() != 3) return false;
  for (int i = 0; i < 3; ++i)
    v[i] = static_cast<float>(std::atof(parts[i].c_str()));
  return true;
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
      continue;
    }
    if (a == "-o") {
      o.output = value("-o");
      continue;
    }
    if (a == "-W" || a == "--width") {
      o.width = std::atoi(value("-W").c_str());
      o.widthSet = true;
      continue;
    }
    if (a == "-H" || a == "--height") {
      o.height = std::atoi(value("-H").c_str());
      o.heightSet = true;
      continue;
    }
    if (a == "--ao-distance") {
      o.aoDistance = static_cast<float>(std::atof(value("--ao-distance").c_str()));
      continue;
    }
    if (a == "--ao-samples") {
      o.aoSamples = std::atoi(value("--ao-samples").c_str());
      continue;
    }
    if (a == "--ao-res") {
      std::string v = value("--ao-res");
      if (v == "full")
        o.aoResDiv = 0;
      else if (v == "out")
        o.aoResDiv = -1;  // resolved to the supersample factor
      else
        fail("--ao-res expects full/out");
      continue;
    }
    if (a == "--ao-res-debug") {
      std::string v = value("--ao-res-debug");
      if (o.ok && !parseBool(v, o.aoResDebug))
        fail("--ao-res-debug expects on/off");
      continue;
    }
    if (a == "--ao-intensity") {
      o.aoIntensity =
          static_cast<float>(std::atof(value("--ao-intensity").c_str()));
      continue;
    }
    if (a == "--ao-falloff") {
      o.aoFalloffPower =
          static_cast<float>(std::atof(value("--ao-falloff").c_str()));
      continue;
    }
    if (a == "--ao-multiscale") {
      std::string v = value("--ao-multiscale");
      if (o.ok && !parseBool(v, o.aoMultiScale))
        fail("--ao-multiscale expects on/off");
      continue;
    }
    if (a == "--ao-bent-normal") {
      std::string v = value("--ao-bent-normal");
      if (o.ok && !parseBool(v, o.aoBentNormal))
        fail("--ao-bent-normal expects on/off");
      continue;
    }
    if (a == "--ao-sky") {
      std::string v = value("--ao-sky");
      if (o.ok && !parseHexColor(v, o.aoSkyColor))
        fail("--ao-sky expects #RRGGBB");
      continue;
    }
    if (a == "--ao-ground") {
      std::string v = value("--ao-ground");
      if (o.ok && !parseHexColor(v, o.aoGroundColor))
        fail("--ao-ground expects #RRGGBB");
      continue;
    }
    if (a == "--ao-camera-up") {
      std::string v = value("--ao-camera-up");
      if (o.ok && !parseBool(v, o.aoUseCameraUp))
        fail("--ao-camera-up expects on/off");
      continue;
    }
    if (a == "--ao-up") {
      std::string v = value("--ao-up");
      if (o.ok && !parseVec3(v, o.aoUp)) fail("--ao-up expects x,y,z");
      continue;
    }
    if (a == "--ao-multibounce") {
      std::string v = value("--ao-multibounce");
      if (o.ok && !parseBool(v, o.aoMultibounce))
        fail("--ao-multibounce expects on/off");
      continue;
    }
    if (a == "--ao-ld") {
      std::string v = value("--ao-ld");
      if (o.ok && !parseBool(v, o.aoLowDiscrepancy))
        fail("--ao-ld expects on/off");
      continue;
    }
    if (a == "--ao-diffuse") {
      o.aoDiffuseFactor =
          static_cast<float>(std::atof(value("--ao-diffuse").c_str()));
      continue;
    }
    if (a == "--ao-write-aov") {
      std::string v = value("--ao-write-aov");
      if (o.ok && !parseBool(v, o.aoWriteAov))
        fail("--ao-write-aov expects on/off");
      continue;
    }
    if (a == "--gi") {
      std::string v = value("--gi");
      if (o.ok && !parseBool(v, o.gi)) fail("--gi expects on/off");
      continue;
    }
    if (a == "--gi-samples") {
      o.giSamples = std::atoi(value("--gi-samples").c_str());
      continue;
    }
    if (a == "--gi-bounces") {
      o.giBounces = std::atoi(value("--gi-bounces").c_str());
      continue;
    }
    if (a == "--gi-max-dist") {
      o.giMaxDistance =
          static_cast<float>(std::atof(value("--gi-max-dist").c_str()));
      continue;
    }
    if (a == "--gi-intensity") {
      o.giIntensity =
          static_cast<float>(std::atof(value("--gi-intensity").c_str()));
      continue;
    }
    if (a == "--gi-env-intensity") {
      o.giEnvIntensity =
          static_cast<float>(std::atof(value("--gi-env-intensity").c_str()));
      continue;
    }
    if (a == "--gi-accuracy") {
      o.giAccuracy =
          static_cast<float>(std::atof(value("--gi-accuracy").c_str()));
      continue;
    }
    if (a == "--gi-spacing") {
      o.giRecordSpacing =
          static_cast<float>(std::atof(value("--gi-spacing").c_str()));
      continue;
    }
    if (a == "--gi-normal-reject") {
      o.giNormalReject =
          static_cast<float>(std::atof(value("--gi-normal-reject").c_str()));
      continue;
    }
    if (a == "--gi-component-reject") {
      std::string v = value("--gi-component-reject");
      if (o.ok && !parseBool(v, o.giComponentReject))
        fail("--gi-component-reject expects on/off");
      continue;
    }
    if (a == "--gi-seed-per-vertex") {
      std::string v = value("--gi-seed-per-vertex");
      if (o.ok && !parseBool(v, o.giSeedPerVertex))
        fail("--gi-seed-per-vertex expects on/off");
      continue;
    }
    if (a == "--gi-gradients") {
      std::string v = value("--gi-gradients");
      if (o.ok && !parseBool(v, o.giGradients))
        fail("--gi-gradients expects on/off");
      continue;
    }
    if (a == "--gi-outlier-reject") {
      std::string v = value("--gi-outlier-reject");
      if (o.ok && !parseBool(v, o.giOutlierReject))
        fail("--gi-outlier-reject expects on/off");
      continue;
    }
    if (a == "--integrator") {
      std::string v = value("--integrator");
      if (v == "cache")
        o.giIntegrator = 0;
      else if (v == "pt1" || v == "pt2") {
        // Asking for pt1/pt2 EXPLICITLY implies the GI pipeline: the gi gate
        // drives the ambient zeroing, the _amb_frac energy rebalance and the
        // GI AOV plumbing, which they share with the cache -- without it a
        // bare "--integrator pt1" would silently render with no GI at all.
        // The implication lives here, on the explicit flag, and NOT on the
        // giIntegrator value: pt1 is the default, so keying off it would turn
        // GI on for every render. A later --gi off still wins.
        o.giIntegrator = (v == "pt2") ? 2 : 1;
        o.gi = true;
      } else
        fail("--integrator expects cache/pt1/pt2");
      continue;
    }
    if (a == "--material") {
      std::string v = value("--material");
      if (v == "pov")
        o.materialModel = 0;
      else if (v == "principled")
        o.materialModel = 1;
      else
        fail("--material expects pov/principled");
      continue;
    }
    if (a == "--pt2-pattern") {
      std::string v = value("--pt2-pattern");
      if (v == "sobol")
        o.pt2Pattern = 0;
      else if (v == "bluenoise")
        o.pt2Pattern = 1;
      else
        fail("--pt2-pattern expects sobol/bluenoise");
      continue;
    }
    if (a == "--pt2-emissive") {
      std::string v = value("--pt2-emissive");
      if (o.ok && !parseBool(v, o.pt2Emissive))
        fail("--pt2-emissive expects on/off");
      continue;
    }
    if (a == "--pt2-rounds") {
      o.pt2Rounds = std::atoi(value("--pt2-rounds").c_str());
      if (o.ok && o.pt2Rounds < 0) fail("--pt2-rounds expects >= 0");
      continue;
    }
    if (a == "--pt2-radius") {
      o.pt2Radius = static_cast<float>(std::atof(value("--pt2-radius").c_str()));
      if (o.ok && o.pt2Radius <= 0.0f) fail("--pt2-radius expects > 0");
      continue;
    }
    if (a == "--pt2-unbiased") {
      std::string v = value("--pt2-unbiased");
      if (o.ok && !parseBool(v, o.pt2Unbiased))
        fail("--pt2-unbiased expects on/off");
      continue;
    }
    if (a == "--pt2-mcap") {
      o.pt2MCap = static_cast<float>(std::atof(value("--pt2-mcap").c_str()));
      if (o.ok && o.pt2MCap < 1.0f) fail("--pt2-mcap expects >= 1");
      continue;
    }
    if (a == "--pt2-wclamp") {
      o.pt2WClamp = static_cast<float>(std::atof(value("--pt2-wclamp").c_str()));
      if (o.ok && o.pt2WClamp < 0.0f) fail("--pt2-wclamp expects >= 0 (0 = off)");
      continue;
    }
    if (a == "--pt2-adaptive") {
      std::string v = value("--pt2-adaptive");
      if (o.ok && !parseBool(v, o.pt2Adaptive))
        fail("--pt2-adaptive expects on/off");
      continue;
    }
    if (a == "--pt2-adaptive-thresh") {
      o.pt2AdaptiveThresh =
          static_cast<float>(std::atof(value("--pt2-adaptive-thresh").c_str()));
      if (o.ok && o.pt2AdaptiveThresh <= 0.0f)
        fail("--pt2-adaptive-thresh expects > 0");
      continue;
    }
    if (a == "--pt2-adaptive-mul") {
      o.pt2AdaptiveMul = std::atoi(value("--pt2-adaptive-mul").c_str());
      if (o.ok && o.pt2AdaptiveMul < 2) fail("--pt2-adaptive-mul expects >= 2");
      continue;
    }
    if (a == "--pt2-reflect") {
      std::string v = value("--pt2-reflect");
      if (o.ok && !parseBool(v, o.pt2Reflect))
        fail("--pt2-reflect expects on/off");
      continue;
    }
    if (a == "--pt2-emissive-nee") {
      std::string v = value("--pt2-emissive-nee");
      if (o.ok && !parseBool(v, o.pt2EmissiveNee))
        fail("--pt2-emissive-nee expects on/off");
      continue;
    }
    if (a == "--pt2-glossy") {
      std::string v = value("--pt2-glossy");
      if (o.ok && !parseBool(v, o.pt2Glossy))
        fail("--pt2-glossy expects on/off");
      continue;
    }
    if (a == "--pt2-glossy-spp") {
      o.pt2GlossySpp = std::atoi(value("--pt2-glossy-spp").c_str());
      if (o.ok && o.pt2GlossySpp < 1) fail("--pt2-glossy-spp expects >= 1");
      continue;
    }
    if (a == "--pt2-glossy-denoise") {
      std::string v = value("--pt2-glossy-denoise");
      if (o.ok && !parseBool(v, o.pt2GlossyDenoise))
        fail("--pt2-glossy-denoise expects on/off");
      continue;
    }
    if (a == "--quality") {
      // pt1 quality preset: expands to --integrator pt1 plus spp/resolution/
      // bounces at the point of appearance, so later explicit flags override
      // individual values (put --quality first).
      std::string v = value("--quality");
      // draft/high gather at OUTPUT resolution with stratified sampling and
      // the silhouette-rim patch (user-approved 2026-07: visually matches
      // the old full-hi-res-grid presets at ~12x less cost). ultra stays on
      // the full supersampled grid as the reference preset.
      // Every preset asks for pt1 explicitly, so -- like --integrator pt1 --
      // it turns the GI gate on (a preset with no GI would be meaningless).
      if (v == "draft") {
        o.giIntegrator = 1;
        o.gi = true;
        o.pt1GatherDiv = -1;  // output resolution
        o.pt1Spp = 8;
        o.giBounces = 1;
        o.pt1Ld = true;
      } else if (v == "high") {
        o.giIntegrator = 1;
        o.gi = true;
        o.pt1GatherDiv = -1;  // output resolution
        o.pt1Spp = 32;
        o.giBounces = 2;
        o.pt1Ld = true;
      } else if (v == "ultra") {
        o.giIntegrator = 1;
        o.gi = true;
        o.pt1HalfRes = false;
        o.pt1GatherDiv = 0;  // legacy derivation -> pt1HalfRes = full-res grid
        o.pt1Spp = 256;
        o.giBounces = 3;
      } else {
        fail("--quality expects draft/high/ultra");
      }
      continue;
    }
    if (a == "--spp") {
      o.pt1Spp = std::atoi(value("--spp").c_str());
      continue;
    }
    if (a == "--indirect-res") {
      std::string v = value("--indirect-res");
      if (v == "full")
        o.pt1GatherDiv = 1;
      else if (v == "half")
        o.pt1GatherDiv = 2;
      else if (v == "quarter")
        o.pt1GatherDiv = 4;
      else if (v == "out")
        o.pt1GatherDiv = -1;  // resolved to the supersample factor
      else
        fail("--indirect-res expects full/half/quarter/out");
      continue;
    }
    if (a == "--denoise") {
      std::string v = value("--denoise");
      if (o.ok && !parseBool(v, o.pt1Denoise))
        fail("--denoise expects on/off");
      continue;
    }
    if (a == "--sky") {
      std::string v = value("--sky");
      if (v == "uniform")
        o.pt1SkyMode = 0;
      else if (v == "gradient")
        o.pt1SkyMode = 1;
      else
        fail("--sky expects uniform/gradient");
      continue;
    }
    if (a == "--sky-radiance") {
      std::string v = value("--sky-radiance");
      if (o.ok && !parseVec3(v, o.pt1SkyRadiance))
        fail("--sky-radiance expects r,g,b");
      continue;
    }
    if (a == "--seed") {
      o.pt1Seed = static_cast<unsigned>(
          std::strtoul(value("--seed").c_str(), nullptr, 10));
      continue;
    }
    if (a == "--pt1-ld") {
      std::string v = value("--pt1-ld");
      if (o.ok && !parseBool(v, o.pt1Ld)) fail("--pt1-ld expects on/off");
      continue;
    }
    if (a == "--pt1-edge-patch") {
      std::string v = value("--pt1-edge-patch");
      if (o.ok && !parseBool(v, o.pt1EdgePatch))
        fail("--pt1-edge-patch expects on/off");
      continue;
    }
    if (a == "--pt1-patch-thresh") {
      o.pt1EdgePatchThresh =
          static_cast<float>(std::atof(value("--pt1-patch-thresh").c_str()));
      continue;
    }
    if (a == "--pt1-stats") {
      std::string v = value("--pt1-stats");
      if (o.ok && !parseBool(v, o.pt1Stats))
        fail("--pt1-stats expects on/off");
      continue;
    }
    if (a == "--pt1-clamp") {
      o.pt1Clamp = static_cast<float>(std::atof(value("--pt1-clamp").c_str()));
      continue;
    }
    if (a == "--pt1-upsample-normal-pow") {
      o.pt1UpsampleNormalPow = static_cast<float>(
          std::atof(value("--pt1-upsample-normal-pow").c_str()));
      continue;
    }
    if (a == "--pt1-upsample-depth-scale") {
      o.pt1UpsampleDepthScale = static_cast<float>(
          std::atof(value("--pt1-upsample-depth-scale").c_str()));
      continue;
    }
    if (a == "--denoiser") {
      std::string v = value("--denoiser");
      if (v == "none")
        o.denoiser = 0;
      else if (v == "atrous")
        o.denoiser = 1;
      else if (v == "oidn")
        o.denoiser = 2;
      else
        fail("--denoiser expects none/atrous/oidn");
      continue;
    }
    if (a == "--denoise-iters") {
      o.denoiseIters = std::atoi(value("--denoise-iters").c_str());
      continue;
    }
    if (a == "--denoise-sigma-z") {
      o.denoiseSigmaZ = static_cast<float>(std::atof(value("--denoise-sigma-z").c_str()));
      continue;
    }
    if (a == "--denoise-sigma-n") {
      o.denoiseSigmaN = static_cast<float>(std::atof(value("--denoise-sigma-n").c_str()));
      continue;
    }
    if (a == "--denoise-sigma-l") {
      o.denoiseSigmaL = static_cast<float>(std::atof(value("--denoise-sigma-l").c_str()));
      continue;
    }
    if (a == "--denoise-demodulate") {
      std::string v = value("--denoise-demodulate");
      if (o.ok && !parseBool(v, o.denoiseDemodulateAlbedo))
        fail("--denoise-demodulate expects on/off");
      continue;
    }
    if (a == "--oidn-clean-aux") {
      std::string v = value("--oidn-clean-aux");
      if (o.ok && !parseBool(v, o.oidnCleanAux))
        fail("--oidn-clean-aux expects on/off");
      continue;
    }
    if (a == "--oidn-max-memory") {
      o.oidnMaxMemoryMB = std::atoi(value("--oidn-max-memory").c_str());
      continue;
    }
    if (a == "--shadows") {
      std::string v = value("--shadows");
      if (o.ok && !parseBool(v, o.shadows)) fail("--shadows expects on/off");
      continue;
    }
    if (a == "--shadow-samples") {
      o.shadowSamples = std::atoi(value("--shadow-samples").c_str());
      o.shadowSamplesSet = true;
      continue;
    }
    if (a == "--light-radius") {
      o.lightRadius =
          static_cast<float>(std::atof(value("--light-radius").c_str()));
      continue;
    }
    if (a == "--env-light") {
      o.envLights = std::atoi(value("--env-light").c_str());
      continue;
    }
    if (a == "--env-intensity") {
      o.envIntensity =
          static_cast<float>(std::atof(value("--env-intensity").c_str()));
      continue;
    }
    if (a == "--env-key-scale") {
      o.envKeyScale =
          static_cast<float>(std::atof(value("--env-key-scale").c_str()));
      continue;
    }
    if (a == "--env-angle") {
      o.envAngle = static_cast<float>(std::atof(value("--env-angle").c_str()));
      continue;
    }
    if (a == "--pov-gain") {
      o.povGain = static_cast<float>(std::atof(value("--pov-gain").c_str()));
      continue;
    }
    if (a == "--outline-scale") {
      o.outlineScale =
          static_cast<float>(std::atof(value("--outline-scale").c_str()));
      continue;
    }
    if (a == "--supersample") {
      o.supersample = std::atoi(value("--supersample").c_str());
      o.supersampleSet = true;
      continue;
    }
    if (a == "--aa") {
      std::string v = value("--aa");
      if (v == "grid")
        o.aaMode = 0;
      else if (v == "adaptive")
        o.aaMode = 1;
      else
        fail("--aa expects grid/adaptive");
      continue;
    }
    if (a == "--aa-threshold") {
      o.aaThreshold =
          static_cast<float>(std::atof(value("--aa-threshold").c_str()));
      continue;
    }
    if (a == "--aa-depth") {
      o.aaDepth = std::atoi(value("--aa-depth").c_str());
      continue;
    }
    if (a == "--aa-debug") {
      std::string v = value("--aa-debug");
      if (o.ok && !parseBool(v, o.aaDebug))
        fail("--aa-debug expects on/off");
      continue;
    }
    if (a == "--specular-scale") {
      o.specularScale =
          static_cast<float>(std::atof(value("--specular-scale").c_str()));
      o.specularScaleSet = true;
      continue;
    }
    if (a == "--threads") {
      o.threads = std::atoi(value("--threads").c_str());
      continue;
    }
    if (a == "--declare") {
      // --declare name=value (predefined POV constant for the .pov path)
      std::string kv = value("--declare");
      std::size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        fail("--declare expects name=value");
      } else {
        o.declares[kv.substr(0, eq)] =
            std::atof(kv.substr(eq + 1).c_str());
      }
      continue;
    }
    if (a == "--alpha") {
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
      continue;
    }
    if (a == "--list-groups") {
      o.listGroups = true;
      continue;
    }
    if (a == "--edges") {
      std::string v = value("--edges");
      if (o.ok && !parseBool(v, o.edges)) fail("--edges expects on/off");
      continue;
    }
    if (a == "--edges-only") {
      std::string v = value("--edges-only");
      if (o.ok && !parseBool(v, o.edgesOnly))
        fail("--edges-only expects on/off");
      continue;
    }
    if (a == "--edge") {
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
      continue;
    }
    if (a == "--obj-edges") {
      std::string v = value("--obj-edges");
      if (o.ok && !parseBool(v, o.objEdges)) fail("--obj-edges expects on/off");
      continue;
    }
    if (a == "--obj-edge-width") {
      o.objEdgeWidth =
          static_cast<float>(std::atof(value("--obj-edge-width").c_str()));
      continue;
    }
    if (a == "--obj-edge-raise") {
      o.objEdgeRaise =
          static_cast<float>(std::atof(value("--obj-edge-raise").c_str()));
      continue;
    }
    if (a == "--obj-edge-segments") {
      o.objEdgeSegments = std::atoi(value("--obj-edge-segments").c_str());
      continue;
    }
    if (a == "--obj-edge-color") {
      std::string v = value("--obj-edge-color");
      if (o.ok && !parseHexColor(v, o.objEdgeColor))
        fail("--obj-edge-color expects #RRGGBB");
      continue;
    }
    if (a == "--obj-edge-clip") {
      std::string v = value("--obj-edge-clip");
      if (o.ok && !parseBool(v, o.objEdgeClip))
        fail("--obj-edge-clip expects on/off");
      continue;
    }
    if (a == "--obj-edge-hard-deg") {
      o.objEdgeHardDeg =
          static_cast<float>(std::atof(value("--obj-edge-hard-deg").c_str()));
      continue;
    }
    if (a == "--obj-edge-crease-deg") {
      o.objEdgeCreaseDeg =
          static_cast<float>(std::atof(value("--obj-edge-crease-deg").c_str()));
      continue;
    }
    if (a == "--obj-edge-crease-smooth-deg") {
      o.objEdgeCreaseSmoothDeg = static_cast<float>(
          std::atof(value("--obj-edge-crease-smooth-deg").c_str()));
      continue;
    }
    if (a == "--obj-edge-crease-convex-only") {
      std::string v = value("--obj-edge-crease-convex-only");
      if (o.ok && !parseBool(v, o.objEdgeCreaseConvexOnly))
        fail("--obj-edge-crease-convex-only expects on/off");
      continue;
    }
    if (a == "--obj-edge-border-coplanar-deg") {
      o.objEdgeBorderCoplanarDeg = static_cast<float>(
          std::atof(value("--obj-edge-border-coplanar-deg").c_str()));
      continue;
    }
    if (a == "--obj-edge-crease-max-deg") {
      o.objEdgeCreaseMaxDeg = std::atoi(value("--obj-edge-crease-max-deg").c_str());
      continue;
    }
    if (a == "--obj-edge-mesh-sil") {
      std::string v = value("--obj-edge-mesh-sil");
      if (o.ok && !parseBool(v, o.objEdgeMeshSil))
        fail("--obj-edge-mesh-sil expects on/off");
      continue;
    }
    if (a == "--obj-edge-mesh-crease") {
      std::string v = value("--obj-edge-mesh-crease");
      if (o.ok && !parseBool(v, o.objEdgeMeshCrease))
        fail("--obj-edge-mesh-crease expects on/off");
      continue;
    }
    if (a == "--obj-edge-mesh-border") {
      std::string v = value("--obj-edge-mesh-border");
      if (o.ok && !parseBool(v, o.objEdgeMeshBorder))
        fail("--obj-edge-mesh-border expects on/off");
      continue;
    }
    if (a == "--obj-edge-visibility") {
      std::string v = value("--obj-edge-visibility");
      if (o.ok && !parseBool(v, o.objEdgeVisibility))
        fail("--obj-edge-visibility expects on/off");
      continue;
    }
    if (a == "--obj-edge-only") {
      std::string v = value("--obj-edge-only");
      if (o.ok && !parseBool(v, o.objEdgeOnly))
        fail("--obj-edge-only expects on/off");
      continue;
    }
    if (a == "--stroke-thickness") {
      o.strokeThickness =
          static_cast<float>(std::atof(value("--stroke-thickness").c_str()));
      o.strokeThicknessSet = true;
      continue;
    }
    if (a == "--stroke-resample") {
      o.strokeResample =
          static_cast<float>(std::atof(value("--stroke-resample").c_str()));
      o.strokeResampleSet = true;
      continue;
    }
    if (a == "--edge-crease-deg") {
      o.strokeCreaseDeg =
          static_cast<float>(std::atof(value("--edge-crease-deg").c_str()));
      o.strokeCreaseDegSet = true;
      continue;
    }
    if (a == "--stroke-silhouette") {
      std::string v = value("--stroke-silhouette");
      if (o.ok && !parseBool(v, o.strokeSilhouette))
        fail("--stroke-silhouette expects on/off");
      continue;
    }
    if (a == "--stroke-crease") {
      std::string v = value("--stroke-crease");
      if (o.ok && !parseBool(v, o.strokeCrease))
        fail("--stroke-crease expects on/off");
      continue;
    }
    if (a == "--stroke-border") {
      std::string v = value("--stroke-border");
      if (o.ok && !parseBool(v, o.strokeBorder))
        fail("--stroke-border expects on/off");
      continue;
    }
    if (a == "--stroke-taper") {
      std::string v = value("--stroke-taper");
      if (o.ok && !parseBool(v, o.strokeTaper))
        fail("--stroke-taper expects on/off");
      continue;
    }
    if (a == "--stroke-smooth") {
      std::string v = value("--stroke-smooth");
      if (o.ok && !parseBool(v, o.strokeSmooth))
        fail("--stroke-smooth expects on/off");
      continue;
    }
    if (a == "--edge-qi-dots") {
      std::string v = value("--edge-qi-dots");
      if (o.ok && !parseBool(v, o.strokeQiDots))
        fail("--edge-qi-dots expects on/off");
      continue;
    }
    if (a == "--edge-qi-vertex-dots") {
      std::string v = value("--edge-qi-vertex-dots");
      if (o.ok && !parseBool(v, o.strokeQiVertexDots))
        fail("--edge-qi-vertex-dots expects on/off");
      continue;
    }
    if (a == "--edge-qi-vertex-delta") {
      o.strokeQiVertexDelta =
          static_cast<float>(std::atof(value("--edge-qi-vertex-delta").c_str()));
      continue;
    }
    if (a == "--edge-qi-lift") {
      o.strokeQiLift =
          static_cast<float>(std::atof(value("--edge-qi-lift").c_str()));
      continue;
    }
    if (a == "--edge-qi-split") {
      std::string v = value("--edge-qi-split");
      if (o.ok && !parseBool(v, o.strokeQiSplit))
        fail("--edge-qi-split expects on/off");
      continue;
    }
    if (a == "--edge-reject-concave") {
      std::string v = value("--edge-reject-concave");
      if (o.ok && !parseBool(v, o.strokeRejectConcave))
        fail("--edge-reject-concave expects on/off");
      continue;
    }
    if (a == "--stroke-geom-silhouette") {
      std::string v = value("--stroke-geom-silhouette");
      if (o.ok && !parseBool(v, o.strokeGeomSilhouette))
        fail("--stroke-geom-silhouette expects on/off");
      continue;
    }
    if (a == "--stroke-analytic") {
      std::string v = value("--stroke-analytic");
      if (o.ok && !parseBool(v, o.strokeAnalytic))
        fail("--stroke-analytic expects on/off");
      continue;
    }
    if (a == "--stroke-analytic-segments") {
      o.strokeAnalyticSegments =
          std::atoi(value("--stroke-analytic-segments").c_str());
      o.strokeAnalyticSegmentsSet = true;
      continue;
    }
    if (a == "--stroke-self-exclude-rings") {
      o.strokeSelfExcludeRings =
          std::atoi(value("--stroke-self-exclude-rings").c_str());
      o.strokeSelfExcludeRingsSet = true;
      continue;
    }
    if (a == "--stroke-depth-gap") {
      o.strokeDepthGap =
          static_cast<float>(std::atof(value("--stroke-depth-gap").c_str()));
      continue;
    }
    if (a == "--stroke-screen-simplify") {
      o.strokeScreenSimplify = static_cast<float>(
          std::atof(value("--stroke-screen-simplify").c_str()));
      continue;
    }
    if (a == "--stroke-screen-smooth") {
      o.strokeScreenSmooth =
          std::atoi(value("--stroke-screen-smooth").c_str());
      continue;
    }
    if (a == "--stroke-screen-minlen") {
      o.strokeScreenMinLen = static_cast<float>(
          std::atof(value("--stroke-screen-minlen").c_str()));
      continue;
    }
    if (a == "--dump-aov") {
      o.dumpAovPrefix = value("--dump-aov");
      continue;
    }
    if (a == "--keep-baked-edges") {
      std::string v = value("--keep-baked-edges");
      if (o.ok && !parseBool(v, o.keepBakedEdges))
        fail("--keep-baked-edges expects on/off");
      continue;
    }
    if (a == "--transparent-bg") {
      std::string v = value("--transparent-bg");
      if (o.ok && !parseBool(v, o.transparentBackground))
        fail("--transparent-bg expects on/off");
      continue;
    }
    if (a == "--transparency") {
      std::string v = value("--transparency");
      if (o.ok && !parseBool(v, o.transparency))
        fail("--transparency expects on/off");
      continue;
    }
    if (a == "--compare") {
      o.compareMode = true;
      o.compareA = value("--compare");
      o.compareB = value("--compare");
      continue;
    }
    if (a == "--convert") {
      o.convertMode = true;
      o.convertIn = value("--convert");
      o.convertOut = value("--convert");
      continue;
    }
    if (!a.empty() && a[0] == '-') {
      fail("unknown option: " + a);
      continue;
    }
    if (o.input.empty()) {
      o.input = a;
      continue;
    }
    if (true) {
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
      "  --aa <grid|adaptive>     antialiasing: full grid / adaptive refine [grid]\n"
      "  --aa-threshold <float>   adaptive: color contrast that refines  [0.1]\n"
      "  --aa-depth <int>         adaptive: edge lattice (>ss = finer)  [0=ss]\n"
      "  --aa-debug <on|off>      adaptive: dump refinement mask AOV    [off]\n"
      "  --specular-scale <float> cartoon specular x      [.pov: 0 = matte]\n"
      "  --threads <int>          TBB parallelism cap (1 = serial)  [0 = all]\n"
      "  --alpha <ID=value>       section group alpha (e.g. _34_35=0.5),\n"
      "                           blendpng-equivalent multipass blend\n"
      "  --list-groups            list the input's section ids and exit\n"
      "  --edges <on|off>         Freestyle stroke edge pass (sil/crease/border) [off]\n"
      "  --edges-only <on|off>    draw ONLY edges over blank bg, full opacity (verify) [off]\n"
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
      "  --obj-edge-visibility <on|off> object-space hidden-line clip      [off]\n"
      "  --obj-edge-only <on|off> render ONLY edges (drop surface, verify)  [off]\n"
      "  --stroke-thickness <float> stroke full width, final px             [2]\n"
      "  --stroke-resample <float> stroke arc-length resample step, px      [2]\n"
      "  --edge-crease-deg <float> stroke crease dihedral threshold (deg)  [30]\n"
      "  --stroke-silhouette <on|off> stroke silhouette nature             [on]\n"
      "  --stroke-crease <on|off> stroke crease nature                    [off]\n"
      "  --stroke-border <on|off> occluding cross-section boundary lines    [on]\n"
      "  --stroke-taper <on|off>  taper stroke width toward its ends (demo) [off]\n"
      "  --stroke-smooth <on|off> corner-preserving backbone smoothing (demo)[off]\n"
      "  --edge-qi-dots <on|off>  overlay pre-majority QI flags as dots (debug)[off]\n"
      "  --edge-qi-vertex-dots <on|off> overlay raw per-vertex QI vis dots (debug)[off]\n"
      "  --edge-qi-vertex-delta <float> offset vertex QI probe along mesh normal[0]\n"
      "  --edge-qi-lift <float>   production QI normal-lift, abs units (0=legacy)[0]\n"
      "  --edge-qi-split <on|off> with lift: on=per-sample split(B), off=majority(A)[off]\n"
      "  --edge-reject-concave <on|off> drop concave(valley) feature edges[on]\n"
      "  --stroke-geom-silhouette <on|off> geometric per-edge silhouette[off]\n"
      "  --stroke-analytic <on|off> draw sphere/cylinder outlines (ball-stick) [on]\n"
      "  --stroke-analytic-segments <int> sphere ring / cap tessellation   [48]\n"
      "  --stroke-self-exclude-rings <int> QI self-occlusion exclude radius  [6]\n"
      "  --stroke-depth-gap <float> screen: depth-gap slope cutoff, world/px [12]\n"
      "  --stroke-screen-simplify <float> screen: Douglas-Peucker eps, px [0.4]\n"
      "  --stroke-screen-smooth <int> screen: Chaikin corner-cut iterations [2]\n"
      "  --stroke-screen-minlen <float> screen: drop chains shorter than, px [4]\n"
      "  --dump-aov <prefix>      dump AOV images (--edges and/or --ao-write-aov)\n"
      "  --keep-baked-edges <on|off> keep baked POV edges with --edges on (A/B) [off]\n"
      "  --transparent-bg <on|off> transparent background output      [off]\n"
      "  --transparency <on|off>  single-layer transparency walk        [on]\n"
      "  --ao-samples <int>       ambient occlusion rays/hit  [0 = off]\n"
      "  --ao-res <full|out>      AO gather grid: per-hit / per-output-px [full]\n"
      "  --ao-res-debug <on|off>  dump the coarse-AO fallback mask AOV   [off]\n"
      "  --ao-distance <float>    AO occluder radius   [auto from scene]\n"
      "  --ao-intensity <float>   AO strength multiplier        [1.0]\n"
      "  --ao-falloff <power>     AO distance falloff exponent  [0 = binary]\n"
      "  --ao-multiscale <on|off> 3 nested AO radii (contact+shape)   [off]\n"
      "  --ao-bent-normal <on|off> directional sky/ground ambient    [off]\n"
      "  --ao-sky #RRGGBB         bent-normal up-hemisphere tint  [#ffffff]\n"
      "  --ao-ground #RRGGBB      bent-normal down-hemisphere tint[#ffffff]\n"
      "  --ao-camera-up <on|off>  gradient axis = camera up          [on]\n"
      "  --ao-up x,y,z            explicit gradient axis (camera-up off)\n"
      "  --ao-multibounce <on|off> albedo-aware AO (anti over-dark)   [off]\n"
      "  --ao-ld <on|off>         low-discrepancy AO sampling        [off]\n"
      "  --ao-diffuse <frac>      also darken direct diffuse in cavities [0]\n"
      "  --ao-write-aov <on|off>  emit AO/G-buffer AOVs (with --dump-aov) [off]\n"
      "  --shadows <on|off>       cast shadows from lights           [off]\n"
      "  --shadow-samples <int>   shadow rays/light (>1 = soft)       [1]\n"
      "  --light-radius <float>   light angular radius deg (soft)     [0]\n"
      "  --env-light <int>        dome lights (0=off; ~32 = soft sky) [0]\n"
      "  --env-intensity <float>  dome irradiance on exposed point    [1.0]\n"
      "  --env-key-scale <float>  scale scene lights w/ dome (0=dome) [0]\n"
      "  --env-angle <deg>        dome half-angle about camera        [90]\n"
      "  --gi <on|off>            diffuse GI surface irradiance cache [off]\n"
      "  --gi-samples <int>       hemisphere gather rays per record    [64]\n"
      "  --gi-bounces <int>       indirect bounces (1 = one-bounce)     [1]\n"
      "  --gi-gradients <on|off>  Ward-Heckbert gradient interpolation [off]\n"
      "  --gi-outlier-reject <on|off> lift isolated dark cache records  [on]\n"
      "  --denoiser <none|atrous|oidn> GI denoise backend  [atrous when --gi]\n"
      "  --denoise-iters <int>    a-trous wavelet iterations            [5]\n"
      "  --denoise-sigma-z <f>    depth edge-stop sigma                 [1]\n"
      "  --denoise-sigma-n <f>    normal edge-stop exponent           [128]\n"
      "  --denoise-sigma-l <f>    luminance edge-stop sigma             [4]\n"
      "  --denoise-demodulate <on|off> denoise color/albedo            [on]\n"
      "  --oidn-clean-aux <on|off> OIDN treats aux as noise-free        [on]\n"
      "  --oidn-max-memory <MB>   OIDN scratch cap (-1 = OIDN default)  [1024]\n"
      "  --gi-max-dist <world>    gather ray max distance (0 = auto)    [0]\n"
      "  --gi-intensity <float>   indirect gain (1.0 = physical)      [1.0]\n"
      "  --gi-env-intensity <f>   ambient fill mult (auto-calibrated; <1 deeper) [1.0]\n"
      "  --gi-accuracy <a>        interpolation accuracy a           [0.15]\n"
      "  --gi-spacing <world>     record voxel spacing (0 = auto)       [0]\n"
      "  --gi-normal-reject <cos> min dot(n_x,n_rec) to blend a record[0.85]\n"
      "  --gi-component-reject <on|off> reject cross-section records   [on]\n"
      "  --gi-seed-per-vertex <on|off> seed records from mesh verts   [off]\n"
      "  --integrator <cache|pt1|pt2> indirect GI integrator; pt2 = pt1 +\n"
      "                           Sobol/blue-noise sampler + emissive GI;\n"
      "                           cache is experimental (pt1/pt2 imply\n"
      "                           --gi on)                              [pt1]\n"
      "  --material <pov|principled> convert every POV finish to the\n"
      "                           principled GGX subset (lossy: only\n"
      "                           diffuse-only finishes stay bitwise)   [pov]\n"
      "  --pt2-pattern <sobol|bluenoise> pt2 first-bounce sample\n"
      "                           arrangement                     [bluenoise]\n"
      "  --pt2-emissive <on|off>  pt2: emissive geometry lights its\n"
      "                           surroundings (GI transport)           [on]\n"
      "  --pt2-rounds <int>       pt2 ReSTIR spatial resampling rounds\n"
      "                           (experimental; wins only at spp=1)     [0]\n"
      "  --pt2-radius <px>        pt2 round-0 reuse kernel radius on the\n"
      "                           gather grid (halves per round)        [16]\n"
      "  --pt2-unbiased <on|off>  pt2 Z-normalization with visibility\n"
      "                           re-check rays                        [off]\n"
      "  --pt2-mcap <f>           pt2 reservoir history clamp M        [100]\n"
      "  --pt2-adaptive <on|off>  pt2 variance-adaptive spp: refine only\n"
      "                           unconverged pixels                   [off]\n"
      "  --pt2-adaptive-thresh <f> pt2 adaptive noise threshold        [0.15]\n"
      "  --pt2-adaptive-mul <int> pt2 adaptive total budget = mul*spp    [4]\n"
      "  --pt2-reflect <on|off>   pt2 traced mirror reflection (replaces\n"
      "                           the fake reflection*background)        [on]\n"
      "  --pt2-emissive-nee <on|off> pt2 emissive-triangle NEE + MIS\n"
      "                           (off = BSDF-only, for A/B)             [on]\n"
      "  --pt2-glossy <on|off>    pt2 glossy GGX reflection: reflective\n"
      "                           finishes with specular > 0 blur their\n"
      "                           traced reflection to the highlight width\n"
      "                           (specular == 0 stays a mirror)          [on]\n"
      "  --pt2-glossy-spp <int>   pt2 glossy reflection rays per pixel    [8]\n"
      "  --pt2-glossy-denoise <on|off> OIDN on the glossy reflection\n"
      "                           buffer (also needs --denoise on)        [on]\n"
      "  --pt2-wclamp <f>         pt2 contribution weight clamp (0=off)  [0]\n"
      "  --quality <draft|high|ultra> pt1 preset: 8spp out-res ld 1-bounce /\n"
      "                           32spp out-res ld 2-bounce / 256spp full\n"
      "                           3-bounce (put it FIRST; later flags\n"
      "                           override)  [draft-equivalent]\n"
      "  --spp <int>              pt1 gather rays per pixel             [8]\n"
      "  --indirect-res <full|half|quarter|out> pt1 gather grid = render\n"
      "                           grid / {1,2,4,ss} (out = final size)   [out]\n"
      "  --denoise <on|off>       pt1 indirect-only OIDN denoise        [on]\n"
      "  --sky <uniform|gradient> pt1 gather sky model            [uniform]\n"
      "  --sky-radiance r,g,b     pt1 sky tint (x ambient energy)   [1,1,1]\n"
      "  --seed <int>             pt1 per-pixel RNG seed                [0]\n"
      "  --pt1-ld <on|off>        pt1 stratified 1st-bounce sampling    [on]\n"
      "  --pt1-clamp <f>          pt1 per-sample luminance clamp      [0=off]\n"
      "  --pt1-edge-patch <on|off> full-res re-gather of silhouette rims\n"
      "                           the reduced gather grid cannot resolve  [on]\n"
      "  --pt1-stats <on|off>     print pt1 denoiser stage split (debug) [off]\n"
      "  --pt1-upsample-normal-pow <f> upsample normal edge-stop       [32]\n"
      "  --pt1-upsample-depth-scale <f> upsample depth edge-stop     [0.02]\n"
      "  --compare <a> <b>        print PSNR/SSIM between two PPM files\n"
      "  --convert <in> <out>     convert a PPM to PNG/PPM\n"
      "  -h, --help               show this help\n",
      prog);
}

}  // namespace umbreon
