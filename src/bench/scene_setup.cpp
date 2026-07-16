#include "scene_setup.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>

#include "edges/object_space_edges.hpp"
#include "geom/mesh2_reader.hpp"
#include "pov/pov_scene_reader.hpp"

namespace umbreon {
namespace {

// Resolve a path referenced from inside a .pov file relative to that file's
// directory (POV resolves #include paths this way).
std::string resolveRelative(const std::string& base, const std::string& rel) {
  if (!rel.empty() && rel[0] == '/') return rel;  // already absolute
  std::size_t slash = base.find_last_of('/');
  std::string dir = (slash == std::string::npos) ? "." : base.substr(0, slash);
  return dir + "/" + rel;
}

}  // namespace

bool buildSceneFromPov(Options& opt, Scene& scene, RenderOptions& ropt,
                       std::vector<std::string>& groupNames) {
  // ---- reproduce the CueMol POV-Ray viewing setup. ----
  const int W = opt.widthSet ? opt.width : 300;
  const int H = opt.heightSet ? opt.height : 300;

  // GI energy balance: mirror the .pov's #ifdef(_radiosity) branch. The
  // umbreon default lighting puts ALL energy in the direct lights
  // (_amb_frac=0), so GI -- which only replaces the constant ambient with an
  // occlusion-aware gather -- has nothing to occlude and is nearly a no-op
  // (pixel-identical to the flat render). When GI is on, adopt the POV
  // radiosity balance: move half the energy into the ambient the GI gathers
  // and dim the direct lights to match (POV _radiosity: _light_inten=1.6,
  // _amb_frac=0.5, _flash_frac=0.5). Only the non-rad DEFAULTS are bumped, so
  // an explicit --declare still wins.
  if (opt.gi) {
    auto giDefault = [&](const char* k, double nonRad, double rad) {
      auto it = opt.declares.find(k);
      if (it == opt.declares.end() || it->second == nonRad)
        opt.declares[k] = rad;
    };
    giDefault("_light_inten", 1.3, 1.6);
    giDefault("_amb_frac", 0.0, 0.5);
    giDefault("_flash_frac", 0.6, 0.5);
  }

  umbreon::PovParseOptions popt;
  popt.imageWidth = W;
  popt.imageHeight = H;
  popt.predefined = opt.declares;
  std::printf("reading POV scene from %s\n", opt.input.c_str());
  umbreon::PovSceneResult ps = umbreon::readPovScene(opt.input, popt);
  if (ps.includePath.empty())
    throw std::runtime_error(
        "no '#declare _scene = #include \"...\"' found in " + opt.input);

  const std::string incPath = resolveRelative(opt.input, ps.includePath);
  std::printf("  geometry include: %s\n", incPath.c_str());
  umbreon::SceneGeometry geo = umbreon::readGeometryFromFile(incPath, ps.declares);
  std::printf("  %zu triangles, %zu spheres, %zu cylinders\n",
              geo.mesh.triangleCount(), geo.spheres.size(),
              geo.cylinders.size());

  // --list-groups: report the transparency groups (CueMol sections) so the
  // user can target one with --alpha, then exit.
  if (opt.listGroups) {
    const std::size_t ng = geo.groupNames.size();
    std::vector<int> triN(ng, 0), sphN(ng, 0), cylN(ng, 0);
    for (std::size_t t = 0; t < geo.mesh.triangleCount(); ++t) {
      std::size_t g = geo.mesh.groupForTri(t);
      if (g < ng) triN[g]++;
    }
    for (const umbreon::Sphere& s : geo.spheres)
      if (s.group < ng) sphN[s.group]++;
    for (const umbreon::Cylinder& c : geo.cylinders)
      if (c.group < ng) cylN[c.group]++;
    std::printf("transparency groups (sections) in %s:\n", incPath.c_str());
    for (std::size_t i = 0; i < ng; ++i) {
      const std::string& nm = geo.groupNames[i];
      std::printf("  %-16s  tris=%d spheres=%d cylinders=%d\n",
                  (i == 0 && nm.empty()) ? "(default)" : nm.c_str(),
                  triN[i], sphN[i], cylN[i]);
    }
    return false;
  }

  scene.mesh = std::move(geo.mesh);
  scene.spheres = std::move(geo.spheres);
  scene.cylinders = std::move(geo.cylinders);
  if (opt.outlineScale != 1.0f) {
    for (umbreon::Sphere& s : scene.spheres) s.radius *= opt.outlineScale;
    for (umbreon::Cylinder& c : scene.cylinders) c.radius *= opt.outlineScale;
  }

  // Scene-scaled default AO radius (used when --ao-distance is not given):
  // 0.7 * the geometry bounding-box diagonal, matching the .inc scene
  // builder. Without this the .pov scene keeps the 1e20 placeholder and AO
  // would search the entire scene.
  umbreon::Aabb aoBounds = scene.mesh.bounds();
  for (const umbreon::Sphere& sp : scene.spheres) {
    const umbreon::Vec3 r{sp.radius, sp.radius, sp.radius};
    aoBounds.extend(sp.center - r);
    aoBounds.extend(sp.center + r);
  }
  for (const umbreon::Cylinder& cy : scene.cylinders) {
    aoBounds.extend(cy.p0);
    aoBounds.extend(cy.p1);
  }
  if (aoBounds.valid())
    scene.aoDistance = std::max(aoBounds.diagonal() * 0.7f, 1.0e-3f);

  // Apply per-section group alpha (--alpha ID=value). The section is
  // resolved against the group names recovered by the parser and recorded
  // in Scene::groupBlend; render() then realizes it as the blendpng-
  // equivalent multi-pass post-blend (one extra full pass per section,
  // rendered opaque, blended into the final image with weight `value`).
  // The geometry's intrinsic (fragment) opacity is left untouched.
  if (!opt.sectionAlpha.empty()) {
    std::map<std::string, int> gidx;
    for (std::size_t i = 0; i < geo.groupNames.size(); ++i)
      gidx[geo.groupNames[i]] = static_cast<int>(i);
    for (const auto& kv : opt.sectionAlpha) {
      auto it = gidx.find(kv.first);
      if (it == gidx.end()) {
        std::fprintf(stderr,
                     "warning: section '%s' not found (try --list-groups)\n",
                     kv.first.c_str());
        continue;
      }
      const int g = it->second;
      const float a = kv.second;
      scene.groupBlend.push_back(
          umbreon::GroupBlend{static_cast<uint16_t>(g), a});
      std::printf(
          "  alpha override: %s = %.3f (group %d, blendpng multipass)\n",
          kv.first.c_str(), a, g);
    }
  }
  // Keep the section names for the post-block --edge resolution (the edge
  // styles need ropt.strokeEdges.defaultStyle, populated after this block).
  groupNames = geo.groupNames;
  scene.camera = ps.camera;
  scene.lights = ps.lights;
  scene.background = ps.background;
  scene.fog = ps.fog;

  // Ambient light color/energy. Without GI this is the neutral white the
  // material finish-ambient term multiplies. With GI it carries the ambient
  // ENERGY the gather occludes: _light_inten * _amb_frac (the POV radiosity
  // dome's emission), white, scaled by the same exposure gain as the direct
  // lights so the direct/ambient split stays balanced. The GI environment
  // radiance is this color * --gi-env-intensity; the constant-ambient term it
  // would otherwise feed is dropped on the GI path (see hit_shader).
  if (opt.gi) {
    const double ambE =
        opt.declares["_light_inten"] * opt.declares["_amb_frac"];
    const float a = static_cast<float>(ambE) * opt.povGain;
    scene.ambientColor = umbreon::Vec3{a, a, a};
  } else {
    scene.ambientColor = umbreon::Vec3{1.0f, 1.0f, 1.0f};
  }

  for (umbreon::DistantLight& L : scene.lights) L.intensity *= opt.povGain;
  scene.ambientIntensity = opt.povGain;

  // Always display-encode with assumed_gamma 2.2, regardless of what the
  // .pov declares (CueMol exports assumed_gamma 1.0 for the radiosity path
  // and 2.2 elsewhere; the POV-Ray radiosity reference is tone-matched at
  // 2.2). Overriding here keeps umbreon's output tone consistent across
  // scenes instead of swinging brightness with the file's gamma tag.
  scene.assumedGamma = 2.2f;
  (void)ps.assumedGamma;

  const std::string camDesc =
      scene.camera.orthographic
          ? "orthographic height=" + std::to_string(scene.camera.height)
          : "perspective fovy=" + std::to_string(scene.camera.fovy);
  std::printf(
      "  camera: %s  bg <%.3f,%.3f,%.3f>  lights=%zu  ambient=%.3f\n",
      camDesc.c_str(), scene.background.x, scene.background.y,
      scene.background.z, scene.lights.size(), scene.ambientIntensity);
  for (std::size_t i = 0; i < scene.lights.size(); ++i) {
    const umbreon::DistantLight& L = scene.lights[i];
    std::printf("    light %zu: dir <%.3f,%.3f,%.3f>  intensity %.3f\n", i,
                L.direction.x, L.direction.y, L.direction.z, L.intensity);
  }
  if (scene.fog.enabled) {
    std::printf(
        "    fog (linear): start %.3f  end %.3f  color <%.3f,%.3f,%.3f>\n",
        scene.fog.start, scene.fog.end, scene.fog.color.x,
        scene.fog.color.y, scene.fog.color.z);
  }
  std::printf("    assumed_gamma %.3f\n", scene.assumedGamma);

  ropt.width = W;
  ropt.height = H;
  // Per-material POV finishes drive specular now (F_MetalA, phong groups),
  // so the .pov path keeps the RenderOptions specularScale default (1.0)
  // unless the user overrides it explicitly.
  if (opt.specularScaleSet) ropt.specularScale = opt.specularScale;
  std::printf("rendering %dx%d  backend=umbreon (embree)\n", W, H);
  return true;
}

void applyEdgeOptions(const Options& opt, Scene& scene, RenderOptions& ropt,
                      const std::vector<std::string>& groupNames) {
  // Freestyle STROKE edges (--edges). The single strokeEdges.enable flag is the
  // master gate for the whole --edges pipeline (G-buffer AOV capture, the
  // stroke pass, and the baked-edge removal below). Off => byte-identical
  // default path. The --stroke-* knobs are wired only when edges are on.
  // --edges-only is a verification mode that implies --edges on.
  const bool edgesOn = opt.edges || opt.edgesOnly;
  ropt.strokeEdges.enable = edgesOn;
  if (edgesOn) {
    ropt.strokeEdges.edgesOnly = opt.edgesOnly;
    ropt.strokeEdges.silhouette = opt.strokeSilhouette;
    ropt.strokeEdges.crease = opt.strokeCrease;
    ropt.strokeEdges.border = opt.strokeBorder;
    ropt.strokeEdges.taper = opt.strokeTaper;
    ropt.strokeEdges.smooth = opt.strokeSmooth;
    ropt.strokeEdges.debugQiDots = opt.strokeQiDots;
    ropt.strokeEdges.debugQiVertexDots = opt.strokeQiVertexDots;
    ropt.strokeEdges.debugQiVertexDelta = opt.strokeQiVertexDelta;
    ropt.strokeEdges.qiNormalLift = opt.strokeQiLift;
    ropt.strokeEdges.qiSplit = opt.strokeQiSplit;
    ropt.strokeEdges.rejectConcaveEdges = opt.strokeRejectConcave;
    ropt.strokeEdges.meshGeomSilhouette = opt.strokeGeomSilhouette;
    ropt.strokeEdges.analytic = opt.strokeAnalytic;
    if (opt.strokeAnalyticSegmentsSet)
      ropt.strokeEdges.analyticSegments = opt.strokeAnalyticSegments;
    if (opt.strokeSelfExcludeRingsSet)
      ropt.strokeEdges.selfExcludeRings = opt.strokeSelfExcludeRings;
    if (opt.strokeThicknessSet)
      ropt.strokeEdges.thickness =
          static_cast<int>(opt.strokeThickness + 0.5f);
    if (opt.strokeResampleSet)
      ropt.strokeEdges.resampleStepPx =
          static_cast<int>(opt.strokeResample + 0.5f);
    if (opt.strokeCreaseDegSet)
      ropt.strokeEdges.creaseAngleDeg = opt.strokeCreaseDeg;
    // Screen-source chain extraction + tuning.
    ropt.strokeEdges.screenDepthGapPx = opt.strokeDepthGap;
    ropt.strokeEdges.screenSimplifyPx = opt.strokeScreenSimplify;
    ropt.strokeEdges.screenSmoothIters = opt.strokeScreenSmooth;
    ropt.strokeEdges.screenMinLenPx = opt.strokeScreenMinLen;

    // Per-section STROKE styling. The stroke pass (applyStrokeEdges) maps each
    // EdgeNature onto an EdgeStyle::cls[] slot (Silhouette->Silhouette,
    // Border->Object, Crease->Crease) and reads Scene::groupEdgeStyle[group].
    // Seed defaultStyle so a bare "--edges on" enables exactly the natures the
    // --stroke-* toggles select, inheriting the global stroke color/thickness;
    // then resolve --edge ID=spec overrides against the section names (parallel
    // to the --alpha loop, warn-on-miss). With groupEdgeStyle populated the
    // stroke pass styles per section; an empty table would fall back to the
    // single global stroke color.
    const int kSilSlot = static_cast<int>(umbreon::EdgeClass::Silhouette);
    const int kObjSlot = static_cast<int>(umbreon::EdgeClass::Object);
    const int kCreaseSlot = static_cast<int>(umbreon::EdgeClass::Crease);
    umbreon::EdgeStyle& ds = ropt.strokeEdges.defaultStyle;
    auto seedSlot = [&](int slot, bool on) {
      umbreon::EdgeClassStyle& cs = ds.cls[slot];
      cs.enabled = on;
      cs.color[0] = ropt.strokeEdges.color[0];
      cs.color[1] = ropt.strokeEdges.color[1];
      cs.color[2] = ropt.strokeEdges.color[2];
      cs.opacity = ropt.strokeEdges.opacity;
      cs.width = static_cast<float>(ropt.strokeEdges.thickness);
    };
    seedSlot(kSilSlot, opt.strokeSilhouette);
    seedSlot(kObjSlot, opt.strokeBorder);
    seedSlot(kCreaseSlot, opt.strokeCrease);

    scene.groupEdgeStyle.assign(groupNames.size(), ds);
    if (!opt.sectionEdge.empty()) {
      std::map<std::string, int> gidx;
      for (std::size_t i = 0; i < groupNames.size(); ++i)
        gidx[groupNames[i]] = static_cast<int>(i);
      for (const auto& kv : opt.sectionEdge) {
        auto it = gidx.find(kv.first);
        if (it == gidx.end()) {
          std::fprintf(stderr,
                       "warning: section '%s' not found (try --list-groups)\n",
                       kv.first.c_str());
          continue;
        }
        scene.groupEdgeStyle[it->second] = kv.second;
        std::printf("  edge override: section %s (group %d)\n",
                    kv.first.c_str(), it->second);
      }
    }
  }
  if (ropt.strokeEdges.enable) {
    // Remove baked POV edge primitives (edge_line/edge_line2 -> open
    // cylinders, tagged fromEdgeMacro) so they do not double-draw against the
    // Freestyle STROKE edges. PER-SECTION policy: a baked edge is dropped ONLY
    // when its group resolves to a section whose stroke EdgeStyle enables ANY
    // drawn nature (Silhouette / Border->Object / Crease slot). A bare
    // "--edges on" enables silhouette+crease+border for every section and so
    // drops all baked POV outlines -- the clean drop-in replacement (strokes in
    // their place). A section whose --edge override disables every nature keeps
    // its baked POV outlines. Capped bonds (open == false,
    // GeomKind::CylinderCapped) are NEVER removed. --keep-baked-edges on
    // suppresses the filter entirely (A/B). Runs only with --edges on; with
    // edges off this block does not execute, so nothing is filtered
    // (byte-identical default).
    if (!opt.keepBakedEdges &&
        (!scene.cylinders.empty() || !scene.spheres.empty())) {
      const std::size_t ng = scene.groupEdgeStyle.size();
      const int kSil = static_cast<int>(umbreon::EdgeClass::Silhouette);
      const int kObj = static_cast<int>(umbreon::EdgeClass::Object);
      const int kCrease = static_cast<int>(umbreon::EdgeClass::Crease);
      auto sectionRemovesBaked = [&](std::uint16_t g) -> bool {
        if (g >= ng) return false;  // unaddressable group: keep baked edges
        const umbreon::EdgeStyle& es = scene.groupEdgeStyle[g];
        return es.cls[kSil].enabled || es.cls[kObj].enabled ||
               es.cls[kCrease].enabled;
      };
      const std::size_t before = scene.cylinders.size();
      scene.cylinders.erase(
          std::remove_if(scene.cylinders.begin(), scene.cylinders.end(),
                         [&](const umbreon::Cylinder& c) {
                           // Never drop capped bonds or non-edge cylinders.
                           return c.fromEdgeMacro && c.open &&
                                  sectionRemovesBaked(c.group);
                         }),
          scene.cylinders.end());
      const std::size_t removed = before - scene.cylinders.size();
      // Also drop the baked silhouette JOINT-DOT spheres (writePoint, tagged
      // fromEdgeMacro) under the SAME per-section policy: CueMol rounds each
      // edge polyline joint with a small black sphere, so removing only the
      // cylinders leaves those spheres as black beads sitting on top of the
      // stroke edges. Atom balls are not fromEdgeMacro and are never touched.
      const std::size_t beforeS = scene.spheres.size();
      scene.spheres.erase(
          std::remove_if(scene.spheres.begin(), scene.spheres.end(),
                         [&](const umbreon::Sphere& s) {
                           return s.fromEdgeMacro &&
                                  sectionRemovesBaked(s.group);
                         }),
          scene.spheres.end());
      const std::size_t removedS = beforeS - scene.spheres.size();
      if (removed > 0 || removedS > 0)
        std::printf(
            "  baked POV edges removed: %zu cylinders, %zu joint dots "
            "(sections with a stroke nature enabled)\n",
            removed, removedS);
    }
  }
}

void applyShadingOptions(const Options& opt, const Scene& scene,
                         RenderOptions& ropt) {
  // Single-layer transparency controls.
  ropt.transparency = opt.transparency;
  ropt.transparentBackground = opt.transparentBackground;

  // Ambient occlusion (both paths). Off unless --ao-samples > 0; the radius is
  // an explicit --ao-distance, else the scene-scaled default (scene builder for
  // .inc, geometry bounds for .pov). AO darkens only the ambient term.
  ropt.aoSamples = opt.aoSamples;
  ropt.aoIntensity = opt.aoIntensity;
  ropt.aoDistance = (opt.aoDistance > 0.0f) ? opt.aoDistance : scene.aoDistance;
  ropt.aoFalloffPower = opt.aoFalloffPower;
  ropt.aoMultiScale = opt.aoMultiScale;
  ropt.aoBentNormal = opt.aoBentNormal;
  for (int i = 0; i < 3; ++i) {
    ropt.aoSkyColor[i] = opt.aoSkyColor[i];
    ropt.aoGroundColor[i] = opt.aoGroundColor[i];
    ropt.aoUp[i] = opt.aoUp[i];
  }
  ropt.aoUseCameraUp = opt.aoUseCameraUp;
  ropt.aoMultibounce = opt.aoMultibounce;
  ropt.aoLowDiscrepancy = opt.aoLowDiscrepancy;
  ropt.aoDiffuseFactor = opt.aoDiffuseFactor;
  ropt.aoWriteAov = opt.aoWriteAov;
  // Coarse-grid AO: -1 "out" sentinel is resolved to the supersample factor
  // (and gi-checked) inside renderFrame.
  ropt.aoResDiv = opt.aoResDiv;
  ropt.aoResDebug = opt.aoResDebug;
  ropt.shadows = opt.shadows;
  ropt.shadowSamples = opt.shadowSamples;
  ropt.lightRadius = opt.lightRadius;
  ropt.envLights = opt.envLights;
  ropt.envIntensity = opt.envIntensity;
  ropt.envKeyScale = opt.envKeyScale;
  ropt.envAngle = opt.envAngle;

  // Diffuse GI: surface irradiance cache (steps 1-3: cache build + fill +
  // debug AOVs; the final composite is not wired yet, so color is unchanged).
  ropt.gi = opt.gi;
  ropt.giSamples = opt.giSamples;
  ropt.giBounces = opt.giBounces;
  ropt.giMaxDistance = opt.giMaxDistance;
  ropt.giIntensity = opt.giIntensity;
  ropt.giEnvIntensity = opt.giEnvIntensity;
  ropt.giAccuracy = opt.giAccuracy;
  ropt.giRecordSpacing = opt.giRecordSpacing;
  ropt.giNormalReject = opt.giNormalReject;
  ropt.giComponentReject = opt.giComponentReject;
  ropt.giSeedPerVertex = opt.giSeedPerVertex;
  ropt.giGradients = opt.giGradients;
  ropt.giOutlierReject = opt.giOutlierReject;
  // pt1 integrator selection + knobs. pt1 is the default; giIntegrator == 0
  // selects the experimental irradiance cache instead, and the pt1 fields are
  // then never read.
  ropt.giIntegrator = opt.giIntegrator;
  ropt.pt1Spp = opt.pt1Spp;
  ropt.pt1HalfRes = opt.pt1HalfRes;
  ropt.pt1GatherDiv = opt.pt1GatherDiv;
  ropt.pt1EdgePatch = opt.pt1EdgePatch;
  ropt.pt1EdgePatchThresh = opt.pt1EdgePatchThresh;
  ropt.pt1Denoise = opt.pt1Denoise;
  ropt.pt1Seed = opt.pt1Seed;
  ropt.pt1SkyMode = opt.pt1SkyMode;
  for (int i = 0; i < 3; ++i) ropt.pt1SkyRadiance[i] = opt.pt1SkyRadiance[i];
  ropt.pt1UpsampleNormalPow = opt.pt1UpsampleNormalPow;
  ropt.pt1UpsampleDepthScale = opt.pt1UpsampleDepthScale;
  ropt.pt1Ld = opt.pt1Ld;
  ropt.pt1Clamp = opt.pt1Clamp;
  ropt.pt1Stats = opt.pt1Stats;
  // pt2 knobs (giIntegrator == 2; layered onto the same gather pass).
  ropt.pt2Pattern = opt.pt2Pattern;
  ropt.pt2Emissive = opt.pt2Emissive;
  ropt.pt2Rounds = opt.pt2Rounds;
  ropt.pt2Radius = opt.pt2Radius;
  ropt.pt2Unbiased = opt.pt2Unbiased;
  ropt.pt2MCap = opt.pt2MCap;
  ropt.pt2WClamp = opt.pt2WClamp;
  ropt.pt2Adaptive = opt.pt2Adaptive;
  ropt.pt2AdaptiveThresh = opt.pt2AdaptiveThresh;
  ropt.pt2AdaptiveMul = opt.pt2AdaptiveMul;
  // GI-conditional denoise default: unset (-1) becomes atrous when the CACHE
  // integrator runs GI, None otherwise. An explicit --denoiser (0/1/2) is
  // honored as-is. On the pt1/pt2 paths the default is None: they denoise
  // their indirect irradiance buffer themselves (--denoise, pre-composite),
  // so a final-color denoise on top would smooth the same signal twice.
  ropt.denoiser =
      opt.denoiser >= 0
          ? opt.denoiser
          : ((ropt.gi && ropt.giIntegrator == 0) ? 1 : 0);
  ropt.denoiseIters = opt.denoiseIters;
  ropt.denoiseSigmaZ = opt.denoiseSigmaZ;
  ropt.denoiseSigmaN = opt.denoiseSigmaN;
  ropt.denoiseSigmaL = opt.denoiseSigmaL;
  ropt.denoiseDemodulateAlbedo = opt.denoiseDemodulateAlbedo;
  ropt.oidnCleanAux = opt.oidnCleanAux;
  ropt.oidnMaxMemoryMB = opt.oidnMaxMemoryMB;
  if (ropt.gi && ropt.giIntegrator >= 1) {
    // Gather-grid label: explicit divisor / "out" sentinel / legacy
    // pt1HalfRes-derived (see RenderOptions::pt1GatherDiv).
    char gridDesc[32];
    if (ropt.pt1GatherDiv < 0)
      std::snprintf(gridDesc, sizeof(gridDesc), "out");
    else if (ropt.pt1GatherDiv == 0)
      std::snprintf(gridDesc, sizeof(gridDesc), "%s",
                    ropt.pt1HalfRes ? "half" : "full");
    else if (ropt.pt1GatherDiv == 1)
      std::snprintf(gridDesc, sizeof(gridDesc), "full");
    else
      std::snprintf(gridDesc, sizeof(gridDesc), "1/%d", ropt.pt1GatherDiv);
    if (ropt.giIntegrator == 2)
      std::printf(
          "  diffuse GI: pt2 path-traced gather, %d spp, %d bounce%s, %s res, "
          "%s sampling, restir %dx%s, emissive %s, denoise %s, "
          "intensity %.2f, env %.2f\n",
          ropt.pt1Spp, ropt.giBounces, ropt.giBounces > 1 ? "s" : "",
          gridDesc, ropt.pt2Pattern == 1 ? "blue-noise" : "sobol",
          ropt.pt2Rounds, ropt.pt2Unbiased ? " unbiased" : "",
          ropt.pt2Emissive ? "on" : "off",
          ropt.pt1Denoise ? "on" : "off", ropt.giIntensity,
          ropt.giEnvIntensity);
    else
      std::printf(
          "  diffuse GI: pt1 path-traced gather, %d spp, %d bounce%s, %s res, "
          "ld %s, denoise %s, intensity %.2f, env %.2f\n",
          ropt.pt1Spp, ropt.giBounces, ropt.giBounces > 1 ? "s" : "",
          gridDesc, ropt.pt1Ld ? "on" : "off",
          ropt.pt1Denoise ? "on" : "off", ropt.giIntensity,
          ropt.giEnvIntensity);
  }
  else if (ropt.gi)
    std::printf(
        "  diffuse GI: irradiance cache, %d samples/record, intensity %.2f, "
        "env %.2f%s\n",
        ropt.giSamples, ropt.giIntensity, ropt.giEnvIntensity,
        ropt.giSeedPerVertex ? " (per-vertex seed)" : "");
  if (ropt.aoSamples > 0)
    std::printf(
        "  ambient occlusion: %d samples, radius %.3f, intensity %.2f%s\n",
        ropt.aoSamples, ropt.aoDistance, ropt.aoIntensity,
        ropt.aoResDiv != 0 ? ", coarse grid (--ao-res out)" : "");
  if (ropt.shadows) {
    if (ropt.lightRadius > 0.0f && ropt.shadowSamples > 1)
      std::printf("  soft shadows: %d samples, light radius %.2f deg\n",
                  ropt.shadowSamples, ropt.lightRadius);
    else
      std::printf("  hard shadows: on\n");
  }

  // Supersampling factor. umbreon::render() renders at ss x the output
  // resolution so the thin silhouette lines antialias like POV-Ray; the .pov
  // path defaults to 3x (a 3x3 box matches POV-Ray's Antialias_Depth=3 edge
  // coverage best).
  const int povDefaultSs = 3;
  const int ss = std::max(
      1, !opt.supersampleSet ? povDefaultSs : opt.supersample);
  ropt.supersample = ss;
  const int finalW = ropt.width, finalH = ropt.height;
  if (ss > 1)
    std::printf("  supersample %dx (%dx%d -> %dx%d)\n", ss, finalW * ss,
                finalH * ss, finalW, finalH);

  // Adaptive antialiasing (--aa adaptive): center-sample + discontinuity-
  // driven refinement instead of the full supersample grid. render() falls
  // back to grid for --gi (unvalidated combination, warns on stderr).
  ropt.aaMode = opt.aaMode;
  ropt.aaThreshold = opt.aaThreshold;
  ropt.aaDepth = opt.aaDepth;
  ropt.aaDebug = opt.aaDebug;
  if (opt.aaMode == 1)
    std::printf("  antialias: adaptive (threshold %.3g, depth %d%s)\n",
                opt.aaThreshold, opt.aaDepth > 0 ? opt.aaDepth : ss,
                opt.aaDebug ? ", mask dump" : "");
}

void applyObjectSpaceEdges(const Options& opt, Scene& scene,
                           RenderOptions& ropt) {
  if (opt.objEdges) {
    // Replace the baked POV outlines with generated ones: drop the baked
    // edge_line cylinders AND their joint-dot spheres (both fromEdgeMacro) so
    // the generated edges do not double-draw against them. --keep-baked-edges
    // suppresses this for an A/B comparison.
    if (!opt.keepBakedEdges) {
      const std::size_t bc = scene.cylinders.size(), bs = scene.spheres.size();
      scene.cylinders.erase(
          std::remove_if(scene.cylinders.begin(), scene.cylinders.end(),
                         [](const umbreon::Cylinder& c) { return c.fromEdgeMacro; }),
          scene.cylinders.end());
      scene.spheres.erase(
          std::remove_if(scene.spheres.begin(), scene.spheres.end(),
                         [](const umbreon::Sphere& s) { return s.fromEdgeMacro; }),
          scene.spheres.end());
      const std::size_t rc = bc - scene.cylinders.size();
      const std::size_t rs = bs - scene.spheres.size();
      if (rc > 0 || rs > 0)
        std::printf("  baked POV edges removed: %zu cylinders, %zu joint dots\n", rc, rs);
    }
    umbreon::ObjectSpaceEdgeOptions silOpt;
    silOpt.enable = true;
    silOpt.width = opt.objEdgeWidth;
    silOpt.raise = opt.objEdgeRaise;
    silOpt.segments = opt.objEdgeSegments;
    silOpt.clip = opt.objEdgeClip;
    silOpt.creaseAngleDeg = opt.objEdgeCreaseDeg;
    silOpt.meshHardEdgeDeg = opt.objEdgeHardDeg;
    silOpt.meshSilhouette = opt.objEdgeMeshSil;
    silOpt.meshCrease = opt.objEdgeMeshCrease;
    silOpt.meshBorder = opt.objEdgeMeshBorder;
    silOpt.visibilityClip = opt.objEdgeVisibility;
    silOpt.meshCreaseSmoothVetoDeg = opt.objEdgeCreaseSmoothDeg;
    silOpt.meshCreaseConvexOnly = opt.objEdgeCreaseConvexOnly;
    silOpt.meshBorderCoplanarVetoDeg = opt.objEdgeBorderCoplanarDeg;
    silOpt.meshCreaseMaxDegree = opt.objEdgeCreaseMaxDeg;
    for (int k = 0; k < 3; ++k) silOpt.color[k] = opt.objEdgeColor[k];
    if (opt.objEdgeOnly) {
      // Verification path: render ONLY the generated edge cylinders. The
      // silhouette extraction needs the surfaces present, but they must be
      // gone before the trace -- the inverse ordering of the integrated pass.
      // So here (and only here) we call the extraction directly, strip the
      // surfaces, and leave the integrated pass disabled (edges are already in
      // scene.cylinders). The render()-driven production path is the else branch.
      const std::size_t before = scene.cylinders.size();
      umbreon::generateObjectSpaceEdges(scene, silOpt);
      scene.cylinders.erase(
          scene.cylinders.begin(),
          scene.cylinders.begin() + static_cast<std::ptrdiff_t>(before));
      scene.spheres.clear();
      scene.mesh = umbreon::Mesh{};
      std::printf("  --obj-edge-only: rendering %zu edge cylinders only\n",
                  scene.cylinders.size());
    } else {
      // Production path: hand the options to render(), symmetric with method A.
      ropt.objectSpaceEdges = silOpt;
      std::printf(
          "  object-space silhouette edges: via render() "
          "(width %.3f, raise %.3f, segments %d)\n",
          silOpt.width, silOpt.raise, silOpt.segments);
    }
  }
}

}  // namespace umbreon
