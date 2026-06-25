// umbreon CLI: read a CueMol .pov scene (or a legacy .inc mesh), render it with
// the umbreon Embree backend, and write the image. Also provides PPM compare
// and PPM->PNG convert utility modes.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <tbb/global_control.h>
#include <tbb/version.h>

#include "cli.hpp"
#include "geom/mesh2_reader.hpp"
#include "geom/scene_builder.hpp"
#include "image/image_io.hpp"
#include "pov/pov_scene_reader.hpp"
#include "povexport/pov_writer.hpp"
#include "render/object_space_edges.hpp"
#include "umbreon.hpp"

namespace {
bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Resolve a path referenced from inside a .pov file relative to that file's
// directory (POV resolves #include paths this way).
std::string resolveRelative(const std::string& base, const std::string& rel) {
  if (!rel.empty() && rel[0] == '/') return rel;  // already absolute
  std::size_t slash = base.find_last_of('/');
  std::string dir = (slash == std::string::npos) ? "." : base.substr(0, slash);
  return dir + "/" + rel;
}

}  // namespace

int main(int argc, char** argv) {
  umbreon::Options opt = umbreon::parseCli(argc, argv);
  if (opt.showHelp) {
    umbreon::printUsage(argv[0]);
    return 0;
  }
  if (!opt.ok) {
    std::fprintf(stderr, "error: %s\n\n", opt.error.c_str());
    umbreon::printUsage(argv[0]);
    return 2;
  }

  // Convert mode: read a PPM, write it in the requested format, and exit.
  if (opt.convertMode) {
    try {
      umbreon::ImageRGB img = umbreon::readPpm(opt.convertIn);
      umbreon::writeImage(opt.convertOut, img.width, img.height, img.rgb.data(), 3);
      std::printf("converted %s -> %s\n", opt.convertIn.c_str(),
                  opt.convertOut.c_str());
    } catch (const std::exception& e) {
      std::fprintf(stderr, "error: %s\n", e.what());
      return 1;
    }
    return 0;
  }

  // Image-compare mode: print metrics between two PPM files and exit.
  if (opt.compareMode) {
    try {
      umbreon::ImageRGB a = umbreon::readPpm(opt.compareA);
      umbreon::ImageRGB b = umbreon::readPpm(opt.compareB);
      umbreon::CompareResult cr = umbreon::compareImages(a, b);
      std::printf("PSNR %.2f dB   SSIM %.4f   (%s vs %s)\n", cr.psnr, cr.ssim,
                  opt.compareA.c_str(), opt.compareB.c_str());
    } catch (const std::exception& e) {
      std::fprintf(stderr, "error: %s\n", e.what());
      return 1;
    }
    return 0;
  }

  try {
    const bool povMode = endsWith(opt.input, ".pov");
    umbreon::Scene scene;
    umbreon::RenderOptions ropt;
    // Section names recovered by the .pov geometry parser (empty for the .inc
    // path); used to resolve --edge / per-section styling after the input block.
    std::vector<std::string> groupNames;

    if (povMode) {
      // ---- .pov path: reproduce the CueMol POV-Ray viewing setup. ----
      const int W = opt.widthSet ? opt.width : 300;
      const int H = opt.heightSet ? opt.height : 300;

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
        return 0;
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

      // Apply per-section opacity overrides (--alpha ID=value). The section is
      // resolved against the group names recovered by the parser; every
      // primitive in that group gets the given opacity so the renderer
      // composites it as one transparent layer.
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
          // Group alpha MULTIPLIES the intrinsic (fragment) opacity, so the two
          // are orthogonal: group 0.5 x fragment 0.5 = 0.25. A fully opaque
          // fragment (1.0) just becomes the group alpha (no change vs before).
          for (std::size_t t = 0; t < scene.mesh.triangleCount(); ++t)
            if (scene.mesh.groupForTri(t) == g)
              for (int k = 0; k < 3; ++k) scene.mesh.colors[3 * t + k].w *= a;
          for (umbreon::Sphere& s : scene.spheres)
            if (s.group == g) s.color.w *= a;
          for (umbreon::Cylinder& c : scene.cylinders)
            if (c.group == g) c.color.w *= a;
          // Mark the section as an additive single-layer "veil" (group alpha).
          scene.veilGroups.push_back(static_cast<uint16_t>(g));
          std::printf("  alpha override: %s *= %.3f (group %d, veil)\n",
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

      scene.ambientColor = umbreon::Vec3{1.0f, 1.0f, 1.0f};

      for (umbreon::DistantLight& L : scene.lights) L.intensity *= opt.povGain;
      scene.ambientIntensity = opt.povGain;

      scene.assumedGamma = ps.assumedGamma;

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
            "    fog: type %d  distance %.3f  color <%.3f,%.3f,%.3f>  "
            "offset %.3f  up <%.2f,%.2f,%.2f>\n",
            scene.fog.type, scene.fog.distance, scene.fog.color.x,
            scene.fog.color.y, scene.fog.color.z, scene.fog.offset,
            scene.fog.up.x, scene.fog.up.y, scene.fog.up.z);
      }
      std::printf("    assumed_gamma %.3f\n", scene.assumedGamma);

      ropt.width = W;
      ropt.height = H;
      // Per-material POV finishes drive specular now (F_MetalA, phong groups),
      // so the .pov path keeps the RenderOptions specularScale default (1.0)
      // unless the user overrides it explicitly.
      if (opt.specularScaleSet) ropt.specularScale = opt.specularScale;
      std::printf("rendering %dx%d  backend=umbreon (embree)\n", W, H);
    } else {
      // ---- .inc path: legacy auto-framed scene with the instance grid. ----
      std::printf("reading mesh2 from %s\n", opt.input.c_str());
      umbreon::Mesh mesh = umbreon::readMesh2FromFile(opt.input);
      std::printf("  %zu triangles, %zu vertices (de-indexed)\n",
                  mesh.triangleCount(), mesh.vertexCount());

      umbreon::BuildOptions bopt;
      bopt.gridN = opt.gridN;
      bopt.spacing = opt.spacing;
      bopt.lightIntensity = opt.lightIntensity;
      bopt.ambientIntensity = opt.ambientIntensity;
      scene = umbreon::buildScene(mesh, bopt);
      std::printf("  grid %d^3 = %zu instances, %zu effective triangles\n",
                  opt.gridN, scene.instanceCount(), scene.effectiveTriangles());

      // Optionally emit an equivalent POV-Ray scene for benchmarking.
      if (!opt.emitPov.empty()) {
        umbreon::PovWriteOptions povOpt;
        povOpt.width = opt.width;
        povOpt.height = opt.height;
        povOpt.radiosity = opt.povRadiosity;
        umbreon::writePovScene(opt.emitPov, scene, povOpt);
        std::printf("wrote POV-Ray scene %s (radiosity %s)\n",
                    opt.emitPov.c_str(), opt.povRadiosity ? "on" : "off");
      }

      ropt.width = opt.width;
      ropt.height = opt.height;
      std::printf("rendering %dx%d\n", ropt.width, ropt.height);
    }

    // Single-layer transparency controls (apply to both input paths).
    ropt.transparency = opt.transparency;
    ropt.transparentBackground = opt.transparentBackground;

    // Freestyle STROKE edges (--edges). The single strokeEdges.enable flag is the
    // master gate for the whole --edges pipeline (G-buffer AOV capture, the
    // stroke pass, and the baked-edge removal below). Off => byte-identical
    // default path. The --stroke-* knobs are wired only when edges are on.
    ropt.strokeEdges.enable = opt.edges;
    if (opt.edges) {
      ropt.strokeEdges.silhouette = opt.strokeSilhouette;
      ropt.strokeEdges.crease = opt.strokeCrease;
      ropt.strokeEdges.border = opt.strokeBorder;
      if (opt.strokeThicknessSet)
        ropt.strokeEdges.thickness =
            static_cast<int>(opt.strokeThickness + 0.5f);
      if (opt.strokeResampleSet)
        ropt.strokeEdges.resampleStepPx =
            static_cast<int>(opt.strokeResample + 0.5f);
      if (opt.strokeCreaseDegSet)
        ropt.strokeEdges.creaseAngleDeg = opt.strokeCreaseDeg;

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
      if (!opt.keepBakedEdges && !scene.cylinders.empty()) {
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
        if (removed > 0)
          std::printf(
              "  baked POV edges removed: %zu cylinders (sections with a "
              "stroke nature enabled)\n",
              removed);
      }
    }

    // Ambient occlusion (both paths). Off unless --ao-samples > 0; the radius is
    // an explicit --ao-distance, else the scene-scaled default (scene builder for
    // .inc, geometry bounds for .pov). AO darkens only the ambient term.
    ropt.aoSamples = opt.aoSamples;
    ropt.aoIntensity = opt.aoIntensity;
    ropt.aoDistance = (opt.aoDistance > 0.0f) ? opt.aoDistance : scene.aoDistance;
    ropt.shadows = opt.shadows;
    ropt.shadowSamples = opt.shadowSamples;
    ropt.lightRadius = opt.lightRadius;
    if (ropt.aoSamples > 0)
      std::printf(
          "  ambient occlusion: %d samples, radius %.3f, intensity %.2f\n",
          ropt.aoSamples, ropt.aoDistance, ropt.aoIntensity);
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
        1, (povMode && !opt.supersampleSet) ? povDefaultSs : opt.supersample);
    ropt.supersample = ss;
    const int finalW = ropt.width, finalH = ropt.height;
    if (ss > 1)
      std::printf("  supersample %dx (%dx%d -> %dx%d)\n", ss, finalW * ss,
                  finalH * ss, finalW, finalH);

    // Optional TBB parallelism cap for a no-rebuild speed comparison:
    // --threads 1 runs the row-parallel render serially, --threads N caps at N,
    // 0 leaves TBB at its default (all cores). global_control must outlive the
    // render() call below, so keep it alive in this scope.
    std::unique_ptr<tbb::global_control> tbbLimit;
    if (opt.threads > 0)
      tbbLimit = std::make_unique<tbb::global_control>(
          tbb::global_control::max_allowed_parallelism,
          static_cast<std::size_t>(opt.threads));

    // Analytic OBJECT-SPACE silhouette edges. The silhouette is camera-dependent,
    // so this runs only now -- after scene.camera is assigned and the scene is
    // fully assembled (geometry, per-section alpha, screen-space edge filtering) --
    // and strictly BEFORE render(). It computes each sphere/cylinder's n.v==0
    // contour in 3D and APPENDS it as thin flat-black open cylinders, so the ray
    // tracer below handles visibility/occlusion/AA/fog for the edges for free.
    // Off (the default) => nothing appended => byte-identical default render.
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
      silOpt.meshCreaseSmoothVetoDeg = opt.objEdgeCreaseSmoothDeg;
      silOpt.meshCreaseConvexOnly = opt.objEdgeCreaseConvexOnly;
      silOpt.meshBorderCoplanarVetoDeg = opt.objEdgeBorderCoplanarDeg;
      silOpt.meshCreaseMaxDegree = opt.objEdgeCreaseMaxDeg;
      for (int k = 0; k < 3; ++k) silOpt.color[k] = opt.objEdgeColor[k];
      const std::size_t before = scene.cylinders.size();
      umbreon::generateObjectSpaceEdges(scene, silOpt);
      std::printf(
          "  object-space silhouette edges: +%zu cylinders "
          "(width %.3f, raise %.3f, segments %d)\n",
          scene.cylinders.size() - before, silOpt.width, silOpt.raise,
          silOpt.segments);
    }

    // umbreon backend: primary rays + direct POV local shading + fog + gamma.
    // This is exactly what CueMol will call (no POV-Ray SDL involved).
    // Report the TBB runtime backing the parallel render: the renderer
    // parallelizes image rows with tbb::parallel_for (and Embree's tasking uses
    // TBB too). TBB_runtime_version() reflects the actually-linked library;
    // max parallelism reflects the --threads cap above.
    std::printf(
        "  parallel backend: TBB %s (interface %d), max parallelism %zu\n",
        TBB_runtime_version(), TBB_runtime_interface_version(),
        tbb::global_control::active_value(
            tbb::global_control::max_allowed_parallelism));
    umbreon::FrameResult frame = umbreon::render(scene, ropt);
    std::printf("  render time:  %.3f s\n", frame.renderSeconds);
    if (scene.fog.enabled)
      std::printf("  applied fog (type %d, distance %.3f)\n", scene.fog.type,
                  scene.fog.distance);
    if (povMode && std::fabs(scene.assumedGamma - 1.0f) > 1.0e-4f)
      std::printf("  applied assumed_gamma %.3f\n", scene.assumedGamma);

    umbreon::writeImage(opt.output, frame.width, frame.height,
                        frame.color.data(), 4);
    std::printf("wrote %s\n", opt.output.c_str());

    // Debug AOV dump (verification only): false-color the G-buffer the edge pass
    // captured. Requires --edges on (otherwise the AOVs are empty). The edge
    // AOVs stay at the SUPERSAMPLE resolution (hiW x hiH), unlike the downsampled
    // color, so dump at those dims (recovered from the buffer size).
    if (!opt.dumpAovPrefix.empty()) {
      if (!ropt.strokeEdges.enable) {
        std::fprintf(stderr,
                     "warning: --dump-aov ignored (needs --edges on)\n");
      } else if (frame.objectId.empty()) {
        std::fprintf(stderr, "warning: --dump-aov: no edge AOVs captured\n");
      } else {
        const int hiW = finalW * ss;
        const int hiH = finalH * ss;
        const std::size_t np = static_cast<std::size_t>(hiW) * hiH;
        const uint32_t kBg = 0xFFFFFFFFu;
        // Deterministic id -> RGB false color (background sentinel -> black).
        auto idColor = [&](uint32_t id, float* rgb) {
          if (id == kBg) { rgb[0] = rgb[1] = rgb[2] = 0.0f; return; }
          uint32_t h = id * 2654435761u;  // Knuth multiplicative hash
          rgb[0] = ((h >> 16) & 0xFF) / 255.0f;
          rgb[1] = ((h >> 8) & 0xFF) / 255.0f;
          rgb[2] = (h & 0xFF) / 255.0f;
        };
        std::vector<float> oimg(np * 3), mimg(np * 3), nimg(np * 3), zimg(np * 3);
        // viewZ range over real (non-background) hits for normalization.
        float zmin = 1e30f, zmax = -1e30f;
        for (std::size_t i = 0; i < np; ++i) {
          if (frame.objectId[i] == kBg) continue;
          const float z = frame.viewZ[i];
          zmin = std::min(zmin, z);
          zmax = std::max(zmax, z);
        }
        const float zspan = (zmax > zmin) ? (zmax - zmin) : 1.0f;
        for (std::size_t i = 0; i < np; ++i) {
          idColor(frame.objectId[i], &oimg[i * 3]);
          idColor(frame.materialId[i], &mimg[i * 3]);
          // normal*0.5+0.5 (background = the zero vector -> mid grey).
          for (int c = 0; c < 3; ++c)
            nimg[i * 3 + c] = frame.normal[i * 3 + c] * 0.5f + 0.5f;
          // Normalized view-z (near=0..far=1); background sentinel -> black.
          float zn = 0.0f;
          if (frame.objectId[i] != kBg)
            zn = (frame.viewZ[i] - zmin) / zspan;
          zimg[i * 3 + 0] = zimg[i * 3 + 1] = zimg[i * 3 + 2] = zn;
        }
        umbreon::writeImage(opt.dumpAovPrefix + "_objectId.png", hiW, hiH,
                            oimg.data(), 3);
        umbreon::writeImage(opt.dumpAovPrefix + "_materialId.png", hiW, hiH,
                            mimg.data(), 3);
        umbreon::writeImage(opt.dumpAovPrefix + "_normal.png", hiW, hiH,
                            nimg.data(), 3);
        umbreon::writeImage(opt.dumpAovPrefix + "_viewZ.png", hiW, hiH,
                            zimg.data(), 3);
        std::printf("  dumped AOVs: %s_{objectId,materialId,normal,viewZ}.png "
                    "(%dx%d)\n",
                    opt.dumpAovPrefix.c_str(), hiW, hiH);
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
