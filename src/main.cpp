// umbreon CLI: read a CueMol .pov scene (or a legacy .inc mesh), render it with
// the umbreon Embree backend, and write the image. Also provides PPM compare
// and PPM->PNG convert utility modes.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "cli.hpp"
#include "geom/mesh2_reader.hpp"
#include "geom/scene_builder.hpp"
#include "image/image_io.hpp"
#include "pov/pov_scene_reader.hpp"
#include "povexport/pov_writer.hpp"
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
      ropt.aoDistance =
          (opt.aoDistance > 0.0f) ? opt.aoDistance : scene.aoDistance;
      ropt.spp = opt.spp;
      ropt.accumFrames = opt.accumFrames;
      ropt.flatten = opt.flatten;
      ropt.flipNormals = opt.flipNormals;
      std::printf("rendering %dx%d  accum=%d\n", ropt.width, ropt.height,
                  ropt.accumFrames);
    }

    // Single-layer transparency controls (apply to both input paths).
    ropt.transparency = opt.transparency;
    ropt.transparentBackground = opt.transparentBackground;

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

    // umbreon backend: primary rays + direct POV local shading + fog + gamma.
    // This is exactly what CueMol will call (no POV-Ray SDL involved).
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
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
