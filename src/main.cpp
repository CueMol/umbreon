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
    float povAssumedGamma = 1.0f;  // POV global_settings assumed_gamma (pov path)

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

      scene.mesh = std::move(geo.mesh);
      scene.spheres = std::move(geo.spheres);
      scene.cylinders = std::move(geo.cylinders);
      if (opt.outlineScale != 1.0f) {
        for (umbreon::Sphere& s : scene.spheres) s.radius *= opt.outlineScale;
        for (umbreon::Cylinder& c : scene.cylinders) c.radius *= opt.outlineScale;
      }
      scene.camera = ps.camera;
      scene.lights = ps.lights;
      scene.background = ps.background;
      scene.fog = ps.fog;

      // umbreon reproduces POV local illumination directly: POV-native light
      // intensities (no pi factor); the shader applies ambient =
      // material.ambient * pigment with ambient_light <1,1,1>.
      for (umbreon::DistantLight& L : scene.lights) L.intensity *= opt.povGain;
      scene.ambientIntensity = opt.povGain;  // POV ambient_light radiance
      scene.ambientColor = umbreon::Vec3{1.0f, 1.0f, 1.0f};

      povAssumedGamma = ps.assumedGamma;

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
      std::printf("    assumed_gamma %.3f\n", povAssumedGamma);

      ropt.width = W;
      ropt.height = H;
      ropt.renderer = "embree";
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
      ropt.aoSamples = opt.aoSamples;
      ropt.aoDistance =
          (opt.aoDistance > 0.0f) ? opt.aoDistance : scene.aoDistance;
      ropt.spp = opt.spp;
      ropt.accumFrames = opt.accumFrames;
      ropt.flatten = opt.flatten;
      ropt.flipNormals = opt.flipNormals;
      ropt.renderer = opt.renderer;
      std::printf("rendering %dx%d  aoSamples=%d  aoDistance=%.3f  accum=%d\n",
                  ropt.width, ropt.height, ropt.aoSamples, ropt.aoDistance,
                  ropt.accumFrames);
    }

    // Supersampling factor. umbreon::render() renders at ss x the output
    // resolution so the thin silhouette lines antialias like POV-Ray; the .pov
    // path defaults to 3x (a 3x3 box matches POV-Ray's Antialias_Depth=3 edge
    // coverage best).
    int povDefaultSs = 2;
    if (ropt.renderer == "embree") povDefaultSs = 3;
    const int ss = std::max(
        1, (povMode && !opt.supersampleSet) ? povDefaultSs : opt.supersample);
    ropt.supersample = ss;
    ropt.assumedGamma = povMode ? povAssumedGamma : 1.0f;
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
    if (povMode && std::fabs(povAssumedGamma - 1.0f) > 1.0e-4f)
      std::printf("  applied assumed_gamma %.3f\n", povAssumedGamma);

    umbreon::writeImage(opt.output, frame.width, frame.height,
                        frame.color.data(), 4);
    std::printf("wrote %s\n", opt.output.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
