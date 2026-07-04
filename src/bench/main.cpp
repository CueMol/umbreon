// umbreon CLI: read a CueMol .pov scene (which may #include .inc geometry),
// render it with the umbreon Embree backend, and write the image. Also provides
// PPM compare and PPM->PNG convert utility modes.
//
// The CLI-to-renderer translation lives in scene_setup.cpp (Scene /
// RenderOptions wiring), modes.cpp (--convert / --compare), timing_report.cpp
// (pt1 timing) and aov_dump.cpp (debug images); this file is the driver.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <tbb/global_control.h>
#include <tbb/version.h>

#include "aov_dump.hpp"
#include "cli.hpp"
#include "image/image_io.hpp"
#include "modes.hpp"
#include "scene_setup.hpp"
#include "timing_report.hpp"
#include "umbreon.hpp"

namespace {
bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
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

  // --integrator pt1 implies the GI pipeline: the gi gate drives the ambient
  // zeroing, the _amb_frac energy rebalance and the GI AOV plumbing, all of
  // which pt1 shares with the cache. Without this a bare "--integrator pt1"
  // would silently render with no GI at all.
  if (opt.giIntegrator == 1) opt.gi = true;

  if (opt.convertMode) return umbreon::runConvertMode(opt);
  if (opt.compareMode) return umbreon::runCompareMode(opt);

  try {
    if (!endsWith(opt.input, ".pov")) {
      std::fprintf(stderr,
                   "error: input must be a .pov scene file (got '%s')\n",
                   opt.input.c_str());
      return 2;
    }
    umbreon::Scene scene;
    umbreon::RenderOptions ropt;
    // Section names recovered by the .pov geometry parser; used to resolve
    // --edge / per-section styling.
    std::vector<std::string> groupNames;

    // CLI -> Scene / RenderOptions translation (scene_setup.cpp), in the
    // historical block order. buildSceneFromPov returns false when
    // --list-groups printed its report.
    if (!umbreon::buildSceneFromPov(opt, scene, ropt, groupNames)) return 0;
    umbreon::applyEdgeOptions(opt, scene, ropt, groupNames);
    umbreon::applyShadingOptions(opt, scene, ropt);

    // Optional TBB parallelism cap for a no-rebuild speed comparison:
    // --threads 1 runs the row-parallel render serially, --threads N caps at N,
    // 0 leaves TBB at its default (all cores). global_control must outlive the
    // render() call below, so keep it alive in this scope.
    std::unique_ptr<tbb::global_control> tbbLimit;
    if (opt.threads > 0)
      tbbLimit = std::make_unique<tbb::global_control>(
          tbb::global_control::max_allowed_parallelism,
          static_cast<std::size_t>(opt.threads));

    umbreon::applyObjectSpaceEdges(opt, scene, ropt);

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
    const auto tRender0 = std::chrono::high_resolution_clock::now();
    umbreon::FrameResult frame = umbreon::render(scene, ropt);
    const auto tRender1 = std::chrono::high_resolution_clock::now();
    std::printf("  render time:  %.3f s\n", frame.renderSeconds);
    if (frame.aoCoarseSeconds > 0.0)
      std::printf("  coarse AO pre-pass:  %.3f s\n", frame.aoCoarseSeconds);

    umbreon::reportPt1Timing(
        ropt, frame,
        std::chrono::duration<double>(tRender1 - tRender0).count());
    if (scene.fog.enabled)
      std::printf("  applied linear fog (start %.3f, end %.3f)\n",
                  scene.fog.start, scene.fog.end);
    if (std::fabs(scene.assumedGamma - 1.0f) > 1.0e-4f)
      std::printf("  applied assumed_gamma %.3f\n", scene.assumedGamma);

    umbreon::writeImage(opt.output, frame.width, frame.height,
                        frame.color.data(), 4);
    std::printf("wrote %s\n", opt.output.c_str());

    umbreon::dumpDebugImages(opt, ropt, frame);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
