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
#include <thread>
#include <vector>

#include <tbb/global_control.h>
#include <tbb/version.h>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

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

// True when stderr is an interactive terminal, so the in-place progress bar
// (which uses '\r') is worth drawing; a pipe/file/CI run gets no bar.
bool stderrIsTty() {
#if defined(_WIN32)
  return _isatty(_fileno(stderr)) != 0;
#else
  return isatty(fileno(stderr)) != 0;
#endif
}

// Redraw the single-line progress bar on stderr (leading '\r', no newline).
void drawProgressBar(float frac, umbreon::RenderPhase phase) {
  frac = std::fmax(0.0f, std::fmin(1.0f, frac));
  constexpr int kWidth = 24;
  const int filled = static_cast<int>(frac * kWidth + 0.5f);
  char bar[kWidth + 1];
  for (int i = 0; i < kWidth; ++i) bar[i] = (i < filled) ? '#' : '-';
  bar[kWidth] = '\0';
  // %-11s pads the phase name (longest is "Postprocess") so shorter names never
  // leave stale characters behind on the reused line.
  std::fprintf(stderr, "\r  render [%s] %3d%%  %-11s", bar,
               static_cast<int>(frac * 100.0f + 0.5f), umbreon::toString(phase));
  std::fflush(stderr);
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
    // Render asynchronously and poll a text progress bar (TTY only; stderr keeps
    // stdout clean). This is functionally identical to the blocking render() but
    // exercises the async API end-to-end. scene/ropt are copied into the task, so
    // they stay valid for the reporting below.
    const bool showBar = stderrIsTty();
    umbreon::RenderTask task = umbreon::renderAsync(scene, ropt);
    while (!task.wait_for(std::chrono::milliseconds(100))) {
      if (showBar) drawProgressBar(task.progress(), task.phase());
    }
    umbreon::FrameResult frame = task.get();
    if (showBar) {
      drawProgressBar(1.0f, umbreon::RenderPhase::Done);
      std::fprintf(stderr, "\n");
    }
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
