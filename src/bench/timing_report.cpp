#include "timing_report.hpp"

#include <cstdio>
#include <exception>
#include <filesystem>

namespace umbreon {

// pt1 stage timing: table to stdout + outputs/timing.json. `direct` is 0 by
// architecture (direct shading is fused into the primary loop, reported under
// `primary`); `total` is the wall time of render().
void reportPt1Timing(const RenderOptions& ropt, const FrameResult& frame,
                     double totalSeconds) {
  if (!(ropt.gi && ropt.giIntegrator >= 1)) return;  // pt1 and pt2 both report
  umbreon::Pt1Timing t = frame.pt1Timing;
  t.total = totalSeconds;
  std::printf(
      "  pt1 timing: bvh_build %.3f  primary %.3f  direct %.3f  gather "
      "%.3f  denoise %.3f  upsample %.3f  total %.3f (s)\n",
      t.bvhBuild, t.primary, t.direct, t.gather, t.denoise, t.upsample,
      t.total);
  try {
    std::filesystem::create_directories("outputs");
    if (std::FILE* jf = std::fopen("outputs/timing.json", "w")) {
      const umbreon::Pt1RayCounts& rc = frame.pt1Rays;
      const double totalRays = static_cast<double>(
          rc.gatherRays + rc.neeRays + rc.gbufferRays);
      std::fprintf(jf,
                   "{\n"
                   "  \"bvh_build\": %.6f,\n"
                   "  \"primary\": %.6f,\n"
                   "  \"direct\": %.6f,\n"
                   "  \"gather\": %.6f,\n"
                   "  \"denoise\": %.6f,\n"
                   "  \"upsample\": %.6f,\n"
                   "  \"total\": %.6f,\n"
                   "  \"rays_gather\": %llu,\n"
                   "  \"rays_nee\": %llu,\n"
                   "  \"rays_gbuffer\": %llu,\n"
                   "  \"nee_fraction\": %.4f,\n"
                   "  \"mrays_per_sec\": %.2f,\n"
                   "  \"note\": \"direct shading is fused into the "
                   "primary-ray loop; its cost is under 'primary'\"\n"
                   "}\n",
                   t.bvhBuild, t.primary, t.direct, t.gather, t.denoise,
                   t.upsample, t.total,
                   static_cast<unsigned long long>(rc.gatherRays),
                   static_cast<unsigned long long>(rc.neeRays),
                   static_cast<unsigned long long>(rc.gbufferRays),
                   totalRays > 0.0 ? rc.neeRays / totalRays : 0.0,
                   t.gather > 0.0 ? totalRays / t.gather / 1.0e6 : 0.0);
      std::fclose(jf);
      std::printf("  wrote outputs/timing.json\n");
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "warning: could not write outputs/timing.json (%s)\n",
                 e.what());
  }
}

}  // namespace umbreon
