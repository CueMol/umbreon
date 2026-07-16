// Internal: turns resolved RenderOptions into per-phase cost estimates, so
// RenderProgress::fraction() can weight the progress bar by where the time
// actually goes. Implementation detail; may change without notice (not part of
// the installed public API).
//
// Both renderFrame (which declares the phase plan) and EmbreeRenderer (which
// budgets the GI phase across its sub-stages) must agree on these numbers, so
// they live here rather than at either call site.
//
// The constants are wall-clock seconds calibrated on data/1ab0_scene1.pov at
// 1200x1200 (macOS arm64, 8 threads, Release). They are deliberately COARSE.
// Every long phase is instrumented, so a mis-estimate only changes the PACE of
// the bar; it cannot freeze it. Getting the ORDER right -- GI dominates
// whenever it is on -- is what matters, and getting it exact is not possible in
// any case: the ratio between OIDN and Embree throughput is hardware dependent.
#pragma once

#include <algorithm>

#include "render/progress_slice.hpp"
#include "render/render_options.hpp"
#include "render/render_progress.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// --- calibration, in seconds (see the file note on precision) ---

// primary 0.244s over a 12.96 Mpix render grid.
constexpr double kPrimaryPerPixel = 19.0e-9;
// gather 0.263s at spp=8 and 0.925s at spp=32, both over a 1.44 Mpix gather
// grid -> 22.8 / 20.1 ns per sample-pixel. Deliberately NO giBounces factor:
// one constant fits both bounces=1 and bounces=2, because continuation rays
// fire only on a hit and the second bounce hits far less often. A spp*bounces
// term would make this ~2x worse, not better.
constexpr double kGatherPerSamplePixel = 22.0e-9;
// OIDN 1.08-1.16s over a 1.44 Mpix gather grid, and spp-INDEPENDENT: the
// network cost is per pixel, not per sample. This is why the denoise dominates
// GI at low spp and the gather dominates at high spp.
constexpr double kDenoisePerPixel = 0.80e-6;
// upsample 0.073s over a 12.96 Mpix render grid.
constexpr double kUpsamplePerPixel = 5.6e-9;
// The gather-grid G-buffer is a primary trace with no shading.
constexpr double kGbufferPerPixel = 8.0e-9;
constexpr double kCompositePerPixel = 2.0e-9;
// Box downsample, measured directly: 0.157s at 17 channels and 0.038s at 4
// channels, both over 12.96 Mpix -> 0.71 / 0.73 ns per pixel-channel.
constexpr double kDownsamplePerPixelChannel = 0.70e-9;
// gamma encode 0.0083s over a 1.44 Mpix final image.
constexpr double kGammaPerPixel = 5.8e-9;
// a-trous denoise 0.44s over a 1.44 Mpix final image.
constexpr double kAtrousPerPixel = 0.31e-6;
// Embree device + BVH build, ~0.003s for 40k triangles.
constexpr double kSetupSeconds = 0.003;
// pt2 ReSTIR spatial resampling: per streamed neighbor (target pdf + Jacobian
// + reservoir update, no rays), and per visibility ray in unbiased mode.
constexpr double kSpatialPerNeighborPixel = 40.0e-9;
constexpr double kSpatialRayPerPixel = 150.0e-9;
// "edge patch re-gathered 11138 rim pixel(s) (0.1% of grid)".
constexpr double kEdgePatchPixelFrac = 0.001;
// The irradiance cache is experimental and its pass is NOT instrumented (see
// giCostEstimate), so this only has to make the GI phase SHARE honest: measured
// ~7.4s of an 8.27s `--gi on` render at 1200x1200 with giSamples=64 over a
// 12.96 Mpix grid. Record count really scales with the scene diagonal, not with
// pixels, so treat this as an order-of-magnitude figure.
constexpr double kCacheGiPerSamplePixel = 8.9e-9;
// Stroke edges are vector tracing rather than per-pixel work, so this is not
// calibrated -- just enough to keep the phase's share non-zero.
constexpr double kEdgesPerPixel = 3.0e-9;
// Coarse-AO pre-pass: one AO ray fan per low-res pixel.
constexpr double kCoarseAoPerSamplePixel = 15.0e-9;

// Gather-grid divisor relative to the RENDER grid. MUST mirror the resolution
// done in runPt1GiPass (embree_renderer.cpp): the estimate and the pass have to
// agree on the grid size, so share the rule here instead of restating it.
inline int pt1GatherDivisor(const RenderOptions& opt) noexcept {
  int k = opt.pt1GatherDiv;
  if (k == 0) k = opt.pt1HalfRes ? 2 : 1;  // legacy pt1HalfRes derivation
  if (k < 1) k = 1;                        // incl. the unresolved -1 sentinel
  return k;
}

// Estimated seconds per pt1 GI sub-stage. W/H are the RENDER grid (the
// supersampled hi-res dims the renderer works at).
struct GiCostEstimate {
  double gbuffer = 0.0;
  double gather = 0.0;
  double spatial = 0.0;  // pt2 ReSTIR spatial resampling rounds
  double denoise = 0.0;
  double upsample = 0.0;
  double edgePatch = 0.0;
  double composite = 0.0;
  double total() const noexcept {
    return gbuffer + gather + spatial + denoise + upsample + edgePatch +
           composite;
  }
};

inline GiCostEstimate giCostEstimate(const RenderOptions& opt, int W,
                                     int H) noexcept {
  GiCostEstimate e;
  if (!opt.gi || W <= 0 || H <= 0) return e;
  const double nHi = static_cast<double>(W) * static_cast<double>(H);

  if (opt.giIntegrator == 0) {
    // Irradiance cache: experimental and off the default path, so its internals
    // are not instrumented and the whole pass is one lump. pt2 (== 2) shares
    // the pt1 pass and falls through to the per-substage estimate below.
    e.gather = kCacheGiPerSamplePixel * nHi * std::max(1, opt.giSamples);
    return e;
  }

  const int k = pt1GatherDivisor(opt);
  // Integer ceil, matching the gather grid runPt1GiPass allocates.
  const int lowW = (W + k - 1) / k;
  const int lowH = (H + k - 1) / k;
  const double nLow = static_cast<double>(lowW) * static_cast<double>(lowH);
  const int spp = std::max(1, opt.pt1Spp);
  if (k > 1) {
    // Reduced gather grid: private G-buffer, then joint-bilateral upsample back
    // to the render grid, then optionally re-gather the silhouette rims.
    e.gbuffer = kGbufferPerPixel * nLow;
    e.upsample = kUpsamplePerPixel * nHi;
    if (opt.pt1EdgePatch)
      e.edgePatch = kGatherPerSamplePixel * nHi * kEdgePatchPixelFrac * spp *
                    std::max(1, opt.pt1EdgePatchSppMul);
  }
  e.gather = kGatherPerSamplePixel * nLow * spp;
  if (opt.giIntegrator == 2 && opt.pt2Rounds > 0) {
    // ReSTIR spatial: pure buffer arithmetic per streamed neighbor (8 the
    // first round, 5 after); the unbiased mode adds one occlusion ray per
    // contributor per round.
    const double neighbors =
        8.0 + 5.0 * static_cast<double>(opt.pt2Rounds - 1);
    e.spatial = kSpatialPerNeighborPixel * nLow * neighbors;
    if (opt.pt2Unbiased)
      e.spatial += kSpatialRayPerPixel * nLow *
                   (neighbors + static_cast<double>(opt.pt2Rounds));
  }
  if (opt.pt1Denoise) e.denoise = kDenoisePerPixel * nLow;
  e.composite = kCompositePerPixel * nHi;
  return e;
}

// The GlobalIllum phase's unit budget, sliced across the pt1 sub-stages in
// EXECUTION order. Units are estimated MICROSECONDS: an integer budget fine
// enough that even the cheapest sub-stage still gets a non-zero span. A
// sub-stage the pass skips estimates to 0 and so occupies an empty slice.
struct GiProgressBudget {
  std::uint64_t total = 1;
  ProgressSlice gbuffer;
  ProgressSlice gather;
  ProgressSlice spatial;
  ProgressSlice denoise;
  ProgressSlice upsample;
  ProgressSlice edgePatch;
  ProgressSlice composite;
};

inline GiProgressBudget makeGiBudget(RenderProgress* progress,
                                     const GiCostEstimate& est) noexcept {
  GiProgressBudget b;
  std::uint64_t at = 0;
  auto slice = [&](double seconds) {
    ProgressSlice s;
    s.progress = progress;
    s.base = at;
    s.span = static_cast<std::uint64_t>(std::max(0.0, seconds) * 1.0e6);
    at += s.span;
    return s;
  };
  b.gbuffer = slice(est.gbuffer);
  b.gather = slice(est.gather);
  b.spatial = slice(est.spatial);
  b.denoise = slice(est.denoise);
  b.upsample = slice(est.upsample);
  b.edgePatch = slice(est.edgePatch);
  b.composite = slice(est.composite);
  b.total = std::max<std::uint64_t>(1, at);
  return b;
}

// Estimated seconds per render phase.
struct RenderCostEstimate {
  double setup = 0.0;
  double coarseAo = 0.0;
  double primary = 0.0;
  double globalIllum = 0.0;
  double edges = 0.0;
  double postprocess = 0.0;
};

// `hi` must be the NORMALIZED options the renderer actually runs (renderFrame's
// `hi`: supersampled dims, sentinels resolved); `ss` is the supersample factor.
inline RenderCostEstimate renderCostEstimate(const Scene& scene,
                                             const RenderOptions& hi,
                                             int ss) noexcept {
  RenderCostEstimate e;
  const int ssc = std::max(1, ss);
  const double nHi =
      static_cast<double>(hi.width) * static_cast<double>(hi.height);
  const double nFinal = nHi / (static_cast<double>(ssc) * ssc);

  e.setup = kSetupSeconds;

  // One camera ray + fused direct shading per hi-res pixel. This ignores
  // shadow / AO / transparency rays, so a shadowed, AO-heavy scene really costs
  // more than this says. That only re-paces the bar -- the primary loop is
  // row-instrumented, so it cannot freeze.
  e.primary = kPrimaryPerPixel * nHi;

  if (hi.aoResDiv > 1) {
    const double nAo =
        nHi / (static_cast<double>(hi.aoResDiv) * hi.aoResDiv);
    e.coarseAo = kCoarseAoPerSamplePixel * nAo * std::max(1, hi.aoSamples);
  }

  // GI runs only when there is something to gather from. renderFrame cannot ask
  // the renderer (meshPresent()/realCsgPresent() need the built scene), so
  // approximate. Over-counting only makes the bar finish early; it never
  // freezes, and under-counting is impossible here.
  const bool giRuns =
      hi.gi && (scene.mesh.triangleCount() > 0 ||
                (hi.giIntegrator >= 1 &&
                 (!scene.spheres.empty() || !scene.cylinders.empty())));
  if (giRuns) e.globalIllum = giCostEstimate(hi, hi.width, hi.height).total();

  if (hi.strokeEdges.enable) e.edges = kEdgesPerPixel * nHi;

  // Box-downsample every live AOV channel at hi-res, then denoise and gamma at
  // the final resolution. Channel counts mirror the buffers renderFrame
  // actually downsamples (color 4; GI adds normal + position/indirect/
  // giRecordViz/giOcclusion, except that stroke edges keep normal at hi-res).
  double post = kGammaPerPixel * nFinal;
  if (ssc > 1) {
    int chans = 4;
    if (giRuns) chans += hi.strokeEdges.enable ? 10 : 13;
    if (hi.aoSamples > 0) chans += 6;
    post += kDownsamplePerPixelChannel * nHi * chans;
  }
  // DenoiserBackend: 0 = None, 1 = AtrousBilateral, 2 = OIDN.
  if (hi.denoiser == 2)
    post += kDenoisePerPixel * nFinal;
  else if (hi.denoiser != 0)
    post += kAtrousPerPixel * nFinal;
  e.postprocess = post;

  return e;
}

// Cost estimate -> phase plan. RenderProgress normalizes the shares, so the
// estimated seconds pass through unscaled.
inline RenderPhasePlan toPhasePlan(const RenderCostEstimate& c) noexcept {
  RenderPhasePlan p;
  p.setup = static_cast<float>(c.setup);
  p.coarseAo = static_cast<float>(c.coarseAo);
  p.primary = static_cast<float>(c.primary);
  p.globalIllum = static_cast<float>(c.globalIllum);
  p.edges = static_cast<float>(c.edges);
  p.postprocess = static_cast<float>(c.postprocess);
  return p;
}

}  // namespace detail
}  // namespace umbreon
