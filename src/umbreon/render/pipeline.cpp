#include "render/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <stdexcept>

#include "edges/object_space_edges.hpp"
#include "edges/stroke_edges.hpp"
#include "postprocess/fog.hpp"
#include "postprocess/image_ops.hpp"
#include "experimental/irradiance_cache/denoise.hpp"
#include "render/embree_renderer.hpp"
#include "render/progress_cost_model.hpp"

namespace umbreon {

FrameResult renderFrame(const Scene& sceneIn, const RenderOptions& opt,
                        RenderProgress* progress) {
  // The two NPR edge methods both draw the silhouette and would double-ink if
  // run together (stroke ribbons over object-space edge cylinders); reject the
  // combination rather than silently picking one.
  if (opt.strokeEdges.enable && opt.objectSpaceEdges.enable)
    throw std::runtime_error(
        "umbreon::render: strokeEdges and objectSpaceEdges are mutually "
        "exclusive; enable at most one");

  // Method B (object-space edges): emit the analytic/mesh silhouette as "open"
  // edge cylinders that the tracer occludes for free. The pass mutates the scene
  // (appends to Scene::cylinders) and is camera dependent, so run it here -- on a
  // PRIVATE copy, keeping render()'s `const Scene&` contract -- before tracing.
  // The copy is paid only when the pass is enabled.
  Scene objEdgeScene;
  const Scene* scenePtr = &sceneIn;
  if (opt.objectSpaceEdges.enable) {
    objEdgeScene = sceneIn;
    generateObjectSpaceEdges(objEdgeScene, opt.objectSpaceEdges);
    scenePtr = &objEdgeScene;
  }
  const Scene& scene = *scenePtr;

  const int ss = std::max(1, opt.supersample);
  const int finalW = opt.width, finalH = opt.height;

  // Render at the supersampled resolution; the camera frames identically.
  RenderOptions hi = opt;
  hi.width = finalW * ss;
  hi.height = finalH * ss;
  // pt1 "output resolution" gather sentinel: the renderer only sees the hi-res
  // grid, so resolve -1 to the supersample factor here (ss == 1 -> full res).
  if (hi.pt1GatherDiv < 0) hi.pt1GatherDiv = ss;
  // Adaptive AA is not validated with the GI integrators yet: both consume
  // per-hi-res-pixel seeds (position/normal/albedo/giRefl), and replicated
  // blocks would feed them blockwise-constant guides (plausibly fine for the
  // world-space cache, unvalidated for the pt1 full-res gather). Fall back to
  // the grid path with a warning. Normalizing HERE keeps the group-alpha
  // multipass consistent (every pass sees the same normalized options).
  if (hi.aaMode == 1 && hi.gi) {
    std::fprintf(stderr,
                 "warning: --aa adaptive is not supported with --gi yet; "
                 "falling back to grid supersampling\n");
    hi.aaMode = 0;
  }
  // Coarse-AO "output resolution" sentinel: resolve -1 to the supersample
  // factor (ss == 1 -> 1 = plain inline AO, correct degenerate semantics).
  if (hi.aoResDiv < 0) hi.aoResDiv = ss;
  // Coarse AO + GI is unvalidated (GI drops the mesh ambient term AO
  // modulates, while cache-integrator primitives keep ambient+AO -- a mixed
  // interaction). Fall back to inline AO with a warning; normalizing HERE
  // keeps the group-alpha multipass consistent (every pass sees the same
  // normalized options).
  if (hi.aoResDiv > 1 && hi.gi) {
    std::fprintf(stderr,
                 "warning: --ao-res out is not supported with --gi yet; "
                 "falling back to full-resolution AO\n");
    hi.aoResDiv = 0;
  }

  // Declare where this render's time will actually go, so fraction() weights the
  // bar by the real cost profile instead of a fixed table: with GI on the GI
  // phase is ~75-90% of the render, with GI off the time is Primary's. A phase
  // that does not run this pass gets a zero share and so leaves no gap (the
  // CoarseAo pre-pass is entered unconditionally but is a no-op by default).
  // Declared here, before the first beginPhase(), and only when a caller asked
  // for progress -- the 2-arg render() path stays free of all of this.
  if (progress)
    progress->setPhasePlan(
        detail::toPhasePlan(detail::renderCostEstimate(scene, hi, ss)));

  EmbreeRenderer renderer;
  FrameResult frame = renderer.render(scene, hi, progress);
  // Setup / CoarseAo / Primary / GlobalIllum phases ran inside render(); if it
  // was cancelled mid-flight the buffers are partial -- skip the post-passes and
  // return what we have (frame.cancelled is already set).
  if (frame.cancelled) return frame;

  // OpenGL linear fog at full (supersampled) resolution, before downsampling, so
  // the box-average mirrors antialiased, fogged samples. Uses the plane eye-z
  // AOV (viewZ); transparent backgrounds fade coverage instead of baking fog.
  if (scene.fog.enabled && !frame.viewZ.empty()) {
    applyFog(scene.fog, frame.width, frame.height, 4, frame.color.data(),
             frame.viewZ.data(), opt.transparentBackground);
  }

  // Freestyle-style stroke edges (--edges): vectorize per-pixel edge AOVs via
  // the screen-space crack tracer. Composited over frame.color in LINEAR space,
  // BEFORE the box-downsample, so antialiasing works. Gated on the master flag;
  // with edges off this is never entered, keeping the default path byte-identical.
  if (opt.strokeEdges.enable) {
    if (progress) {
      progress->beginPhase(RenderPhase::Edges);
      if (progress->cancelRequested()) {
        frame.cancelled = true;
        return frame;
      }
    }
    // VERIFICATION (--edges-only): blank the surface color to the scene
    // background BEFORE the stroke pass, so only the edges are drawn. The AOVs
    // captured above (viewZ/objectId/normal/surfAlpha) are untouched, so the
    // extracted line set is identical to the production render. Alpha = 0 for a
    // transparent background (premultiplied), else opaque.
    if (opt.strokeEdges.edgesOnly) {
      const float a = opt.transparentBackground ? 0.0f : 1.0f;
      const float bgr = scene.background.x, bgg = scene.background.y,
                  bgb = scene.background.z;
      const std::size_t npix =
          static_cast<std::size_t>(frame.width) * frame.height;
      for (std::size_t p = 0; p < npix; ++p) {
        frame.color[p * 4 + 0] = bgr * a;
        frame.color[p * 4 + 1] = bgg * a;
        frame.color[p * 4 + 2] = bgb * a;
        frame.color[p * 4 + 3] = a;
      }
    }
    applyStrokeEdges(frame, scene, opt, OcclusionQuery{}, OcclusionQuery{});
  }

  // Fog / downsample / denoise / gamma: the finishing pass. One last cancel
  // check at its boundary; the steps themselves are not row-instrumented.
  if (progress) {
    progress->beginPhase(RenderPhase::Postprocess);
    if (progress->cancelRequested()) {
      frame.cancelled = true;
      return frame;
    }
  }

  if (ss > 1) {
    frame.color = boxDownsample(frame.color, frame.width, frame.height, 4, ss);
    if (!frame.albedo.empty())
      frame.albedo =
          boxDownsample(frame.albedo, frame.width, frame.height, 3, ss);
    // Edge AOVs (normal/viewZ/objectId/materialId) are a hi-res set: the edge
    // pass runs at supersample resolution before this downsample, and box-
    // averaging integer ids is meaningless. So when edges are on, leave them at
    // hi-res; only the legacy normal AOV path downsamples. frame.width/height
    // below become the FINAL color dims.
    if (!frame.normal.empty() && !opt.strokeEdges.enable)
      frame.normal =
          boxDownsample(frame.normal, frame.width, frame.height, 3, ss);
    // AO AOVs are a continuous hi-res set: box-averaging them to the output
    // resolution is exactly the supersample denoise the AO relies on (more
    // effective samples per output pixel). Done before width/height become final.
    if (!frame.contactAo.empty()) {
      frame.contactAo =
          boxDownsample(frame.contactAo, frame.width, frame.height, 1, ss);
      frame.shapeAo =
          boxDownsample(frame.shapeAo, frame.width, frame.height, 1, ss);
      frame.avgHitDist =
          boxDownsample(frame.avgHitDist, frame.width, frame.height, 1, ss);
      frame.bentNormal =
          boxDownsample(frame.bentNormal, frame.width, frame.height, 3, ss);
    }
    // GI cache AOVs (continuous): downsample to the output resolution like the
    // other guide channels. position is world-space, so the box average is a
    // mild edge blend, acceptable for a debug/guide buffer.
    if (!frame.indirect.empty()) {
      frame.position =
          boxDownsample(frame.position, frame.width, frame.height, 3, ss);
      frame.indirect =
          boxDownsample(frame.indirect, frame.width, frame.height, 3, ss);
      frame.giRecordViz =
          boxDownsample(frame.giRecordViz, frame.width, frame.height, 3, ss);
      frame.giOcclusion =
          boxDownsample(frame.giOcclusion, frame.width, frame.height, 1, ss);
    }
    frame.width = finalW;
    frame.height = finalH;
  }

  // Edge-aware denoise on the linear HDR color, at final resolution, before the
  // gamma encode (OIDN/SVGF operate in linear HDR; the supersample box-average is
  // the primary denoise, this is the finishing pass). No-op when denoiser == None,
  // keeping the default render byte-identical. OIDN falls back to the built-in
  // a-trous when the library was not compiled in or fails at runtime; the
  // backend that actually ran is recorded in frame.denoiserUsed.
  if (opt.denoiser == static_cast<int>(DenoiserBackend::OIDN)) {
#ifdef UMBREON_HAVE_OIDN
    if (denoiseOidn(frame, opt)) {
      frame.denoiserUsed = static_cast<int>(DenoiserBackend::OIDN);
    } else {
      // OIDN failed at runtime (device/filter error): fall back instead of
      // silently skipping the denoise (behavior change from the old code,
      // which left the frame un-denoised on this rare path).
      denoiseAtrous(frame, opt);
      frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
    }
#else
    std::fprintf(stderr,
                 "warning: OIDN denoiser backend not built (UMBREON_WITH_OIDN "
                 "off); falling back to the built-in a-trous denoiser\n");
    denoiseAtrous(frame, opt);
    frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
#endif
  } else if (opt.denoiser != static_cast<int>(DenoiserBackend::None)) {
    denoiseAtrous(frame, opt);
    frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
  }

  applyAssumedGamma(frame, scene.assumedGamma);
  return frame;
}

}  // namespace umbreon
