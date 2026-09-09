#include "../log.hpp"
#include "render/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "edges/object_space_edges.hpp"
#include "edges/stroke_edges.hpp"
#include "npr/hatch_shade.hpp"
#include "postprocess/fog.hpp"
#include "postprocess/image_ops.hpp"
#include "experimental/irradiance_cache/denoise.hpp"
#include "render/embree_renderer.hpp"
#include "render/progress_cost_model.hpp"

namespace umbreon {

// The RAW stage: object-space edges, the normalized (supersampled) options,
// the Embree render, the hatch base, fog and the hi-res stroke edge pass --
// i.e. the frame while it is still at the supersampled resolution and in
// linear light, with per-sample coverage still intact. Split from the
// finishing stage so a caller that must combine several renders (the
// group-alpha per-pixel blend in umbreon.cpp) can composite them HERE, before
// the box-downsample turns per-sample coverage into partial pixels, and then
// finish once.
RawFrame renderFrameRaw(const Scene& sceneIn, const RenderOptions& opt,
                        RenderProgress* progress, RTCDevice sharedDevice) {
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
    generateObjectSpaceEdges(objEdgeScene, opt.objectSpaceEdges, progress);
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
    umbreon::logMessage(umbreon::LogLevel::Warning,
                 "--aa adaptive is not supported with --gi yet; "
                 "falling back to grid supersampling");
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
    umbreon::logMessage(umbreon::LogLevel::Warning,
                 "--ao-res out is not supported with --gi yet; "
                 "falling back to full-resolution AO");
    hi.aoResDiv = 0;
  }
  // Tone hatching (--hatch): Ink mode discards the shaded color entirely, so
  // GI would be wasted work, and the binarizing ink composite breaks the
  // color denoisers' smooth-illumination assumptions -- force both off
  // (Over mode keeps the color visible, so an explicit --gi is respected
  // there). Normalizing HERE keeps the group-alpha multipass consistent and
  // precedes the cost model, so the progress phase plan stays honest. An
  // empty layer list resolves to the pen-cross preset for the same reason
  // (every pass must see the same layers).
  if (hi.hatch.enable) {
    if (hi.hatch.mode == HatchMode::Ink) {
      if (hi.gi) {
        umbreon::logMessage(umbreon::LogLevel::Warning,
                     "--hatch ink does not use GI; disabling --gi");
        hi.gi = false;
      }
      hi.denoiser = static_cast<int>(DenoiserBackend::None);
      hi.pt1Denoise = false;
    }
    if (hi.hatch.layers.empty()) applyHatchPreset(hi.hatch, "pen-cross");
    hi.hatch.transparentBackground = hi.transparentBackground;
    hi.hatch.displayGamma = scene.assumedGamma;
    // Hi-res ink (--hatch-res hi) display-encodes the frame BEFORE the
    // downsample, so the linear-domain color denoisers cannot run after it.
    if (hi.hatch.inkHiRes) {
      hi.denoiser = static_cast<int>(DenoiserBackend::None);
      hi.pt1Denoise = false;
    }
    // Per-section styles may need the albedo AOV even when the global
    // base/ink do not (Scene::groupHatchStyle overrides them per group).
    for (const GroupHatchStyle& g : scene.groupHatchStyle)
      if (g.enable && (g.base == HatchBase::Albedo ||
                       g.ink == HatchInk::FromAlbedo))
        hi.hatch.sectionNeedsAlbedo = true;
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

  // What this pass is actually about to do. A host's render log otherwise
  // shows only phase names, which say where the render is but not what it was
  // asked for -- and the settings that decide the cost (the supersampled grid,
  // the GI sample count, the gather resolution) are exactly what someone
  // reading a slow render's log needs to see.
  logMessage(LogLevel::Info,
             "render %dx%d ss%d (grid %dx%d), %zu tris / %zu spheres / "
             "%zu cylinders%s%s",
             finalW, finalH, ss, hi.width, hi.height, scene.mesh.triangleCount(),
             scene.spheres.size(), scene.cylinders.size(),
             hi.strokeEdges.enable ? ", edges" : "",
             hi.aoSamples > 0 ? ", AO" : "");
  if (hi.gi) {
    // gatherDiv was resolved above: 1 = the full hi-res grid, ss = one gather
    // per OUTPUT pixel (the fast path), k = an explicit 1/k grid.
    logMessage(LogLevel::Info,
               "  GI pt%d %dspp, gather grid 1/%d of the render grid%s",
               hi.giIntegrator, hi.pt1Spp, hi.pt1GatherDiv,
               hi.pt1Denoise ? ", OIDN denoise" : "");
  }

  EmbreeRenderer renderer;
  // Borrow the caller's device when one was supplied (group-alpha multipass),
  // so its passes do not each re-initialise Embree.
  renderer.setSharedDevice(sharedDevice);
  FrameResult frame = renderer.render(scene, hi, progress);
  // Setup / CoarseAo / Primary / GlobalIllum phases ran inside render(); if it
  // was cancelled mid-flight the buffers are partial -- skip the post-passes and
  // return what we have (frame.cancelled is already set).
  if (frame.cancelled) return RawFrame{std::move(frame), hi, ss, finalW, finalH};

  // Tone hatching, Ink mode: paint the flat base (paper / first-hit albedo)
  // over every surface pixel NOW -- before fog and the stroke edge pass --
  // in LINEAR hi-res space, the same pattern as the --edges-only blanking.
  // The contour ink then composites over the flat base and survives the
  // final hatch composite (which only multiplies ink in); replacing the base
  // at the end of the pipeline instead would erase every silhouette line
  // drawn over a surface, leaving only the outer contour. This is also what
  // keeps the flat base UNSHADED: the shaded color is discarded here and the
  // hatch density alone carries the tone.
  if (hi.hatch.enable && hi.hatch.mode == HatchMode::Ink &&
      !frame.hatchMask.empty()) {
    // Paper color is display-encoded; applyAssumedGamma later applies
    // pow(v, g), so paint pow(d, 1/g) for a round trip (g ~ 1 paints d).
    const float g = scene.assumedGamma;
    const bool gammaOn = std::fabs(g - 1.0f) > 1e-4f;
    float paperLin[3];
    for (int k = 0; k < 3; ++k) {
      const float d =
          std::min(1.0f, std::max(0.0f, hi.hatch.paperColor[k]));
      paperLin[k] = (gammaOn && d > 0.0f) ? std::pow(d, 1.0f / g) : d;
    }
    // Per-section styling: the hi-res section-id buffer selects each
    // pixel's style (base choice, or "leave this section shaded").
    const bool perSection =
        !scene.groupHatchStyle.empty() && !frame.hatchGroup.empty();
    const std::size_t npix =
        static_cast<std::size_t>(frame.width) * frame.height;
    for (std::size_t p = 0; p < npix; ++p) {
      if (frame.hatchMask[p] <= 0.5f) continue;  // background: untouched
      const GroupHatchStyle* st = nullptr;
      if (perSection) {
        const std::uint16_t g = frame.hatchGroup[p];
        if (g != 0xFFFFu && g < scene.groupHatchStyle.size())
          st = &scene.groupHatchStyle[g];
        if (st != nullptr && !st->enable) continue;  // keeps its shading
      }
      const HatchBase baseSel = st != nullptr ? st->base : hi.hatch.base;
      float b[3];
      if (baseSel == HatchBase::Albedo && !frame.albedo.empty()) {
        // Paint the LINEAR albedo: applyAssumedGamma below then displays it
        // exactly like every other surface color, i.e. the flat base is the
        // pigment as this pipeline shows it (fully lit, unshaded). Encoding
        // it here with a different curve would desaturate the fill.
        for (int k = 0; k < 3; ++k) {
          b[k] = frame.albedo[p * 3 + k];
          if (hi.hatch.albedoQuantize > 1) {
            const float n = static_cast<float>(hi.hatch.albedoQuantize);
            b[k] = std::round(b[k] * n) / n;
          }
        }
      } else {
        b[0] = paperLin[0];
        b[1] = paperLin[1];
        b[2] = paperLin[2];
      }
      // Keep the premultiplied convention: coverage stays in alpha, the
      // painted base carries it in RGB (opaque background => alpha 1).
      const float a = frame.color[p * 4 + 3];
      const float s = hi.transparentBackground ? a : 1.0f;
      frame.color[p * 4 + 0] = b[0] * s;
      frame.color[p * 4 + 1] = b[1] * s;
      frame.color[p * 4 + 2] = b[2] * s;
    }
  }

  // OpenGL linear fog at full (supersampled) resolution, before downsampling, so
  // the box-average mirrors antialiased, fogged samples. Uses the plane eye-z
  // AOV (viewZ); transparent backgrounds fade coverage instead of baking fog.
  if (scene.fog.enabled && !frame.viewZ.empty()) {
    applyFog(scene.fog, frame.width, frame.height, 4, frame.color.data(),
             frame.viewZ.data(), opt.transparentBackground);
    // Hatch tone fog: fade the tone toward paper with the SAME fog factor,
    // so distant marks thin out and (via inkShadeDark) lighten -- the ink
    // analogue of the fogged silhouette stroke color. Hi-res, before the
    // downsample, like the color fog above.
    if (hi.hatch.enable && hi.hatch.toneFog && !frame.hatchTone.empty()) {
      const std::size_t npix =
          static_cast<std::size_t>(frame.width) * frame.height;
      for (std::size_t p = 0; p < npix; ++p) {
        const float vz = frame.viewZ[p];
        if (vz <= 0.0f) continue;  // background sentinel
        const float f = fogFactor(scene.fog, vz);
        frame.hatchTone[p] = 1.0f - (1.0f - frame.hatchTone[p]) * f;
      }
    }
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
        return RawFrame{std::move(frame), hi, ss, finalW, finalH};
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
    // The renderer's live BVH backs the extractor's fold probe (a segment
    // occlusion test per strongNdelta-rescue candidate crack); the plain
    // any-hit path is used (no exclude faces / filters).
    // The edge pass polls `progress` for cancellation at its stage / loop
    // boundaries; a cancelled pass returns early with a partial line set and
    // the Postprocess boundary check below flags frame.cancelled.
    applyStrokeEdges(frame, scene, opt,
                     [&renderer](const Vec3& p, const Vec3& q,
                                 const int* excludeFaces, int nExclude) {
                       return renderer.occluded(p, q, excludeFaces, nExclude);
                     },
                     OcclusionQuery{}, progress);
  }

  return RawFrame{std::move(frame), hi, ss, finalW, finalH};
}

// The FINISHING stage: hi-res ink, box-downsample to the output resolution,
// denoise, assumed_gamma and the output-resolution ink composite. `hi` is the
// normalized options renderFrameRaw resolved; `opt` is the caller's, and each
// is read exactly where the single-pass pipeline read it.
void finishFrame(FrameResult& frame, const Scene& scene,
                 const RenderOptions& opt, const RenderOptions& hi, int ss,
                 int finalW, int finalH, RenderProgress* progress) {  // Fog / downsample / denoise / gamma: the finishing pass. One last cancel
  // check at its boundary; the steps themselves are not row-instrumented.
  if (progress) {
    progress->beginPhase(RenderPhase::Postprocess);
    if (progress->cancelRequested()) {
      frame.cancelled = true;
      return;
    }
  }

  // Hi-res ink (--hatch-res hi, the default): display-encode and lay the
  // strokes at the SUPERSAMPLED resolution, then let the box downsample
  // average them into a fine drawing-like grain. The layer parameters stay
  // in FINAL-resolution pixel units -- they are converted to the hi-res
  // grid HERE, so the look is invariant under the supersample factor and
  // the effective pitch floor is 2/ss output px (the 2 px min-feature
  // clamp applies on the hi-res grid). The AA filter width (edgeSoftness)
  // is NOT converted: it is a device-pixel quantity, and the downsample
  // already supplies the output-space filtering.
  bool inkDone = false;
  if (hi.hatch.enable && hi.hatch.inkHiRes && !frame.hatchTone.empty()) {
    applyAssumedGamma(frame, scene.assumedGamma);
    HatchOptions inkOpt = hi.hatch;
    if (ss > 1) {
      const float s = static_cast<float>(ss);
      for (HatchLayer& l : inkOpt.layers) {
        l.spacingPx *= s;
        l.widthPx *= s;
        l.mark.wobbleAmpPx *= s;
        l.mark.wobbleWavePx *= s;
        l.mark.strokeLenPx *= s;
        l.mark.strokeGapPx *= s;
        l.mark.toothScalePx *= s;
      }
      inkOpt.uvScale *= s;  // the UV shares the layer parameters' units
    }
    applyHatch(frame.width, frame.height, frame.color.data(),
               frame.hatchTone.data(), frame.hatchMask.data(),
               frame.albedo.empty() ? nullptr : frame.albedo.data(), inkOpt,
               frame.hatchGroup.empty() ? nullptr : frame.hatchGroup.data(),
               /*groupSs=*/1,
               scene.groupHatchStyle.empty() ? nullptr
                                             : scene.groupHatchStyle.data(),
               scene.groupHatchStyle.size(),
               frame.hatchUv.empty() ? nullptr : frame.hatchUv.data());
    inkDone = true;
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
    // GI AOVs (continuous): downsample to the output resolution like the
    // other guide channels. position is world-space, so the box average is a
    // mild edge blend, acceptable for a debug/guide buffer. Each buffer is
    // gated on its own presence: the debug pair (giRecordViz/giOcclusion) is
    // only allocated under giWriteAov, and boxDownsample reads its source
    // unconditionally, so a shared gate would walk empty vectors.
    if (!frame.indirect.empty()) {
      frame.position =
          boxDownsample(frame.position, frame.width, frame.height, 3, ss);
      frame.indirect =
          boxDownsample(frame.indirect, frame.width, frame.height, 3, ss);
    }
    if (!frame.giRecordViz.empty())
      frame.giRecordViz =
          boxDownsample(frame.giRecordViz, frame.width, frame.height, 3, ss);
    if (!frame.giOcclusion.empty())
      frame.giOcclusion =
          boxDownsample(frame.giOcclusion, frame.width, frame.height, 1, ss);
    // Hatch AOVs (continuous): the tone box-average IS the tone
    // antialiasing (raise ss and the tone smooths while the ink, laid at
    // final resolution below, keeps its pixel-exact width), and the mask
    // average gives the silhouette-coverage AA of the ink composite. With
    // hi-res ink (--hatch-res hi) the ink was already composited above, so
    // like the edge G-buffer these AOVs stay at their hi-res size.
    if (!inkDone && !frame.hatchTone.empty())
      frame.hatchTone =
          boxDownsample(frame.hatchTone, frame.width, frame.height, 1, ss);
    if (!inkDone && !frame.hatchMask.empty())
      frame.hatchMask =
          boxDownsample(frame.hatchMask, frame.width, frame.height, 1, ss);
    // The UV is a continuous surface coordinate, so the box average is the
    // right reconstruction (it blurs mildly across a silhouette, which the
    // mask already suppresses there).
    if (!inkDone && !frame.hatchUv.empty())
      frame.hatchUv =
          boxDownsample(frame.hatchUv, frame.width, frame.height, 2, ss);
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
    umbreon::logMessage(umbreon::LogLevel::Warning,
                 "OIDN denoiser backend not built (UMBREON_WITH_OIDN "
                 "off); falling back to the built-in a-trous denoiser");
    denoiseAtrous(frame, opt);
    frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
#endif
  } else if (opt.denoiser != static_cast<int>(DenoiserBackend::None)) {
    denoiseAtrous(frame, opt);
    frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
  }

  if (!inkDone) applyAssumedGamma(frame, scene.assumedGamma);
  // Tone-hatching ink composite (--hatch, default --hatch-res out): AFTER
  // the gamma encode, because ink/paper colors are display-encoded values
  // composited in display space (the same rule as the group-alpha
  // blendpng-equivalent blend). The tone was generated hi-res in the hit
  // shader and box-downsampled above, so the binarization here happens
  // once, at the final resolution (pixel-exact stroke widths).
  if (hi.hatch.enable && !inkDone && !frame.hatchTone.empty())
    applyHatch(frame.width, frame.height, frame.color.data(),
               frame.hatchTone.data(), frame.hatchMask.data(),
               frame.albedo.empty() ? nullptr : frame.albedo.data(), hi.hatch,
               frame.hatchGroup.empty() ? nullptr : frame.hatchGroup.data(),
               ss,
               scene.groupHatchStyle.empty() ? nullptr
                                             : scene.groupHatchStyle.data(),
               scene.groupHatchStyle.size(),
               frame.hatchUv.empty() ? nullptr : frame.hatchUv.data());
}

FrameResult renderFrame(const Scene& scene, const RenderOptions& opt,
                        RenderProgress* progress, RTCDevice sharedDevice) {
  RawFrame raw = renderFrameRaw(scene, opt, progress, sharedDevice);
  if (raw.frame.cancelled) return std::move(raw.frame);
  finishFrame(raw.frame, scene, opt, raw.hi, raw.ss, raw.finalW, raw.finalH,
              progress);
  return std::move(raw.frame);
}

}  // namespace umbreon
