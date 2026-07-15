// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// FrameResult (linear HDR framebuffer + AOV channels) and the pt1 timing/ray
// diagnostics it carries, re-exported through render/render_types.hpp. Pure
// C++17 data, no rendering-library dependency.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace umbreon {

// Per-stage wall-clock seconds of the pt1 integrator pipeline. bvhBuild and
// primary are filled on every render (cheap timers around existing stages);
// gather/denoise/upsample only when the pt1 integrator runs. direct stays 0 in
// this architecture: direct shading is fused into the primary-ray loop, so its
// cost is reported under `primary` (the stage split the pt1 plan asks for
// cannot separate them without restructuring the shared pixel loop).
struct Pt1Timing {
  double bvhBuild = 0.0;
  double primary = 0.0;   // primary rays + fused direct shading
  double direct = 0.0;    // always 0 (see above)
  double gather = 0.0;
  double denoise = 0.0;
  double upsample = 0.0;
  double total = 0.0;     // filled by the caller (wall time around render())
};

// pt1 ray counts for the frame (gather intersects, NEE shadow rays, half-res
// G-buffer primaries). Filled only when the pt1 integrator runs; counting is
// per-pixel-flushed so its overhead is negligible (see Pt1RayStats in
// pt1_integrator.hpp).
struct Pt1RayCounts {
  std::uint64_t gatherRays = 0;
  std::uint64_t gatherHits = 0;
  std::uint64_t neeRays = 0;
  std::uint64_t neeOccluded = 0;
  std::uint64_t gbufferRays = 0;
};

// Rendered frame: linear HDR color plus AOV channels, top-left pixel origin.
struct FrameResult {
  int width = 0;
  int height = 0;
  std::vector<float> color;   // width*height*4 linear HDR RGBA
  std::vector<float> albedo;  // width*height*3
  std::vector<float> normal;  // width*height*3 world-space
  std::vector<float> depth;   // width*height   ray distance from camera
  // Edge G-buffer AOVs: sized and written ONLY when RenderOptions::strokeEdges
  // is enabled (otherwise left empty, keeping the default path byte-identical).
  std::vector<float> viewZ;          // width*height   linear view-z (edge-only)
  std::vector<std::uint32_t> objectId;    // width*height   per-pixel object id
  std::vector<std::uint32_t> materialId;  // width*height   per-pixel material id
  // First-hit surface opacity (fragment alpha, including any group alpha
  // override baked into the scene colors). Sized and written ONLY when
  // strokeEdges is enabled. The stroke edge pass multiplies each chain
  // vertex's stroke opacity by this value, so an edge derived from a
  // transparent surface inks with that surface's transparency (per-vertex
  // alpha varies linearly along the stroke via the standard lerp).
  std::vector<float> surfAlpha;           // width*height   first-hit opacity
  // AO / surface-irradiance-cache AOVs: sized and written ONLY when
  // RenderOptions::aoWriteAov is on (else left empty, keeping the default path
  // byte-identical). albedo/normal above are the OIDN guide; these are the AO
  // contact/shape split, the bent normal and the mean occluder distance.
  std::vector<float> contactAo;   // width*height   small-radius (contact) openness
  std::vector<float> shapeAo;     // width*height   mid+large-radius openness
  std::vector<float> bentNormal;  // width*height*3 average unoccluded direction
  std::vector<float> avgHitDist;  // width*height   mean occluder distance (world)
  // Surface-irradiance-cache AOVs: sized and written ONLY when RenderOptions::gi
  // is on (else empty, keeping the default path byte-identical). `position` is the
  // world-space first hit (cache spatial key / denoise guide); `indirect` is the
  // interpolated indirect irradiance E_cached (debug / denoise demodulation);
  // `giRecordViz` is a debug false-color of the nearest cache record's effective
  // radius R_i (bright = small radius = dense records, e.g. in concavities).
  std::vector<float> position;    // width*height*3 world-space first-hit position
  std::vector<float> indirect;    // width*height*3 interpolated E_cached
  std::vector<float> giRecordViz; // width*height*3 record-radius (log R_i) heatmap
  std::vector<float> giOcclusion; // width*height   gather occlusion fraction (AO-like)
  // Adaptive-AA refinement mask debug AOV: sized (width/ss)*(height/ss) -- one
  // value per OUTPUT pixel, 1 = refined, 0 = replicated -- and written ONLY when
  // aaMode == 1 and aaDebug is on (else empty). Never downsampled.
  std::vector<float> aaMask;
  // Coarse-AO fallback mask debug AOV: hi-res width*height, 1 = the first hit's
  // bilateral lookup was rejected and the pixel gathered inline (silhouette
  // rim / transparency / sub-cell feature). Written ONLY when aoResDiv > 1 and
  // aoResDebug is on (else empty). Never downsampled.
  std::vector<float> aoPatchMask;
  double renderSeconds = 0.0;
  // Coarse-AO pre-pass wall time (low-res G-buffer trace + per-cell gather);
  // 0 unless aoResDiv > 1 activated the coarse grid.
  double aoCoarseSeconds = 0.0;
  std::size_t effectiveTriangles = 0;
  // True when the render was cooperatively cancelled (RenderProgress::
  // requestCancel) before finishing: the buffers hold a PARTIAL frame (rows past
  // the cancel point, and skipped post-passes, are unfilled). Always false for a
  // render that ran to completion, so the default path is unchanged.
  bool cancelled = false;
  // pt1 stage timing (zero-filled unless the pt1 integrator ran; bvhBuild and
  // primary are recorded on every render since the timers are free).
  Pt1Timing pt1Timing;
  Pt1RayCounts pt1Rays;
  // Which denoiser actually ran (DenoiserBackend as int, matching
  // RenderOptions::denoiser: 0=None, 1=AtrousBilateral, 2=OIDN). denoiserUsed
  // is the final-color denoise stage: 2 only when the out-of-process OIDN
  // worker really processed the frame; 1 covers both an explicit a-trous
  // request AND every OIDN fallback (client not built, worker missing / dead /
  // error); 0 when no final-color denoise ran. pt1DenoiserUsed reports the pt1
  // integrator's internal indirect-irradiance (E) denoise (RenderOptions::
  // pt1Denoise) the same way -- this is the field CueMol's GI path cares about,
  // since giDenoise maps to pt1Denoise. Group-alpha multipass renders report
  // the final (carrier) pass. Additive fields: pixel output is unaffected.
  int denoiserUsed = 0;
  int pt1DenoiserUsed = 0;
};

}  // namespace umbreon
