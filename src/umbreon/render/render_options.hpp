// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// RenderOptions for umbreon::render(), re-exported through
// render/render_types.hpp. Pure C++17 data, no rendering-library dependency.
#pragma once

#include "render/edge_types.hpp"

namespace umbreon {

// Options for umbreon::render(). Every field here is honored by the renderer;
// the defaults reproduce the POV-faithful look with all secondary effects off
// (so a default-constructed RenderOptions yields plain primary-ray shading).
struct RenderOptions {
  // --- output ---
  int width = 1024;   // final image width  (pixels)
  int height = 768;   // final image height (pixels)
  // Supersampling factor: render at width*ss x height*ss and box-average down to
  // width x height in linear space (antialiasing). 1 = off.
  int supersample = 1;

  // --- adaptive antialiasing (--aa adaptive) --- default 0 (grid) keeps the
  // full-grid supersample path untouched, so flag-less output stays byte-
  // identical. Mode 1 (adaptive) shades ONE center subpixel per output pixel
  // first, flags output pixels whose neighborhood shows a discontinuity
  // (geomID/group change, color contrast > aaThreshold, normal delta, depth
  // crack), then shades every subpixel of flagged pixels only; unflagged
  // blocks replicate the center result. Fully deterministic (no jitter): the
  // flagged region is bitwise-identical to the grid render, and thread count
  // never changes the output. Not supported with gi yet (renderFrame falls
  // back to grid with a warning).
  int aaMode = 0;             // 0 = grid (legacy full supersample), 1 = adaptive
  float aaThreshold = 0.1f;   // per-channel linear color contrast that flags a pair
  // Edge-quality lattice for FLAGGED pixels: subdivide each output pixel
  // aaDepth x aaDepth (rounded up to a multiple of ss) instead of ss x ss.
  // 0 or ss = same density as the supersample grid (bitwise-equal there);
  // > ss = finer edge sampling than the grid at unchanged flat-region cost.
  int aaDepth = 0;
  bool aaDebug = false;       // fill FrameResult::aaMask (refinement mask AOV)

  // --- ambient occlusion (mesh hits only; modulates the ambient term) ---
  // Default 0 = AO off, so flag-less output stays the bit-exact POV-matched
  // local shading. AO never darkens flat outline primitives (spheres/cylinders).
  int aoSamples = 0;           // AO rays per mesh hit; 0 = off
  float aoDistance = 1.0e20f;  // AO occluder search radius (ray tfar / world units)
  float aoIntensity = 1.0f;    // AO strength: aoFactor = 1 - aoIntensity*(1-rawAO)

  // --- AO quality enhancements (all default to the legacy binary single-scale
  // behavior). When aoEnhanced() is false the legacy computeAO runs and the
  // output is bit-identical to the pre-enhancement renderer; any non-default
  // flag below switches the hit shader to the computeAOQuality estimator.
  float aoFalloffPower = 0.0f;   // 0 = binary (legacy); >0 => (max(0,1-t/R))^power
  bool aoMultiScale = false;     // false = single radius (aoDistance); true = 3-scale
  bool aoBentNormal = false;     // directional ambient from the avg unoccluded dir
  float aoSkyColor[3] = {1.0f, 1.0f, 1.0f};     // up-hemisphere tint (x ambient)
  float aoGroundColor[3] = {1.0f, 1.0f, 1.0f};  // down-hemisphere tint
  bool aoUseCameraUp = true;     // gradient axis = camera up (view-stable)
  float aoUp[3] = {0.0f, 1.0f, 0.0f};  // explicit gradient axis when !aoUseCameraUp
  bool aoMultibounce = false;    // albedo-aware GTAO cubic (anti over-darkening)
  bool aoLowDiscrepancy = false; // Hammersley + per-pixel Cranley-Patterson rotation
  float aoDiffuseFactor = 0.0f;  // 0 = ambient-only; >0 also darkens direct diffuse
  bool aoWriteAov = false;       // emit AO/G-buffer AOVs into FrameResult (phase 5)

  // --- coarse-grid AO (--ao-res out) --- gather the AO once per COARSE cell
  // (the hi-res render grid divided by aoResDiv) on a low-res first-hit
  // G-buffer; every shading hit interpolates it with a normal/depth-guided
  // bilateral lookup and falls back to the exact inline gather where the
  // guides reject (silhouette rims, transparency layers, sub-cell features).
  // 0 or 1 = inline per-hit gather (default, byte-identical); -1 = "output
  // resolution" sentinel, resolved to the supersample factor by renderFrame
  // (the pt1GatherDiv pattern); k > 1 = explicit divisor. AO rays drop by
  // ~aoResDiv^2 in smooth regions. Not supported with gi yet (renderFrame
  // falls back with a warning).
  int aoResDiv = 0;
  bool aoResDebug = false;  // fill FrameResult::aoPatchMask (fallback pixels)

  // True when any AO enhancement is requested. Drives the hit shader's
  // enhanced-vs-legacy branch: false => bit-exact legacy computeAO path.
  bool aoEnhanced() const {
    return aoFalloffPower > 0.0f || aoMultiScale || aoBentNormal ||
           aoMultibounce || aoLowDiscrepancy || aoDiffuseFactor > 0.0f;
  }

  // --- diffuse GI: adaptive surface irradiance cache (mesh hits only) ---
  // Default gi == false => no cache pass at all, byte-identical to the local-
  // shading render. When on, a deterministic set of surface cache records is
  // placed and filled by hemisphere gather (see
  // experimental/irradiance_cache/irradiance_cache.hpp);
  // the interpolated indirect irradiance is exposed as the `indirect` AOV.
  // NOTE (steps 1-3): the gather + cache are built and visualized via AOVs, but
  // the final composite ([E], L += gi*kd*E_cached) is NOT wired yet, so a gi==on
  // render produces the SAME color as gi==off (only the AOVs differ).
  bool gi = false;              // MASTER gate; false => no GI work, byte-identical
  int giSamples = 64;           // hemisphere gather rays per cache record
  int giBounces = 1;            // 1 = one-bounce; >1 = multi-bounce (later step)
  float giMaxDistance = 0.0f;   // gather ray tfar; 0 => auto (scene diagonal)
  float giIntensity = 1.0f;     // indirect gain (physical 1.0; user knob, no 1/pi)
  float giEnvIntensity = 1.0f;  // MULTIPLIER on the environment (sky/ground miss)
                                // radiance, which is scene.ambientColor * this.
                                // scene.ambientColor carries the ambient light
                                // ENERGY GI gathers occlusion-aware (the harness
                                // sets it from the lighting energy balance when GI
                                // is on); 1.0 uses it as-is, <1 deepens occlusion
                                // contrast, >1 adds fill. Scales ONLY the miss
                                // term; surface bounce / color bleeding stay full.
  float giAccuracy = 0.15f;     // interpolation accuracy a (max influence = a*R_i)
  float giRecordSpacing = 0.0f; // voxel seed world spacing; 0 => auto (diag*0.007)
  bool giGradients = false;     // Ward-Heckbert rotational/translational gradients
  bool giOutlierReject = true;  // lift isolated fully-occluded dark cache records
  bool giAdaptive = false;      // adaptive voxel refinement (later step; unused now)
  float giNormalReject = 0.85f; // min dot(n_x, n_rec) to accept a record
  bool giComponentReject = true;// reject records of a different component (leak)
  bool giSeedPerVertex = false; // true => seed from mesh vertices (view-independent)

  // --- pt1: path-traced indirect integrator (per-pixel one-bounce gather; see
  // experimental/pt1/pt1_integrator.hpp). This is the DEFAULT indirect
  // integrator. giIntegrator == 2 selects pt2, which layers an Owen-scrambled
  // Sobol / blue-noise sampler, emissive-at-bounce and (in progress) ReSTIR-GI
  // spatial resampling onto the same gather core (experimental/pt2/).
  // giIntegrator == 0 selects the older irradiance cache, experimental and
  // kept for comparison only. Either way the integrator runs only when gi is
  // on.
  int giIntegrator = 1;  // 1 = pt1 (default), 2 = pt2, 0 = irradiance cache
  int pt1Spp = 8;               // gather rays per pixel
  bool pt1HalfRes = true;       // LEGACY gather-grid selector, consulted only
                                // when pt1GatherDiv == 0: half the render grid
                                // + joint bilateral upsample (false=full res)
  bool pt1Denoise = true;       // OIDN denoise of the indirect irradiance
                                // buffer (pre-composite, at gather resolution)
  unsigned pt1Seed = 0;         // deterministic per-pixel RNG seed
  int pt1SkyMode = 0;           // 0 = uniform sky; 1 = gradient (zenith =
                                // pt1SkyRadiance, ground = aoGroundColor)
  float pt1SkyRadiance[3] = {1.0f, 1.0f, 1.0f};  // sky tint (x ambient energy)
  // Joint bilateral upsample edge-stops (half-res mode): normal weight
  // max(0,dot(Nf,Nh))^pow and depth weight exp(-|zf-zh|/(scale*zf+1e-6)).
  float pt1UpsampleNormalPow = 32.0f;
  float pt1UpsampleDepthScale = 0.02f;
  // Stratified first-bounce sampling (Hammersley + per-pixel Cranley-Patterson
  // shift, the AO sampler's scheme) and per-sample luminance firefly clamp
  // (0 = off). Stratification is on by default -- it is what the --quality
  // presets use: same ray count, less variance. The clamp stays off because it
  // biases the result.
  bool pt1Ld = true;
  float pt1Clamp = 0.0f;
  // Gather-grid divisor relative to the RENDER grid (which is the supersampled
  // hi-res grid): the indirect irradiance is gathered on a ceil(W/k) x
  // ceil(H/k) grid and joint-bilateral-upsampled back (the E field is
  // low-frequency; the denoise and the ss box-downsample smooth it anyway).
  //   0  = legacy: derive 1 or 2 from pt1HalfRes
  //   k>=1 = explicit divisor (1 = gather on the render grid)
  //   -1 = "output resolution" (DEFAULT): resolved to the supersample factor by
  //        renderFrame, so the gather grid matches the FINAL image size. This
  //        is what the --quality draft/high presets use: visually equivalent to
  //        the full hi-res grid at a fraction of the cost.
  int pt1GatherDiv = -1;
  // Re-gather silhouette-rim pixels at FULL resolution when the reduced
  // gather grid has no compatible sample for them (the joint-bilateral guide
  // weights die at depth/normal discontinuities whose surface the low grid
  // never sampled -- historically those pixels copied a wrong-surface E or
  // went black). The rim set is a few percent of the frame, so the patch
  // gather costs little. Applies only when the gather grid is reduced.
  bool pt1EdgePatch = true;
  // spp multiplier for the edge-patch re-gather. Patched rim pixels skip the
  // low-grid denoise, so they carry raw Monte-Carlo variance; oversampling
  // just those pixels (0.1-0.2% of the frame) is nearly free and cuts the
  // rim speckle variance by the same factor.
  int pt1EdgePatchSppMul = 4;
  // Also patch LOW-CONFIDENCE upsampled pixels: total guide weight below this
  // threshold (0 = only dead-weight pixels). The upsample weight sums to ~1
  // for well-supported interior pixels.
  float pt1EdgePatchThresh = 0.3f;
  // Print pt1 diagnostics to stderr: the OIDN stage split (device / filter /
  // execute) inside the denoiser. Ray counts are always collected (negligible
  // cost, reported via FrameResult::pt1Rays); this flag only adds the prints.
  bool pt1Stats = false;

  // --- pt2 (giIntegrator == 2): extensions layered onto the pt1 gather core.
  // The pt1* knobs above (spp, gather grid, denoise, sky, edge patch, seed)
  // all apply to pt2 as well; these fields only control what pt2 adds.
  // First-bounce sample arrangement: 0 = an independent Owen-scrambled Sobol
  // sequence per pixel; 1 = blue noise (ONE global sequence walked along a
  // hierarchically shuffled Morton curve, so the residual error distributes
  // as blue noise in screen space -- kinder to OIDN and the eye).
  int pt2Pattern = 1;
  // Add Material::emission at gather bounce vertices, so emissive geometry
  // lights its surroundings. Not a double count: the direct pass applies
  // emission only as self-illumination on camera-visible pixels and never
  // transports it to other surfaces.
  bool pt2Emissive = true;
  // ReSTIR-GI spatial resampling rounds (0 = OFF, the default). Implemented
  // and deterministic, but MEASURED NOT TO PAY OFF in umbreon's regime
  // (stills, LD-stratified gather at >= 4 spp, sky-dominant diffuse, OIDN):
  // on 1ab0_scene1 at spp=8 the reservoir estimator lands ~7-8 dB BELOW the
  // plain gather mean, and only wins at spp=1 (+1.3 dB). Spatial-only ReSTIR
  // compresses a pixel's spp samples into ONE survivor; without the temporal
  // accumulation the reference implementations lean on (M up to ~30 across
  // frames), that discard outweighs the neighborhood reuse. Kept as an
  // opt-in experiment and as the reservoir substrate for a future temporal
  // mode (pt3 / RenderSession).
  int pt2Rounds = 0;
  // Round-0 kernel radius in GATHER-grid pixels (halves each round, floor 3).
  float pt2Radius = 16.0f;
  // Z-normalization with one visibility ray per contributor (Ouyang eq. 16):
  // removes the reuse visibility bias (slightly lightened contact shadows)
  // at ~K+1 occlusion rays per pixel per round.
  bool pt2Unbiased = false;
  // Clamp on a streamed reservoir's M (bounds how much history a single
  // reservoir can claim), and an optional clamp on the finalized contribution
  // weight W (0 = off). W-clamping breaks the exact luminance cancellation of
  // the creation-stage reservoir (grazing winners legitimately carry a large
  // W), so it is OFF by default; the MIS normalization already bounds reuse.
  float pt2MCap = 100.0f;
  float pt2WClamp = 0.0f;
  // Variance-adaptive spp (Cycles' split-buffer scheme): after the base
  // gather, pixels whose mean-vs-half-mean luminance difference exceeds
  // pt2AdaptiveThresh * normalization get ONE refinement pass that continues
  // their sample sequence, up to a total of pt2AdaptiveMul * pt1Spp samples.
  // Concentrates rays in the noisy minority (pockets, contact shadows) at a
  // hard cost bound. Mutually exclusive with pt2Rounds > 0 (the reservoir
  // bookkeeping does not compose with a two-pass merge).
  bool pt2Adaptive = false;
  float pt2AdaptiveThresh = 0.15f;
  int pt2AdaptiveMul = 4;
  // Traced mirror reflection (full-PT track, stage 1): surfaces with
  // Material::reflection > 0 trace one mirror ray per pixel in a GI post-pass
  // and composite reflection * L(hit: NEE direct + ambient approx; miss: the
  // gather sky), replacing shadeLocal's fake reflection*background term. POV's
  // reflection is a sharp mirror, so one deterministic ray suffices (no spp).
  bool pt2Reflect = true;

  // --- denoise (post-pass on the linear HDR color, after downsample / before
  // gamma) --- denoiser == 0 (None) => no-op, byte-identical to the un-denoised
  // render. AtrousBilateral (1) is the built-in, zero-dependency edge-avoiding
  // a-trous (Dammertz 2010 + SVGF edge-stops); OIDN (2) is an optional backend
  // (later step). The edge-stop guides are the GI/AO G-buffer (position, normal)
  // plus the color's own luminance, so the smoothing follows the cache's residual
  // noise without crossing depth/normal/illumination edges.
  int denoiser = 0;             // DenoiserBackend: 0=None, 1=AtrousBilateral, 2=OIDN
  int denoiseIters = 5;         // a-trous wavelet iterations (2^i dilation)
  float denoiseSigmaZ = 1.0f;   // depth/position edge-stop sigma
  float denoiseSigmaN = 128.0f; // normal edge-stop exponent
  float denoiseSigmaL = 4.0f;   // luminance edge-stop sigma
  bool denoiseDemodulateAlbedo = true;  // denoise color/albedo, then re-multiply
  bool oidnCleanAux = true;     // OIDN: primary-hit albedo/normal are noise-free
  // OIDN scratch memory cap in MB (< 0 = OIDN default, no cap). OIDN denoises
  // in internal overlapping tiles to stay under the cap, which bounds every
  // SINGLE allocation regardless of resolution. Required inside Electron:
  // PartitionAlloc aborts (SIGTRAP) on single allocations > ~2 GiB, and the
  // uncapped default allocates one ~2.5 GiB arena at 13 MP. Measured with
  // 1024: max single allocation ~0.9-1.0 GiB up to 8K, ~+23% denoise time at
  // 13 MP. NOTE: the value changes the tile layout and thus the output bits
  // at resolutions large enough to trigger tiling -- keep it fixed for
  // reproducibility (small frames are tiling-free and bit-identical).
  int oidnMaxMemoryMB = 1024;

  // --- shadows (per-light visibility; never applied to outline primitives) ---
  bool shadows = false;        // cast shadows from the lights; false = off
  int shadowSamples = 1;       // shadow rays per light (>1 = soft area light)
  float lightRadius = 0.0f;    // light angular radius (deg); >0 = soft shadows

  // --- environment dome lighting (synthetic, computed in the renderer; NOT read
  // from the scene/POV) --- A hemisphere of distant lights around the camera-
  // forward axis approximating sky/environment illumination: exposed surfaces are
  // lit softly from many directions (form-revealing, no frontal "flashlight"),
  // while recesses and buried atoms darken via real shadows. This is the lighting
  // route to the radiosity-like "clean exposed + dark recess" look without the
  // muddiness of darkening direct diffuse by openness (aoDiffuseFactor). Opt-in:
  // envLights == 0 leaves the scene's own lights untouched (byte-identical). When
  // on, shadows are implicitly active and the dome lights are diffuse-only (no
  // specular). The scene's own lights are scaled by envKeyScale (0 = dome only).
  int envLights = 0;           // dome light count (0 = off; ~24-48 for smooth)
  float envIntensity = 1.0f;   // diffuse irradiance on a fully-exposed (camera-facing) point
  float envKeyScale = 0.0f;    // scale applied to the scene's own lights when the dome is on
  float envAngle = 90.0f;      // dome half-angle around camera-forward (deg)

  // --- shading ---
  float specularScale = 1.0f;  // multiplies each material's specular weight

  // --- transparency (single-pass front-to-back compositing) ---
  // When on, the renderer walks hits front-to-back and composites every
  // transparent fragment ("over", fragment alpha). Off = opaque only. Group
  // alpha (CueMol sections) is separate: Scene::groupBlend renders one extra
  // pass per group and blends the final frames (blendpng equivalence).
  bool transparency = true;
  // When on, the background contributes 0 coverage so the output alpha equals the
  // accumulated transparent coverage (POV "_transpbg"); default = opaque bg.
  bool transparentBackground = false;
  // Safety ceiling on transparent hits walked per primary ray. Normal
  // termination is the opacity early-out (accumulated alpha >= ~1); this only
  // bites pathological deep stacks. The renderer warns once if a ray hits it.
  int maxTransparentLayers = 256;

  // --- Freestyle-style stroke edges (--edges) --- defaulted OFF (enable ==
  // false). When off, no edge AOV is allocated and applyStrokeEdges is never
  // invoked, so output is byte-identical to the no-edge path. This single flag
  // is the master gate for the whole --edges pipeline (G-buffer AOV capture,
  // the stroke pass, and the baked-edge removal); see StrokeEdgeOptions.
  StrokeEdgeOptions strokeEdges;

  // --- Analytic OBJECT-SPACE edges (--obj-edges) --- defaulted OFF. The
  // counterpart of strokeEdges: render() runs generateObjectSpaceEdges()
  // internally (on a private scene copy) before tracing. Mutually exclusive with
  // strokeEdges -- enabling both throws std::runtime_error (they double-draw).
  ObjectSpaceEdgeOptions objectSpaceEdges;
};

}  // namespace umbreon
