// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt1: path-traced indirect diffuse integrator (one bounce, per-pixel
// brute-force cosine-hemisphere gather). FROZEN as the regression anchor:
// pt1's own behavior is bit-identical to its 2026-07 form forever, while pt2
// (integrator/pt2/, the DEFAULT since 2026-07) layers its extensions onto the
// same gather core through the Pt1GatherExt seam below. Historically an
// ALTERNATIVE to the irradiance cache post-pass: the main render loop keeps
// computing direct lighting unchanged, and pt1 replaces only the cache's
// placement/fill/interpolation stages with a per-pixel gather using the SAME
// radiance evaluators (oneBounceRadiance / environmentRadiance), so the
// integrators are unit-compatible for A/B comparison by construction. That
// shared core is why this header still includes
// experimental/irradiance_cache/ (IrradianceCacheParams, environmentRadiance
// and the ray helpers live there); extracting the shared types into a neutral
// header is a follow-up, not part of the directory move.
//
// Energy convention (matches the cache, see irradiance_cache.hpp): the E
// buffer stores E_stored = mean(L_i) over cosine-weighted samples = E_true/pi,
// and the composite multiplies by the receiver reflectance kd*pigment WITHOUT
// a 1/pi. The plan's estimator (pi/N)*sum(L_i) maps to E_stored=(1/N)*sum(L_i).
//
// Sky/emission division (double-counting guards):
//   - The sky enters ONLY through the gather's miss term (environmentRadiance);
//     the direct pass has no sky. Env-dome lights (--env-light) are DIRECT
//     distant lights, so combining them with any GI integrator double-counts
//     the sky energy -- the renderer warns on that combination.
//   - A gather-ray hit contributes its REFLECTED direct light only (no
//     emission): light travelling from a source straight to the receiver is
//     already counted by the direct pass, so adding emission at the bounce
//     point would double-count it. oneBounceRadiance implements exactly this.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range2d.h>

#include "ao/ambient_occlusion.hpp"
#include "experimental/irradiance_cache/irradiance_cache.hpp"
#include "render/progress_slice.hpp"
#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

// Umbrella: the integrator is split into the G-buffer half (data types +
// tracePt1GBuffer) and the gather half (pt1EvalVertex / pt1GatherPoint /
// gatherPt1Grid); both stay header-inline. This header keeps providing the
// full historical surface plus the sole out-of-line function below.
#include "integrator/pt1/pt1_gather.hpp"
#include "integrator/pt1/pt1_gbuffer.hpp"

namespace umbreon {
namespace detail {

// Denoise the E buffer (w*h*3 indirect irradiance, pre-composite) in place:
// NaN/Inf scrub, then OIDN with albedo/normal guides (a-trous fallback when
// the build has no OIDN). albedo is the receiver reflectance kd*pigment in
// [0,1] (the giRefl side-channel at full res, the pt1 G-buffer at half res);
// normal marks background with zero vectors; position feeds the a-trous depth
// edge-stop. Any guide may be null. Defined in pt1_denoise.cpp (NOT inline:
// UMBREON_HAVE_OIDN is a target-private macro, see there). Returns the
// DenoiserBackend that actually ran (2=OIDN, 1=a-trous fallback, 0=degenerate
// no-op or a cancelled filter) for FrameResult::pt1DenoiserUsed.
// `prog` animates the bar through the OIDN filter and cancels it on request;
// see denoiseOidn.
int denoisePt1E(int w, int h, std::vector<float>& E, const float* albedo,
                const float* normal, const float* position,
                const RenderOptions& opt,
                const ProgressSlice* prog = nullptr);

}  // namespace detail
}  // namespace umbreon
