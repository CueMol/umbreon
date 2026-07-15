// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Edge-aware denoise post-pass for the diffuse-GI residual noise. Runs on the
// final-resolution linear HDR color AFTER the supersample downsample and BEFORE
// the assumed-gamma encode (OIDN/SVGF operate in linear HDR). The supersample
// box-average is the primary denoise; this is the finishing pass that removes the
// low-frequency irradiance-cache blotch the box-average leaves behind.
#pragma once

#include "render/render_types.hpp"

namespace umbreon {

// Denoiser backend selector (RenderOptions::denoiser holds the int value).
enum class DenoiserBackend { None = 0, AtrousBilateral = 1, OIDN = 2 };

// Built-in, dependency-free edge-avoiding a-trous wavelet denoiser (Dammertz
// 2010) with SVGF edge-stop functions. Smooths frame.color in place using the
// G-buffer guides (frame.position as depth, frame.normal) plus the color's own
// luminance and a spatial variance estimate, so flat noisy regions converge while
// depth/normal/illumination edges are preserved. No-op when a guide is missing.
// When opt.denoiseDemodulateAlbedo and frame.albedo is present, the illumination
// (color / albedo) is denoised and re-multiplied, so per-section flat colors are
// not blurred across their boundaries. Background pixels (alpha ~ 0) are left
// untouched. Deterministic: a pure function of the input buffers.
void denoiseAtrous(FrameResult& frame, const RenderOptions& opt);

// Intel Open Image Denoise backend (DenoiserBackend::OIDN), out of process:
// the umbreon_oidn_worker executable is spawned once (lazily) and driven over
// pipes + shared memory by ipc/oidn_client.hpp; libumbreon itself does not
// link OIDN. Defined only when the library is built with UMBREON_HAVE_OIDN
// (CMake option UMBREON_WITH_OIDN + a located Boost package); callers must
// guard the call with the same macro. Denoises frame.color (linear HDR) with
// the "RT" filter, using frame.albedo / frame.normal as clean auxiliary
// guides. Background pixels (no geometry) keep their original color. When the
// worker is unavailable or fails, falls back to denoiseAtrous with a stderr
// warning (see RenderOptions::oidnWorkerPath for how the worker is located).
// Returns true when the worker processed the frame, false when it fell back to
// a-trous (callers use this to fill FrameResult::denoiserUsed = 2 vs 1).
bool denoiseOidn(FrameResult& frame, const RenderOptions& opt);

}  // namespace umbreon
