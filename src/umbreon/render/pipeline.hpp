// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// The umbreon frame pipeline: orchestrates a single render -- supersample ->
// Embree primary-ray direct shading -> POV ground fog -> Freestyle stroke edges
// -> linear box-downsample -> assumed_gamma -> final linear HDR FrameResult.
// Split out of umbreon.cpp so the public umbreon.cpp stays a thin entry point.
// Internal API (not installed); the public surface is umbreon::render().
#pragma once

#include <embree4/rtcore.h>

#include "render/render_types.hpp"
#include "scene.hpp"

namespace umbreon {

// The frame at the stage boundary between the RAW pipeline (render, fog, hi-res
// stroke edges) and the FINISHING pipeline (downsample, denoise, gamma): still
// at the supersampled grid and in linear light, so per-sample coverage and the
// hi-res AOVs (objectId / viewZ) still describe single surfaces. The group-alpha
// per-pixel blend composites here; a plain render just finishes it.
struct RawFrame {
  FrameResult frame;
  /// Options as renderFrameRaw normalized them (supersampled width/height, the
  /// resolved gather / AO / hatch sentinels). finishFrame needs them.
  RenderOptions hi;
  int ss = 1;
  /// The FINAL (output) size; frame is ss times that until finishFrame runs.
  int finalW = 0;
  int finalH = 0;
};

// Run the raw stage and return the frame at that boundary. A cancelled render
// comes back with frame.cancelled set and the later stages skipped.
RawFrame renderFrameRaw(const Scene& scene, const RenderOptions& opt,
                        RenderProgress* progress = nullptr,
                        RTCDevice sharedDevice = nullptr);

// Run the finishing stage in place: hi-res ink, box-downsample to finalW/finalH,
// denoise, assumed_gamma, ink composite. `hi` / `ss` / `finalW` / `finalH` come
// from the RawFrame; `opt` is the caller's original options.
void finishFrame(FrameResult& frame, const Scene& scene,
                 const RenderOptions& opt, const RenderOptions& hi, int ss,
                 int finalW, int finalH, RenderProgress* progress = nullptr);

// Run the full frame pipeline at opt.supersample and return the final LINEAR HDR
// framebuffer (top-left pixel origin). opt.width/height are the FINAL output size.
// When `progress` is non-null it receives phase/row updates and is polled for
// cooperative cancellation at pass boundaries (a cancelled render returns a
// partial frame with cancelled == true). Null keeps the default path unchanged.
//
// `sharedDevice` lets a caller that renders several frames back to back -- the
// group-alpha multipass in umbreon.cpp -- supply ONE RTCDevice for all of them
// instead of paying Embree's device setup per frame. The caller keeps
// ownership; nullptr creates and releases a device per frame, as before.
FrameResult renderFrame(const Scene& scene, const RenderOptions& opt,
                        RenderProgress* progress = nullptr,
                        RTCDevice sharedDevice = nullptr);

}  // namespace umbreon
