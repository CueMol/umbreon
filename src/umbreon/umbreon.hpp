// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// umbreon: a CueMol-embeddable offline renderer backend. Build a Scene
// (geometry, camera, lights, material, fog), call render(), and get a linear
// HDR framebuffer. Depends only on Intel Embree 4 + TBB -- no POV-Ray SDL and
// no OSPRay. This is the public surface intended for static linking into
// libcuemol2; the bench harness drives the same API behind its .pov/.inc parser.
#pragma once

#include <chrono>
#include <memory>

#include "render/render_types.hpp"
#include "scene.hpp"

// Library version (kept in sync with project() in CMakeLists.txt).
#define UMBREON_VERSION_MAJOR 0
#define UMBREON_VERSION_MINOR 1
#define UMBREON_VERSION_PATCH 0

namespace umbreon {

// High-level render. Runs: supersample (opt.supersample) -> Embree primary-ray
// direct shading -> POV ground fog (scene.fog) -> linear box-downsample ->
// assumed_gamma (opt.assumedGamma). opt.width/height are the FINAL output size.
// Returns the final LINEAR HDR framebuffer (top-left pixel origin). The pipeline
// itself lives in render/pipeline.hpp; the image post-process helpers
// (boxDownsample / applyAssumedGamma / srgbEncode8) in postprocess/image_ops.hpp.
FrameResult render(const Scene& scene, const RenderOptions& opt);

// As above, but reports progress and honors cooperative cancellation through
// `progress` (see render/render_progress.hpp). Runs SYNCHRONOUSLY on the calling
// thread -- pass the same `progress` to another thread to poll fraction()/phase()
// or requestCancel() while this call is in flight. On cancel it returns a partial
// FrameResult with cancelled == true; otherwise progress reaches phase() == Done.
// The 2-arg overload above is byte-identical to this one with a fresh progress
// that is never cancelled.
FrameResult render(const Scene& scene, const RenderOptions& opt,
                   RenderProgress& progress);

// Asynchronous render handle. renderAsync() spawns a background worker thread and
// returns immediately; poll progress()/phase()/done(), cancel() cooperatively,
// and get() the finished FrameResult (which rethrows any exception the render
// threw). Move-only. The destructor requests cancel and joins the worker, so
// dropping a RenderTask never leaks the thread. Reads are lock-free.
class RenderTask {
 public:
  RenderTask(RenderTask&&) noexcept;
  RenderTask& operator=(RenderTask&&) noexcept;
  ~RenderTask();
  RenderTask(const RenderTask&) = delete;
  RenderTask& operator=(const RenderTask&) = delete;

  // Overall completion in [0, 1] (RenderProgress::fraction of the worker).
  float progress() const noexcept;
  RenderPhase phase() const noexcept;
  // True once the worker has finished (completed, cancelled, or threw).
  bool done() const noexcept;
  // Request cooperative cancellation; get() then yields a partial, cancelled frame.
  void cancel() noexcept;

  // Block up to `timeout` for completion; returns done().
  bool wait_for(std::chrono::milliseconds timeout) const;
  // Block until the worker finishes.
  void wait() const;
  // Join the worker and return its FrameResult; rethrows a render exception.
  // Call at most once.
  FrameResult get();

 private:
  struct Impl;
  explicit RenderTask(std::unique_ptr<Impl> impl) noexcept;
  friend RenderTask renderAsync(Scene scene, RenderOptions opt);
  std::unique_ptr<Impl> p_;
};

// Start rendering `scene` with `opt` on a background thread and return a handle
// immediately. The scene and options are taken by value (move to avoid the copy),
// so the caller need not keep them alive.
RenderTask renderAsync(Scene scene, RenderOptions opt);

}  // namespace umbreon
