#include "umbreon.hpp"

#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

#include "blend/group_blend.hpp"
#include "log.hpp"
#include "render/pipeline.hpp"

namespace umbreon {
namespace {

/// Report a finished GI render's per-stage cost. Info level, so it reaches a
/// host that installed a log sink and stays off the CLI's stderr (the bench
/// prints its own report from the same FrameResult).
void logPt1Timing(const RenderOptions& opt, const Pt1Timing& t,
                  const Pt1RayCounts& rays, std::uint64_t passCount) {
  if (!opt.gi) return;
  logMessage(LogLevel::Info,
             "pt1 timing (%llu pass%s): bvh_build %.3f  primary %.3f  "
             "direct %.3f  gather %.3f  denoise %.3f  upsample %.3f  "
             "total %.3f (s)",
             static_cast<unsigned long long>(passCount),
             passCount == 1 ? "" : "es", t.bvhBuild, t.primary, t.direct,
             t.gather, t.denoise, t.upsample, t.total);
  // Ray counts turn the timing into a rate, which is what tells a slow render
  // from a big one: the same seconds mean different things at 5 and at 50
  // Mrays/s. Only the LAST pass's counters survive in the carrier frame, so
  // this is per-pass rather than a total.
  const double total = double(rays.gatherRays + rays.neeRays + rays.gbufferRays);
  if (total > 0.0 && t.total > 0.0) {
    logMessage(LogLevel::Info,
               "  last pass: %.2f Mrays (gather %.2f, NEE %.2f, gbuffer %.2f)",
               total / 1e6, double(rays.gatherRays) / 1e6,
               double(rays.neeRays) / 1e6, double(rays.gbufferRays) / 1e6);
  }
}

}  // namespace

// Public entry point: the full frame pipeline lives in render/pipeline.cpp
// (renderFrame), the group-alpha (CueMol section) transparency blend in
// blend/group_blend.cpp, and the image post-process helpers in
// postprocess/image_ops.cpp.
//
// Shared body for both public render() overloads. `progress` is null for the
// zero-overhead 2-arg path; when non-null it is threaded into renderFrame (phase
// / row progress + cooperative cancel) and marked Done on a successful finish.
static FrameResult renderImpl(const Scene& scene, const RenderOptions& opt,
                              RenderProgress* progress) {
  // No group alpha: one pass, no blending. Every transparent surface then
  // composites front-to-back "over" (fragment alpha) inside that pass.
  std::uint64_t passCount = 1;
  FrameResult f = scene.groupBlend.empty()
                      ? renderFrame(scene, opt, progress)
                      : renderGroupBlend(scene, opt, progress, passCount);
  logPt1Timing(opt, f.pt1Timing, f.pt1Rays, passCount);
  if (progress && !f.cancelled) progress->markDone();
  return f;
}

// Public entry points -----------------------------------------------------------

FrameResult render(const Scene& scene, const RenderOptions& opt) {
  return renderImpl(scene, opt, nullptr);
}

FrameResult render(const Scene& scene, const RenderOptions& opt,
                   RenderProgress& progress) {
  return renderImpl(scene, opt, &progress);
}

// Background render handle (PIMPL) -----------------------------------------------
// Impl owns the worker thread, the progress channel, and the marshalled result /
// exception. Its destructor cancels and joins, so RenderTask's move and destroy
// are trivial (defaulted) and never leak the thread.
struct RenderTask::Impl {
  RenderProgress progress;
  Scene scene;
  RenderOptions opt;
  FrameResult result;
  std::exception_ptr err;
  std::atomic<bool> done{false};
  std::mutex m;
  std::condition_variable cv;
  std::thread worker;

  ~Impl() {
    if (worker.joinable()) {
      progress.requestCancel();
      worker.join();
    }
  }
};

RenderTask::RenderTask(std::unique_ptr<Impl> impl) noexcept
    : p_(std::move(impl)) {}
RenderTask::RenderTask(RenderTask&&) noexcept = default;
RenderTask& RenderTask::operator=(RenderTask&&) noexcept = default;
RenderTask::~RenderTask() = default;

float RenderTask::progress() const noexcept { return p_->progress.fraction(); }
RenderPhase RenderTask::phase() const noexcept { return p_->progress.phase(); }
bool RenderTask::done() const noexcept {
  return p_->done.load(std::memory_order_acquire);
}
void RenderTask::cancel() noexcept { p_->progress.requestCancel(); }

bool RenderTask::wait_for(std::chrono::milliseconds timeout) const {
  std::unique_lock<std::mutex> lk(p_->m);
  return p_->cv.wait_for(lk, timeout, [this] {
    return p_->done.load(std::memory_order_acquire);
  });
}

void RenderTask::wait() const {
  std::unique_lock<std::mutex> lk(p_->m);
  p_->cv.wait(lk,
              [this] { return p_->done.load(std::memory_order_acquire); });
}

FrameResult RenderTask::get() {
  if (p_->worker.joinable()) p_->worker.join();
  if (p_->err) std::rethrow_exception(p_->err);
  return std::move(p_->result);
}

RenderTask renderAsync(Scene scene, RenderOptions opt) {
  auto impl = std::make_unique<RenderTask::Impl>();
  impl->scene = std::move(scene);
  impl->opt = std::move(opt);
  RenderTask::Impl* raw = impl.get();
  // Start the worker LAST: it reads scene/opt, which must be fully in place.
  impl->worker = std::thread([raw] {
    try {
      raw->result = render(raw->scene, raw->opt, raw->progress);
    } catch (...) {
      raw->err = std::current_exception();
    }
    {
      std::lock_guard<std::mutex> lk(raw->m);
      raw->done.store(true, std::memory_order_release);
    }
    raw->cv.notify_all();
  });
  return RenderTask(std::move(impl));
}

}  // namespace umbreon
