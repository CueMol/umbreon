#include "umbreon.hpp"

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

#include "postprocess/image_ops.hpp"
#include "render/pipeline.hpp"
#include "log.hpp"
#include "render/scene_build.hpp"  // embreeErrorCallback

namespace umbreon {
namespace {

// Owns one RTCDevice for the lifetime of a multipass render. Embree's device
// setup (task scheduler init) is per-device, not per-scene, so the group-alpha
// passes share one instead of each building and tearing down their own.
class SharedDevice {
 public:
  SharedDevice() : device_(rtcNewDevice(nullptr)) {
    // A null device is not fatal here: renderFrame falls back to creating its
    // own per pass, which is exactly the previous behaviour.
    // Same error handler renderFrame installs on a device it owns, so a
    // shared device reports Embree errors identically.
    if (device_) rtcSetDeviceErrorFunction(device_, detail::embreeErrorCallback, nullptr);
  }
  ~SharedDevice() {
    if (device_) rtcReleaseDevice(device_);
  }
  SharedDevice(const SharedDevice&) = delete;
  SharedDevice& operator=(const SharedDevice&) = delete;
  RTCDevice get() const { return device_; }

 private:
  RTCDevice device_;
};

// Rewrite `out` as `s` minus every primitive whose group is flagged in `hide`
// (indexed by group id; ids beyond the mask are kept). Vertex buffers are kept
// wholesale -- only the triangle index list and its per-tri side tables shrink
// -- so unreferenced vertices stay in the buffers, which the tracer never
// visits.
//
// `out` is the multipass WORKING COPY, not a fresh Scene: it already carries
// s's vertex buffers (positions / normals / colors are identical in every
// pass and can be tens of MB), so a pass rewrites only the primitive lists
// instead of copying the whole mesh again. `meshDropped` tracks the one case
// that invalidates that -- a pass whose geometry is entirely hidden has to
// clear the mesh -- so the next pass knows to restore the buffers.
void applyHideGroups(const Scene& s, const std::vector<uint8_t>& hide,
                     Scene& out, bool& meshDropped) {
  auto hidden = [&](uint16_t g) {
    return g < hide.size() && hide[g] != 0;
  };

  if (meshDropped) {
    out.mesh = s.mesh;  // a previous pass cleared it (see below)
    meshDropped = false;
  }

  const std::size_t ntri = s.mesh.triangleCount();
  std::vector<uint32_t> idx;
  std::vector<uint8_t> mat;
  std::vector<uint16_t> grp;
  idx.reserve(ntri * 3);
  for (std::size_t t = 0; t < ntri; ++t) {
    if (hidden(s.mesh.groupForTri(t))) continue;
    for (int k = 0; k < 3; ++k)
      idx.push_back(s.mesh.cornerVertex(t * 3 + static_cast<std::size_t>(k)));
    if (!s.mesh.triMaterialId.empty()) mat.push_back(s.mesh.triMaterialId[t]);
    if (!s.mesh.triGroupId.empty()) grp.push_back(s.mesh.triGroupId[t]);
  }
  if (idx.empty()) {
    // Every triangle was hidden. An empty `index` means the de-indexed
    // (soup) fallback, which would resurrect ALL original vertices as
    // triangles -- drop the mesh entirely instead.
    out.mesh = Mesh{};
    meshDropped = true;
  } else {
    out.mesh.index = std::move(idx);
    out.mesh.triMaterialId = std::move(mat);
    out.mesh.triGroupId = std::move(grp);
  }

  out.spheres.clear();
  for (const Sphere& sp : s.spheres)
    if (!hidden(sp.group)) out.spheres.push_back(sp);
  out.cylinders.clear();
  for (const Cylinder& cy : s.cylinders)
    if (!hidden(cy.group)) out.cylinders.push_back(cy);
}

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
// (renderFrame), and the image post-process helpers in postprocess/image_ops.cpp.
//
// Group-alpha (CueMol section) transparency is realized HERE, as the closed
// form of CueMol's blendpng postprocess (blendpng.cpp: solvebeta + the
// front-to-back lerp chain reduce exactly to):
//   out = (1 - sum_i a_i) * render(scene minus every blend group)
//       + sum_i a_i * render(scene with group i kept, other blend groups hidden)
// Each pass runs the FULL pipeline -- direct shading, GI, fog, edges, denoise,
// gamma -- on its own geometry subset, and the blend combines the final
// display-encoded framebuffers, exactly like blendpng combines the finished
// PNG layers. GI therefore sees each pass's geometry consistently: the
// background pass gathers without the blend groups occluding, each layer pass
// with its group fully opaque.
// Shared body for both public render() overloads. `progress` is null for the
// zero-overhead 2-arg path; when non-null it is threaded into renderFrame (phase
// / row progress + cooperative cancel) and marked Done on a successful finish.
static FrameResult renderImpl(const Scene& scene, const RenderOptions& opt,
                              RenderProgress* progress) {
  if (scene.groupBlend.empty()) {
    FrameResult f = renderFrame(scene, opt, progress);
    logPt1Timing(opt, f.pt1Timing, f.pt1Rays, 1);
    if (progress && !f.cancelled) progress->markDone();
    return f;
  }

  float sumA = 0.0f;
  uint16_t maxGroup = 0;
  for (const GroupBlend& gb : scene.groupBlend) {
    sumA += gb.alpha;
    maxGroup = std::max(maxGroup, gb.group);
  }
  // The background weight is NEGATIVE once the blend weights sum to more than
  // 1, and that is the correct value rather than an error to clamp away. What
  // makes the blend faithful is that the pass weights sum to exactly 1:
  // geometry outside every blend group appears identically in all passes, so
  // it is reproduced unchanged only while (1 - sum) + sum == 1. Clamping the
  // background weight to 0 leaves the total at `sum`, which scales the whole
  // frame by that factor and clips opaque geometry to white. blendpng, whose
  // closed form this is, lets the same coefficient go negative (its
  // solvebeta + front-to-back lerp chain produces 1 - sum(beta) directly).
  const float bgW = 1.0f - sumA;

  std::vector<uint8_t> hideAll(static_cast<std::size_t>(maxGroup) + 1, 0);
  for (const GroupBlend& gb : scene.groupBlend) hideAll[gb.group] = 1;

  // Accumulate w * pass color into `acc` -- RGB in the sRGB-ENCODED domain
  // (blendpng blends the finished 8-bit PNGs, whose RGB is the sRGB encode of
  // FrameResult.color; alpha is stored linear in the PNG and blends as-is).
  // The LAST rendered pass is kept whole as the carrier frame so the
  // non-color outputs (edge G-buffer, GI guides, depth) come from a real
  // render -- the final layer pass, which for the common single-group case is
  // the full scene. Zero-weight passes still render: skipping them would
  // silently change which pass carries those.
  // One device for every pass of this render (see SharedDevice).
  const SharedDevice device;

  // One working scene for every pass: the vertex buffers are pass-invariant, so
  // they are copied once here and each pass rewrites only its primitive lists
  // (applyHideGroups). Nested blending is off in a pass by construction.
  Scene work = scene;
  work.groupBlend.clear();
  bool workMeshDropped = false;

  FrameResult carrier;
  std::vector<float> acc;
  double seconds = 0.0;
  Pt1Timing timing{};
  // Each blend group renders as its own full-pipeline pass; report progress as
  // one slot per pass (background + one per group) so fraction() spans them.
  const std::uint64_t passCount = 1 + scene.groupBlend.size();
  std::uint64_t passIndex = 0;
  bool cancelled = false;
  auto addPass = [&](const std::vector<uint8_t>& hide, float w) {
    if (cancelled) return;  // a prior pass was cancelled: stop the chain
    if (progress) progress->beginPass(passIndex, passCount);
    ++passIndex;
    // Group-alpha transparency costs one FULL render per blend group plus one
    // for the background, which is the single most surprising thing about a
    // transparent scene's render time. Say so, per pass.
    logMessage(LogLevel::Info, "group-alpha pass %llu/%llu (weight %.3f)",
               static_cast<unsigned long long>(passIndex),
               static_cast<unsigned long long>(passCount), w);
    applyHideGroups(scene, hide, work, workMeshDropped);
    FrameResult f = renderFrame(work, opt, progress, device.get());
    if (f.cancelled) cancelled = true;
    seconds += f.renderSeconds;
    timing.bvhBuild += f.pt1Timing.bvhBuild;
    timing.primary += f.pt1Timing.primary;
    timing.direct += f.pt1Timing.direct;
    timing.gather += f.pt1Timing.gather;
    timing.denoise += f.pt1Timing.denoise;
    timing.upsample += f.pt1Timing.upsample;
    timing.total += f.pt1Timing.total;
    if (acc.empty()) acc.assign(f.color.size(), 0.0f);
    const std::size_t n = std::min(acc.size(), f.color.size()) / 4;
    for (std::size_t p = 0; p < n; ++p) {
      for (int c = 0; c < 3; ++c)
        acc[p * 4 + c] += w * srgbEncodeF(f.color[p * 4 + c]);
      acc[p * 4 + 3] += w * f.color[p * 4 + 3];
    }
    carrier = std::move(f);
  };

  addPass(hideAll, bgW);
  for (const GroupBlend& gb : scene.groupBlend) {
    std::vector<uint8_t> hide = hideAll;
    hide[gb.group] = 0;  // keep this group (opaque), hide the other layers
    addPass(hide, gb.alpha);
  }

  // Map the blended sRGB values back to FrameResult's linear-ish domain so
  // the image writer's own sRGB encode reproduces the blend exactly. The
  // blend is affine and every encoded input sits in [0, 1], so only a > 1
  // weight sum needs the lower clamp. Alpha is coverage, clamped to [0, 1].
  const std::size_t npix = acc.size() / 4;
  for (std::size_t p = 0; p < npix; ++p) {
    for (int c = 0; c < 3; ++c)
      acc[p * 4 + c] = srgbDecodeF(std::fmax(0.0f, acc[p * 4 + c]));
    acc[p * 4 + 3] =
        std::fmin(1.0f, std::fmax(0.0f, acc[p * 4 + 3]));
  }

  carrier.color = std::move(acc);
  carrier.renderSeconds = seconds;
  carrier.pt1Timing = timing;
  carrier.cancelled = cancelled;
  logPt1Timing(opt, timing, carrier.pt1Rays, passCount);
  if (progress && !cancelled) progress->markDone();
  return carrier;
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
