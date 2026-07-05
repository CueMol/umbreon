// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// RenderProgress: a lock-free progress + cooperative-cancel channel for
// umbreon::render(). The renderer WRITES (beginPhase/advance/markDone); a UI
// thread READS (fraction/phase/...) and may requestCancel(). Every field is a
// std::atomic, so reads and cancel are safe from any thread without locking.
// Passing a RenderProgress is optional: the 2-arg render() carries zero progress
// overhead. Pure C++17, no rendering-library dependency.
#pragma once

#include <atomic>
#include <cstdint>

namespace umbreon {

// Coarse render pipeline phase, in execution order. fraction() weights these so
// the value climbs 0 -> 1 across a render. A phase the render skips (e.g.
// GlobalIllum when GI is off) is simply never entered.
enum class RenderPhase : int {
  Idle = 0,
  Setup,        // Embree device + BVH build
  CoarseAo,     // optional coarse-AO pre-pass
  Primary,      // per-pixel primary rays + direct shading (the dominant cost)
  GlobalIllum,  // GI / pt1 indirect post-pass
  Edges,        // NPR stroke / object-space edges
  Postprocess,  // fog, downsample, denoise, gamma
  Done,
};

// Thread-safe (lock-free) progress + cancel channel. Non-copyable; pass by
// reference. Cheap to default-construct.
class RenderProgress {
 public:
  RenderProgress() = default;
  RenderProgress(const RenderProgress&) = delete;
  RenderProgress& operator=(const RenderProgress&) = delete;

  // --- reader side (any thread; all noexcept, lock-free) ---
  // Overall completion in [0, 1], weighted across phases and (for group-alpha
  // multipass) passes. Monotonic non-decreasing over a render. The per-phase
  // weights are NOMINAL (Primary dominates); this is a "roughly how far" figure
  // for a progress bar, not a time estimate.
  float fraction() const noexcept;
  RenderPhase phase() const noexcept {
    return static_cast<RenderPhase>(phase_.load(std::memory_order_relaxed));
  }
  // Completed / total units within the current phase (e.g. image rows). total is
  // never 0 (reported as 1 when a phase is tracked only at its boundary).
  std::uint64_t unitsDone() const noexcept {
    return unitsDone_.load(std::memory_order_relaxed);
  }
  std::uint64_t unitsTotal() const noexcept {
    return unitsTotal_.load(std::memory_order_relaxed);
  }

  // --- cancel side (UI -> renderer, cooperative) ---
  // Request that the running render stop early. The renderer polls this at row
  // and pass boundaries and returns a partial FrameResult with cancelled == true
  // (see FrameResult::cancelled). Idempotent; safe from any thread.
  void requestCancel() noexcept {
    cancel_.store(true, std::memory_order_relaxed);
  }
  bool cancelRequested() const noexcept {
    return cancel_.load(std::memory_order_relaxed);
  }

  // --- writer side (called by the renderer internals; not for consumers) ---
  // Enter multipass slot `passIndex` of `passCount` (group-alpha section blend).
  // The default is a single pass [0, 1). Resets the per-phase counters.
  void beginPass(std::uint64_t passIndex, std::uint64_t passCount) noexcept {
    passCount_.store(passCount ? passCount : 1, std::memory_order_relaxed);
    passIndex_.store(passIndex, std::memory_order_relaxed);
    unitsTotal_.store(1, std::memory_order_relaxed);
    unitsDone_.store(0, std::memory_order_relaxed);
    phase_.store(static_cast<int>(RenderPhase::Idle), std::memory_order_relaxed);
  }
  // Enter phase `p`; `unitsTotal` is the count advance() will sum to (e.g. image
  // rows). Leave at the default 1 for a phase tracked only at its boundary.
  void beginPhase(RenderPhase p, std::uint64_t unitsTotal = 1) noexcept {
    unitsTotal_.store(unitsTotal ? unitsTotal : 1, std::memory_order_relaxed);
    unitsDone_.store(0, std::memory_order_relaxed);
    phase_.store(static_cast<int>(p), std::memory_order_relaxed);
  }
  // Add `n` completed units to the current phase (a plain atomic increment, so
  // it never perturbs the deterministic pixel values).
  void advance(std::uint64_t n) noexcept {
    unitsDone_.fetch_add(n, std::memory_order_relaxed);
  }
  // Mark the whole render finished: fraction() -> 1, phase() -> Done.
  void markDone() noexcept {
    passIndex_.store(0, std::memory_order_relaxed);
    passCount_.store(1, std::memory_order_relaxed);
    unitsTotal_.store(1, std::memory_order_relaxed);
    unitsDone_.store(1, std::memory_order_relaxed);
    phase_.store(static_cast<int>(RenderPhase::Done), std::memory_order_relaxed);
  }

 private:
  std::atomic<int> phase_{static_cast<int>(RenderPhase::Idle)};
  std::atomic<std::uint64_t> unitsDone_{0};
  std::atomic<std::uint64_t> unitsTotal_{1};
  std::atomic<std::uint64_t> passIndex_{0};
  std::atomic<std::uint64_t> passCount_{1};
  std::atomic<bool> cancel_{false};
};

// Human-readable phase name (e.g. for a CLI progress bar). Never null.
const char* toString(RenderPhase p) noexcept;

}  // namespace umbreon
