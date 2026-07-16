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

// Coarse render pipeline phase, in execution order. fraction() weights these
// with a RenderPhasePlan so the value climbs 0 -> 1 across a render. A phase
// the render skips (e.g. GlobalIllum when GI is off) is simply never entered.
enum class RenderPhase : int {
  Idle = 0,
  Setup,        // Embree device + BVH build
  CoarseAo,     // optional coarse-AO pre-pass
  Primary,      // per-pixel primary rays + direct shading
  GlobalIllum,  // GI / pt1 indirect post-pass (dominant whenever GI is on)
  Edges,        // NPR stroke / object-space edges
  Postprocess,  // fog, downsample, denoise, gamma
  Done,
};

// Relative cost shares of the phases within one render pass. The renderer
// declares these per pass (RenderProgress::setPhasePlan) from the resolved
// RenderOptions, so the bar tracks where the time actually goes: GI dominates
// whenever it is on, while a GI-off render spends its time in Primary. A fixed
// table cannot express both.
//
// Shares are RELATIVE and may use any unit -- fraction() normalizes by their
// sum -- so the renderer simply passes estimated seconds. A ZERO share means
// the phase does not run in this pass: it collapses to an empty span, so a
// skipped phase leaves no gap instead of jumping the bar.
//
// The defaults reproduce the historical fixed table.
struct RenderPhasePlan {
  float setup = 0.05f;
  float coarseAo = 0.05f;
  float primary = 0.65f;
  float globalIllum = 0.15f;
  float edges = 0.05f;
  float postprocess = 0.05f;
};

// Thread-safe (lock-free) progress + cancel channel. Non-copyable; pass by
// reference. Cheap to default-construct.
class RenderProgress {
 public:
  RenderProgress() = default;
  RenderProgress(const RenderProgress&) = delete;
  RenderProgress& operator=(const RenderProgress&) = delete;

  // --- reader side (any thread; all noexcept, lock-free) ---
  // Overall completion in [0, 1], weighted by the phase plan (see setPhasePlan)
  // and, for group-alpha multipass, across passes. Monotonic non-decreasing over
  // a render. The plan is an ESTIMATE, so this is a "roughly how far" figure for
  // a progress bar rather than an exact time prediction -- but a phase that
  // costs most of the render is given most of the bar.
  float fraction() const noexcept;
  // The plan currently in effect, renormalized to sum to 1 (a segmented UI can
  // size its phase ticks from this). Never returns an all-zero plan.
  RenderPhasePlan phasePlan() const noexcept;
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
  // Declare the relative phase costs for this pass (see RenderPhasePlan). Call
  // before the pass's first beginPhase(): while the phase is Idle, fraction() is
  // passIndex/passCount under ANY plan, so a reader cannot observe a
  // plan-induced drop. Shares that are negative or non-finite count as 0; an
  // all-zero plan falls back to the defaults.
  void setPhasePlan(const RenderPhasePlan& plan) noexcept;
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
  // Monotonic seek: raise the completed-unit count to `unitsDone`, never lower
  // it. Needed wherever progress arrives as an ABSOLUTE position instead of an
  // increment -- OIDN's filter monitor reports a [0, 1] fraction, and may do so
  // from worker threads out of order -- and to land a sub-stage exactly on the
  // end of its slice after per-chunk rounding has undershot it.
  void advanceTo(std::uint64_t unitsDone) noexcept {
    std::uint64_t cur = unitsDone_.load(std::memory_order_relaxed);
    while (cur < unitsDone &&
           !unitsDone_.compare_exchange_weak(cur, unitsDone,
                                             std::memory_order_relaxed,
                                             std::memory_order_relaxed)) {
    }
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
  // The plan lives in ONE atomic word rather than six: fraction() prefix-sums
  // the shares, so a torn read (half old plan, half new) would be precisely a
  // non-monotonic blip in the bar. Six 10-bit permille slots -- Setup, CoarseAo,
  // Primary, GlobalIllum, Edges, Postprocess; Idle and Done always weigh 0 --
  // fit in 60 bits, and one 64-bit load keeps the reader path cheap. 10 bits
  // resolves 0.1%, enough for the default table to sum to exactly 1000 and so
  // reproduce the historical fractions with no rounding drift.
  static constexpr int kPlanSlots = 6;
  static constexpr int kPlanBits = 10;
  static constexpr std::uint64_t kPlanMask = (1ull << kPlanBits) - 1;
  static constexpr std::uint64_t kPlanPermille = 1000;
  // Default shares in permille, in slot order: Setup 50, CoarseAo 50,
  // Primary 650, GlobalIllum 150, Edges 50, Postprocess 50. Sums to exactly
  // 1000, which is what makes the historical fractions exact.
  static constexpr std::uint64_t kDefaultPlanPacked =
      50ull | (50ull << kPlanBits) | (650ull << (2 * kPlanBits)) |
      (150ull << (3 * kPlanBits)) | (50ull << (4 * kPlanBits)) |
      (50ull << (5 * kPlanBits));

  std::atomic<int> phase_{static_cast<int>(RenderPhase::Idle)};
  std::atomic<std::uint64_t> unitsDone_{0};
  std::atomic<std::uint64_t> unitsTotal_{1};
  std::atomic<std::uint64_t> passIndex_{0};
  std::atomic<std::uint64_t> passCount_{1};
  std::atomic<std::uint64_t> plan_{kDefaultPlanPacked};
  std::atomic<bool> cancel_{false};
};

// Human-readable phase name (e.g. for a CLI progress bar). Never null.
const char* toString(RenderPhase p) noexcept;

}  // namespace umbreon
