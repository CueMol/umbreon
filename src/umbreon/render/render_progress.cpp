#include "render/render_progress.hpp"

#include <algorithm>
#include <cmath>

namespace umbreon {
namespace {

// Slot index of a phase within the packed plan. The phases are contiguous in
// execution order, so the enum value maps directly: Idle -> -1 (nothing before
// it) and Done -> kPlanSlots (everything before it), which is exactly what the
// prefix sum in fraction() needs to yield 0 and 1.
int planSlot(RenderPhase p) noexcept { return static_cast<int>(p) - 1; }

}  // namespace

void RenderProgress::setPhasePlan(const RenderPhasePlan& plan) noexcept {
  const float in[kPlanSlots] = {plan.setup, plan.coarseAo, plan.primary,
                                plan.globalIllum, plan.edges, plan.postprocess};
  // Negative / non-finite shares count as zero (a caller's cost model must not
  // be able to make the bar run backwards).
  double sum = 0.0;
  for (int i = 0; i < kPlanSlots; ++i)
    if (std::isfinite(in[i]) && in[i] > 0.0f) sum += static_cast<double>(in[i]);
  if (!(sum > 0.0)) {  // an all-zero plan carries no information: keep defaults
    plan_.store(kDefaultPlanPacked, std::memory_order_relaxed);
    return;
  }

  std::uint64_t packed = 0;
  for (int i = 0; i < kPlanSlots; ++i) {
    const double share =
        (std::isfinite(in[i]) && in[i] > 0.0f) ? static_cast<double>(in[i]) : 0.0;
    // fraction() divides by the ACTUAL stored sum, so per-slot rounding drift
    // is self-correcting -- no largest-remainder pass needed.
    std::uint64_t q = static_cast<std::uint64_t>(
        share / sum * static_cast<double>(kPlanPermille) + 0.5);
    packed |= std::min<std::uint64_t>(q, kPlanMask) << (i * kPlanBits);
  }
  // The largest share is at least sum/kPlanSlots, so it cannot round to 0 and
  // this is unreachable; guard anyway rather than store a divide-by-zero plan.
  if (packed == 0) packed = kDefaultPlanPacked;
  plan_.store(packed, std::memory_order_relaxed);
}

RenderPhasePlan RenderProgress::phasePlan() const noexcept {
  std::uint64_t packed = plan_.load(std::memory_order_relaxed);
  if (packed == 0) packed = kDefaultPlanPacked;
  std::uint64_t w[kPlanSlots];
  std::uint64_t total = 0;
  for (int i = 0; i < kPlanSlots; ++i) {
    w[i] = (packed >> (i * kPlanBits)) & kPlanMask;
    total += w[i];
  }
  RenderPhasePlan out;
  if (total == 0) return out;  // unreachable (see above); return the defaults
  const float inv = 1.0f / static_cast<float>(total);
  out.setup = static_cast<float>(w[0]) * inv;
  out.coarseAo = static_cast<float>(w[1]) * inv;
  out.primary = static_cast<float>(w[2]) * inv;
  out.globalIllum = static_cast<float>(w[3]) * inv;
  out.edges = static_cast<float>(w[4]) * inv;
  out.postprocess = static_cast<float>(w[5]) * inv;
  return out;
}

float RenderProgress::fraction() const noexcept {
  // One load: a torn read of the plan would be a non-monotonic blip (see the
  // note on plan_).
  std::uint64_t packed = plan_.load(std::memory_order_relaxed);
  if (packed == 0) packed = kDefaultPlanPacked;
  std::uint64_t w[kPlanSlots];
  std::uint64_t total = 0;
  for (int i = 0; i < kPlanSlots; ++i) {
    w[i] = (packed >> (i * kPlanBits)) & kPlanMask;
    total += w[i];
  }

  // Cumulative weight of the earlier phases, so fraction() is monotonic in
  // phase order. A zero-share phase contributes no span: entering it leaves the
  // bar exactly where the previous phase ended, so a skipped phase is a no-op
  // rather than a jump.
  const int slot = planSlot(phase());
  std::uint64_t base = 0;
  for (int i = 0; i < slot && i < kPlanSlots; ++i) base += w[i];
  const std::uint64_t span =
      (slot >= 0 && slot < kPlanSlots) ? w[slot] : 0;  // Idle / Done: no span

  const std::uint64_t units = unitsTotal_.load(std::memory_order_relaxed);
  const std::uint64_t done = unitsDone_.load(std::memory_order_relaxed);
  float within = 0.0f;
  if (units > 0) {
    within = static_cast<float>(done) / static_cast<float>(units);
    within = std::min(1.0f, std::max(0.0f, within));
  }
  const float inPass =
      (static_cast<float>(base) + static_cast<float>(span) * within) /
      static_cast<float>(total);  // [0, 1] within a pass

  const std::uint64_t passCount =
      std::max<std::uint64_t>(1, passCount_.load(std::memory_order_relaxed));
  const std::uint64_t passIndex = std::min<std::uint64_t>(
      passCount - 1, passIndex_.load(std::memory_order_relaxed));
  const float f =
      (static_cast<float>(passIndex) + inPass) / static_cast<float>(passCount);
  return std::min(1.0f, std::max(0.0f, f));
}

const char* toString(RenderPhase p) noexcept {
  switch (p) {
    case RenderPhase::Idle:        return "Idle";
    case RenderPhase::Setup:       return "Setup";
    case RenderPhase::CoarseAo:    return "CoarseAO";
    case RenderPhase::Primary:     return "Primary";
    case RenderPhase::GlobalIllum: return "GI";
    case RenderPhase::Edges:       return "Edges";
    case RenderPhase::Postprocess: return "Postprocess";
    case RenderPhase::Done:        return "Done";
  }
  return "Unknown";
}

}  // namespace umbreon
