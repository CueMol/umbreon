#include "render/render_progress.hpp"

#include <algorithm>

namespace umbreon {
namespace {

// Nominal weight of each phase within one render pass; the entries sum to 1.
// These are rough cost shares (Primary dominates) used only to make fraction()
// climb smoothly -- they are NOT a time estimate. `base` is the cumulative
// weight of all earlier phases, so fraction() is monotonic in phase order.
struct PhaseSpan {
  float base;
  float weight;
};

PhaseSpan phaseSpan(RenderPhase p) noexcept {
  switch (p) {
    case RenderPhase::Idle:        return {0.00f, 0.00f};
    case RenderPhase::Setup:       return {0.00f, 0.05f};
    case RenderPhase::CoarseAo:    return {0.05f, 0.05f};
    case RenderPhase::Primary:     return {0.10f, 0.65f};
    case RenderPhase::GlobalIllum: return {0.75f, 0.15f};
    case RenderPhase::Edges:       return {0.90f, 0.05f};
    case RenderPhase::Postprocess: return {0.95f, 0.05f};
    case RenderPhase::Done:        return {1.00f, 0.00f};
  }
  return {0.00f, 0.00f};
}

}  // namespace

float RenderProgress::fraction() const noexcept {
  const PhaseSpan span = phaseSpan(phase());
  const std::uint64_t total = unitsTotal_.load(std::memory_order_relaxed);
  const std::uint64_t done = unitsDone_.load(std::memory_order_relaxed);
  float within = 0.0f;
  if (total > 0) {
    within = static_cast<float>(done) / static_cast<float>(total);
    within = std::min(1.0f, std::max(0.0f, within));
  }
  const float inPass = span.base + span.weight * within;  // [0, 1] within a pass

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
