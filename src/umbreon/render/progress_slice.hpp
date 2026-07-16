// Internal: a contiguous slice of a phase's unit budget, handed to one
// sub-stage. Implementation detail; may change without notice (not installed).
//
// The GlobalIllum phase is not one loop but a chain of sub-stages -- G-buffer,
// gather, denoise, upsample, rim patch, composite -- whose costs differ by
// orders of magnitude and reorder with spp (the denoise dominates at spp=8, the
// gather at spp=32). Each sub-stage gets a FIXED [base, base + span) slice of
// the phase budget and always consumes exactly its span. So a wrong sub-stage
// estimate only changes the PACE inside GI, never the endpoint: the phase lands
// on exactly 1.0 either way, and fraction() stays monotone with no clamping
// heuristic anywhere.
#pragma once

#include <algorithm>
#include <cstdint>

#include "render/render_progress.hpp"

namespace umbreon {
namespace detail {

struct ProgressSlice {
  RenderProgress* progress = nullptr;  // null => the zero-overhead 2-arg render
  std::uint64_t base = 0;
  std::uint64_t span = 0;

  bool cancelled() const noexcept {
    return progress && progress->cancelRequested();
  }

  // Worker threads: report that `done` out of `total` units of THIS sub-stage's
  // work finished. A plain relaxed atomic add (RenderProgress::advance), so it
  // cannot perturb the deterministic pixel values or the thread-count
  // invariance. Per-chunk rounding always floors, i.e. undershoots the span;
  // finish() tops the remainder up.
  void addWork(std::uint64_t done, std::uint64_t total) const noexcept {
    if (progress && total) progress->advance(span * done / total);
  }

  // Report an ABSOLUTE position in [0, 1] within this sub-stage. Used by OIDN's
  // filter monitor, which hands out a fraction rather than an increment and may
  // do so from several worker threads out of order -- advanceTo() is a monotone
  // seek, so a stale report is simply ignored.
  void reportAbs(double f) const noexcept {
    if (!progress) return;
    const double c = std::min(1.0, std::max(0.0, f));
    progress->advanceTo(base + static_cast<std::uint64_t>(
                                   static_cast<double>(span) * c));
  }

  // Driver thread, at the sub-stage boundary: land exactly on the slice end,
  // absorbing the per-chunk truncation above. advanceTo() never rewinds, so
  // this cannot double-count work already reported.
  void finish() const noexcept {
    if (progress) progress->advanceTo(base + span);
  }
};

}  // namespace detail
}  // namespace umbreon
