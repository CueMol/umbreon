// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Group-alpha (CueMol section) transparency: the multipass blend behind
// Scene::groupBlend.
//
// A section is rendered OPAQUE in its own pass and the passes are combined
// afterwards, instead of blending each overlapping primitive of the section
// over the next (which double-darkens the overlaps). Blend entries that share
// an alpha are ONE VEIL sharing one pass, so the veil is weighed once.
//
// Both modes plan the same passes -- one background pass (every veil hidden)
// plus one per veil -- and differ only in where and how the passes are
// combined (RenderOptions::groupBlendMode):
//
//   LayerWeights (default)
//     Sum the FINISHED, display-encoded frames with GLOBAL weights: the closed
//     form of CueMol's blendpng postprocess (solvebeta + the front-to-back lerp
//     chain),
//         out = (1 - sum_i a_i) * B + sum_i a_i * S_i
//     with the background weight allowed to go negative so the pass weights sum
//     to exactly 1. Collapsed per pixel, a pixel covered by the veil set K sees
//     `1 - sum_{i in K} a_i` of the background -- correct while that sum stays
//     under 1, inverted (dark reads brighter than light) where it does not.
//
//   PerPixel
//     Combine at the RAW stage (supersampled, before the box-downsample) with
//     weights built from the veils that actually cover each sample:
//         T = prod_{i in K} (1 - a_i)          background weight
//         w_i = a_i * (1 - T) / sum_{j in K} a_j
//     Non-negative and summing to 1 for any alphas and any K, so nothing
//     inverts. The domain is the same display-encoded one LayerWeights blends
//     in, and for |K| = 1 the weights are already its weights, so a sample
//     under a single veil comes out IDENTICAL: the difference is confined to
//     overlaps, where the background keeps its physical transmittance instead
//     of going negative. Coverage is read from the pass depths, which is why
//     the composite has to happen at the raw stage: after the downsample a
//     pixel is a mix of samples and "which veils cover it" is no longer a
//     yes/no per surface.
#pragma once

#include <cstdint>

#include "render/pipeline.hpp"
#include "scene.hpp"

namespace umbreon {

/// Composite the group-alpha passes of `scene`. Precondition:
/// `!scene.groupBlend.empty()`. `passCount` reports how many full renders were
/// spent (background + one per veil), for the caller's timing log. A cancelled
/// render returns the partial carrier frame with `cancelled` set.
FrameResult renderGroupBlend(const Scene& scene, const RenderOptions& opt,
                             RenderProgress* progress,
                             std::uint64_t& passCount);

}  // namespace umbreon
