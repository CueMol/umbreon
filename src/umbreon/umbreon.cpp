#include "umbreon.hpp"

#include <algorithm>
#include <cstdio>
#include <vector>

#include "postprocess/image_ops.hpp"
#include "render/pipeline.hpp"

namespace umbreon {
namespace {

// Copy `s` with every primitive whose group is flagged in `hide` removed
// (indexed by group id; ids beyond the mask are kept). Vertex buffers are
// shared wholesale -- only the triangle index list and its per-tri side tables
// shrink -- so unreferenced vertices stay in the buffers, which the tracer
// never visits.
Scene hideGroups(const Scene& s, const std::vector<uint8_t>& hide) {
  auto hidden = [&](uint16_t g) {
    return g < hide.size() && hide[g] != 0;
  };

  Scene out = s;
  out.groupBlend.clear();  // each pass renders plain (no nested blending)

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
  } else {
    out.mesh.index = std::move(idx);
    out.mesh.triMaterialId = std::move(mat);
    out.mesh.triGroupId = std::move(grp);
  }

  out.spheres.erase(
      std::remove_if(out.spheres.begin(), out.spheres.end(),
                     [&](const Sphere& sp) { return hidden(sp.group); }),
      out.spheres.end());
  out.cylinders.erase(
      std::remove_if(out.cylinders.begin(), out.cylinders.end(),
                     [&](const Cylinder& cy) { return hidden(cy.group); }),
      out.cylinders.end());
  return out;
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
FrameResult render(const Scene& scene, const RenderOptions& opt) {
  if (scene.groupBlend.empty()) return renderFrame(scene, opt);

  float sumA = 0.0f;
  uint16_t maxGroup = 0;
  for (const GroupBlend& gb : scene.groupBlend) {
    sumA += gb.alpha;
    maxGroup = std::max(maxGroup, gb.group);
  }
  if (sumA > 1.0f + 1.0e-4f)
    std::fprintf(stderr,
                 "warning: group-alpha blend weights sum to %.3f (> 1); the "
                 "background pass weight is clamped to 0\n",
                 sumA);
  const float bgW = std::fmax(0.0f, 1.0f - sumA);

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
  FrameResult carrier;
  std::vector<float> acc;
  double seconds = 0.0;
  Pt1Timing timing{};
  auto addPass = [&](const Scene& ps, float w) {
    FrameResult f = renderFrame(ps, opt);
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

  addPass(hideGroups(scene, hideAll), bgW);
  for (const GroupBlend& gb : scene.groupBlend) {
    std::vector<uint8_t> hide = hideAll;
    hide[gb.group] = 0;  // keep this group (opaque), hide the other layers
    addPass(hideGroups(scene, hide), gb.alpha);
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
  return carrier;
}

}  // namespace umbreon
