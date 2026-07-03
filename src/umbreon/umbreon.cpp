#include "umbreon.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

#include "postprocess/image_ops.hpp"
#include "render/blend_reuse.hpp"
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

// --blend-reuse verify: byte-compare two frames' image buffers (timing
// metadata excluded) and report the first mismatching buffer to stderr.
// Returns true when identical.
bool framesIdentical(const FrameResult& a, const FrameResult& b,
                     const char** what) {
  auto same = [](const auto& x, const auto& y) {
    return x.size() == y.size() &&
           (x.empty() ||
            std::memcmp(x.data(), y.data(),
                        x.size() * sizeof(x[0])) == 0);
  };
  *what = "";
  if (a.width != b.width || a.height != b.height) { *what = "dims"; return false; }
  if (!same(a.color, b.color)) { *what = "color"; return false; }
  if (!same(a.depth, b.depth)) { *what = "depth"; return false; }
  if (!same(a.viewZ, b.viewZ)) { *what = "viewZ"; return false; }
  if (!same(a.normal, b.normal)) { *what = "normal"; return false; }
  if (!same(a.albedo, b.albedo)) { *what = "albedo"; return false; }
  if (!same(a.objectId, b.objectId)) { *what = "objectId"; return false; }
  if (!same(a.surfAlpha, b.surfAlpha)) { *what = "surfAlpha"; return false; }
  if (!same(a.position, b.position)) { *what = "position"; return false; }
  if (!same(a.indirect, b.indirect)) { *what = "indirect"; return false; }
  if (!same(a.giOcclusion, b.giOcclusion)) { *what = "giOcclusion"; return false; }
  return true;
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

  // Every pass renders with the same options; for pt1, pin the gather epsilon
  // scale to the FULL scene's mesh diagonal so it does not vary with the
  // pass's geometry subset (see RenderOptions::pt1EpsT -- applied to naive
  // and reuse multipass alike, so the two stay bit-comparable).
  RenderOptions passOpt = opt;
  if (opt.gi && opt.giIntegrator == 1 && opt.pt1EpsT <= 0.0f) {
    const Aabb fullBounds = scene.mesh.bounds();
    passOpt.pt1EpsT = fullBounds.valid() ? fullBounds.diagonal() : 1.0f;
  }

  std::vector<uint8_t> hideAll(static_cast<std::size_t>(maxGroup) + 1, 0);
  for (const GroupBlend& gb : scene.groupBlend) hideAll[gb.group] = 1;

  // Cross-pass reuse eligibility (RenderOptions::blendReuse). The exclusions
  // fall back to the naive multipass, which is trivially bit-exact:
  //  - irradiance-cache GI: world-space shared records, not pixel-separable;
  //  - objectSpaceEdges: edge GEOMETRY derives from each pass's scene subset,
  //    so removing a group can change edges far from it (unboundable);
  //  - > 32 groups: the touch word is uint32.
  const bool reusable = opt.blendReuse != 0 &&
                        !(opt.gi && opt.giIntegrator == 0) &&
                        !opt.objectSpaceEdges.enable &&
                        scene.groupBlend.size() <= 32;
  detail::BlendProbeHolder probes;
  detail::BlendReuseContext ctx;
  if (reusable) {
    probes = detail::buildBlendProbes(scene);
    ctx.mode = detail::BlendReuseContext::Mode::Capture;
    ctx.probe = &probes.scenes;
  }

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
  int passIndex = 0;
  const int passCount = 1 + static_cast<int>(scene.groupBlend.size());
  auto addPass = [&](const Scene& ps, float w,
                     detail::BlendReuseContext* rc = nullptr) {
    FrameResult f = renderFrame(ps, passOpt, rc);
    std::fprintf(stderr,
                 "blend pass %d/%d: weight %.3f  %zu tris  %.3f s\n",
                 ++passIndex, passCount, w, ps.mesh.triangleCount(),
                 f.renderSeconds);
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

  // Background pass. Under reuse it runs in Capture mode: rays are ghost-
  // probed against the blend groups and the trace-stage outputs snapshot into
  // `ctx` for the layer passes.
  addPass(hideGroups(scene, hideAll), bgW, reusable ? &ctx : nullptr);
  if (reusable) {
    // Per-group dirty fractions (diagnostics + the go/no-go datapoint for the
    // reuse phases): the fraction of hi-res pixels whose rays touched each
    // group during the background pass.
    const std::size_t nHi = ctx.touch.size();
    const std::size_t nHalf = ctx.touchHalf.size();
    for (std::size_t i = 0; i < scene.groupBlend.size(); ++i) {
      std::size_t dirty = 0, dirtyHalf = 0;
      for (uint32_t t : ctx.touch) dirty += (t >> i) & 1u;
      for (uint32_t t : ctx.touchHalf) dirtyHalf += (t >> i) & 1u;
      const double pctHi =
          nHi ? 100.0 * static_cast<double>(dirty) / static_cast<double>(nHi)
              : 0.0;
      if (nHalf)
        std::fprintf(stderr,
                     "blend-reuse: group %u dirty %.1f%% (hi) %.1f%% (half)\n",
                     scene.groupBlend[i].group, pctHi,
                     100.0 * static_cast<double>(dirtyHalf) /
                         static_cast<double>(nHalf));
      else
        std::fprintf(stderr, "blend-reuse: group %u dirty %.1f%% (hi)\n",
                     scene.groupBlend[i].group, pctHi);
    }
  }
  // Layer passes. Under reuse each runs in Reuse mode with its dirty mask
  // (the group's touch bit from the background pass); clean pixels are copied
  // from the capture snapshot inside the renderer, and the pt1 gather is
  // restricted the same way (with the half-grid mask when half-res ran).
  const bool reuseLayers = reusable;
  std::size_t groupIdx = 0;
  for (const GroupBlend& gb : scene.groupBlend) {
    std::vector<uint8_t> hide = hideAll;
    hide[gb.group] = 0;  // keep this group (opaque), hide the other layers
    const Scene layer = hideGroups(scene, hide);
    if (!reuseLayers) {
      addPass(layer, gb.alpha);
    } else {
      std::vector<uint8_t> active(ctx.touch.size());
      for (std::size_t p = 0; p < active.size(); ++p)
        active[p] = static_cast<uint8_t>((ctx.touch[p] >> groupIdx) & 1u);
      std::vector<uint8_t> activeHalf(ctx.touchHalf.size());
      for (std::size_t p = 0; p < activeHalf.size(); ++p)
        activeHalf[p] = static_cast<uint8_t>((ctx.touchHalf[p] >> groupIdx) & 1u);
      ctx.mode = detail::BlendReuseContext::Mode::Reuse;
      ctx.active = &active;
      ctx.activeHalf = activeHalf.empty() ? nullptr : &activeHalf;
      // verify mode (--blend-reuse verify): ALSO render the layer full-frame
      // and require byte-identity -- the direct probe-soundness check.
      FrameResult check;
      if (opt.blendReuse == 2) check = renderFrame(layer, passOpt);
      addPass(layer, gb.alpha, &ctx);
      if (opt.blendReuse == 2) {
        const char* what = nullptr;
        if (framesIdentical(carrier, check, &what)) {
          std::fprintf(stderr, "blend-reuse verify: group %u OK\n", gb.group);
        } else {
          std::fprintf(stderr,
                       "blend-reuse verify: group %u MISMATCH in '%s' -- the "
                       "reuse mask is unsound for this scene, please report\n",
                       gb.group, what);
        }
      }
    }
    ++groupIdx;
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
