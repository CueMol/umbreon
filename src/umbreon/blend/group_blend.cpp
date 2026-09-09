#include "blend/group_blend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "log.hpp"
#include "postprocess/image_ops.hpp"   // srgbEncodeF / srgbDecodeF
#include "render/scene_build.hpp"      // embreeErrorCallback

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
    if (device_)
      rtcSetDeviceErrorFunction(device_, detail::embreeErrorCallback, nullptr);
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
  auto hidden = [&](uint16_t g) { return g < hide.size() && hide[g] != 0; };

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

/// One veil: the blend entries that share an alpha, and that alpha.
struct Veil {
  float alpha = 1.0f;
  std::vector<uint16_t> groups;
};

/// Entries whose alphas agree within this are the same veil. It absorbs scene
/// round-trip noise only; values a user actually set apart stay apart.
constexpr float kAlphaEps = 1.0e-4f;

/// Bucket the blend entries into veils, in FIRST APPEARANCE order: the host's
/// section order therefore fixes both the pass order and each veil's alpha, so
/// the bucketing is deterministic for a given input. A NaN alpha buckets per
/// entry and still poisons the weight sum, exactly as it does unbucketed.
std::vector<Veil> bucketVeils(const std::vector<GroupBlend>& entries) {
  std::vector<Veil> veils;
  for (const GroupBlend& gb : entries) {
    auto it = std::find_if(veils.begin(), veils.end(), [&](const Veil& v) {
      return std::fabs(v.alpha - gb.alpha) <= kAlphaEps;
    });
    if (it == veils.end())
      veils.push_back(Veil{gb.alpha, {gb.group}});
    else
      it->groups.push_back(gb.group);
  }
  return veils;
}

// Runs the passes of one blend and owns everything both modes share: the Embree
// device, the working scene, the per-pass log line, progress, cancellation and
// the timing totals. It hands back each pass at the RAW stage; a mode that wants
// a finished frame (LayerWeights) calls finishFrame on it, and one that
// composites first (PerPixel) finishes once at the end.
class PassRunner {
 public:
  PassRunner(const Scene& scene, const RenderOptions& opt,
             RenderProgress* progress, std::uint64_t passCount)
      : scene_(scene),
        opt_(opt),
        progress_(progress),
        passCount_(passCount),
        work_(scene) {
    // Nested blending is off in a pass by construction.
    work_.groupBlend.clear();
  }

  /// Render one pass: the geometry left visible by `hide`, at the raw stage.
  /// `label` describes the pass for the log (weights are the mode's business).
  /// A pass rendered after a cancellation returns an empty frame.
  RawFrame run(const std::vector<uint8_t>& hide, const std::string& label) {
    if (cancelled_) return RawFrame{};  // a prior pass was cancelled
    if (progress_) progress_->beginPass(passIndex_, passCount_);
    ++passIndex_;
    // Group-alpha transparency costs one FULL render per veil plus one for the
    // background, which is the single most surprising thing about a transparent
    // scene's render time. Say so, per pass.
    logMessage(LogLevel::Info, "group-alpha pass %llu/%llu (%s)",
               static_cast<unsigned long long>(passIndex_),
               static_cast<unsigned long long>(passCount_), label.c_str());
    applyHideGroups(scene_, hide, work_, workMeshDropped_);
    RawFrame raw = renderFrameRaw(work_, opt_, progress_, device_.get());
    if (raw.frame.cancelled) cancelled_ = true;
    seconds_ += raw.frame.renderSeconds;
    timing_.bvhBuild += raw.frame.pt1Timing.bvhBuild;
    timing_.primary += raw.frame.pt1Timing.primary;
    timing_.direct += raw.frame.pt1Timing.direct;
    timing_.gather += raw.frame.pt1Timing.gather;
    timing_.denoise += raw.frame.pt1Timing.denoise;
    timing_.upsample += raw.frame.pt1Timing.upsample;
    timing_.total += raw.frame.pt1Timing.total;
    return raw;
  }

  /// Run the finishing stage a mode skipped, with the options this pass used.
  void finish(RawFrame& raw) const {
    if (raw.frame.cancelled) return;  // renderFrame stops here too
    finishFrame(raw.frame, scene_, opt_, raw.hi, raw.ss, raw.finalW, raw.finalH,
                progress_);
  }

  bool cancelled() const { return cancelled_; }
  double seconds() const { return seconds_; }
  const Pt1Timing& timing() const { return timing_; }

 private:
  const Scene& scene_;
  const RenderOptions& opt_;
  RenderProgress* progress_;
  std::uint64_t passCount_;
  std::uint64_t passIndex_ = 0;
  bool cancelled_ = false;
  // One device for every pass of this render (see SharedDevice).
  const SharedDevice device_;
  // One working scene for every pass: the vertex buffers are pass-invariant, so
  // they are copied once here and each pass rewrites only its primitive lists.
  Scene work_;
  bool workMeshDropped_ = false;
  double seconds_ = 0.0;
  Pt1Timing timing_{};
};

/// `weight 0.600, 2 group(s)` -- the LayerWeights per-pass log label.
std::string weightLabel(float w, std::size_t nGroups) {
  char buf[64];
  if (nGroups == 0)
    std::snprintf(buf, sizeof(buf), "weight %.3f", double(w));
  else
    std::snprintf(buf, sizeof(buf), "weight %.3f, %zu group(s)", double(w),
                  nGroups);
  return std::string(buf);
}

/// `veil alpha 0.600, 2 group(s)` -- the PerPixel per-pass log label (its
/// weights are per sample, so a single number would be a fiction).
std::string veilLabel(float a, std::size_t nGroups) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "veil alpha %.3f, %zu group(s)", double(a),
                nGroups);
  return std::string(buf);
}

// --- LayerWeights: global weights over the finished frames -------------------

FrameResult blendLayerWeights(const std::vector<Veil>& veils,
                              const std::vector<uint8_t>& hideAll, float bgW,
                              PassRunner& runner) {
  // Accumulate w * pass color into `acc` -- RGB in the sRGB-ENCODED domain
  // (blendpng blends the finished 8-bit PNGs, whose RGB is the sRGB encode of
  // FrameResult.color; alpha is stored linear in the PNG and blends as-is).
  // The LAST rendered pass is kept whole as the carrier frame so the
  // non-color outputs (edge G-buffer, GI guides, depth) come from a real
  // render -- the final veil pass, which for the common single-alpha case
  // shows every translucent group, i.e. the full scene. Zero-weight passes
  // still render: skipping them would silently change which pass carries
  // those.
  FrameResult carrier;
  std::vector<float> acc;
  auto addPass = [&](const std::vector<uint8_t>& hide, float w,
                     std::size_t nGroups) {
    RawFrame raw = runner.run(hide, weightLabel(w, nGroups));
    runner.finish(raw);
    FrameResult f = std::move(raw.frame);
    if (acc.empty()) acc.assign(f.color.size(), 0.0f);
    const std::size_t n = std::min(acc.size(), f.color.size()) / 4;
    for (std::size_t p = 0; p < n; ++p) {
      for (int c = 0; c < 3; ++c)
        acc[p * 4 + c] += w * srgbEncodeF(f.color[p * 4 + c]);
      acc[p * 4 + 3] += w * f.color[p * 4 + 3];
    }
    carrier = std::move(f);
  };

  addPass(hideAll, bgW, 0);
  for (const Veil& v : veils) {
    std::vector<uint8_t> hide = hideAll;
    // Keep every group of this veil (opaque), hide the other veils.
    for (const uint16_t g : v.groups) hide[g] = 0;
    addPass(hide, v.alpha, v.groups.size());
  }

  // Map the blended sRGB values back to FrameResult's linear-ish domain so
  // the image writer's own sRGB encode reproduces the blend exactly. The
  // blend is affine and every encoded input sits in [0, 1], so only a > 1
  // weight sum needs the lower clamp. Alpha is coverage, clamped to [0, 1].
  const std::size_t npix = acc.size() / 4;
  for (std::size_t p = 0; p < npix; ++p) {
    for (int c = 0; c < 3; ++c)
      acc[p * 4 + c] = srgbDecodeF(std::fmax(0.0f, acc[p * 4 + c]));
    acc[p * 4 + 3] = std::fmin(1.0f, std::fmax(0.0f, acc[p * 4 + 3]));
  }

  carrier.color = std::move(acc);
  return carrier;
}

// --- PerPixel: raw-stage composite with per-sample coverage ------------------

/// Does this veil pass show its veil at this sample, rather than the same
/// background the background pass shows? Both passes trace the SAME opaque
/// geometry (only veils are hidden or shown), so an opaque hit gives the same
/// distance in both, and the veil covers the sample exactly when the pass's
/// frontmost hit is closer than the background's -- or the background has none.
/// `depth` is the ray distance; 0 means nothing was hit.
inline bool veilCovers(float dBg, float dVeil) {
  if (!(dVeil > 0.0f)) return false;  // this pass hit nothing here
  if (!(dBg > 0.0f)) return true;     // ... and the background did not either
  return dVeil < dBg * (1.0f - 1.0e-5f);
}

FrameResult blendPerPixel(const std::vector<Veil>& veils,
                          const std::vector<uint8_t>& hideAll,
                          PassRunner& runner) {
  // The background pass is the carrier: its non-color outputs describe the
  // scene minus the veils, and its depth is the reference every coverage test
  // is made against. It must therefore run FIRST (LayerWeights carries the last
  // pass instead, which is why the two modes differ in what the AOVs describe).
  RawFrame bg = runner.run(hideAll, "background");
  if (bg.frame.cancelled) return std::move(bg.frame);

  const std::size_t nsamp =
      static_cast<std::size_t>(bg.frame.width) * bg.frame.height;
  const bool doAlbedo = bg.frame.albedo.size() >= nsamp * 3;
  // Per sample: the veils' weighted color, the sum of their alphas, and the
  // transmittance product that becomes the background's own weight.
  std::vector<float> acc(nsamp * 4, 0.0f);
  std::vector<float> accAlbedo(doAlbedo ? nsamp * 3 : 0, 0.0f);
  std::vector<float> sumA(nsamp, 0.0f);
  std::vector<float> trans(nsamp, 1.0f);

  for (const Veil& v : veils) {
    std::vector<uint8_t> hide = hideAll;
    for (const uint16_t g : v.groups) hide[g] = 0;
    RawFrame f = runner.run(hide, veilLabel(v.alpha, v.groups.size()));
    if (f.frame.cancelled) break;  // partial: composite what we have
    const std::size_t n =
        std::min(nsamp, static_cast<std::size_t>(f.frame.width) *
                            f.frame.height);
    for (std::size_t p = 0; p < n; ++p) {
      if (!veilCovers(bg.frame.depth[p], f.frame.depth[p])) continue;
      for (int c = 0; c < 4; ++c)
        acc[p * 4 + c] += v.alpha * f.frame.color[p * 4 + c];
      if (doAlbedo && f.frame.albedo.size() >= n * 3)
        for (int c = 0; c < 3; ++c)
          accAlbedo[p * 3 + c] += v.alpha * f.frame.albedo[p * 3 + c];
      sumA[p] += v.alpha;
      trans[p] *= 1.0f - v.alpha;
    }
  }

  // out = T * background + (the veils' color, re-weighted to share 1 - T).
  // Linear light, before the downsample and the gamma encode -- unlike
  // LayerWeights, which mixes the display-encoded finished frames.
  for (std::size_t p = 0; p < nsamp; ++p) {
    const float s = sumA[p];
    if (!(s > 0.0f)) continue;  // no veil covers this sample: background stands
    const float T = trans[p];
    const float k = (1.0f - T) / s;
    for (int c = 0; c < 4; ++c)
      bg.frame.color[p * 4 + c] = T * bg.frame.color[p * 4 + c] + k * acc[p * 4 + c];
    if (doAlbedo)
      for (int c = 0; c < 3; ++c)
        bg.frame.albedo[p * 3 + c] =
            T * bg.frame.albedo[p * 3 + c] + k * accAlbedo[p * 3 + c];
  }

  runner.finish(bg);
  return std::move(bg.frame);
}

}  // namespace

FrameResult renderGroupBlend(const Scene& scene, const RenderOptions& opt,
                             RenderProgress* progress,
                             std::uint64_t& passCount) {
  const std::vector<Veil> veils = bucketVeils(scene.groupBlend);

  uint16_t maxGroup = 0;
  for (const GroupBlend& gb : scene.groupBlend)
    maxGroup = std::max(maxGroup, gb.group);
  // The background pass hides every listed group; a veil pass then shows its
  // own again. This stays ENTRY based: a group is hidden because some veil owns
  // it, whichever veil that is.
  std::vector<uint8_t> hideAll(static_cast<std::size_t>(maxGroup) + 1, 0);
  for (const GroupBlend& gb : scene.groupBlend) hideAll[gb.group] = 1;

  float sumA = 0.0f;
  for (const Veil& v : veils) sumA += v.alpha;
  const float bgW = 1.0f - sumA;
  const bool perPixel =
      opt.groupBlendMode == static_cast<int>(GroupBlendMode::PerPixel);

  // One line for the whole table: this is where the veil / weight contract is
  // reported, so a host does not have to restate (or recompute) it.
  logMessage(LogLevel::Info,
             "group-alpha: %zu blend group(s) -> %zu veil(s), sum %.3f, "
             "bg weight %.3f, %s",
             scene.groupBlend.size(), veils.size(), double(sumA), double(bgW),
             perPixel ? "per-pixel" : "layer weights");

  // The background weight is NEGATIVE once the veil weights sum to more than
  // 1, and in LayerWeights that is the correct value rather than an error to
  // clamp away. What makes the blend faithful is that the pass weights sum to
  // exactly 1: geometry outside every blend group appears identically in all
  // passes, so it is reproduced unchanged only while (1 - sum) + sum == 1.
  // Clamping the background weight to 0 leaves the total at `sum`, which scales
  // the whole frame by that factor and clips opaque geometry to white. blendpng,
  // whose closed form this is, lets the same coefficient go negative (its
  // solvebeta + front-to-back lerp chain produces 1 - sum(beta) directly).
  //
  // The sum runs over VEILS, so it can only pass 1 with two or more DISTINCT
  // alphas -- and then only the pixels where those veils OVERLAP are wrong (the
  // background coefficient there is 1 minus the alphas covering that pixel).
  // PerPixel computes exactly that coefficient and never goes negative, so the
  // warning is for LayerWeights only.
  if (!perPixel && bgW < -kAlphaEps) {
    logMessage(LogLevel::Warning,
               "group-alpha veil weights sum to %.3f (> 1): background weight "
               "%.3f. Where the veils overlap, geometry behind them is "
               "composited with a negative weight and can invert or clip; "
               "lower a section alpha to keep the sum <= 1, or use the "
               "per-pixel group blend.",
               double(sumA), double(bgW));
  }

  passCount = 1 + veils.size();
  PassRunner runner(scene, opt, progress, passCount);
  FrameResult out = perPixel ? blendPerPixel(veils, hideAll, runner)
                             : blendLayerWeights(veils, hideAll, bgW, runner);
  out.renderSeconds = runner.seconds();
  out.pt1Timing = runner.timing();
  out.cancelled = runner.cancelled();
  return out;
}

}  // namespace umbreon
