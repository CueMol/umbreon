#include "umbreon.hpp"

#include <algorithm>
#include <cmath>

#include "render/stroke_edges.hpp"
#include "render/embree_renderer.hpp"
#include "render/fog.hpp"

namespace umbreon {

std::vector<float> boxDownsample(const std::vector<float>& src, int w, int h,
                                 int channels, int ss) {
  const int ow = w / ss, oh = h / ss;
  std::vector<float> out(static_cast<std::size_t>(ow) * oh * channels, 0.0f);
  const float inv = 1.0f / static_cast<float>(ss * ss);
  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      for (int c = 0; c < channels; ++c) {
        float acc = 0.0f;
        for (int dy = 0; dy < ss; ++dy) {
          for (int dx = 0; dx < ss; ++dx) {
            std::size_t si =
                (static_cast<std::size_t>(y * ss + dy) * w + (x * ss + dx)) *
                    channels + c;
            // Contain a NaN/Inf subpixel to itself: a non-finite sample
            // contributes 0 instead of poisoning the whole block average (the
            // divisor stays ss*ss, matching OSPRay zeroing NaN before accum).
            const float v = src[si];
            acc += std::isfinite(v) ? v : 0.0f;
          }
        }
        out[(static_cast<std::size_t>(y) * ow + x) * channels + c] = acc * inv;
      }
    }
  }
  return out;
}

void applyAssumedGamma(FrameResult& frame, float g) {
  if (std::fabs(g - 1.0f) <= 1.0e-4f) return;
  const std::size_t px = static_cast<std::size_t>(frame.width) * frame.height;
  for (std::size_t i = 0; i < px; ++i) {
    for (int c = 0; c < 3; ++c) {
      float v = frame.color[i * 4 + c];
      frame.color[i * 4 + c] = (v > 0.0f) ? std::pow(v, g) : 0.0f;
    }
  }
}

namespace {
std::uint8_t toSrgb8(float v) {
  v = std::min(1.0f, std::max(0.0f, v));
  const float s = (v <= 0.0031308f)
                      ? 12.92f * v
                      : 1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f;
  const int i = static_cast<int>(s * 255.0f + 0.5f);
  return static_cast<std::uint8_t>(std::min(255, std::max(0, i)));
}
}  // namespace

std::vector<std::uint8_t> srgbEncode8(const FrameResult& frame, int channels) {
  const std::size_t px = static_cast<std::size_t>(frame.width) * frame.height;
  std::vector<std::uint8_t> out(px * channels);
  for (std::size_t i = 0; i < px; ++i) {
    for (int c = 0; c < 3 && c < channels; ++c)
      out[i * channels + c] = toSrgb8(frame.color[i * 4 + c]);
    if (channels == 4) {
      const float a = std::min(1.0f, std::max(0.0f, frame.color[i * 4 + 3]));
      out[i * 4 + 3] = static_cast<std::uint8_t>(a * 255.0f + 0.5f);
    }
  }
  return out;
}

FrameResult render(const Scene& scene, const RenderOptions& opt) {
  const int ss = std::max(1, opt.supersample);
  const int finalW = opt.width, finalH = opt.height;

  // Render at the supersampled resolution; the camera frames identically.
  RenderOptions hi = opt;
  hi.width = finalW * ss;
  hi.height = finalH * ss;

  EmbreeRenderer renderer;
  FrameResult frame = renderer.render(scene, hi);

  // POV ground fog at full (supersampled) resolution, before downsampling, so
  // the box-average mirrors POV-Ray averaging antialiased, fogged samples.
  if (scene.fog.enabled && !frame.depth.empty()) {
    applyFog(scene.fog, scene.camera, frame.width, frame.height, 4,
             frame.color.data(), frame.depth.data());
  }

  // Freestyle-style stroke edges (--edges, NEW): extract/chain/visibility/ribbon
  // composited over the color in LINEAR space, here -- BEFORE the downsample --
  // so the box-average antialiases them. The Embree scene is kept ALIVE in
  // `renderer` through this pass (see EmbreeRenderer) for ray-cast visibility.
  // Gated on the master flag; with edges off this is never entered, keeping the
  // default render path byte-identical. --edges drives the stroke pipeline.
  if (opt.strokeEdges.enable) {
    // Bind the ray-cast visibility query to the live BVH kept alive in
    // `renderer` (see EmbreeRenderer): occluded(P, target, selfFaces) is the QI
    // test, excluding the edge's own incident mesh faces (Freestyle self-face
    // exclusion). The renderer holds the mesh geomID needed to match those faces.
    // Freestyle TANGENTIAL-occluder rejection threshold: a QI ray hit on a face
    // grazed nearly edge-on (|dir . normalize(Ng)| <= this cosine) is a numerical
    // degeneracy, not a real occluder. Match Freestyle's literal value -- its
    // ComputeRayCastingVisibility counts a hit only when fabs(u*normal) > 0.0001
    // (already cited in embree_renderer.cpp). This is a pure DEGENERACY GUARD, NOT
    // an angular cull: the silhouette's OWN faces are dropped by face-ID
    // (excludeFaceFilter step 1) plus the unbiased true-surface QI origin (fix B,
    // camBias=0), NOT by an angle. The former 0.1 (~5.7deg) discarded a REAL front
    // occluder seen near its OWN silhouette (|dir.Ng|->0), so a back line just
    // inside that silhouette wrongly voted VISIBLE -- the hidden band. Relies on
    // fix B (true-surface origin) + fix D (face-ID self-exclude keeps each strand's
    // own grazing faces) so flat strands self-hide by id, not by this angle.
    constexpr float kQiGrazeCosEps = 1.0e-4f;
    const OcclusionQuery occluded = [&renderer](const Vec3& p, const Vec3& q,
                                                const int* excludeFaces,
                                                int nExclude) {
      return renderer.occluded(p, q, excludeFaces, nExclude, 1.0e-4f,
                               kQiGrazeCosEps);
    };
    applyStrokeEdges(frame, scene, opt, occluded);
  }

  if (ss > 1) {
    frame.color = boxDownsample(frame.color, frame.width, frame.height, 4, ss);
    if (!frame.albedo.empty())
      frame.albedo =
          boxDownsample(frame.albedo, frame.width, frame.height, 3, ss);
    // Edge AOVs (normal/viewZ/objectId/materialId) are a hi-res set: the edge
    // pass runs at supersample resolution before this downsample, and box-
    // averaging integer ids is meaningless. So when edges are on, leave them at
    // hi-res; only the legacy normal AOV path downsamples. frame.width/height
    // below become the FINAL color dims.
    if (!frame.normal.empty() && !opt.strokeEdges.enable)
      frame.normal =
          boxDownsample(frame.normal, frame.width, frame.height, 3, ss);
    frame.width = finalW;
    frame.height = finalH;
  }

  applyAssumedGamma(frame, scene.assumedGamma);
  return frame;
}

}  // namespace umbreon
