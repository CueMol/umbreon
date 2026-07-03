// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Cross-pass reuse state for the group-alpha multipass blend (see
// Scene::groupBlend and RenderOptions::blendReuse). The orchestrator in
// umbreon.cpp renders the BACKGROUND pass in Capture mode -- every ray is
// ghost-probed against per-group blend BVHs (shading/blend_probe.hpp) and the
// renderer snapshots its trace-stage outputs -- then renders each LAYER pass
// in Reuse mode with a per-pass dirty mask: pixels whose rays never touched
// the pass's group are copied from the snapshot instead of recomputed, which
// is bit-exact because all per-pixel sampling is deterministic.
#pragma once

#include <cstdint>
#include <vector>

#include <embree4/rtcore.h>

#include "experimental/pt1/pt1_integrator.hpp"  // Pt1GBuffer
#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "scene.hpp"
#include "shading/blend_probe.hpp"

namespace umbreon {
namespace detail {

// Raw pt1 state captured from the background pass, PRE-denoise (denoisePt1E
// mutates E in place, so snapshots are taken right after gatherPt1Grid).
// Only the arrays of the active resolution mode are filled.
struct Pt1ReuseBundle {
  // full-res mode (pt1HalfRes == false)
  std::vector<float> Eraw;    // hiW*hiH*3 gathered E, pre-denoise
  std::vector<float> occRaw;  // hiW*hiH   gather occlusion fraction
  // half-res mode (pt1HalfRes == true)
  Pt1GBuffer gbuf;             // private half-res G-buffer (post-trace)
  std::vector<float> EhRaw;    // halfW*halfH*3 gathered E, pre-denoise
  std::vector<float> occhRaw;  // halfW*halfH
};

// Everything the multipass orchestrator threads through renderFrame ->
// EmbreeRenderer::render. One instance lives for the whole blended frame: the
// background pass fills it (Capture), every layer pass reads it (Reuse) with
// its own dirty mask.
struct BlendReuseContext {
  enum class Mode { Capture, Reuse };
  Mode mode = Mode::Capture;

  // Capture input (owned by the orchestrator; null in Reuse mode).
  const BlendProbeScenes* probe = nullptr;

  // Capture outputs. Dimensions are the HI-RES render grid (final * ss).
  int hiW = 0, hiH = 0;
  int halfW = 0, halfH = 0;         // pt1 half grid (0 unless half-res ran)
  std::vector<uint32_t> touch;      // hiW*hiH per-pixel group-touch bits
  std::vector<uint32_t> touchHalf;  // halfW*halfH (pt1 half-res only)
  // Trace-stage snapshot: a copy of the renderer's FrameResult taken at the
  // END of EmbreeRenderer::render (after the GI composite, before the
  // pipeline's fog/edges/downsample/denoise/gamma stages).
  FrameResult raw;
  // GI side channels (local to the renderer, not part of FrameResult).
  std::vector<float> giRefl;    // hiW*hiH*3
  std::vector<uint8_t> giElig;  // hiW*hiH
  Pt1ReuseBundle pt1;

  // Reuse inputs, set by the orchestrator per layer pass: 1 = compute,
  // 0 = copy from the snapshot.
  const std::vector<uint8_t>* active = nullptr;      // hiW*hiH
  const std::vector<uint8_t>* activeHalf = nullptr;  // halfW*halfH
};

// hideGroups' inverse: keep ONLY primitives whose group == g (mesh triangles,
// spheres, cylinders); everything else is removed. Used to build the
// per-group probe BVHs.
Scene keepOnlyGroup(const Scene& s, uint16_t g);

// Owns the probe device + one committed blend-only BVH per groupBlend entry
// (in Scene::groupBlend order). The BuiltScene side tables are kept alive
// alongside the scenes; release order in the destructor is scenes then device.
struct BlendProbeHolder {
  RTCDevice device = nullptr;
  BlendProbeScenes scenes;
  std::vector<BuiltScene> built;

  BlendProbeHolder() = default;
  BlendProbeHolder(const BlendProbeHolder&) = delete;
  BlendProbeHolder& operator=(const BlendProbeHolder&) = delete;
  BlendProbeHolder(BlendProbeHolder&& o) noexcept { *this = std::move(o); }
  BlendProbeHolder& operator=(BlendProbeHolder&& o) noexcept;
  ~BlendProbeHolder();
};

BlendProbeHolder buildBlendProbes(const Scene& scene);

}  // namespace detail
}  // namespace umbreon
