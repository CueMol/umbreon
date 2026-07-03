#include "render/blend_reuse.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace umbreon {
namespace detail {

Scene keepOnlyGroup(const Scene& s, uint16_t g) {
  Scene out = s;
  out.groupBlend.clear();

  const std::size_t ntri = s.mesh.triangleCount();
  std::vector<uint32_t> idx;
  std::vector<uint8_t> mat;
  std::vector<uint16_t> grp;
  for (std::size_t t = 0; t < ntri; ++t) {
    if (s.mesh.groupForTri(t) != g) continue;
    for (int k = 0; k < 3; ++k)
      idx.push_back(s.mesh.cornerVertex(t * 3 + static_cast<std::size_t>(k)));
    if (!s.mesh.triMaterialId.empty()) mat.push_back(s.mesh.triMaterialId[t]);
    if (!s.mesh.triGroupId.empty()) grp.push_back(s.mesh.triGroupId[t]);
  }
  if (idx.empty()) {
    // No triangle carries the group. An empty `index` means the de-indexed
    // (soup) fallback, which would resurrect ALL vertices -- drop the mesh.
    out.mesh = Mesh{};
  } else {
    out.mesh.index = std::move(idx);
    out.mesh.triMaterialId = std::move(mat);
    out.mesh.triGroupId = std::move(grp);
  }

  out.spheres.erase(
      std::remove_if(out.spheres.begin(), out.spheres.end(),
                     [&](const Sphere& sp) { return sp.group != g; }),
      out.spheres.end());
  out.cylinders.erase(
      std::remove_if(out.cylinders.begin(), out.cylinders.end(),
                     [&](const Cylinder& cy) { return cy.group != g; }),
      out.cylinders.end());
  return out;
}

BlendProbeHolder& BlendProbeHolder::operator=(BlendProbeHolder&& o) noexcept {
  if (this != &o) {
    for (RTCScene s : scenes.groupScene)
      if (s) rtcReleaseScene(s);
    if (device) rtcReleaseDevice(device);
    device = o.device;
    scenes = std::move(o.scenes);
    built = std::move(o.built);
    o.device = nullptr;
    o.scenes.groupScene.clear();
    o.built.clear();
  }
  return *this;
}

BlendProbeHolder::~BlendProbeHolder() {
  for (RTCScene s : scenes.groupScene)
    if (s) rtcReleaseScene(s);
  if (device) rtcReleaseDevice(device);
}

BlendProbeHolder buildBlendProbes(const Scene& scene) {
  BlendProbeHolder h;
  h.device = rtcNewDevice(nullptr);
  if (!h.device) throw std::runtime_error("blend probe: rtcNewDevice failed");
  rtcSetDeviceErrorFunction(h.device, embreeErrorCallback, nullptr);
  h.built.reserve(scene.groupBlend.size());
  h.scenes.groupScene.reserve(scene.groupBlend.size());
  for (const GroupBlend& gb : scene.groupBlend) {
    // An empty group builds an empty committed scene: every probe misses,
    // which is exactly the correct "cannot affect any pixel" answer.
    h.built.push_back(
        buildEmbreeScene(h.device, keepOnlyGroup(scene, gb.group)));
    h.scenes.groupScene.push_back(h.built.back().scene);
  }
  return h;
}

}  // namespace detail
}  // namespace umbreon
