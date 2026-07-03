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
  h.scenes.groupBounds.reserve(scene.groupBlend.size());
  for (const GroupBlend& gb : scene.groupBlend) {
    // An empty group builds an empty committed scene: every probe misses,
    // which is exactly the correct "cannot affect any pixel" answer (its
    // never-extended AABB rejects every segment before the BVH is asked).
    const Scene gs = keepOnlyGroup(scene, gb.group);
    h.built.push_back(buildEmbreeScene(h.device, gs));
    h.scenes.groupScene.push_back(h.built.back().scene);
    // Conservative world bounds of the group's geometry. The filtered mesh
    // shares the FULL vertex buffers, so extend only over the vertices its
    // index actually references (Mesh::bounds() would cover every group).
    Aabb b;
    for (uint32_t v : gs.mesh.index) b.extend(gs.mesh.positions[v]);
    for (const Sphere& sp : gs.spheres) {
      const Vec3 r{sp.radius, sp.radius, sp.radius};
      b.extend(sp.center - r);
      b.extend(sp.center + r);
    }
    for (const Cylinder& cy : gs.cylinders) {
      const Vec3 r{cy.radius, cy.radius, cy.radius};
      b.extend(cy.p0 - r);
      b.extend(cy.p0 + r);
      b.extend(cy.p1 - r);
      b.extend(cy.p1 + r);
    }
    // The scene builder bakes one copy per instance offset (legacy .inc grid
    // path); the bounds must cover every baked copy.
    if (!gs.instanceOffsets.empty() && b.valid()) {
      const Aabb base = b;
      for (const Vec3& off : gs.instanceOffsets) {
        b.extend(base.lo + off);
        b.extend(base.hi + off);
      }
    }
    h.scenes.groupBounds.push_back(b);
  }
  return h;
}

}  // namespace detail
}  // namespace umbreon
