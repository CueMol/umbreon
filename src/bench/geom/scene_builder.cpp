#include "geom/scene_builder.hpp"

#include <algorithm>
#include <cmath>

namespace umbreon {

Scene buildScene(const Mesh& mesh, const BuildOptions& opt) {
  Scene scene;
  scene.mesh = mesh;
  scene.background = opt.background;
  scene.ambientIntensity = opt.ambientIntensity;
  scene.ambientColor = Vec3{1.0f, 1.0f, 1.0f};

  // --- base geometry bounds ---
  Aabb base = mesh.bounds();
  if (!base.valid()) base = Aabb{Vec3{-1, -1, -1}, Vec3{1, 1, 1}};
  Vec3 ext = base.extent();
  float pitch = std::max({ext.x, ext.y, ext.z, 1e-4f}) * opt.spacing;

  // --- N x N x N instance grid, centered on the origin ---
  const int n = std::max(1, opt.gridN);
  const float c0 = (n - 1) * 0.5f;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      for (int k = 0; k < n; ++k) {
        scene.instanceOffsets.push_back(Vec3{(i - c0) * pitch,
                                             (j - c0) * pitch,
                                             (k - c0) * pitch});
      }
    }
  }

  // --- combined bounds of the whole grid ---
  Aabb world;
  for (const Vec3& off : scene.instanceOffsets) {
    world.extend(base.lo + off);
    world.extend(base.hi + off);
  }

  // --- camera framing the whole grid ---
  Vec3 center = world.center();
  float radius = 0.5f * world.diagonal();
  if (radius <= 0.0f) radius = 1.0f;

  // AO ray distance scaled to the whole grid so that occlusion between
  // neighbouring instances is captured, not just within a single mesh.
  scene.aoDistance = std::max(world.diagonal() * 0.7f, 1.0e-3f);

  float az = radians(opt.cameraAzimuth);
  float el = radians(opt.cameraElevation);
  Vec3 toCam = normalize(Vec3{std::cos(el) * std::sin(az), std::sin(el),
                              std::cos(el) * std::cos(az)});
  float dist = radius * opt.fitMargin / std::sin(radians(opt.fovy * 0.5f));

  scene.camera.position = center + toCam * dist;
  scene.camera.direction = normalize(center - scene.camera.position);
  scene.camera.up = Vec3{0.0f, 1.0f, 0.0f};
  scene.camera.fovy = opt.fovy;

  // --- key light from over the camera's upper shoulder ---
  Vec3 right = normalize(cross(scene.camera.direction, scene.camera.up));
  Vec3 keyFrom = normalize(toCam + scene.camera.up * 1.1f + right * 0.35f);
  DistantLight key;
  key.direction = keyFrom * -1.0f;  // travel direction: light -> scene
  key.color = Vec3{1.0f, 1.0f, 1.0f};
  key.intensity = opt.lightIntensity;
  scene.lights.push_back(key);

  return scene;
}

}  // namespace umbreon
