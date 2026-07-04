// Shared scene builders and helpers for the render integration tests
// (test_render_*.cpp), formerly file-local in the monolithic test_render.cpp.
#pragma once

#include <cmath>
#include <cstddef>

#include "umbreon.hpp"

inline bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

// A flat quad in the z=0 plane spanning [-2,2]^2, facing +Z (toward the camera),
// with a single pigment color. Two de-indexed triangles, material 0.2/0.8.
inline umbreon::Mesh makeQuad(umbreon::Vec4 color) {
  using umbreon::Vec3;
  umbreon::Mesh m;
  const Vec3 p00{-2, -2, 0}, p10{2, -2, 0}, p11{2, 2, 0}, p01{-2, 2, 0};
  const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
  const Vec3 n{0, 0, 1};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;
  return m;
}

// Orthographic camera at +Z looking down -Z. height=4 frames y in [-2,2]; with a
// square image that frames x in [-2,2] as well, so the quad fills the view.
inline umbreon::Camera makeOrthoCam() {
  umbreon::Camera c;
  c.position = {0, 0, 10};
  c.direction = {0, 0, -1};
  c.up = {0, 1, 0};
  c.orthographic = true;
  c.height = 4.0f;
  return c;
}

// A single head-on distant light traveling -Z (lighting the +Z faces) at half
// intensity. With material 0.2/0.8 and a head-on normal (N.L = 1) this gives
// lit = 0.2 + 0.8 * 1.0 * 0.5 = 0.6, so out = pigment * 0.6.
inline umbreon::DistantLight makeKeyLight() {
  umbreon::DistantLight l;
  l.direction = {0, 0, -1};
  l.color = {1, 1, 1};
  l.intensity = 0.5f;
  return l;
}

// Build a scene with one shaded material sphere at the origin (radius 1), framed
// head-on by the ortho camera so the center pixel hits the apex with N=(0,0,1),
// V=(0,0,1) and the key light L=(0,0,1). Every cos term is then 1, so shadeLocal
// reduces to the closed-form values the material checks below assert against.
inline umbreon::Scene makeMaterialSphereScene(umbreon::Vec4 color,
                                       const umbreon::Material& mat,
                                       umbreon::Vec3 bg) {
  umbreon::Scene sc;
  sc.camera = makeOrthoCam();
  sc.lights.push_back(makeKeyLight());
  sc.background = bg;
  umbreon::Sphere sp;
  sp.center = {0, 0, 0};
  sp.radius = 1.0f;
  sp.color = color;
  sp.material = mat;
  sc.spheres.push_back(sp);
  return sc;
}

// Index of the center pixel of a 5x5 image, in float-RGBA element units.
constexpr std::size_t kCenterRgba = (2 * 5 + 2) * 4;
constexpr std::size_t kCenterPix = 2 * 5 + 2;
