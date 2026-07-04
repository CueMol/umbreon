// Screen projection shared by the edge passes. See screen_projection.hpp.
#include "edges/screen_projection.hpp"

#include <cmath>

namespace umbreon {

ScreenProj makeScreenProj(const Camera& cam, int w, int h) {
  ScreenProj sp;
  sp.pos = cam.position;
  sp.dir = normalize(cam.direction);
  sp.right = normalize(cross(sp.dir, cam.up));
  sp.up = normalize(cross(sp.right, sp.dir));
  sp.ortho = cam.orthographic;
  const float aspect = static_cast<float>(w) / static_cast<float>(h);
  sp.halfH = cam.height * 0.5f;
  sp.halfW = sp.halfH * aspect;
  sp.persHalfH = std::tan(radians(cam.fovy) * 0.5f);
  sp.persHalfW = sp.persHalfH * aspect;
  sp.W = w;
  sp.H = h;
  return sp;
}

bool worldToScreen(const ScreenProj& sp, const Vec3& P, float& x, float& y,
                   float& vz) {
  const Vec3 d = P - sp.pos;
  const float zc = dot(d, sp.dir);
  float u, v;
  if (sp.ortho) {
    u = dot(d, sp.right) / sp.halfW;
    v = dot(d, sp.up) / sp.halfH;
    vz = zc;
  } else {
    if (zc <= 1.0e-6f) return false;  // at/behind the eye
    u = (dot(d, sp.right) / zc) / sp.persHalfW;
    v = (dot(d, sp.up) / zc) / sp.persHalfH;
    vz = zc;
  }
  x = (u + 1.0f) * 0.5f * static_cast<float>(sp.W) - 0.5f;  // top-left origin
  y = (1.0f - v) * 0.5f * static_cast<float>(sp.H) - 0.5f;
  return true;
}

}  // namespace umbreon
