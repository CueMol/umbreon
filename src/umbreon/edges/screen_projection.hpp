// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Screen projection shared by the edge passes.
//
// worldToScreen is the algebraic inverse of the renderer's pixel->ray map
// (embree_renderer.cpp). It maps a world point to a top-left-origin pixel
// coordinate plus a linear view-z in the SAME units as the AOV firstViewZ, so
// a later visibility compare is apples-to-apples.
//
// Pure C++17, no rendering-library dependency.
#pragma once

#include "scene.hpp"

namespace umbreon {

// Camera projection basis built once per frame. Mirrors the renderer's camera
// basis (embree_renderer.cpp:81-89). halfW/halfH are the ORTHO image-plane
// half-extents (world units); persHalfW/persHalfH are the perspective half-
// extents at unit forward distance. W/H are the (hi-res) pixel dims.
struct ScreenProj {
  Vec3 pos;       // camera position
  Vec3 dir;       // normalized forward (view) axis
  Vec3 right;     // normalized image-plane right axis
  Vec3 up;        // normalized image-plane up axis
  bool ortho = false;
  float halfW = 1.0f, halfH = 1.0f;          // ortho half-extents (world units)
  float persHalfW = 1.0f, persHalfH = 1.0f;  // persp half-extents at unit depth
  int W = 0, H = 0;                          // pixel dimensions
};

// Build the projection basis for `cam` at pixel resolution w x h.
ScreenProj makeScreenProj(const Camera& cam, int w, int h);

// Project world point P to a top-left-origin pixel coordinate (x,y) and a linear
// view-z (vz). Algebraic inverse of the renderer's pixel->ray map. Returns false
// only for a perspective point at/behind the eye (zc <= ~0).
bool worldToScreen(const ScreenProj& sp, const Vec3& P, float& x, float& y,
                   float& vz);

}  // namespace umbreon
