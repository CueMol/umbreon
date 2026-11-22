// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Outline-mode far-side depth rule, shared by both edge methods.
//
// An Outline-mode section (the CueMol GL "silhouette" edge type) draws a
// contour line only against the background. A surface behind the line hides
// it -- unless that surface lies beyond `outlineFarVz` (linear view-z), where
// fog has dissolved it into the background, so the line stays. The screen
// method applies this to the far side of a same-section depth step
// (screen_edge_classify.cpp); the object method to the first surface behind an
// outline edge line along the primary ray (shading/transparency.hpp). Each
// method looks up a section's mode in its own style table; only this depth
// comparison is shared.
#pragma once

namespace umbreon {

inline bool outlineFarSideHidden(float farVz, float outlineFarVz) {
  return farVz <= outlineFarVz;
}

}  // namespace umbreon
