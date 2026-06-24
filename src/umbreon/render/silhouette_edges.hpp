// Analytic OBJECT-SPACE silhouette edges for spheres and cylinders.
//
// For each analytic primitive (Sphere, Cylinder) this computes the 3D locus
// where the surface normal is perpendicular to the view direction (the n.v == 0
// contour, i.e. the CueMol "edge"/silhouette) and EMITS it as thin flat-black
// "open" cylinders appended to Scene::cylinders. The existing Embree ray tracer
// then handles visibility / occlusion / antialiasing / fog / transparency for
// these edges for free -- exactly like the baked POV edge_line cylinders.
//
// The silhouette is CAMERA DEPENDENT (it is defined relative to the viewer), so
// generateSilhouetteEdges() must run AFTER Scene::camera is assigned and the
// scene is otherwise fully assembled, but BEFORE umbreon::render().
//
// Pure C++17, no rendering-library dependency.
#pragma once

namespace umbreon {

struct Scene;

// Options for the analytic silhouette-edge pass. enable == false (the default)
// means generateSilhouetteEdges() is a no-op, so the render stays byte-identical
// to the no-edge path.
struct SilEdgeOptions {
  bool enable = false;   // master gate; false => no-op, byte-identical default
  float width = 0.03f;   // edge cylinder radius, world units
  float raise = 0.0f;    // outward offset of the contour, world units
  int segments = 48;     // ring tessellation (sphere ring / cylinder is 2 lines)
  float color[3] = {0.0f, 0.0f, 0.0f};  // edge color, linear RGB (w = 1 opacity)
  // Union-boundary clip: drop the parts of each primitive's silhouette that lie
  // INSIDE another primitive's solid, so connecting primitives (a bond entering
  // an atom) join along the intersection instead of crossing -- the per-primitive
  // "junction notch" otherwise left at coincident depth. Sampled along each
  // segment at ~`width` spacing (finer => cleaner, like a higher tessellation).
  bool clip = true;
};

// Append analytic object-space silhouette edges for every original Sphere and
// Cylinder in `scene` (a SNAPSHOT of the counts taken before appending, so the
// generated edges are never themselves silhouetted). Reads scene.spheres,
// scene.cylinders and scene.camera; appends Cylinder edges to scene.cylinders.
// No-op when opt.enable is false.
void generateSilhouetteEdges(Scene& scene, const SilEdgeOptions& opt);

}  // namespace umbreon
