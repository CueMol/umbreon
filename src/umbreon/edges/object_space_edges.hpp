// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Analytic OBJECT-SPACE silhouette edges for spheres and cylinders.
//
// This is the implementation of the OBJECT-SPACE (3D geometry) edge method
// (--obj-edges). Its public knob is ObjectSpaceEdgeOptions in
// render/render_types.hpp; render() drives this pass internally via
// RenderOptions::objectSpaceEdges. Its counterpart is the Freestyle-style STROKE
// method in edges/stroke_edges.hpp. The two are independent; render() rejects
// enabling both at once (they would double-draw).
//
// For each analytic primitive (Sphere, Cylinder) this computes the 3D locus
// where the surface normal is perpendicular to the view direction (the n.v == 0
// contour, i.e. the CueMol "edge"/silhouette) and EMITS it as thin flat-black
// "open" cylinders appended to Scene::cylinders. The existing Embree ray tracer
// then handles visibility / occlusion / antialiasing / fog / transparency for
// these edges for free -- exactly like the baked POV edge_line cylinders.
//
// The silhouette is CAMERA DEPENDENT (it is defined relative to the viewer), so
// generateObjectSpaceEdges() must run AFTER Scene::camera is assigned and the
// scene is otherwise fully assembled, but BEFORE the trace (render() handles
// this ordering on a private scene copy).
#pragma once

#include "render/render_types.hpp"  // ObjectSpaceEdgeOptions

namespace umbreon {

struct Scene;

// Append analytic object-space silhouette edges for every original Sphere and
// Cylinder, AND the silhouette/crease/border edges of scene.mesh, in `scene` (a
// SNAPSHOT of the counts is taken before appending, so the generated edges are
// never themselves silhouetted). Primitives tagged fromEdgeMacro (baked POV
// outlines) are skipped as sources. Reads scene.spheres, scene.cylinders,
// scene.mesh and scene.camera; appends Cylinder edges to scene.cylinders. No-op
// when opt.enable is false.
void generateObjectSpaceEdges(Scene& scene, const ObjectSpaceEdgeOptions& opt);

}  // namespace umbreon
