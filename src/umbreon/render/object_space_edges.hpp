// Analytic OBJECT-SPACE silhouette edges for spheres and cylinders.
//
// This is the OBJECT-SPACE (3D geometry) edge method, selected by --obj-edges.
// Its counterpart is the SCREEN-SPACE (image-post-process) method in
// render/screen_space_edges.hpp (ScreenSpaceEdgeOptions, --edges). The two are
// independent; never enable both at once (they would double-draw).
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
// scene is otherwise fully assembled, but BEFORE umbreon::render().
//
// Pure C++17, no rendering-library dependency.
#pragma once

namespace umbreon {

struct Scene;

// Options for the analytic silhouette-edge pass. enable == false (the default)
// means generateObjectSpaceEdges() is a no-op, so the render stays byte-identical
// to the no-edge path.
struct ObjectSpaceEdgeOptions {
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

  // --- triangle-mesh edges (ribbon / SES / cartoon) ---
  // The mesh silhouette is the SMOOTH n.v == 0 contour: per interpolated VERTEX
  // normal a DotP = n.v is taken at each face vertex, and where it changes sign
  // across a face the zero-crossing is interpolated and connected through the
  // face (Freestyle WXFaceLayer::BuildSmoothEdge). This follows the shaded
  // silhouette smoothly instead of snapping to mesh edges (which is what CueMol's
  // face-normal extraction does, leaving a faceted line). Crease and border edges
  // DO lie on mesh edges and are emitted there.
  //
  // STRATEGY (geometry only, no color): the SMOOTH SILHOUETTE is the primary
  // edge and reproduces the CueMol OpenGL "outline" look on its own. CREASE and
  // BORDER over-ink a smooth ribbon (helix-barrel facet hatching, strip-seam
  // dashes on the coil tube, valley lines at SS-element junctions), so they are
  // GEOMETRICALLY GATED to fire only on genuine features:
  //   * a crease is a real sharp FOLD only where the interpolated vertex normals
  //     across the edge actually DISAGREE (a smooth-shaded facet seam has them
  //     near-parallel => tessellation, not a fold) -> meshCreaseSmoothVetoDeg;
  //   * a real outline fold is CONVEX (a ridge bulging toward the viewer); the
  //     concave valleys are where two ribbon strips meet at a junction step, which
  //     CueMol's builder marks NO-EDGE -> meshCreaseConvexOnly drops them;
  //   * a border that continues smoothly into another border edge at each end (a
  //     near-collinear, near-coplanar chain) is an internal strip SEAM, not a
  //     geometric terminus -> meshBorderCoplanarVetoDeg suppresses those, keeping
  //     only true open boundaries (cap rims, strand termini).
  // Struct defaults keep the new geometric gates OFF (neutral) so the bare-library
  // crease/border semantics are unchanged; the ribbon-tuned values that reproduce
  // the clean CueMol OpenGL outline live in the CLI (Options::objEdge*), which is
  // the user-facing knob for this feature.
  bool meshSilhouette = true;  // smooth n.v==0 contour through faces (primary)
  bool meshCrease = true;      // sharp folds (gated below), face-normal dihedral
  bool meshBorder = true;      // open boundary edges (gated below)
  // Hard-edge angle (degrees). CueMol ribbon meshes are deliberately NOT
  // water-tight: a sharp (rectangular beta-sheet) cross-section duplicates its
  // box-corner vertices with normals this far apart or more to encode the angular
  // shape + flat shading. The mesh-silhouette pass uses this twofold: (1) incident
  // corner normals at a welded position that differ by MORE than this are kept in
  // SEPARATE smoothing clusters (not averaged), so the smooth n.v==0 contour is
  // not computed from a meaningless diagonal and stops breaking into dashes on
  // sharp ribbons; (2) an interior edge whose two FACE normals differ by more than
  // this is a HARD edge, drawn on the silhouette by the CueMol-style face-normal
  // straddle test (one face front-, the other back-facing) -- a crisp continuous
  // box-edge line the per-vertex smooth contour cannot produce. A smooth tube has
  // all normals within this angle, so it is unaffected.
  float meshHardEdgeDeg = 40.0f;
  float creaseAngleDeg = 30.0f;  // dihedral threshold for a crease edge (degrees)
  // Smooth-facet veto: suppress a face-normal crease when BOTH faces' normals lie
  // within this angle of the shared edge's interpolated vertex normals (the mesh
  // is smooth-shaded across the edge => the dihedral is tessellation facetting,
  // not a CueMol-style crease). 0 disables the veto. Degrees.
  float meshCreaseSmoothVetoDeg = 0.0f;
  // Keep only CONVEX creases (ridges that bulge toward the average outward
  // normal); drop CONCAVE creases (valleys), the geometric stand-in for CueMol's
  // MFMOD_MESHXX no-edge junction-step faces.
  bool meshCreaseConvexOnly = false;
  // Coplanar-continuation border veto: suppress a border edge whose two endpoints
  // each continue into another border edge that is near-collinear (a smooth border
  // chain) -- an internal strip seam, not a true terminus. The angle is the max
  // bend (in degrees) of the border chain that still counts as "smoothly
  // continuing". 0 disables the veto.
  float meshBorderCoplanarVetoDeg = 0.0f;
  // Crease-cluster degree filter: drop a crease edge incident to a vertex where
  // MORE than this many crease edges meet. A clean fold LINE has crease degree
  // <=2 along it (<=~4 at a junction); a CAP/terminus blob radiates many creases
  // from one hub vertex. Removes the tube/chain-end cap scribbles geometrically
  // while keeping fold lines. 0 disables (emit every gated crease).
  int meshCreaseMaxDegree = 0;
};

// Append analytic object-space silhouette edges for every original Sphere and
// Cylinder, AND the silhouette/crease/border edges of scene.mesh, in `scene` (a
// SNAPSHOT of the counts is taken before appending, so the generated edges are
// never themselves silhouetted). Primitives tagged fromEdgeMacro (baked POV
// outlines) are skipped as sources. Reads scene.spheres, scene.cylinders,
// scene.mesh and scene.camera; appends Cylinder edges to scene.cylinders. No-op
// when opt.enable is false.
void generateObjectSpaceEdges(Scene& scene, const ObjectSpaceEdgeOptions& opt);

}  // namespace umbreon
