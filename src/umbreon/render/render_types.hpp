// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// Renderer-agnostic options and framebuffer result shared by the umbreon
// (Embree) renderer and the bench harness. Pure C++17, no
// rendering-library dependency.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace umbreon {

// --- NPR edge styling types (shared) --------------------------------------
// These styling types are consumed by the Freestyle-style STROKE edge pass
// (render/stroke_edges.cpp, --edges): the stroke pipeline maps each EdgeNature
// onto an EdgeClass slot for per-section coloring (see Scene::groupEdgeStyle).
// They are also consumed by the OBJECT-SPACE (3D geometry) edge pass via
// ObjectSpaceEdgeOptions (defined below); its implementation lives in
// edges/object_space_edges.hpp (--obj-edges).
//
// The five edge classes. The stroke pass uses Silhouette / Object (border) /
// Crease as its per-nature styling slots and composites in a fixed precedence
// order (most-specific structural edge wins). `Count` is the array size, not a
// class.
enum class EdgeClass : uint8_t {
  Silhouette = 0,    // object vs background boundary
  Disconnected = 1,  // same object, depth discontinuity (signature Warabi line)
  Object = 2,        // different object id across the boundary
  Material = 3,      // different material id across the boundary
  Crease = 4,        // ridge/valley fold on the world-normal field
  Count = 5
};

// Independent styling for one edge class. Color is linear RGB (composited in
// linear space); width is the mask dilation radius in hi-res (supersample) px.
struct EdgeClassStyle {
  bool enabled = false;
  float color[3] = {0.0f, 0.0f, 0.0f};  // linear RGB
  float opacity = 1.0f;                 // 0..1
  float width = 2.0f;                   // dilation radius, hi-res px
};

// Per CueMol section (per transparency group): a bundle of the five class
// styles. A section without an explicit override uses
// StrokeEdgeOptions::defaultStyle.
struct EdgeStyle {
  EdgeClassStyle cls[static_cast<int>(EdgeClass::Count)];
};

// --- Freestyle-style stroke edge rendering (--edges) ----------------------
// Options for the STROKE edge pipeline (render/stroke_edges.cpp), the
// Freestyle-faithful replacement for the retired per-pixel screen-space pass.
// It extracts topology-tagged feature edges (silhouette/crease/border), chains
// them into continuous polylines, computes ray-cast visibility against the live
// Embree BVH, then rasterizes variable-width ribbons composited in linear space.
// Per-section styling reuses EdgeStyle/EdgeClassStyle/Scene::groupEdgeStyle.
//
// The master `enable` flag gates ALL new work: a default-constructed
// StrokeEdgeOptions (enable == false) runs no extra pass, so the renderer output
// stays byte-identical to the no-edge path.
struct StrokeEdgeOptions {
  bool enable = false;  // MASTER gate; false => zero new work, byte-identical

  // --- which natures to extract/draw ---
  bool silhouette = true;  // smooth n.v==0 contour + hard-edge straddle
  // Interior fold (dihedral) creases. DEFAULT OFF to match CueMol2, whose crease
  // line is gated on creaseLimit > 0 and defaults to -1 (off): a crease would
  // over-ink the deliberate degenerate-vertex sharp edges of a rectangular ribbon
  // sheet body (the n.v silhouette already outlines it). Enable with
  // --stroke-crease on when a faceted crease look is wanted.
  bool crease = false;
  bool border = true;      // open boundary edges (one incident face)

  // --- feature-edge extraction params (mirror ObjectSpaceEdgeOptions) ---
  // Ray-cast visibility is analytic, so no 3D lift is needed (raise == 0).
  float raise = 0.0f;                      // outward contour offset, world units
  // DRAW the analytic silhouettes of the analytic primitives (spheres/cylinders):
  // their n.v==0 contour is emitted as Silhouette feature segments and MERGED into
  // the same chain/visibility/ribbon pipeline as the mesh edges, so ball-and-stick
  // geometry is outlined by --edges too (with true cross-primitive QI hidden-line
  // against the live BVH). ON by default so a bare "--edges on" outlines mesh +
  // spheres + cylinders alike.
  //
  // NOTE this flag gates only RASTERIZATION. The analytic silhouettes are ALWAYS
  // extracted (when `silhouette` is on) and ALWAYS participate in the visibility
  // pass, because an analytic primitive's silhouette is a real occluder boundary:
  // it splits the mesh ViewEdges at the occlusion boundary so the QI per-span vote
  // hides the part of a ribbon edge passing BEHIND a ball/stick. So `analytic =
  // false` (--stroke-analytic off) gives a MESH-ONLY outline whose hidden-line is
  // still correct behind ball-and-stick (the ball-stick simply gets no outline) --
  // it does NOT reintroduce the leak a naive mesh-only pass would have.
  bool analytic = true;
  int analyticSegments = 48;               // sphere ring / cap circle tessellation
  float meshHardEdgeDeg = 40.0f;           // hard-edge straddle / cluster split
  float creaseAngleDeg = 30.0f;            // crease dihedral threshold (degrees)
  float meshCreaseSmoothVetoDeg = 35.0f;   // smooth-facet crease veto (0 = off)
  bool meshCreaseConvexOnly = true;        // keep convex creases, drop valleys
  float meshBorderCoplanarVetoDeg = 35.0f; // coplanar-continuation border veto
  int meshCreaseMaxDegree = 4;             // drop crease hubs above this degree
  // Silhouette method (--stroke-geom-silhouette). false (default) = SMOOTH contour
  // (interpolated n.v==0) + hard-edge straddle; matches the smooth-shaded outline.
  // true = GEOMETRIC per-edge silhouette (face-normal straddle on ALL mesh edges):
  // follows mesh edges at mesh resolution (denser at grazing folds) but is faceted
  // and breaks up under grazing QI. See ExtractParams::geomSilhouette.
  bool meshGeomSilhouette = false;
  // Drop CONCAVE (valley) feature edges across ALL natures (crease + hard-edge
  // silhouette) via the two adjacent faces' geometric normals (--edge-reject-
  // concave). On by default for stroke edges.
  bool rejectConcaveEdges = true;
  // QI self-occlusion exclude radius (edge-adjacency rings over the true surface;
  // see ExtractParams::selfExcludeRings). >0 stops a twisted ribbon over-hiding its
  // own silhouette where its across-width fold self-occludes it, without leaking a
  // genuine (geodesically-far) occluder. Default 6 ≈ a ribbon's width in faces;
  // 0 restores the former {f0,f1}-only self-exclude.
  int selfExcludeRings = 6;

  // --- visibility (FREESTYLE-FAITHFUL image-space hidden-line) ---
  // Visibility is decided in two complementary image-space stages, run per chain
  // by applyStrokeEdges (NO primary z-buffer / G-buffer is read, keeping this
  // distinct from --obj-edges):
  //  (A) Quantitative Invisibility: a ray cast from each feature SEGMENT's 3D
  //      center toward the eye, counting solid surfaces, with the segment's OWN
  //      incident mesh faces excluded as self-occluders (Freestyle
  //      ViewMapBuilder self/adjacent-face skip, live via the Embree argument
  //      filter). qi>0 => hidden.
  //  (B) a 2D crossing pass over the PROJECTED feature segments: where two drawn
  //      lines cross in screen space, the farther one (larger eye-space view-z
  //      at the crossing) is hidden (Freestyle CreateTVertex).
  // A stroke point is visible iff QI says visible AND it is not inside a (B)
  // hidden notch. There are no tuning knobs: the zTol that guards a coincident-
  // depth junction is derived from the mesh mean edge length internally.

  // --- stylization ---
  // Stroke geometry in FINAL-resolution pixels; applyStrokeEdges scales these by
  // the supersample factor since it runs on the hi-res (pre-downsample) frame, so
  // a line keeps its requested final width at any --supersample.
  int thickness = 2;        // stroke FULL width, final px (phase-1 const)
  int resampleStepPx = 2;   // arc-length resample step, final px
  float color[3] = {0.0f, 0.0f, 0.0f};  // default linear RGB
  float opacity = 1.0f;                 // 0..1
  // Demo stylization shader: taper each stroke's width toward its two ends as a
  // function of the curvilinear abscissa u (a "spindle"/calligraphic look). Off by
  // default so the stroke pass stays byte-identical; --stroke-taper on enables it.
  bool taper = false;
  // Demo GEOMETRY shader: Laplacian-smooth the backbone to remove tessellation-
  // induced jaggedness, while PRESERVING endpoints and sharp corners (turn angle
  // above a threshold) so genuine angular features are not rounded. Off by default
  // (byte-identical); --stroke-smooth on enables it.
  bool smooth = false;

  // Per-section styling: a section without an override uses defaultStyle. The
  // stroke pipeline maps each EdgeNature onto a styling slot in EdgeStyle (see
  // Scene::groupEdgeStyle).
  EdgeStyle defaultStyle;

  // DEBUG visualization: overlay one dot per sub-span at its screen midpoint,
  // colored by the PRE-majority QI class vs the FINAL (post per-ViewEdge majority)
  // decision, so the majority rule's effect is visible on the render:
  //   green  = pre-visible, drawn   (correctly shown)
  //   blue   = pre-hidden,  dropped (correctly hidden)
  //   red    = pre-hidden,  drawn   (LEAK: majority dragged a hidden span visible)
  //   orange = pre-visible, dropped (over-hidden by the majority)
  // Off by default (no overlay, output unchanged). --edge-qi-dots on enables it.
  bool debugQiDots = false;

  // DEBUG (--edge-qi-vertex-dots): overlay the RAW per-backbone-VERTEX QI result
  // (one ray per vertex, no sub-span pooling or per-ViewEdge majority) as dots --
  // blue = hidden, green = visible -- to verify the bare QI test is self-consistent
  // vertex by vertex. Off by default (no overlay, output unchanged).
  bool debugQiVertexDots = false;

  // DEBUG (--edge-qi-vertex-delta): when nonzero, the --edge-qi-vertex-dots probe
  // is pushed off the surface by this distance (world units) along the ORIGINAL
  // mesh vertex normal before the visibility ray is cast (analogue of the old
  // eye-ward camBias, but along the surface normal). 0 = probe at the vertex.
  float debugQiVertexDelta = 0.0f;

  // PRODUCTION QI normal-lift (--edge-qi-lift), ABSOLUTE world units. When > 0 the
  // per-ViewEdge QI sampling (subSpanHidden) pushes each sample point off the
  // surface by this distance along the INTERPOLATED input-mesh vertex normal (GPU-
  // shader-style lerp of the segment's two endpoint normals) and casts a PURE
  // occlusion ray (no self-face exclude / grazing / coplanar). This replaces those
  // self-occlusion heuristics with a geometric self-surface clearance, fixing the
  // hidden-line leak at tight folds. Absolute (not mesh-relative) because molecular
  // coordinates have a fixed Angstrom scale. Default 0 = legacy heuristic QI
  // (concave-reject alone is the production fix; lift is opt-in for tight folds).
  // Falls back to legacy per-sample when the interpolated normal is degenerate
  // (e.g. analytic sphere/cylinder chains with no mesh normal).
  float qiNormalLift = 0.0f;

  // QI aggregation mode when qiNormalLift > 0 (--edge-qi-split). true = approach B:
  // per-sample visibility split into runs at transitions (no majority), so a hidden
  // fold-back on a partly-visible edge is cut off. false (default) = approach A:
  // per-ViewEdge majority (legacy aggregation, just with the lifted pure sample).
  // Ignored when qiNormalLift == 0.
  bool qiSplit = false;
};

// --- Analytic OBJECT-SPACE silhouette edges (--obj-edges) ------------------
// Options for the analytic object-space edge pass (edges/object_space_edges.cpp).
// This is the counterpart of the Freestyle STROKE method (StrokeEdgeOptions):
// for each analytic primitive (Sphere, Cylinder) and the triangle mesh it emits
// the n.v == 0 silhouette (plus mesh crease/border edges) as thin flat-black
// "open" cylinders that the Embree ray tracer then occludes/antialiases for free.
// Camera dependent. Driven through RenderOptions::objectSpaceEdges; render()
// runs the pass internally (on a private scene copy) before tracing. The two
// edge methods are mutually exclusive -- enabling both is an error (they would
// double-draw). enable == false (the default) is a no-op, byte-identical to the
// no-edge path. Pure data; defined here so it is part of the installed public
// API surface (the extraction implementation header stays internal).
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

  // Object-space HIDDEN-LINE clip for the triangle-mesh edges (CueMol
  // RendIntData_AABBTree visibility/clipping ported to Embree). When false (the
  // default) each mesh FeatureSeg is emitted verbatim and the ray tracer alone
  // occludes the resulting cylinders -- byte-identical to the legacy path. When
  // true, a throwaway mesh BVH (edges/edge_mesh_bvh) splits each edge into its
  // VISIBLE spans before emission, so an edge raised off the surface (or behind a
  // transparent surface) is correctly hidden where it dips behind geometry,
  // exactly as CueMol pre-removed the hidden parts before emitting edge_line.
  // Analytic sphere/cylinder silhouettes still use the union-boundary clip + ray
  // tracer (they are not part of the triangle mesh).
  bool visibilityClip = false;
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

// Options for umbreon::render(). Every field here is honored by the renderer;
// the defaults reproduce the POV-faithful look with all secondary effects off
// (so a default-constructed RenderOptions yields plain primary-ray shading).
struct RenderOptions {
  // --- output ---
  int width = 1024;   // final image width  (pixels)
  int height = 768;   // final image height (pixels)
  // Supersampling factor: render at width*ss x height*ss and box-average down to
  // width x height in linear space (antialiasing). 1 = off.
  int supersample = 1;

  // --- ambient occlusion (mesh hits only; modulates the ambient term) ---
  // Default 0 = AO off, so flag-less output stays the bit-exact POV-matched
  // local shading. AO never darkens flat outline primitives (spheres/cylinders).
  int aoSamples = 0;           // AO rays per mesh hit; 0 = off
  float aoDistance = 1.0e20f;  // AO occluder search radius (ray tfar / world units)
  float aoIntensity = 1.0f;    // AO strength: aoFactor = 1 - aoIntensity*(1-rawAO)

  // --- AO quality enhancements (all default to the legacy binary single-scale
  // behavior). When aoEnhanced() is false the legacy computeAO runs and the
  // output is bit-identical to the pre-enhancement renderer; any non-default
  // flag below switches the hit shader to the computeAOQuality estimator.
  float aoFalloffPower = 0.0f;   // 0 = binary (legacy); >0 => (max(0,1-t/R))^power
  bool aoMultiScale = false;     // false = single radius (aoDistance); true = 3-scale
  bool aoBentNormal = false;     // directional ambient from the avg unoccluded dir
  float aoSkyColor[3] = {1.0f, 1.0f, 1.0f};     // up-hemisphere tint (x ambient)
  float aoGroundColor[3] = {1.0f, 1.0f, 1.0f};  // down-hemisphere tint
  bool aoUseCameraUp = true;     // gradient axis = camera up (view-stable)
  float aoUp[3] = {0.0f, 1.0f, 0.0f};  // explicit gradient axis when !aoUseCameraUp
  bool aoMultibounce = false;    // albedo-aware GTAO cubic (anti over-darkening)
  bool aoLowDiscrepancy = false; // Hammersley + per-pixel Cranley-Patterson rotation
  float aoDiffuseFactor = 0.0f;  // 0 = ambient-only; >0 also darkens direct diffuse
  bool aoWriteAov = false;       // emit AO/G-buffer AOVs into FrameResult (phase 5)

  // True when any AO enhancement is requested. Drives the hit shader's
  // enhanced-vs-legacy branch: false => bit-exact legacy computeAO path.
  bool aoEnhanced() const {
    return aoFalloffPower > 0.0f || aoMultiScale || aoBentNormal ||
           aoMultibounce || aoLowDiscrepancy || aoDiffuseFactor > 0.0f;
  }

  // --- diffuse GI: adaptive surface irradiance cache (mesh hits only) ---
  // Default gi == false => no cache pass at all, byte-identical to the local-
  // shading render. When on, a deterministic set of surface cache records is
  // placed and filled by hemisphere gather (see render/irradiance_cache.hpp);
  // the interpolated indirect irradiance is exposed as the `indirect` AOV.
  // NOTE (steps 1-3): the gather + cache are built and visualized via AOVs, but
  // the final composite ([E], L += gi*kd*E_cached) is NOT wired yet, so a gi==on
  // render produces the SAME color as gi==off (only the AOVs differ).
  bool gi = false;              // MASTER gate; false => no GI work, byte-identical
  int giSamples = 64;           // hemisphere gather rays per cache record
  int giBounces = 1;            // 1 = one-bounce; >1 = multi-bounce (later step)
  float giMaxDistance = 0.0f;   // gather ray tfar; 0 => auto (scene diagonal)
  float giIntensity = 1.0f;     // indirect gain (physical 1.0; user knob, no 1/pi)
  float giAccuracy = 0.15f;     // interpolation accuracy a (max influence = a*R_i)
  float giRecordSpacing = 0.0f; // voxel seed world spacing; 0 => auto (diag*0.007)
  bool giGradients = false;     // gradient interpolation (later step; unused now)
  bool giAdaptive = false;      // adaptive voxel refinement (later step; unused now)
  float giNormalReject = 0.85f; // min dot(n_x, n_rec) to accept a record
  bool giComponentReject = true;// reject records of a different component (leak)
  bool giSeedPerVertex = false; // true => seed from mesh vertices (view-independent)

  // --- shadows (per-light visibility; never applied to outline primitives) ---
  bool shadows = false;        // cast shadows from the lights; false = off
  int shadowSamples = 1;       // shadow rays per light (>1 = soft area light)
  float lightRadius = 0.0f;    // light angular radius (deg); >0 = soft shadows

  // --- shading ---
  float specularScale = 1.0f;  // multiplies each material's specular weight

  // --- transparency (single-pass front-to-back compositing) ---
  // When on, the renderer walks hits front-to-back and composites every
  // transparent fragment ("over"), with groups in Scene::veilGroups instead laid
  // additively as single-layer "veils" (CueMol blendpng). Off = opaque only.
  bool transparency = true;
  // When on, the background contributes 0 coverage so the output alpha equals the
  // accumulated transparent coverage (POV "_transpbg"); default = opaque bg.
  bool transparentBackground = false;
  // Safety ceiling on transparent hits walked per primary ray. Normal
  // termination is the opacity early-out (accumulated alpha >= ~1); this only
  // bites pathological deep stacks. The renderer warns once if a ray hits it.
  int maxTransparentLayers = 256;

  // --- Freestyle-style stroke edges (--edges) --- defaulted OFF (enable ==
  // false). When off, no edge AOV is allocated and applyStrokeEdges is never
  // invoked, so output is byte-identical to the no-edge path. This single flag
  // is the master gate for the whole --edges pipeline (G-buffer AOV capture,
  // the stroke pass, and the baked-edge removal); see StrokeEdgeOptions.
  StrokeEdgeOptions strokeEdges;

  // --- Analytic OBJECT-SPACE edges (--obj-edges) --- defaulted OFF. The
  // counterpart of strokeEdges: render() runs generateObjectSpaceEdges()
  // internally (on a private scene copy) before tracing. Mutually exclusive with
  // strokeEdges -- enabling both throws std::runtime_error (they double-draw).
  ObjectSpaceEdgeOptions objectSpaceEdges;
};

// Rendered frame: linear HDR color plus AOV channels, top-left pixel origin.
struct FrameResult {
  int width = 0;
  int height = 0;
  std::vector<float> color;   // width*height*4 linear HDR RGBA
  std::vector<float> albedo;  // width*height*3
  std::vector<float> normal;  // width*height*3 world-space
  std::vector<float> depth;   // width*height   ray distance from camera
  // Edge G-buffer AOVs: sized and written ONLY when RenderOptions::strokeEdges
  // is enabled (otherwise left empty, keeping the default path byte-identical).
  std::vector<float> viewZ;          // width*height   linear view-z (edge-only)
  std::vector<std::uint32_t> objectId;    // width*height   per-pixel object id
  std::vector<std::uint32_t> materialId;  // width*height   per-pixel material id
  // AO / surface-irradiance-cache AOVs: sized and written ONLY when
  // RenderOptions::aoWriteAov is on (else left empty, keeping the default path
  // byte-identical). albedo/normal above are the OIDN guide; these are the AO
  // contact/shape split, the bent normal and the mean occluder distance.
  std::vector<float> contactAo;   // width*height   small-radius (contact) openness
  std::vector<float> shapeAo;     // width*height   mid+large-radius openness
  std::vector<float> bentNormal;  // width*height*3 average unoccluded direction
  std::vector<float> avgHitDist;  // width*height   mean occluder distance (world)
  // Surface-irradiance-cache AOVs: sized and written ONLY when RenderOptions::gi
  // is on (else empty, keeping the default path byte-identical). `position` is the
  // world-space first hit (cache spatial key / denoise guide); `indirect` is the
  // interpolated indirect irradiance E_cached (debug / denoise demodulation);
  // `giRecordViz` is a debug false-color of the nearest cache record's effective
  // radius R_i (bright = small radius = dense records, e.g. in concavities).
  std::vector<float> position;    // width*height*3 world-space first-hit position
  std::vector<float> indirect;    // width*height*3 interpolated E_cached
  std::vector<float> giRecordViz; // width*height*3 record-radius (log R_i) heatmap
  std::vector<float> giOcclusion; // width*height   gather occlusion fraction (AO-like)
  double renderSeconds = 0.0;
  std::size_t effectiveTriangles = 0;
};

}  // namespace umbreon
