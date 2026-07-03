// libumbreon PUBLIC API header (installed). Part of the supported public
// API surface; keep in sync with install(FILES) in CMakeLists.txt.
// Core scene data structures shared across the prototype.
// Pure C++17, no rendering-library dependency.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "render/render_types.hpp"

namespace umbreon {

// --------------------------------------------------------------------------
// Minimal vector math
// --------------------------------------------------------------------------
struct Vec3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  Vec3() = default;
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return a * s; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
  float l = length(a);
  return l > 0.0f ? a * (1.0f / l) : a;
}

// Length-flooring normalize: divides by sqrt(max(dot, tiny)) so a zero or
// degenerate input yields a finite (zero) vector instead of NaN/Inf. Mirrors
// OSPRay's rkcommon safe_normalize. Intended for secondary-ray / sampling-frame
// geometry; the plain normalize() above stays on the primary-ray path.
inline Vec3 safeNormalize(Vec3 a) {
  // 0x1.0p-126f is the smallest normal float (rkcommon flt_min); flooring dot
  // here guarantees a finite reciprocal square root for any finite input.
  float s = 1.0f / std::sqrt(std::fmax(dot(a, a), 0x1.0p-126f));
  return a * s;
}

// As above but returns `fallback` (assumed already unit) when `a` is shorter
// than ~1e-9, so a degenerate input gets a usable direction rather than zero.
inline Vec3 safeNormalize(Vec3 a, Vec3 fallback) {
  float d = dot(a, a);
  if (d < 1.0e-18f) return fallback;  // length < 1e-9
  return a * (1.0f / std::sqrt(d));
}

// Orthonormal basis around a unit normal n (n becomes the local +z axis). For
// hemisphere sampling at a surface a local sample (sx, sy, sz) maps to the world
// direction sx*t + sy*b + sz*n. Built with the larger-component branch (Duff et
// al. / OSPRay rkcommon frame()) so the seed axis is never near-parallel to n;
// safeNormalize guards the exactly-degenerate case (yields a finite, non-NaN
// frame). Assumes n is approximately unit length.
struct Frame {
  Vec3 t, b, n;
};

inline Frame frameFromNormal(Vec3 n) {
  const Vec3 seed0{0.0f, n.z, -n.y};
  const Vec3 seed1{-n.z, 0.0f, n.x};
  const Vec3 t = safeNormalize(std::fabs(n.x) < std::fabs(n.y) ? seed0 : seed1,
                               Vec3{1.0f, 0.0f, 0.0f});
  const Vec3 b = cross(n, t);
  return {t, b, n};
}

inline float radians(float deg) { return deg * 0.01745329252f; }
inline float degrees(float rad) { return rad * 57.2957795131f; }

// RGB color plus opacity (alpha). Linear color space.
struct Vec4 {
  float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
  Vec4() = default;
  Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
};

// Axis-aligned bounding box.
struct Aabb {
  Vec3 lo{1e30f, 1e30f, 1e30f};
  Vec3 hi{-1e30f, -1e30f, -1e30f};
  void extend(Vec3 p) {
    lo = {std::fmin(lo.x, p.x), std::fmin(lo.y, p.y), std::fmin(lo.z, p.z)};
    hi = {std::fmax(hi.x, p.x), std::fmax(hi.y, p.y), std::fmax(hi.z, p.z)};
  }
  bool valid() const { return lo.x <= hi.x; }
  Vec3 center() const { return (lo + hi) * 0.5f; }
  Vec3 extent() const { return hi - lo; }
  float diagonal() const { return length(extent()); }
};

// --------------------------------------------------------------------------
// Geometry / material
// --------------------------------------------------------------------------

// Material derived from a POV-Ray "finish" block. A single material per mesh.
struct Material {
  float ambient = 0.2f;     // POV finish ambient
  float diffuse = 0.8f;     // POV finish diffuse
  float specular = 0.0f;    // POV finish specular (POV default is 0)
  float roughness = 0.02f;  // POV finish roughness (smaller = sharper highlight)
  float brilliance = 1.0f;  // POV finish brilliance (diffuse exponent)
  float phong = 0.0f;       // POV finish phong amount
  float phongSize = 40.0f;  // POV finish phong_size exponent
  bool metallic = false;    // POV finish metallic (tint highlight by pigment)
  float reflection = 0.0f;  // POV finish reflection amount
  float emission = 0.0f;    // POV finish emission

  // FLAT outline preset: ambient 1, diffuse 0, specular 0. With ambientColor
  // (1,1,1) this yields out = pigment color exactly (raw flat color).
  static Material flatOutline() {
    Material m;
    m.ambient = 1.0f;
    m.diffuse = 0.0f;
    m.specular = 0.0f;
    m.roughness = 0.02f;
    m.brilliance = 1.0f;
    m.phong = 0.0f;
    m.phongSize = 40.0f;
    m.metallic = false;
    m.reflection = 0.0f;
    m.emission = 0.0f;
    return m;
  }
};

// Indexed triangle mesh. Vertices carry their own position, normal and color;
// triangle corners reference vertices through `index` (3 indices per triangle).
// A vertex is shared only when its position, normal AND color bit-match, so a
// per-face-corner POV "texture_list" index or a hard-edge normal split keeps its
// own vertex -- the rendered image is unchanged from the legacy triangle soup.
//
// `index` is OPTIONAL: when empty the mesh is de-indexed (triangle soup), i.e.
// corner c uses vertex c and triangle i uses vertices [3i, 3i+1, 3i+2]. All the
// hand-built meshes (tests, examples) rely on this fallback and need no index.
struct Mesh {
  std::vector<Vec3> positions;  // per vertex
  std::vector<Vec3> normals;    // per vertex
  std::vector<Vec4> colors;     // per vertex (rgb + opacity)

  // Triangle corner -> vertex. size == 3 * triangleCount, or EMPTY for the
  // trivial de-indexed mapping (corner c -> vertex c).
  std::vector<uint32_t> index;

  // Vertex -> position-class id: vertices welded by POSITION ALONE (the ~1e-4
  // topological weld used for feature-edge extraction, which must ignore the
  // normal/color splits that keep render vertices apart). size == vertexCount(),
  // or EMPTY when no class map was built (extractor then welds on its own).
  std::vector<int32_t> posClass;
  int32_t posClassCount = 0;

  Material material;

  // Per-triangle material: when a file has multiple mesh2 blocks with distinct
  // finishes, each block's material is appended to `materials` and every
  // triangle in that block gets the corresponding index in `triMaterialId`.
  // Empty vectors mean all triangles use `material` (single-material legacy).
  std::vector<Material> materials;
  std::vector<uint8_t> triMaterialId;

  // Per-triangle transparency group (= the CueMol section, e.g. "_34_35").
  // Empty means all triangles are group 0. This is DISTINCT from triMaterialId:
  // one section may hold several mesh2 blocks (distinct materials) that share a
  // single group, and the renderer composites only the frontmost surface PER
  // GROUP (single-layer transparency).
  std::vector<uint16_t> triGroupId;

  std::size_t vertexCount() const { return positions.size(); }
  std::size_t cornerCount() const {
    return index.empty() ? positions.size() : index.size();
  }
  std::size_t triangleCount() const { return cornerCount() / 3; }

  // Triangle corner -> vertex slot. The single seam every consumer routes
  // through; honors the optional `index` (trivial corner==vertex when empty).
  uint32_t cornerVertex(std::size_t corner) const {
    return index.empty() ? static_cast<uint32_t>(corner) : index[corner];
  }

  const Material& materialForTri(std::size_t triIdx) const {
    if (triMaterialId.empty()) return material;
    return materials[triMaterialId[triIdx]];
  }

  uint16_t groupForTri(std::size_t triIdx) const {
    return triGroupId.empty() ? 0 : triGroupId[triIdx];
  }

  Aabb bounds() const {
    Aabb b;
    for (const Vec3& p : positions) b.extend(p);
    return b;
  }

  // Expand to a de-indexed (triangle-soup) copy: every corner gets its own
  // vertex and `index`/`posClass` are dropped, so the feature-edge extractor
  // falls back to its own positional weld. Per-triangle material/group are kept.
  // Used by the regression test to compare the indexed path against the legacy
  // soup path within a single process. Bit-exact: corner values are copied, not
  // recomputed.
  Mesh toDeindexed() const {
    if (index.empty()) return *this;
    Mesh m;
    const std::size_t nCorner = index.size();
    m.positions.reserve(nCorner);
    m.normals.reserve(nCorner);
    m.colors.reserve(nCorner);
    const bool haveNrm = normals.size() == positions.size();
    const bool haveCol = colors.size() == positions.size();
    for (std::size_t c = 0; c < nCorner; ++c) {
      const uint32_t v = index[c];
      m.positions.push_back(positions[v]);
      if (haveNrm) m.normals.push_back(normals[v]);
      if (haveCol) m.colors.push_back(colors[v]);
    }
    m.material = material;
    m.materials = materials;
    m.triMaterialId = triMaterialId;
    m.triGroupId = triGroupId;
    return m;
  }
};

// A shaded sphere (CueMol "ball" / silhouette joint). Color carries rgb+opacity.
struct Sphere {
  Vec3 center;
  float radius = 1.0f;
  Vec4 color{0.0f, 0.0f, 0.0f, 1.0f};
  Material material = Material::flatOutline();
  uint16_t group = 0;  // transparency group (CueMol section); 0 = default
  // Origin tag for baked POV silhouette JOINT dots: the small black spheres
  // CueMol emits at edge-line vertices (writePoint, "<sec>_sl_tex" texture) to
  // round the outline polyline joints. Mirrors Cylinder::fromEdgeMacro so the
  // edge passes can drop them alongside the baked edge_line cylinders (otherwise
  // they survive as black speckles). Set in mesh2_reader; false for atom balls.
  bool fromEdgeMacro = false;
};

// A shaded cylinder/capsule (CueMol "stick" / silhouette edge), rendered as a
// rounded linear curve segment between p0 and p1.
struct Cylinder {
  Vec3 p0, p1;
  float radius = 1.0f;
  Vec4 color{0.0f, 0.0f, 0.0f, 1.0f};  // rgb + opacity at p0 (color.w)
  Material material = Material::flatOutline();
  uint16_t group = 0;  // transparency group (CueMol section); 0 = default
  // Opacity at p1 for an edge_line2 gradient (POV "gradient z" transmit fade).
  // < 0 means uniform opacity (use color.w along the whole segment).
  float opacity1 = -1.0f;
  // POV cap semantics. POV silhouette edges are emitted with the `open` keyword
  // (capless), and the renderer stitches them into ROUND_LINEAR_CURVE chains so
  // joints share a single swept-sphere (no double-cap seam). POV stick bonds and
  // density-mesh wireframes are emitted as plain (CLOSED) cylinders with FLAT
  // disk caps at the exact endpoints; the renderer draws those as independent
  // CONE_LINEAR_CURVE segments so a cap occupies zero axial thickness and stays
  // hidden inside the overlap of consecutive bonds (no protruding round cap).
  // true => `open` (round/chained edge); false => capped (flat-disk bond).
  bool open = false;
  // Origin tag: true only for cylinders produced by the POV edge_line/edge_line2
  // macros (baked-in POV silhouette outlines). Set exclusively in
  // mesh2_reader parseEdgeLine; raw cylinder{} bonds (including a user's open
  // black bond) keep it false. The screen-space edge pass uses this to drop the
  // baked POV outlines so they do not double-draw against the generated edges,
  // applied per section and only when --edges is on (graceful degradation).
  bool fromEdgeMacro = false;
};

// One group-alpha blend entry: CueMol "postprocess" transparency for a whole
// section (transparency group). render() realizes the listed entries with one
// extra full-pipeline pass per group and blends the FINAL display-encoded
// framebuffers -- the closed form of CueMol's blendpng lerp chain (solvebeta):
//   out = (1 - sum_i a_i) * render(scene minus every listed group)
//       + sum_i a_i * render(scene with group i kept, other listed groups hidden)
// The group's geometry renders OPAQUE (colors untouched) inside its own pass;
// fragment alpha (per-vertex color.w / POV native transmit) is orthogonal and
// still composites front-to-back "over" within each pass.
struct GroupBlend {
  uint16_t group = 0;  // transparency group (CueMol section) id
  float alpha = 1.0f;  // blend weight (CueMol group alpha; blendpng beta)
};

// --------------------------------------------------------------------------
// Scene description (parser/builder output, renderer input)
// --------------------------------------------------------------------------

struct Camera {
  Vec3 position{0.0f, 0.0f, 1.0f};
  Vec3 direction{0.0f, 0.0f, -1.0f};  // normalized view direction
  Vec3 up{0.0f, 1.0f, 0.0f};
  float fovy = 40.0f;          // perspective: vertical field of view, degrees
  bool orthographic = false;   // true => orthographic projection
  float height = 1.0f;         // orthographic: image-plane height, world units
};

// Directional light (POV light_source approximated as a distant light).
struct DistantLight {
  Vec3 direction{0.0f, -0.3f, -1.0f};  // direction the light travels
  Vec3 color{1.0f, 1.0f, 1.0f};
  float intensity = 1.0f;
  // POV-Ray: a `shadowless` light is a FILL_LIGHT_SOURCE, which contributes
  // diffuse only -- it produces no specular/phong highlight (trace.cpp gates
  // highlights on Light_Type != FILL_LIGHT_SOURCE). false reproduces that.
  bool castsHighlight = true;
};

// OpenGL linear fog (applied as a depth-based post-process), matching CueMol's
// interactive display. The fog factor is f = clamp((end - z)/(end - start), 0,1)
// over the PLANE eye-z `z`: f=1 at/before `start` (near, unfogged), f=0 at/after
// `end` (far). For an opaque background the visible RGB is mixed toward `color`
// by (1-f); for a transparent background the coverage (alpha) is faded by f
// instead, so no fog color is baked and a different backdrop can be composited
// later. The POV reader restores start/end from CueMol's POV ground-fog hack.
struct Fog {
  bool enabled = false;
  Vec3 color{0.0f, 0.0f, 0.0f};  // fog color (linear) = background color
  float start = 0.0f;            // plane eye-z where fogging begins (unfogged)
  float end = 0.0f;              // plane eye-z where fully the fog color

  // Legacy POV fog fields: still parsed for compatibility. `distance` feeds the
  // start/end restoration in the POV reader; type/offset/alt/up (the POV
  // ground-fog hack that approximated the linear GL fog) are no longer used by
  // the renderer.
  float distance = 1.0e30f;      // POV `distance` = slabDepth / 3
  int type = 1;
  float offset = 0.0f;
  float alt = 0.0f;
  Vec3 up{0.0f, 1.0f, 0.0f};
};

struct Scene {
  Mesh mesh;                          // base geometry (single copy)
  std::vector<Sphere> spheres;        // CueMol balls / silhouette joints
  std::vector<Cylinder> cylinders;    // CueMol sticks / silhouette edges
  std::vector<Vec3> instanceOffsets;  // translation per grid instance
  Camera camera;
  std::vector<DistantLight> lights;
  float ambientIntensity = 0.25f;     // constant fill light
  Vec3 ambientColor{1.0f, 1.0f, 1.0f};
  Vec3 background{0.0f, 0.0f, 0.0f};
  Fog fog;                            // optional POV fog (post-process)
  float aoDistance = 1.0e20f;         // AO ray max distance (scene-scaled)
  float assumedGamma = 1.0f;         // POV assumed_gamma (from global_settings)

  // Group-alpha (section) transparency realized as a blendpng-equivalent
  // multi-pass post-blend (see GroupBlend above). Empty (default) => a single
  // render pass and no blending; every transparent surface then composites
  // front-to-back "over" (fragment alpha).
  std::vector<GroupBlend> groupBlend;

  // Per-section (per transparency group) stroke edge style, indexed by group id
  // (objectId >> 2). Sized to groupNames.size() and pre-filled with
  // StrokeEdgeOptions::defaultStyle when --edges is on, then overridden per --edge
  // ID=spec. Empty (the default) means the stroke pass falls back to the global
  // StrokeEdgeOptions::defaultStyle. Only consulted when
  // RenderOptions::strokeEdges.enable is set.
  std::vector<EdgeStyle> groupEdgeStyle;

  std::size_t instanceCount() const { return instanceOffsets.size(); }
  std::size_t effectiveTriangles() const {
    return mesh.triangleCount() * (instanceOffsets.empty() ? 1 : instanceOffsets.size());
  }
};

}  // namespace umbreon
