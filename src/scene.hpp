// Core scene data structures shared across the prototype.
// Pure C++17, no rendering-library dependency.
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

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

// De-indexed triangle mesh: triangle i uses vertices [3i, 3i+1, 3i+2].
// Each corner carries its own position, normal and color so that per-face-corner
// POV "texture_list" indices are reproduced exactly.
struct Mesh {
  std::vector<Vec3> positions;  // size == 3 * triangleCount
  std::vector<Vec3> normals;    // size == 3 * triangleCount
  std::vector<Vec4> colors;     // size == 3 * triangleCount (rgb + opacity)
  Material material;

  std::size_t vertexCount() const { return positions.size(); }
  std::size_t triangleCount() const { return positions.size() / 3; }

  Aabb bounds() const {
    Aabb b;
    for (const Vec3& p : positions) b.extend(p);
    return b;
  }
};

// A shaded sphere (CueMol "ball" / silhouette joint). Color carries rgb+opacity.
struct Sphere {
  Vec3 center;
  float radius = 1.0f;
  Vec4 color{0.0f, 0.0f, 0.0f, 1.0f};
  Material material = Material::flatOutline();
};

// A shaded cylinder/capsule (CueMol "stick" / silhouette edge), rendered as a
// rounded linear curve segment between p0 and p1.
struct Cylinder {
  Vec3 p0, p1;
  float radius = 1.0f;
  Vec4 color{0.0f, 0.0f, 0.0f, 1.0f};
  Material material = Material::flatOutline();
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
};

// POV-Ray fog (applied as a depth-based post-process). The visible surface
// color is blended toward `color` by the transmittance along the view ray.
struct Fog {
  bool enabled = false;
  Vec3 color{0.0f, 0.0f, 0.0f};  // fog color (linear)
  float distance = 1.0e30f;      // POV `distance`: 1/e transmittance distance
  int type = 1;                  // 1 = constant, 2 = ground fog
  float offset = 0.0f;           // ground fog: full density at/below this alt
  float alt = 0.0f;              // ground fog: falloff scale above the offset
  Vec3 up{0.0f, 1.0f, 0.0f};     // ground fog: altitude axis (world space)
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

  std::size_t instanceCount() const { return instanceOffsets.size(); }
  std::size_t effectiveTriangles() const {
    return mesh.triangleCount() * (instanceOffsets.empty() ? 1 : instanceOffsets.size());
  }
};

}  // namespace umbreon
