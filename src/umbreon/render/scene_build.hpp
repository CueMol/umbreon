// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Embree scene construction for the umbreon renderer: turns a umbreon::Scene
// (mesh, spheres, cylinders) into a committed RTCScene plus the primID-indexed
// side tables the shader reads back. This is cold, run-once-per-frame work, so
// it lives in its own translation unit (scene_build.cpp); the hot per-hit
// shading path is in shading.hpp instead.
#pragma once

#include <cstdint>
#include <cstdio>
#include <vector>

#include <embree4/rtcore.h>

#include "scene.hpp"

namespace umbreon {
namespace detail {

// Map an Embree error code to a short string. Embree 4 has no last-message
// query, so the synchronous post-commit check reports the code via this; the
// async callback additionally prints the detailed message Embree supplies.
inline const char* rtcErrorString(RTCError code) {
  switch (code) {
    case RTC_ERROR_NONE: return "no error";
    case RTC_ERROR_UNKNOWN: return "unknown error";
    case RTC_ERROR_INVALID_ARGUMENT: return "invalid argument";
    case RTC_ERROR_INVALID_OPERATION: return "invalid operation";
    case RTC_ERROR_OUT_OF_MEMORY: return "out of memory";
    case RTC_ERROR_UNSUPPORTED_CPU: return "unsupported CPU";
    case RTC_ERROR_CANCELLED: return "cancelled";
    default: return "unrecognized error code";
  }
}

// Device error callback: log async Embree errors (with Embree's own message) to
// stderr so a malformed buffer / unsupported flag / OOM is diagnosable instead
// of corrupting the image silently.
inline void embreeErrorCallback(void* /*userPtr*/, RTCError code,
                                const char* str) {
  std::fprintf(stderr, "embree error %d (%s): %s\n", static_cast<int>(code),
               rtcErrorString(code), (str != nullptr) ? str : "");
}

// Per-geometry kind, recorded against the geomID so the shader knows how to
// color a hit (smooth-shaded mesh vs flat-colored outline primitive).
// Cylinder = POV `open` silhouette edges (ROUND_LINEAR_CURVE, chained, indexed
// by the cyl* side-tables). CylinderCapped = POV capped bonds/wireframes
// (CONE_LINEAR_CURVE, one segment per primID, indexed by the cylCap* tables).
// The two need distinct kinds because Embree restarts primID at 0 per geometry.
enum class GeomKind { Mesh, Sphere, Cylinder, CylinderCapped };

struct GeomRecord {
  GeomKind kind = GeomKind::Mesh;
  RTCGeometry geom = nullptr;  // borrowed handle, for rtcInterpolate on mesh
};

// A committed Embree scene plus the primID-indexed side tables the shader reads
// for the flat outline primitives (spheres and the two cylinder kinds). The
// caller owns `scene` and must rtcReleaseScene() it after rendering.
struct BuiltScene {
  RTCScene scene = nullptr;
  std::vector<GeomRecord> records;  // indexed by geomID

  // Spheres (CueMol balls / silhouette joints), indexed by primID.
  std::vector<Vec4> sphereColor;
  std::vector<Material> sphereMat;
  std::vector<uint16_t> sphereGroup;  // transparency group (section)

  // `open` silhouette edges (ROUND_LINEAR_CURVE), indexed by primID per segment.
  std::vector<Vec4> cylColor;
  std::vector<Material> cylMat;
  std::vector<uint16_t> cylGroup;
  std::vector<float> cylOpacity1;  // p1 opacity (< 0 = uniform)

  // Capped bonds (CONE_LINEAR_CURVE), indexed by primID per segment.
  std::vector<Vec4> cylCapColor;
  std::vector<Material> cylCapMat;
  std::vector<uint16_t> cylCapGroup;
  std::vector<float> cylCapOpacity1;

  // --- screen-space edges: per-primitive global material index side-tables ---
  // Phase-1 (doc 3.3 option b): a RAW per-primitive index per kind (no Material
  // dedup). The global materialId is offset by the running per-kind count so
  // the four kinds occupy disjoint id ranges:
  //   mesh tri  : triMaterialId[primID]                              (uint8)
  //   sphere    : meshMatCount + sphereMatIndex[primID]
  //   cyl       : meshMatCount + sphereMatCount + cylMatIndex[primID]
  //   cylCapped : meshMatCount + sphereMatCount + cylMatCount + cylCapMatIndex
  // Built only when edges are enabled (the gate is in EmbreeRenderer::render).
  std::vector<uint32_t> sphereMatIndex;
  std::vector<uint32_t> cylMatIndex;
  std::vector<uint32_t> cylCapMatIndex;
  uint32_t meshMatCount = 0;    // mesh material slots (<= 256; triMaterialId is uint8)
  uint32_t sphereMatCount = 0;  // sphere material slots (raw == sphere count)
  uint32_t cylMatCount = 0;     // cyl material slots    (raw == open-cyl count)
  uint32_t cylCapMatCount = 0;  // cylCap material slots (raw == capped-cyl count)
};

// Build and commit every Embree geometry for `scene` on `device`, returning the
// committed RTCScene and its side tables. Throws std::runtime_error (after
// releasing the partial scene) if the Embree build reports an error.
// `buildEdgeTables` (= RenderOptions::strokeEdges.enable) additionally fills the
// per-primitive global material-index side-tables used by the edge G-buffer
// capture; when false those tables stay empty (no extra allocation, byte-identical).
BuiltScene buildEmbreeScene(RTCDevice device, const Scene& scene,
                            bool buildEdgeTables = false);

}  // namespace detail
}  // namespace umbreon
