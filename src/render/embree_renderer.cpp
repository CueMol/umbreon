#include "render/embree_renderer.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace umbreon {
namespace {

// Map an Embree error code to a short string. Embree 4 has no last-message
// query, so the synchronous post-commit check below reports the code via this;
// the async callback additionally prints the detailed message Embree supplies.
const char* rtcErrorString(RTCError code) {
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
void embreeErrorCallback(void* /*userPtr*/, RTCError code, const char* str) {
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

Vec3 faceForward(Vec3 n, Vec3 rayDir) {
  // POV/CueMol normals should point toward the viewer; flip if back-facing.
  return (dot(n, rayDir) > 0.0f) ? Vec3{-n.x, -n.y, -n.z} : n;
}

// POV-native light: direction from surface toward the light plus its radiance.
struct Light {
  Vec3 L;      // unit direction from surface toward the light
  Vec3 color;  // light color * intensity
  bool highlight = true;  // false for POV fill (shadowless) lights: diffuse only
  float radius = 0.0f;    // angular radius (radians) for soft shadows; 0 = hard
};

// Map POV "roughness" to a Blinn-Phong specular exponent. POV-Ray's Blinn
// specular uses pow(N.H, 1/roughness). roughness 0.01 -> exp 100.
float blinnExp(float roughness) {
  float r = roughness;
  if (r < 1e-6f) r = 1e-6f;
  float e = 1.0f / r;
  if (e < 1.0f) e = 1.0f;
  if (e > 1.0e6f) e = 1.0e6f;
  return e;
}

// --- Secondary-ray (ambient occlusion) helpers ----------------------------
// Ported from OSPRay's scivis/ao renderer (Apache-2.0): a cosine-weighted
// hemisphere AO estimator with deterministic per-pixel sampling (math only, not
// ISPC). AO is computed on mesh hits and modulates the ambient term; it is gated
// off by default (RenderOptions::aoSamples == 0) so flag-less output is unchanged.

// Scale-adaptive self-intersection epsilon (OSPRay calcEpsilon port): the
// distance to push a secondary-ray origin off the surface so it does not re-hit
// the surface it left. Scales with the hit-point magnitude and ray length, so it
// holds at any camera distance / scene scale (a fixed eps fails when the camera
// sits far from the scene). Same formula the transparency walk uses.
float selfIntersectEps(const Vec3& P, const Vec3& dir, float t) {
  const float dirMax =
      std::fmax(std::fabs(dir.x), std::fmax(std::fabs(dir.y), std::fabs(dir.z)));
  const float epsScale =
      std::fmax(std::fmax(std::fabs(P.x), std::fabs(P.y)),
                std::fmax(std::fabs(P.z), dirMax * t));
  constexpr float kUlpEps = 0x1.0fp-21f;  // ~5.05e-7 (~4 ULP), OSPRay ulpEpsilon
  return epsScale * kUlpEps;
}

// Tiny Encryption Algorithm, 8 rounds: hash two 32-bit seeds into a decorrelated
// pair. Seeded only from (pixel, sample index), so the stream is identical
// regardless of TBB thread count or grain size (deterministic, reproducible AO).
void tea2(uint32_t& v0, uint32_t& v1) {
  uint32_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    sum += 0x9E3779B9u;
    v0 += ((v1 << 4) + 0xA341316Cu) ^ (v1 + sum) ^ ((v1 >> 5) + 0xC8013EA4u);
    v1 += ((v0 << 4) + 0xAD90777Du) ^ (v0 + sum) ^ ((v0 >> 5) + 0x7E95761Eu);
  }
}

// Map a 32-bit hash to a float in [0,1) using the top 24 bits (mantissa width).
float u32ToUnorm(uint32_t u) { return (u >> 8) * 0x1.0p-24f; }

// Cosine-weighted hemisphere sample around frame f (f.n is the surface normal),
// Malley's method (a uniform disk lifted to the hemisphere). The cosine weight
// cancels the estimator's 1/cos, so AO needs no per-sample weighting.
Vec3 cosineSampleHemisphere(float u1, float u2, const Frame& f) {
  const float phi = 6.2831853072f * u1;  // 2*pi
  const float r = std::sqrt(u2);
  const float z = std::sqrt(std::fmax(0.0f, 1.0f - u2));
  const float x = std::cos(phi) * r;
  const float y = std::sin(phi) * r;
  return f.t * x + f.b * y + f.n * z;
}

// Any-hit visibility test along [tnear, tfar]: true if any geometry is hit
// (rtcOccluded1 sets ray.tfar < 0 on a hit). Transparent geometry counts as an
// opaque occluder (binary), as OSPRay scivis does -- cheaper than a second
// transparency walk and visually close.
bool occluded(RTCScene rscene, const Vec3& P, const Vec3& dir, float tnear,
              float tfar) {
  RTCRay r;
  r.org_x = P.x;
  r.org_y = P.y;
  r.org_z = P.z;
  r.dir_x = dir.x;
  r.dir_y = dir.y;
  r.dir_z = dir.z;
  r.tnear = tnear;
  r.tfar = tfar;
  r.mask = 0xFFFFFFFFu;
  r.flags = 0;
  r.time = 0.0f;
  RTCOccludedArguments oargs;
  rtcInitOccludedArguments(&oargs);
  rtcOccluded1(rscene, &r, &oargs);
  return r.tfar < 0.0f;
}

// Ambient-occlusion factor in [0,1] for a hit: 1 = fully open, 0 = fully
// occluded. Casts nSamples cosine-weighted rays over the hemisphere around the
// shading normal Ns; each ray origin is pushed off the surface along the
// GEOMETRIC normal Ng (not Ns: offsetting along an interpolated normal can dip
// the origin below a concave surface and self-hit). aoRadius bounds the search.
// Deterministic from the hi-res pixel (px, py).
float computeAO(RTCScene rscene, const Vec3& P, const Vec3& Ng, const Vec3& Ns,
                int nSamples, float aoRadius, uint32_t px, uint32_t py,
                int wHi) {
  if (nSamples <= 0) return 1.0f;
  const Frame f = frameFromNormal(Ns);
  // Offset axis = geometric normal, face-forwarded to the shading side.
  Vec3 ng = (dot(Ng, Ns) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
  ng = safeNormalize(ng, Ns);
  const float eps = selfIntersectEps(P, ng, 1.0f);
  const Vec3 O = P + ng * eps;
  const uint32_t base = px + py * static_cast<uint32_t>(wHi);
  int hits = 0;
  for (int i = 0; i < nSamples; ++i) {
    uint32_t s0 = base;
    uint32_t s1 = static_cast<uint32_t>(i);
    tea2(s0, s1);
    const Vec3 dir = cosineSampleHemisphere(u32ToUnorm(s0), u32ToUnorm(s1), f);
    if (dot(dir, Ns) < 0.01f) {  // grazing / below the surface: treat as occluded
      ++hits;
      continue;
    }
    if (occluded(rscene, O, dir, eps, aoRadius)) ++hits;
  }
  return 1.0f - static_cast<float>(hits) / static_cast<float>(nSamples);
}

// Per-light shadow factor in [0,1]: 1 = lit, 0 = fully shadowed. Casts a shadow
// ray from the hit point toward the (distant) light; transparent geometry is a
// binary occluder (as OSPRay scivis). The origin is offset along the geometric
// normal Ng (face-forwarded to the lit side) by the adaptive self-intersection
// epsilon, so a surface does not shadow itself. This is the HARD-shadow path
// (one ray); soft area-light sampling is added later.
float computeShadow(RTCScene rscene, const Vec3& P, const Vec3& Ng,
                    const Vec3& N, const Light& l) {
  Vec3 ng = (dot(Ng, N) < 0.0f) ? Vec3{-Ng.x, -Ng.y, -Ng.z} : Ng;
  ng = safeNormalize(ng, N);
  const float eps = selfIntersectEps(P, ng, 1.0f);
  const Vec3 O = P + ng * eps;
  const float far = std::numeric_limits<float>::infinity();  // distant light
  return occluded(rscene, O, l.L, eps, far) ? 0.0f : 1.0f;
}

// Shared POV local-illumination shader (no 1/pi factor). C is the pigment rgb,
// N the face-forwarded shading normal, V the unit direction toward the viewer.
// out = emission*C + aoFactor*ambient*C*ambLight
//       + sum_lights[ diffuse*C*pow(N.L,brilliance)*Lc + specular/phong lobe ]
//       + reflection*background.
// The FLAT preset (ambient 1, diffuse 0, specular 0, phong 0) with ambLight
// (1,1,1) yields out = C, preserving the flat outline/silhouette behavior.
Vec3 shadeLocal(const Material& mat, const Vec3& C, const Vec3& N, const Vec3& V,
                const std::vector<Light>& lights, const Vec3& ambLight,
                const Vec3& bg, float specularScale, float aoFactor,
                const Vec3& P, const Vec3& Ng, RTCScene rscene, bool shadowsOn) {
  Vec3 out{mat.emission * C.x + aoFactor * mat.ambient * C.x * ambLight.x,
           mat.emission * C.y + aoFactor * mat.ambient * C.y * ambLight.y,
           mat.emission * C.z + aoFactor * mat.ambient * C.z * ambLight.z};

  const float exp = blinnExp(mat.roughness);
  for (const Light& l : lights) {
    const float ndl = dot(N, l.L);
    if (ndl <= 0.0f) continue;

    // Per-light shadow factor folded into a local light color Lc, applied to
    // BOTH the diffuse and the highlight below. With shadowsOn == false (or a
    // fully lit point) Lc == l.color bitwise, so the shadow-off render is
    // byte-identical to the pre-shadow output.
    const float shadowFactor =
        shadowsOn ? computeShadow(rscene, P, Ng, N, l) : 1.0f;
    const Vec3 Lc{l.color.x * shadowFactor, l.color.y * shadowFactor,
                  l.color.z * shadowFactor};

    // Diffuse with brilliance as the N.L exponent. Guard pow(0,0).
    float d;
    if (mat.brilliance == 1.0f) {
      d = ndl;
    } else if (mat.brilliance == 0.0f) {
      d = 1.0f;
    } else {
      d = std::pow(ndl, mat.brilliance);
    }
    const float dk = mat.diffuse * d;
    out.x += dk * C.x * Lc.x;
    out.y += dk * C.y * Lc.y;
    out.z += dk * C.z * Lc.z;

    // POV fill (shadowless) lights contribute diffuse only -- no specular/phong
    // (trace.cpp gates highlights on Light_Type != FILL_LIGHT_SOURCE).
    if (!l.highlight) continue;

    // Highlight color. POV "metallic" tints the highlight toward the pigment by
    // an empirical Fresnel factor f(N.L): head-on (f=0) the highlight is fully
    // pigment-tinted, at grazing (f=1) it desaturates to the light color
    // (trace.cpp ComputeSpecularColour/ComputePhongColour). Non-metallic uses
    // the plain light color.
    Vec3 hl;
    if (mat.metallic) {
      float c = ndl;
      if (c > 1.0f) c = 1.0f;
      const float x = std::acos(c) * 0.63661977f;  // (angle)/(pi/2), 0..1
      float f = 0.014567225f / ((x - 1.12f) * (x - 1.12f)) - 0.011612903f;
      if (f < 0.0f) f = 0.0f;
      if (f > 1.0f) f = 1.0f;
      // cs = light * (f + (1-f)*pigment): lerp pigment->white by f.
      hl = Vec3{Lc.x * (f + (1.0f - f) * C.x),
                Lc.y * (f + (1.0f - f) * C.y),
                Lc.z * (f + (1.0f - f) * C.z)};
    } else {
      hl = Lc;
    }

    float specW = 0.0f;  // accumulated scalar specular weight
    // Blinn highlight (POV "specular S roughness R"), gated on specular > 0.
    if (mat.specular > 0.0f) {
      const Vec3 H = normalize(Vec3{l.L.x + V.x, l.L.y + V.y, l.L.z + V.z});
      float nh = dot(N, H);
      if (nh > 0.0f) specW += mat.specular * std::pow(nh, exp);
    }
    // Phong highlight (POV "phong P phong_size PS"), gated on phong > 0.
    // POV-faithful: intensity = phong * pow(R.V, phong_size) with no clamp, so a
    // large phong (e.g. 10000) saturates the channel to a white pip exactly as
    // POV-Ray does (ComputePhongColour). The supersample box-average then mirrors
    // POV's antialiasing of that crest. POV skips the term for tiny reflections
    // at high phong_size (phong_size >= 60 && R.V <= 0.0008).
    if (mat.phong > 0.0f) {
      const Vec3 Rr = 2.0f * ndl * N - l.L;
      float rv = dot(Rr, V);
      if (rv > 0.0f && (mat.phongSize < 60.0f || rv > 0.0008f))
        specW += mat.phong * std::pow(rv, mat.phongSize);
    }
    if (specW > 0.0f) {
      const float s = specW * specularScale;
      out.x += s * hl.x;
      out.y += s * hl.y;
      out.z += s * hl.z;
    }
  }

  // Cheap reflection: add reflection * background (no second ray). On scene4's
  // white background with no env geometry this matches POV non-radiosity.
  if (mat.reflection > 0.0f) {
    out.x += mat.reflection * bg.x;
    out.y += mat.reflection * bg.y;
    out.z += mat.reflection * bg.z;
  }
  return out;
}

}  // namespace

FrameResult EmbreeRenderer::render(const Scene& scene, const RenderOptions& opt) {
  RTCDevice device = rtcNewDevice(nullptr);
  if (!device) throw std::runtime_error("rtcNewDevice failed");
  rtcSetDeviceErrorFunction(device, embreeErrorCallback, nullptr);

  RTCScene rscene = rtcNewScene(device);
  rtcSetSceneFlags(rscene, RTC_SCENE_FLAG_ROBUST);
  // Static offline scene: committed once, then traversed by every primary ray
  // (and every future AO/shadow ray). A one-time HIGH-quality BVH (spatial
  // splits) amortizes over the whole frame, so HIGH is the right default vs
  // Embree's MEDIUM, as OSPRay builds its static scenes. Pure traversal-speed
  // win; it cannot change which primitive a ray hits.
  rtcSetSceneBuildQuality(rscene, RTC_BUILD_QUALITY_HIGH);

  const Mesh& m = scene.mesh;

  // Instance offsets are baked into the geometry (the .pov scenes have none;
  // the legacy grid path replicates each primitive per offset).
  std::vector<Vec3> bakeOffsets = scene.instanceOffsets;
  if (bakeOffsets.empty()) bakeOffsets.push_back(Vec3{0.0f, 0.0f, 0.0f});

  // Map geomID -> kind/handle so the shader can interpolate or read flat color.
  std::vector<GeomRecord> records;

  // --- triangle mesh (de-indexed, replicated per instance offset) ---
  if (m.vertexCount() >= 3) {
    const std::size_t baseV = m.positions.size();
    const std::size_t copies = bakeOffsets.size();
    const std::size_t nV = baseV * copies;
    const std::size_t nT = nV / 3;

    RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    auto* pos = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), nV));
    auto* idx = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned int),
        nT));

    // Two vertex-attribute slots: 0 = shading normal (FLOAT3), 1 = pigment
    // color RGBA (FLOAT4). Interpolated with the hit barycentrics.
    rtcSetGeometryVertexAttributeCount(g, 2);
    auto* nrm = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
        3 * sizeof(float), nV));
    auto* col = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT4,
        4 * sizeof(float), nV));

    for (std::size_t c = 0; c < copies; ++c) {
      const Vec3 off = bakeOffsets[c];
      for (std::size_t v = 0; v < baseV; ++v) {
        const std::size_t d = c * baseV + v;
        const Vec3 p = m.positions[v] + off;
        const Vec3 n = m.normals[v];
        const Vec4 cc = m.colors[v];
        pos[d * 3 + 0] = p.x;
        pos[d * 3 + 1] = p.y;
        pos[d * 3 + 2] = p.z;
        nrm[d * 3 + 0] = n.x;
        nrm[d * 3 + 1] = n.y;
        nrm[d * 3 + 2] = n.z;
        col[d * 4 + 0] = cc.x;
        col[d * 4 + 1] = cc.y;
        col[d * 4 + 2] = cc.z;
        col[d * 4 + 3] = cc.w;
      }
    }
    for (std::size_t t = 0; t < nT; ++t) {
      idx[t * 3 + 0] = static_cast<unsigned int>(3 * t + 0);
      idx[t * 3 + 1] = static_cast<unsigned int>(3 * t + 1);
      idx[t * 3 + 2] = static_cast<unsigned int>(3 * t + 2);
    }

    rtcCommitGeometry(g);
    unsigned int id = rtcAttachGeometry(rscene, g);
    if (id >= records.size()) records.resize(id + 1);
    records[id] = {GeomKind::Mesh, g};
    rtcReleaseGeometry(g);
  }

  // --- spheres (CueMol balls / silhouette joints) ---
  std::vector<Vec4> sphereColor;     // indexed by primID
  std::vector<Material> sphereMat;   // parallel to sphereColor (primID order)
  std::vector<uint16_t> sphereGroup; // parallel: transparency group (section)
  if (!scene.spheres.empty()) {
    const std::size_t n = scene.spheres.size() * bakeOffsets.size();
    RTCGeometry g = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SPHERE_POINT);
    auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
        g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float), n));
    sphereColor.reserve(n);
    sphereMat.reserve(n);
    sphereGroup.reserve(n);
    std::size_t k = 0;
    for (const Vec3& off : bakeOffsets) {
      for (const Sphere& s : scene.spheres) {
        vb[k * 4 + 0] = s.center.x + off.x;
        vb[k * 4 + 1] = s.center.y + off.y;
        vb[k * 4 + 2] = s.center.z + off.z;
        vb[k * 4 + 3] = s.radius;
        sphereColor.push_back(s.color);
        sphereMat.push_back(s.material);
        sphereGroup.push_back(s.group);
        ++k;
      }
    }
    rtcCommitGeometry(g);
    unsigned int id = rtcAttachGeometry(rscene, g);
    if (id >= records.size()) records.resize(id + 1);
    records[id] = {GeomKind::Sphere, g};
    rtcReleaseGeometry(g);
  }

  // --- cylinders: two POV cap semantics, two Embree geometries ---------------
  // POV emits two kinds of cylinders, distinguished by the parser's `open` flag:
  //
  //  (1) `open` (capless) silhouette EDGES -> ROUND_LINEAR_CURVE, chained.
  //      POV draws silhouette edges as a union of short `open` cylinders that
  //      share endpoints. Rendering each as an independent ROUND_LINEAR_CURVE
  //      capsule adds a hemispherical cap at every shared vertex, so two caps
  //      stack at each joint; through a transparent (faded) edge the extra cap
  //      layers multiply the transmittance and show as dark beads at the joints.
  //      We therefore stitch the segments into connected polylines and tag each
  //      with RTC_CURVE_FLAG_NEIGHBOR_*, so Embree drops the internal caps and a
  //      joint becomes a single shared swept-sphere -- POV's seamless union.
  //
  //  (2) CAPPED (CLOSED) bonds/wireframes -> CONE_LINEAR_CURVE, unchained.
  //      POV stick bonds are plain cylinder{p0,p1,r} with FLAT disk caps at the
  //      exact endpoints. Consecutive bonds OVERLAP but do not share endpoints
  //      and carry different colors, so they cannot chain. A ROUND cap would
  //      poke ~radius past its endpoint into the overlap and (with no neighbor
  //      link) never get clipped -- a hemispherical bulge that protrudes through
  //      an overlapping transparent surface (the red arc artifact). A
  //      CONE_LINEAR_CURVE segment ("capped cone, discontinuous at edge
  //      boundaries") has a flat cap of zero axial thickness at each endpoint,
  //      so it hides inside the overlap exactly as POV's disk caps do.
  //
  // The two geometries have independent primID spaces (Embree restarts primID at
  // 0 per geometry), so each gets its own primID-indexed side-tables and its own
  // GeomKind; shadeHit dispatches on the kind.
  std::vector<Vec4> cylColor;      // ROUND edges: indexed by primID (per segment)
  std::vector<Material> cylMat;    // parallel to cylColor (primID order)
  std::vector<uint16_t> cylGroup;  // parallel: transparency group (section)
  std::vector<float> cylOpacity1;  // parallel: p1 opacity (< 0 = uniform)
  std::vector<Vec4> cylCapColor;      // CONE bonds: indexed by primID (per segment)
  std::vector<Material> cylCapMat;    // parallel to cylCapColor (primID order)
  std::vector<uint16_t> cylCapGroup;  // parallel: transparency group (section)
  std::vector<float> cylCapOpacity1;  // parallel: p1 opacity (< 0 = uniform)
  if (!scene.cylinders.empty()) {
    // Partition source cylinders by cap semantics BEFORE chaining so the chain
    // builder only ever sees `open` edges (capped bonds must not be chained).
    std::vector<int> openIdx, capIdx;
    openIdx.reserve(scene.cylinders.size());
    capIdx.reserve(scene.cylinders.size());
    for (int i = 0; i < static_cast<int>(scene.cylinders.size()); ++i) {
      if (scene.cylinders[i].open) openIdx.push_back(i);
      else capIdx.push_back(i);
    }

    // ----- (1) OPEN edges: ROUND_LINEAR_CURVE chained (seam fix, unchanged) ---
    if (!openIdx.empty()) {
      const std::size_t no = openIdx.size();
      const std::size_t segs = no * bakeOffsets.size();
      cylColor.reserve(segs);
      cylMat.reserve(segs);
      cylGroup.reserve(segs);
      cylOpacity1.reserve(segs);

      // Curve buffers, built in chain order. Vertices are shared within a chain
      // (chainLen+1 vertices per chain), so segment j of a chain uses vertices
      // [base+j, base+j+1] and its neighbor flags reference base+j-1 / base+j+2,
      // which stay inside the chain (LEFT only when j>0, RIGHT only when j<n-1).
      std::vector<float> vbuf;          // 4 floats per vertex (x, y, z, radius)
      std::vector<unsigned int> ibuf;   // start-vertex index per segment
      std::vector<unsigned char> fbuf;  // RTCCurveFlags per segment
      vbuf.reserve((segs + no) * 4);
      ibuf.reserve(segs);
      fbuf.reserve(segs);

      const float eps = 1.0e-4f;  // endpoint match tolerance (<< radius ~0.03)
      auto keyOf = [&](const Vec3& p) {
        return std::array<long long, 3>{std::llround(p.x / eps),
                                        std::llround(p.y / eps),
                                        std::llround(p.z / eps)};
      };

      for (const Vec3& off : bakeOffsets) {
        // Baked endpoints for this instance (local index 0..no-1 over openIdx).
        std::vector<Vec3> a0(no), a1(no);
        for (std::size_t i = 0; i < no; ++i) {
          const Cylinder& c = scene.cylinders[openIdx[i]];
          a0[i] = {c.p0.x + off.x, c.p0.y + off.y, c.p0.z + off.z};
          a1[i] = {c.p1.x + off.x, c.p1.y + off.y, c.p1.z + off.z};
        }
        // Vertex incidence: key -> list of (segment, end) with end 0=p0, 1=p1.
        std::map<std::array<long long, 3>, std::vector<std::pair<int, int>>> inc;
        for (int i = 0; i < static_cast<int>(no); ++i) {
          inc[keyOf(a0[i])].push_back({i, 0});
          inc[keyOf(a1[i])].push_back({i, 1});
        }
        auto compatible = [&](int s, int t) {
          const Cylinder& cs = scene.cylinders[openIdx[s]];
          const Cylinder& ct = scene.cylinders[openIdx[t]];
          return cs.group == ct.group &&
                 (cs.opacity1 < 0.0f) == (ct.opacity1 < 0.0f);
        };
        // next[s]: the segment that continues from s.p1 (its p0 meets s.p1 at a
        // degree-2 vertex, same direction, compatible attributes). prev derived.
        std::vector<int> next(no, -1), prev(no, -1);
        for (int s = 0; s < static_cast<int>(no); ++s) {
          const auto& ends = inc[keyOf(a1[s])];
          if (ends.size() != 2) continue;
          int t = -1;
          for (const auto& e : ends)
            if (e.first != s) t = (e.second == 0) ? e.first : -1;
          if (t >= 0 && t != s && compatible(s, t)) next[s] = t;
        }
        for (int s = 0; s < static_cast<int>(no); ++s)
          if (next[s] >= 0) prev[next[s]] = s;

        std::vector<char> visited(no, 0);
        auto emitChain = [&](int start) {
          std::vector<int> chain;
          for (int s = start; s >= 0 && !visited[s]; s = next[s]) {
            visited[s] = 1;
            chain.push_back(s);
          }
          const unsigned int base = static_cast<unsigned int>(vbuf.size() / 4);
          auto pushV = [&](const Vec3& p, float r) {
            vbuf.push_back(p.x);
            vbuf.push_back(p.y);
            vbuf.push_back(p.z);
            vbuf.push_back(r);
          };
          pushV(a0[chain[0]], scene.cylinders[openIdx[chain[0]]].radius);
          for (int cs : chain)
            pushV(a1[cs], scene.cylinders[openIdx[cs]].radius);
          const int n = static_cast<int>(chain.size());
          for (int j = 0; j < n; ++j) {
            ibuf.push_back(base + static_cast<unsigned int>(j));
            unsigned char fl = 0;
            if (j > 0) fl |= RTC_CURVE_FLAG_NEIGHBOR_LEFT;
            if (j < n - 1) fl |= RTC_CURVE_FLAG_NEIGHBOR_RIGHT;
            fbuf.push_back(fl);
            const Cylinder& c = scene.cylinders[openIdx[chain[j]]];
            cylColor.push_back(c.color);
            cylMat.push_back(c.material);
            cylGroup.push_back(c.group);
            cylOpacity1.push_back(c.opacity1);
          }
        };
        // Open chains first (start = no predecessor), then any leftover cycles.
        for (int s = 0; s < static_cast<int>(no); ++s)
          if (prev[s] < 0 && !visited[s]) emitChain(s);
        for (int s = 0; s < static_cast<int>(no); ++s)
          if (!visited[s]) emitChain(s);
      }

      const std::size_t nSeg = ibuf.size();
      const std::size_t nVert = vbuf.size() / 4;
      RTCGeometry g =
          rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE);
      auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
          g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float),
          nVert));
      std::memcpy(vb, vbuf.data(), vbuf.size() * sizeof(float));
      auto* ib = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
          g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(unsigned int),
          nSeg));
      std::memcpy(ib, ibuf.data(), ibuf.size() * sizeof(unsigned int));
      auto* fb = static_cast<unsigned char*>(rtcSetNewGeometryBuffer(
          g, RTC_BUFFER_TYPE_FLAGS, 0, RTC_FORMAT_UCHAR, sizeof(unsigned char),
          nSeg));
      std::memcpy(fb, fbuf.data(), fbuf.size() * sizeof(unsigned char));
      rtcCommitGeometry(g);
      unsigned int id = rtcAttachGeometry(rscene, g);
      if (id >= records.size()) records.resize(id + 1);
      records[id] = {GeomKind::Cylinder, g};
      rtcReleaseGeometry(g);
    }

    // ----- (2) CAPPED bonds: CONE_LINEAR_CURVE, one segment per primID -------
    // Independent (unchained) segments: each control-point pair gets a flat disk
    // cap at p0 and p1. No flags buffer is needed -- a cone is discontinuous at
    // edge boundaries, and the flat caps occupy zero axial thickness so they
    // hide inside the overlap of consecutive bonds regardless of neighbors.
    if (!capIdx.empty()) {
      const std::size_t segs = capIdx.size() * bakeOffsets.size();
      cylCapColor.reserve(segs);
      cylCapMat.reserve(segs);
      cylCapGroup.reserve(segs);
      cylCapOpacity1.reserve(segs);

      std::vector<float> vbuf;         // 4 floats per vertex (x, y, z, radius)
      std::vector<unsigned int> ibuf;  // start-vertex index per segment
      vbuf.reserve(segs * 2 * 4);
      ibuf.reserve(segs);

      for (const Vec3& off : bakeOffsets) {
        for (int idx : capIdx) {
          const Cylinder& c = scene.cylinders[idx];
          const unsigned int base = static_cast<unsigned int>(vbuf.size() / 4);
          vbuf.push_back(c.p0.x + off.x);
          vbuf.push_back(c.p0.y + off.y);
          vbuf.push_back(c.p0.z + off.z);
          vbuf.push_back(c.radius);
          vbuf.push_back(c.p1.x + off.x);
          vbuf.push_back(c.p1.y + off.y);
          vbuf.push_back(c.p1.z + off.z);
          vbuf.push_back(c.radius);
          ibuf.push_back(base);  // segment uses vertices [base, base+1]
          cylCapColor.push_back(c.color);
          cylCapMat.push_back(c.material);
          cylCapGroup.push_back(c.group);
          cylCapOpacity1.push_back(c.opacity1);
        }
      }

      const std::size_t nSeg = ibuf.size();
      const std::size_t nVert = vbuf.size() / 4;
      RTCGeometry g =
          rtcNewGeometry(device, RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE);
      auto* vb = static_cast<float*>(rtcSetNewGeometryBuffer(
          g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float),
          nVert));
      std::memcpy(vb, vbuf.data(), vbuf.size() * sizeof(float));
      auto* ib = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
          g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(unsigned int),
          nSeg));
      std::memcpy(ib, ibuf.data(), ibuf.size() * sizeof(unsigned int));
      rtcCommitGeometry(g);
      unsigned int id = rtcAttachGeometry(rscene, g);
      if (id >= records.size()) records.resize(id + 1);
      records[id] = {GeomKind::CylinderCapped, g};
      rtcReleaseGeometry(g);
    }
  }

  rtcCommitScene(rscene);
  if (RTCError err = rtcGetDeviceError(device); err != RTC_ERROR_NONE) {
    rtcReleaseScene(rscene);
    rtcReleaseDevice(device);
    throw std::runtime_error(std::string("embree scene build failed: ") +
                             rtcErrorString(err));
  }

  // --- camera basis (POV orthographic framing) ---
  const Camera& cam = scene.camera;
  const int W = opt.width, H = opt.height;
  const float aspect = static_cast<float>(W) / static_cast<float>(H);

  const Vec3 dir = normalize(cam.direction);
  const Vec3 right = normalize(cross(dir, cam.up));
  const Vec3 trueUp = normalize(cross(right, dir));

  const float halfH = cam.height * 0.5f;
  const float halfW = halfH * aspect;
  // Perspective fallback half-extents at unit distance from the image plane.
  const float persHalfH = std::tan(radians(cam.fovy) * 0.5f);
  const float persHalfW = persHalfH * aspect;

  // --- POV-native lights: direction the light travels -> direction to light. ---
  std::vector<Light> lights;
  lights.reserve(scene.lights.size());
  for (const DistantLight& dl : scene.lights) {
    Light l;
    l.L = normalize(Vec3{-dl.direction.x, -dl.direction.y, -dl.direction.z});
    l.color = Vec3{dl.color.x * dl.intensity, dl.color.y * dl.intensity,
                   dl.color.z * dl.intensity};
    l.highlight = dl.castsHighlight;
    lights.push_back(l);
  }
  // POV ambient radiance: ambient_light defaults to <1,1,1>; the mesh ambient
  // term is material.ambient * pigment, applied below via ambK.
  const Vec3 ambLight = scene.ambientColor;  // expected <1,1,1> on the embree path

  FrameResult res;
  res.width = W;
  res.height = H;
  res.color.assign(static_cast<std::size_t>(W) * H * 4, 0.0f);
  res.depth.assign(static_cast<std::size_t>(W) * H, 0.0f);
  res.effectiveTriangles = scene.effectiveTriangles();

  const Vec3 bg = {scene.background.x, scene.background.y, scene.background.z};

  // Shade a single hit and report its opacity and transparency group (CueMol
  // section). Used by the front-to-back walk below.
  struct HitShade {
    Vec3 color{0.0f, 0.0f, 0.0f};
    float opacity = 1.0f;
    int group = 0;
  };
  auto shadeHit = [&](const RTCRayHit& rh, const Vec3& rd, const Vec3& org,
                      uint32_t px, uint32_t py) -> HitShade {
    HitShade hs;
    const GeomRecord& rec = records[rh.hit.geomID];
    const Vec3 V = normalize(Vec3{-rd.x, -rd.y, -rd.z});
    if (rec.kind == GeomKind::Mesh) {
      // Interpolate the shading normal and pigment color (slots 0/1).
      float nbuf[3] = {0, 0, 0};
      float cbuf[4] = {0, 0, 0, 1};
      rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                      RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
      rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                      RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
      Vec3 N = faceForward(normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]}), rd);
      const Vec3 C = {cbuf[0], cbuf[1], cbuf[2]};
      const Material& triMat = m.materialForTri(rh.hit.primID);
      // Hit point and GEOMETRIC normal (rh.hit.Ng, not the interpolated shading
      // normal N): shared by the AO and shadow secondary rays. Offsetting the
      // secondary-ray origin along Ng avoids self-hits on concave meshes.
      const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                   org.z + rd.z * rh.ray.tfar};
      const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
      // AO darkens ONLY the ambient term; gated off by default (aoSamples == 0)
      // so the flag-less render stays bit-exact (aoFactor == 1 -> x*1 == x).
      float aoFactor = 1.0f;
      if (opt.aoSamples > 0) {
        const float rawAO = computeAO(rscene, P, Ng, N, opt.aoSamples,
                                      opt.aoDistance, px, py, W);
        aoFactor = 1.0f - opt.aoIntensity * (1.0f - rawAO);
      }
      hs.color = shadeLocal(triMat, C, N, V, lights, ambLight, bg,
                            opt.specularScale, aoFactor, P, Ng, rscene,
                            opt.shadows);
      hs.opacity = cbuf[3];
      hs.group = m.groupForTri(rh.hit.primID);
    } else {
      // Outline / VdW primitives: shade with the per-primitive material.
      // The Embree geometric normal Ng is valid for SPHERE_POINT and for both
      // linear-curve modes (ROUND swept-sphere and CONE capped-cone): in round
      // mode and on the cone surface Ng is the non-normalized geometric surface
      // normal POV shades against (flat-curve tangent semantics do not apply).
      // Each cylinder geometry has its own primID space and side-tables; select
      // the table set by kind (Sphere / Cylinder=round edge / capped=cone bond).
      const bool isSphere = (rec.kind == GeomKind::Sphere);
      const bool isCapped = (rec.kind == GeomKind::CylinderCapped);
      const Vec4& fc = isSphere ? sphereColor[rh.hit.primID]
                       : isCapped ? cylCapColor[rh.hit.primID]
                                  : cylColor[rh.hit.primID];
      const Material& pm = isSphere ? sphereMat[rh.hit.primID]
                           : isCapped ? cylCapMat[rh.hit.primID]
                                      : cylMat[rh.hit.primID];
      Vec3 N = faceForward(
          normalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z}), rd);
      const Vec3 C = {fc.x, fc.y, fc.z};
      // Outline / VdW primitives are flat silhouette geometry: never AO-darkened
      // and never shadowed (aoFactor 1, shadowsOn false). This is the gate.
      const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                   org.z + rd.z * rh.ray.tfar};
      const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
      hs.color = shadeLocal(pm, C, N, V, lights, ambLight, bg,
                            opt.specularScale, 1.0f, P, Ng, rscene, false);
      hs.opacity = fc.w;
      hs.group = isSphere ? sphereGroup[rh.hit.primID]
                 : isCapped ? cylCapGroup[rh.hit.primID]
                            : cylGroup[rh.hit.primID];
      // edge_line2 gradient: opacity varies p0 (fc.w) -> p1 (opacity1) along the
      // segment. For both linear-curve modes rh.hit.u is the axial curve fraction
      // in [0,1], so a linear lerp reproduces POV's "gradient z" transmit fade
      // (uniform when opacity1 < 0). Opaque outlines and uniform bonds
      // (opacity1 < 0) are unaffected.
      if (!isSphere) {
        const float op1 =
            isCapped ? cylCapOpacity1[rh.hit.primID] : cylOpacity1[rh.hit.primID];
        if (op1 >= 0.0f) {
          float u = rh.hit.u;
          if (u < 0.0f) u = 0.0f;
          if (u > 1.0f) u = 1.0f;
          hs.opacity = fc.w * (1.0f - u) + op1 * u;
        }
      }
    }
    return hs;
  };

  // Veil lookup: groups rendered as additive single-layer (group alpha). Every
  // other transparency uses front-to-back "over" (fragment alpha). Empty =>
  // all transparency is "over".
  std::vector<uint8_t> isVeil;
  for (uint16_t g : scene.veilGroups) {
    if (g >= isVeil.size()) isVeil.resize(static_cast<std::size_t>(g) + 1, 0);
    isVeil[g] = 1;
  }

  auto t0 = std::chrono::high_resolution_clock::now();

  // Count primary rays that exhaust the transparent-layer cap so truncation of
  // far transmission is observable (warned once below) rather than silent.
  std::atomic<long long> cappedRays{0};

  // Parallelize over image rows with TBB (CueMol's unified CPU parallel
  // primitive). Each ray is independent and rtcIntersect1 on a committed scene
  // is thread-safe; pixels write to disjoint framebuffer indices.
  tbb::parallel_for(tbb::blocked_range<int>(0, H),
                    [&](const tbb::blocked_range<int>& rows) {
  for (int py = rows.begin(); py != rows.end(); ++py) {
    // Top-left origin: row 0 maps to v = +1 (top), last row to v = -1 (bottom).
    const float v = 1.0f - 2.0f * (static_cast<float>(py) + 0.5f) /
                               static_cast<float>(H);
    for (int px = 0; px < W; ++px) {
      const float u = 2.0f * (static_cast<float>(px) + 0.5f) /
                          static_cast<float>(W) - 1.0f;

      Vec3 org, rd;
      if (cam.orthographic) {
        org = cam.position + right * (u * halfW) + trueUp * (v * halfH);
        rd = dir;
      } else {
        org = cam.position;
        rd = normalize(dir + right * (u * persHalfW) + trueUp * (v * persHalfH));
      }

      const std::size_t pix = (static_cast<std::size_t>(py) * W + px);

      // Single-pass transparency with two coexisting models, selected per hit
      // by the hit's group:
      //   - VEIL groups (group alpha): additive single-layer, frontmost-per-
      //     group, order-independent -- exactly CueMol's blendpng.
      //   - everything else (fragment alpha = intrinsic per-color opacity):
      //     front-to-back "over", every surface composited (no dedup) -- POV
      //     native transmit; group alpha (if any) already multiplied in.
      // Combine: fragments composite over the opaque floor, then the veils are
      // laid additively on top. Reduces to pure "over" (no veils, e.g. scene5),
      // pure additive (all veils, the group-alpha path), or opaque (unchanged).
      Vec3 Cv{0.0f, 0.0f, 0.0f};  // additive (veil) premultiplied color
      float sumBeta = 0.0f;       // sum of veil weights
      Vec3 Cf{0.0f, 0.0f, 0.0f};  // over (fragment) premultiplied color
      float Af = 0.0f;            // over accumulated coverage
      float nearDepth = 0.0f;
      Vec3 base = bg;
      float baseCov = opt.transparentBackground ? 0.0f : 1.0f;

      constexpr int kMaxSeen = 64;  // distinct veil groups per ray
      int seen[kMaxSeen];
      int nseen = 0;

      const float kOpaque = 1.0f - 1e-4f;  // opacity at/above this == opaque
      float tnear = 0.0f;
      const int maxIters = opt.transparency ? (opt.maxTransparentLayers + 1) : 1;

      for (int iter = 0; iter < maxIters; ++iter) {
        RTCRayHit rh;
        rh.ray.org_x = org.x;
        rh.ray.org_y = org.y;
        rh.ray.org_z = org.z;
        rh.ray.dir_x = rd.x;
        rh.ray.dir_y = rd.y;
        rh.ray.dir_z = rd.z;
        rh.ray.tnear = tnear;
        rh.ray.tfar = std::numeric_limits<float>::infinity();
        rh.ray.mask = 0xFFFFFFFFu;
        rh.ray.flags = 0;
        rh.ray.time = 0.0f;
        rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        RTCIntersectArguments iargs;
        rtcInitIntersectArguments(&iargs);
        rtcIntersect1(rscene, &rh, &iargs);

        if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
            rh.hit.geomID >= records.size()) {
          break;  // ray escaped: base stays the background
        }
        if (nearDepth == 0.0f) nearDepth = rh.ray.tfar;

        const HitShade hs = shadeHit(rh, rd, org, static_cast<uint32_t>(px),
                                     static_cast<uint32_t>(py));

        if (!opt.transparency || hs.opacity >= kOpaque) {
          base = hs.color;  // nearest opaque surface = the floor
          baseCov = 1.0f;
          break;
        }

        const bool veil =
            hs.group >= 0 && static_cast<std::size_t>(hs.group) < isVeil.size() &&
            isVeil[hs.group] != 0;
        if (veil) {
          // additive: only the frontmost surface of each veil group
          bool dup = false;
          for (int sidx = 0; sidx < nseen; ++sidx)
            if (seen[sidx] == hs.group) { dup = true; break; }
          if (!dup) {
            if (nseen < kMaxSeen) seen[nseen++] = hs.group;
            const float a = hs.opacity;
            Cv.x += a * hs.color.x;
            Cv.y += a * hs.color.y;
            Cv.z += a * hs.color.z;
            sumBeta += a;
          }
        } else {
          // over: every fragment composited front-to-back (no dedup)
          const float w = (1.0f - Af) * hs.opacity;
          Cf.x += w * hs.color.x;
          Cf.y += w * hs.color.y;
          Cf.z += w * hs.color.z;
          Af += w;
        }
        if (sumBeta >= kOpaque || Af >= kOpaque) break;  // fully saturated
        // Advance just past this surface to find the next one. The step is a
        // self-intersection epsilon: it must clear only floating-point jitter,
        // never a distinct surface sitting just behind this one. A hardcoded
        // step is wrong because the hit-point precision degrades with scale --
        // the camera is ~200 units from the molecule, so the hit coordinates
        // carry an absolute error ~ t * 2^-23. Use a scale-adaptive epsilon, as
        // OSPRay does (modules/cpu/common/DifferentialGeometry.ih, calcEpsilon):
        // max(|hit point|, |dir| * t) scaled by a small float-ULP factor. This
        // adapts to any camera distance / scene scale with no magic constant.
        const Vec3 hitP{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                        org.z + rd.z * rh.ray.tfar};
        const float dirMax = std::fmax(
            std::fabs(rd.x), std::fmax(std::fabs(rd.y), std::fabs(rd.z)));
        const float epsScale =
            std::fmax(std::fmax(std::fabs(hitP.x), std::fabs(hitP.y)),
                      std::fmax(std::fabs(hitP.z), dirMax * rh.ray.tfar));
        constexpr float kUlpEps = 0x1.0fp-21f;  // ~5.05e-7 (~4 ULP), OSPRay ulpEpsilon
        tnear = rh.ray.tfar + epsScale * kUlpEps;
        // Reaching the final allowed iteration after compositing a transparent
        // layer means more surfaces may lie behind that we will not trace:
        // record the truncation for the post-loop warning.
        if (iter == maxIters - 1)
          cappedRays.fetch_add(1, std::memory_order_relaxed);
      }

      // Fragments over the opaque floor; veils laid additively on top. The base
      // (floor / background) contributes COLOR only where it is covered
      // (baseCov), so an opaque background (default) leaves opaque scenes
      // byte-unchanged, while a transparent background (baseCov 0) yields a
      // premultiplied result with alpha = accumulated coverage.
      const float floorW = (1.0f - Af) * baseCov;
      const Vec3 baseEff{Cf.x + floorW * base.x,
                         Cf.y + floorW * base.y,
                         Cf.z + floorW * base.z};
      const float covEff = Af + floorW;
      float vw = 1.0f - sumBeta;
      if (vw < 0.0f) vw = 0.0f;
      float outA = sumBeta + vw * covEff;
      outA = std::fmin(1.0f, std::fmax(0.0f, outA));

      res.color[pix * 4 + 0] = Cv.x + vw * baseEff.x;
      res.color[pix * 4 + 1] = Cv.y + vw * baseEff.y;
      res.color[pix * 4 + 2] = Cv.z + vw * baseEff.z;
      res.color[pix * 4 + 3] = outA;
      res.depth[pix] = nearDepth;
    }
  }
  });

  if (const long long capped = cappedRays.load(std::memory_order_relaxed);
      capped > 0) {
    std::fprintf(stderr,
                 "warning: %lld ray(s) reached the transparent-layer cap (%d); "
                 "transmission past that depth was dropped -- raise "
                 "maxTransparentLayers if it is visible\n",
                 capped, opt.maxTransparentLayers);
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  res.renderSeconds = std::chrono::duration<double>(t1 - t0).count();

  rtcReleaseScene(rscene);
  rtcReleaseDevice(device);
  return res;
}

}  // namespace umbreon
