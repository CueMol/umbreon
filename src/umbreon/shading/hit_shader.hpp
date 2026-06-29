// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// Per-hit shading dispatch for the umbreon renderer: given a primary/transparency
// ray hit, pick the right shading path (smooth-shaded mesh vs flat outline
// primitive), interpolate or read the surface attributes, drive the AO/shadow
// secondary rays, and return the hit's color, opacity and transparency group.
//
// This is on the hot per-hit path, so it is `inline` here: embree_renderer.cpp
// and transparency.hpp include it and the compiler inlines it into the pixel
// loop exactly as if it were still a local lambda (no LTO needed).
#pragma once

#include <cstdint>

#include <embree4/rtcore.h>

#include "render/render_types.hpp"
#include "render/scene_build.hpp"
#include "shading/secondary_rays.hpp"
#include "shading/shading.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// One shaded hit: linear HDR color, its opacity, and the CueMol transparency
// group (section) used by the front-to-back compositor. The `normal`/`objectId`/
// `materialId` fields feed the screen-space edge G-buffer (Stage A); they are
// always populated but only read when RenderOptions::edges is enabled.
struct HitShade {
  Vec3 color{0.0f, 0.0f, 0.0f};
  float opacity = 1.0f;
  int group = 0;
  Vec3 normal{0.0f, 0.0f, 0.0f};   // face-forwarded SMOOTH shading normal (mesh)
                                   // or analytic normal (sphere/cyl); crease AOV
  uint32_t objectId = 0;           // (group << 2) | kindBits
  uint32_t materialId = 0;         // global per-primitive material id
  // AO / G-buffer AOVs, populated only when RenderOptions::aoWriteAov is on
  // (else left at the open/neutral defaults). These are the OIDN guide / surface
  // irradiance cache inputs and AO-tuning debug channels; the color path never
  // reads them, so enabling them does not change the rendered color.
  Vec3 albedo{0.0f, 0.0f, 0.0f};   // pigment color (OIDN guide / cache albedo)
  Vec3 bentNormal{0.0f, 0.0f, 0.0f};  // average unoccluded direction
  float contactAo = 1.0f;          // small-radius (contact) openness
  float shapeAo = 1.0f;            // mid+large-radius (shape) openness
  float avgHitDist = 0.0f;         // mean occluder distance (world units)
};

// Everything shadeHit() reads, gathered once per frame. References point at the
// built Embree scene + side tables, the mesh, the converted lights and the
// render options; copies hold the two scene constants (ambient + background).
struct ShadeContext {
  const BuiltScene& built;
  const Mesh& mesh;
  const std::vector<Light>& lights;
  Vec3 ambLight;
  Vec3 bg;
  const RenderOptions& opt;
  // Resolved up axis for the bent-normal sky/ground ambient gradient: the camera
  // true-up (view-stable) or an explicit world axis. Computed once per frame.
  Vec3 aoUp{0.0f, 1.0f, 0.0f};
};

// Shade a single ray hit. `rh` is the Embree hit, `rd` the ray direction, `org`
// the ray origin, and (px, py) the hi-res pixel (for deterministic AO/shadow
// sampling).
inline HitShade shadeHit(const ShadeContext& c, const RTCRayHit& rh,
                         const Vec3& rd, const Vec3& org, uint32_t px,
                         uint32_t py) {
  HitShade hs;
  RTCScene rscene = c.built.scene;
  const GeomRecord& rec = c.built.records[rh.hit.geomID];
  const Vec3 V = normalize(Vec3{-rd.x, -rd.y, -rd.z});
  if (rec.kind == GeomKind::Mesh) {
    // Interpolate the shading normal and pigment color (slots 0/1).
    float nbuf[3] = {0, 0, 0};
    float cbuf[4] = {0, 0, 0, 1};
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, nbuf, 3);
    rtcInterpolate0(rec.geom, rh.hit.primID, rh.hit.u, rh.hit.v,
                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, cbuf, 4);
    // Face-forward the SMOOTH shading normal using the GEOMETRIC normal Ng, NOT
    // the interpolated normal itself. At grazing incidence on a smoothly curved
    // surface (e.g. a tube bending toward the viewer, seen nearly along its axis)
    // dot(N_interp, rd) is ~0 and noisy across neighboring pixels, so a faceForward
    // keyed on N_interp flips it erratically per pixel -- a jagged dark shading
    // wedge. Ng is piecewise-constant per triangle and changes sign cleanly only at
    // the true silhouette, so keying the flip on Ng is stable. (rh.hit.Ng follows
    // the mesh winding, which CueMol emits consistently outward.)
    const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
    const Vec3 NgF = faceForward(Ng, rd);
    Vec3 N = normalize(Vec3{nbuf[0], nbuf[1], nbuf[2]});
    if (dot(N, NgF) < 0.0f) N = Vec3{-N.x, -N.y, -N.z};
    const Vec3 C = {cbuf[0], cbuf[1], cbuf[2]};
    const Material& triMat = c.mesh.materialForTri(rh.hit.primID);
    // Hit point. The GEOMETRIC normal Ng (declared above) is shared by the AO and
    // shadow secondary rays; offsetting the secondary-ray origin along Ng avoids
    // self-hits on concave meshes.
    const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                 org.z + rd.z * rh.ray.tfar};
    // Self-intersection epsilon for the secondary (AO/shadow) rays: the SAME
    // scale-adaptive value the transparency walk uses, from the PRIMARY ray
    // length rh.ray.tfar. A far camera makes the hit-point error ~ tfar*2^-23,
    // so a fixed / t=1 epsilon is far too small and the surface shadows itself
    // (shadow acne).
    const float secEps = selfIntersectEps(P, rd, rh.ray.tfar);
    // AO darkens ONLY the ambient term; gated off by default (aoSamples == 0)
    // so the flag-less render stays bit-exact (aoFactor == 1 -> x*1 == x).
    // aoFactor is per-channel (Vec3); the legacy path fills {s,s,s} so the
    // ambient float ops are unchanged. With any enhancement requested
    // (aoEnhanced) the multi-scale/falloff estimator runs instead; otherwise the
    // legacy binary computeAO runs verbatim, keeping --ao-samples-only renders
    // bit-identical too.
    Vec3 aoFactor{1.0f, 1.0f, 1.0f};
    Vec3 ambLight = c.ambLight;
    float diffuseAo = 1.0f;  // direct-diffuse AO scale (1 = POV ambient-only)
    AOResult aoAov;          // captured for the AOVs (default = fully open)
    if (c.opt.aoSamples > 0) {
      float openness;
      if (c.opt.aoEnhanced()) {
        AOParams ap;
        ap.nSamples = c.opt.aoSamples;
        ap.radius = c.opt.aoDistance;
        ap.falloffPower = c.opt.aoFalloffPower;
        ap.multiScale = c.opt.aoMultiScale;
        ap.lowDiscrepancy = c.opt.aoLowDiscrepancy;
        aoAov = computeAOQuality(rscene, P, Ng, N, secEps, ap, px, py,
                                 c.opt.width);
        openness = aoAov.openness;
        // Directional ambient: a 2-color sky/ground hemisphere gradient sampled
        // along the bent normal (the average unoccluded direction). A point that
        // opens toward `aoUp` sees the sky tint, one facing away sees the ground
        // tint. With the default white sky == ground this collapses to the plain
        // scene ambient, so enabling only bent normal still looks neutral.
        if (c.opt.aoBentNormal) {
          const float w = 0.5f * (dot(aoAov.bent, c.aoUp) + 1.0f);
          const float gx = c.opt.aoGroundColor[0], sx = c.opt.aoSkyColor[0];
          const float gy = c.opt.aoGroundColor[1], sy = c.opt.aoSkyColor[1];
          const float gz = c.opt.aoGroundColor[2], sz = c.opt.aoSkyColor[2];
          ambLight = Vec3{c.ambLight.x * (gx + (sx - gx) * w),
                          c.ambLight.y * (gy + (sy - gy) * w),
                          c.ambLight.z * (gz + (sz - gz) * w)};
        }
      } else {
        openness = computeAO(rscene, P, Ng, N, secEps, c.opt.aoSamples,
                             c.opt.aoDistance, px, py, c.opt.width);
        // Color stays on the bit-exact legacy path; if AOVs are requested,
        // derive the contact/shape/bent/avgHitDist channels with one extra
        // (single-scale) quality pass that does NOT feed the color.
        if (c.opt.aoWriteAov) {
          AOParams ap;
          ap.nSamples = c.opt.aoSamples;
          ap.radius = c.opt.aoDistance;
          aoAov = computeAOQuality(rscene, P, Ng, N, secEps, ap, px, py,
                                   c.opt.width);
        }
      }
      if (c.opt.aoMultibounce) {
        // Lift each channel by its pigment albedo so light cavities don't crush
        // to black (single-bounce AO assumes a black surface).
        const float ox = aoMultibounce(openness, C.x);
        const float oy = aoMultibounce(openness, C.y);
        const float oz = aoMultibounce(openness, C.z);
        aoFactor = Vec3{1.0f - c.opt.aoIntensity * (1.0f - ox),
                        1.0f - c.opt.aoIntensity * (1.0f - oy),
                        1.0f - c.opt.aoIntensity * (1.0f - oz)};
      } else {
        const float s = 1.0f - c.opt.aoIntensity * (1.0f - openness);
        aoFactor = Vec3{s, s, s};
      }
      // Optional indirect-shadowing approximation: also darken direct diffuse in
      // cavities (off by default => diffuseAo == 1 => bit-exact).
      if (c.opt.aoDiffuseFactor > 0.0f)
        diffuseAo = 1.0f - c.opt.aoDiffuseFactor * (1.0f - openness);
    }
    // Main-path AO / G-buffer AOVs (OIDN guide + surface-irradiance-cache input
    // + AO-tuning debug). Gated on aoWriteAov; the color computed above is
    // unchanged whether or not these are written.
    if (c.opt.aoWriteAov) {
      hs.albedo = C;
      hs.normal = N;
      hs.bentNormal = aoAov.bent;
      hs.contactAo = aoAov.contact;
      hs.shapeAo = aoAov.shape;
      hs.avgHitDist = aoAov.avgHitDist;
    } else if (c.opt.gi) {
      // GI cache seeds need the smooth shading normal and pigment at the first
      // mesh hit even when the AO AOVs are off. This does not feed the color.
      hs.albedo = C;
      hs.normal = N;
    }
    hs.color = shadeLocal(triMat, C, N, V, c.lights, ambLight, c.bg,
                          c.opt.specularScale, aoFactor, diffuseAo, P, Ng, secEps,
                          rscene, c.opt.shadows, c.opt.shadowSamples, px, py);
    hs.opacity = cbuf[3];
    hs.group = c.mesh.groupForTri(rh.hit.primID);
    // Edge G-buffer (only when the stroke edge pass is on; otherwise the
    // material-index side-tables are empty and these reads must not happen).
    if (c.opt.strokeEdges.enable) {
      // Crease AOV: store the SMOOTH interpolated shading normal N (slot-0
      // interpolated, already face-forwarded at line above). The geometric Ng
      // is piecewise-constant per triangle and would crease at EVERY facet edge
      // of a smooth CueMol SES/ribbon mesh; the smooth N only diverges across a
      // genuine geometric crease, which is what the crease class must detect.
      hs.normal = N;
      hs.objectId = (static_cast<uint32_t>(hs.group) << 2) |
                    static_cast<uint32_t>(rec.kind);  // kindBits: Mesh == 0
      // Mesh materialId is the per-triangle index directly (base 0;
      // triMaterialId is uint8 so it never collides with the sphere/cyl offsets
      // above it). For instanced meshes the primID indexes the replicated tris,
      // so reduce it modulo the base triangle count before reading the table.
      if (!c.mesh.triMaterialId.empty()) {
        const std::size_t nt = c.mesh.triMaterialId.size();
        hs.materialId =
            static_cast<uint32_t>(c.mesh.triMaterialId[rh.hit.primID % nt]);
      } else {
        hs.materialId = 0u;
      }
    }
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
    const Vec4& fc = isSphere ? c.built.sphereColor[rh.hit.primID]
                     : isCapped ? c.built.cylCapColor[rh.hit.primID]
                                : c.built.cylColor[rh.hit.primID];
    const Material& pm = isSphere ? c.built.sphereMat[rh.hit.primID]
                         : isCapped ? c.built.cylCapMat[rh.hit.primID]
                                    : c.built.cylMat[rh.hit.primID];
    Vec3 N = faceForward(
        normalize(Vec3{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z}), rd);
    const Vec3 C = {fc.x, fc.y, fc.z};
    // Outline / VdW primitives are flat silhouette geometry: never AO-darkened
    // and never shadowed (aoFactor 1, shadowsOn false). This is the gate.
    const Vec3 P{org.x + rd.x * rh.ray.tfar, org.y + rd.y * rh.ray.tfar,
                 org.z + rd.z * rh.ray.tfar};
    const Vec3 Ng{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
    const float secEps = selfIntersectEps(P, rd, rh.ray.tfar);
    hs.color = shadeLocal(pm, C, N, V, c.lights, c.ambLight, c.bg,
                          c.opt.specularScale, Vec3{1.0f, 1.0f, 1.0f}, 1.0f, P,
                          Ng, secEps, rscene, false, 1, px, py);
    hs.opacity = fc.w;
    hs.group = isSphere ? c.built.sphereGroup[rh.hit.primID]
               : isCapped ? c.built.cylCapGroup[rh.hit.primID]
                          : c.built.cylGroup[rh.hit.primID];
    // Edge G-buffer (only when the edge pass is on; the material-index
    // side-tables are empty otherwise, so these reads must be gated).
    if (c.opt.strokeEdges.enable) {
      hs.normal = N;  // analytic face-forwarded normal: the TRUE smooth normal
                      // of a sphere/cylinder surface (no facet interpolation)
      hs.objectId = (static_cast<uint32_t>(hs.group) << 2) |
                    static_cast<uint32_t>(rec.kind);  // Sphere=1,Cyl=2,CylCap=3
      // Global materialId: per-kind offset above the mesh material block plus the
      // precomputed raw per-primitive index.
      const BuiltScene& b = c.built;
      if (isSphere)
        hs.materialId = b.meshMatCount + b.sphereMatIndex[rh.hit.primID];
      else if (isCapped)
        hs.materialId = b.meshMatCount + b.sphereMatCount + b.cylMatCount +
                        b.cylCapMatIndex[rh.hit.primID];
      else
        hs.materialId =
            b.meshMatCount + b.sphereMatCount + b.cylMatIndex[rh.hit.primID];
    }
    // edge_line2 gradient: opacity varies p0 (fc.w) -> p1 (opacity1) along the
    // segment. For both linear-curve modes rh.hit.u is the axial curve fraction
    // in [0,1], so a linear lerp reproduces POV's "gradient z" transmit fade
    // (uniform when opacity1 < 0). Opaque outlines and uniform bonds
    // (opacity1 < 0) are unaffected.
    if (!isSphere) {
      const float op1 = isCapped ? c.built.cylCapOpacity1[rh.hit.primID]
                                 : c.built.cylOpacity1[rh.hit.primID];
      if (op1 >= 0.0f) {
        float u = rh.hit.u;
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;
        hs.opacity = fc.w * (1.0f - u) + op1 * u;
      }
    }
  }
  return hs;
}

}  // namespace detail
}  // namespace umbreon
