// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt2 glossy GGX indirect reflection (pt2.3-E): for every first-hit pixel
// whose material carries POV `reflection` AND `specular > 0`, sample a GGX
// reflection lobe (Heitz 2018 VNDF importance sampling) whose roughness alpha
// comes from the SAME Blinn-exponent map as the direct pass's highlight
// (pt2GgxAlphaFromRoughness, shading.hpp), so highlight and reflection
// sharpness agree. The per-pixel sample mean lands in a separate E_spec
// RADIANCE buffer (not the E_stored = E_true/pi irradiance convention -- no
// 1/pi, no giReflectance), which the caller optionally OIDN-denoises and
// composites as  color += reflection * E_spec  on exactly these pixels.
//
// Pixels whose lobe degenerates to a mirror (alpha == 0, see
// kPt2GlossyAlphaMin) stay with the exact single-ray mirror pass
// (pt2_reflect.hpp); the two passes partition the reflective pixels by
// reflAlpha, so a specular-free scene reproduces the stage-1 output bitwise.
//
// Each reflected ray's hit is evaluated exactly like the mirror pass:
// shadow-tested NEE direct + albedo * gather-sky (one-bounce ambient
// approximation); no glossy -> glossy chaining (single reflection segment,
// consistent with the gather's energy budget).
//
// Determinism: per-pixel Owen-scrambled Sobol 2D stream for the lobe
// (independent sequence per pixel, decorrelated from the diffuse gather by an
// ASCII-tag seed XOR) and a per-sample tea2 stream for the reflected-hit NEE
// jitter. Pure function of (pixel, sample, frameSeed); disjoint tile writes;
// bit-exact across thread counts by construction.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <embree4/rtcore.h>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "integrator/pt1/pt1_gather.hpp"
#include "render/progress_slice.hpp"
#include "shading/principled.hpp"
#include "shading/secondary_rays.hpp"
#include "scene.hpp"

namespace umbreon {
namespace detail {

// Smith Lambda for isotropic GGX: Lambda(w) with cosT = |cos(theta_w)| against
// the surface normal. Shared by the masking G1 = 1/(1+Lambda) and the
// height-correlated G2 below.
inline float pt2GgxLambda(float alpha, float cosT) {
  const float c2 = std::fmax(cosT * cosT, 1.0e-12f);
  const float t2 = (1.0f - c2) / c2;  // tan^2(theta)
  return 0.5f * (std::sqrt(1.0f + alpha * alpha * t2) - 1.0f);
}

// Anisotropic Smith Lambda for a unit direction w in the (t1, t2, N) tangent
// frame. Isotropic pixels keep calling pt2GgxLambda above (the two are
// mathematically equal at ax == ay but not bitwise -- different op order).
inline float pt2GgxLambdaAniso(float ax, float ay, const Vec3& w) {
  const float z2 = std::fmax(w.z * w.z, 1.0e-12f);
  const float t2 = (ax * ax * w.x * w.x + ay * ay * w.y * w.y) / z2;
  return 0.5f * (std::sqrt(1.0f + t2) - 1.0f);
}

// Heitz 2018 (JCGT 7(4), Appendix A): sample the GGX distribution of VISIBLE
// normals for view direction `wo` (local frame, wo.z > 0). Returns the unit
// microfacet normal wm. With this pdf the reflection estimator's weight
// reduces to G2/G1(wo) -- no D, no Jacobian, numerically robust. The
// anisotropic form (ax, ay); isotropic callers pass (alpha, alpha), which
// runs the exact float ops of the former isotropic version (bitwise-safe).
inline Vec3 pt2SampleGgxVndf(const Vec3& wo, float ax, float ay, float u1,
                             float u2) {
  // Transform to the hemisphere configuration (stretch by alpha).
  const Vec3 Vh = safeNormalize(Vec3{ax * wo.x, ay * wo.y, wo.z});
  // Orthonormal basis around Vh (T1 in the tangent plane of the config).
  const Vec3 T1 = (Vh.z < 0.999f)
                      ? safeNormalize(cross(Vec3{0.0f, 0.0f, 1.0f}, Vh))
                      : Vec3{1.0f, 0.0f, 0.0f};
  const Vec3 T2 = cross(Vh, T1);
  // Parameterization of the projected area (polar disk, warped toward Vh).
  const float r = std::sqrt(u1);
  const float phi = 2.0f * 3.14159265358979323846f * u2;
  const float t1 = r * std::cos(phi);
  float t2 = r * std::sin(phi);
  const float s = 0.5f * (1.0f + Vh.z);
  t2 = (1.0f - s) * std::sqrt(std::fmax(0.0f, 1.0f - t1 * t1)) + s * t2;
  // Reproject onto the hemisphere and unstretch.
  const float t3 =
      std::sqrt(std::fmax(0.0f, 1.0f - t1 * t1 - t2 * t2));
  const Vec3 Nh{t1 * T1.x + t2 * T2.x + t3 * Vh.x,
                t1 * T1.y + t2 * T2.y + t3 * Vh.y,
                t1 * T1.z + t2 * T2.z + t3 * Vh.z};
  return safeNormalize(
      Vec3{ax * Nh.x, ay * Nh.y, std::fmax(0.0f, Nh.z)});
}

// Glossy reflection gather at the RENDER grid: for pixels with
// reflAmt > 0 && reflAlpha > 0, average `spp` VNDF-sampled reflection rays
// into Espec (W*H*3, assigned here). The caller denoises/composites.
// position/normal/depth are the first-hit G-buffer; camPos/camDir + ortho
// reconstruct the incident view ray exactly as the primary loop generated it
// (same reconstruction as pt2ReflectPass).
inline void pt2GlossyPass(const IrradianceCacheParams& p, int W, int H,
                          const std::vector<float>& reflAmt,
                          const std::vector<float>& reflAlpha,
                          const std::vector<float>& reflF0,
                          const std::vector<float>& reflTan,
                          const std::vector<float>& reflAniso,
                          const float* position, const float* normal,
                          const float* depth, bool orthographic,
                          const Vec3& camPos, const Vec3& camDir, bool emissive,
                          int spp, uint32_t frameSeed,
                          std::vector<float>& Espec,
                          const ProgressSlice* prog) {
  const std::size_t npix = static_cast<std::size_t>(W) * H;
  Espec.assign(npix * 3, 0.0f);
  // Decorrelate the glossy Sobol stream from the diffuse gather's per-pixel
  // sequences with an ASCII-tag XOR ('GLSY'), mirroring the sobol-pattern
  // idiom of pt2MakePixelSampler (independent sequence per pixel).
  const uint32_t seedMix = hashU32(frameSeed ^ 0x474c5359u);  // 'GLSY'
  const float invSpp = 1.0f / static_cast<float>(spp);
  tbb::parallel_for(
      tbb::blocked_range2d<int>(0, H, 16, 0, W, 16),
      [&](const tbb::blocked_range2d<int>& r) {
        if (prog && prog->cancelled()) return;
        for (int py = r.rows().begin(); py != r.rows().end(); ++py) {
          for (int px = r.cols().begin(); px != r.cols().end(); ++px) {
            const std::size_t pix = static_cast<std::size_t>(py) * W + px;
            const float alpha = reflAlpha[pix];
            if (reflAmt[pix] <= 0.0f || alpha <= 0.0f) continue;
            const Vec3 P{position[pix * 3 + 0], position[pix * 3 + 1],
                         position[pix * 3 + 2]};
            const Vec3 N{normal[pix * 3 + 0], normal[pix * 3 + 1],
                         normal[pix * 3 + 2]};
            // Incident direction: ortho rays share camDir; perspective rays
            // point from the camera to the hit (same as the mirror pass).
            Vec3 wiV;
            if (orthographic) {
              wiV = camDir;
            } else {
              wiV = safeNormalize(
                  Vec3{P.x - camPos.x, P.y - camPos.y, P.z - camPos.z});
            }
            // Anisotropic frame (principled sphere/cylinder pixels): the
            // rotation-baked tangent from the hit shader replaces the
            // canonical frame and the lobe alphas stretch by the aspect.
            // Isotropic pixels keep every original expression verbatim.
            bool anisoPix = false;
            float ax = alpha, ay = alpha;
            Frame f;
            if (!reflTan.empty()) {
              const Vec3 T{reflTan[pix * 3 + 0], reflTan[pix * 3 + 1],
                           reflTan[pix * 3 + 2]};
              const float aspect = reflAniso[pix];
              if (aspect != 1.0f && dot(T, T) > 0.5f) {
                anisoPix = true;
                f = Frame{T, cross(N, T), N};
                ax = alpha / aspect;
                if (ax > 1.0f) ax = 1.0f;
                ay = alpha * aspect;
              }
            }
            if (!anisoPix) f = frameFromNormal(N);
            // Local frame with N = +z; view direction wo points AWAY from the
            // surface. Clamp wo.z to keep the VNDF configuration valid when
            // the G-buffer normal grazes the view ray.
            const Vec3 woW{-wiV.x, -wiV.y, -wiV.z};
            Vec3 wo{dot(woW, f.t), dot(woW, f.b), dot(woW, f.n)};
            if (wo.z < 1.0e-4f) wo.z = 1.0e-4f;
            wo = safeNormalize(wo);
            const float lamO = anisoPix ? pt2GgxLambdaAniso(ax, ay, wo)
                                        : pt2GgxLambda(alpha, wo.z);
            const float t0 = (depth && depth[pix] > 0.0f) ? depth[pix] : 1.0f;
            const uint32_t seedPix =
                hashU32(static_cast<uint32_t>(pix)) ^ seedMix;
            // Per-pixel Fresnel F0 (principled). POV pixels in a mixed scene
            // carry the neutral (1,1,1), for which Schlick is exactly 1.
            const bool haveF0 = !reflF0.empty();
            const Vec3 F0pix =
                haveF0 ? Vec3{reflF0[pix * 3 + 0], reflF0[pix * 3 + 1],
                              reflF0[pix * 3 + 2]}
                       : Vec3{1.0f, 1.0f, 1.0f};

            Vec3 sum{0.0f, 0.0f, 0.0f};
            for (int s = 0; s < spp; ++s) {
              float u1, u2;
              pt2SobolBurley2D(static_cast<uint32_t>(s), seedPix, &u1, &u2);
              const Vec3 wm = pt2SampleGgxVndf(wo, ax, ay, u1, u2);
              const float owm = dot(wo, wm);
              const Vec3 wl{2.0f * owm * wm.x - wo.x,
                            2.0f * owm * wm.y - wo.y,
                            2.0f * owm * wm.z - wo.z};
              // Below-horizon reflected direction: contributes zero but stays
              // one of the spp candidates (same convention as the gather's
              // horizon guard), keeping the estimator unbiased.
              if (wl.z <= 0.0f) continue;
              // VNDF estimator weight: f*cos/pdf = G2/G1(wo) with
              // height-correlated Smith G2 = 1/(1 + Lo + Li), G1 = 1/(1 + Lo).
              const float lamI = anisoPix ? pt2GgxLambdaAniso(ax, ay, wl)
                                          : pt2GgxLambda(alpha, wl.z);
              const float weight = (1.0f + lamO) / (1.0f + lamO + lamI);
              const Vec3 rdir = safeNormalize(
                  Vec3{f.t.x * wl.x + f.b.x * wl.y + f.n.x * wl.z,
                       f.t.y * wl.x + f.b.y * wl.y + f.n.y * wl.z,
                       f.t.z * wl.x + f.b.z * wl.y + f.n.z * wl.z});
              const float eps = selfIntersectEps(P, rdir, t0);
              const Vec3 O{P.x + N.x * eps, P.y + N.y * eps,
                           P.z + N.z * eps};
              const RTCRayHit rh =
                  intersectFull(p.scene, O, rdir, eps,
                                std::numeric_limits<float>::infinity());
              Vec3 L{0.0f, 0.0f, 0.0f};
              if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
                L = environmentRadiance(p, rdir);
              } else {
                // Per-sample deterministic stream for the area-light NEE
                // jitter at the reflected hit (the mirror pass uses
                // 'RFLT'/'NEE1'; this lobe gets its own tags).
                uint32_t rs0 = static_cast<uint32_t>(pix) ^ 0x474c5359u;
                uint32_t rs1 = 0x4e454532u + static_cast<uint32_t>(s);
                tea2(rs0, rs1);
                Pt1Vertex v;
                if (pt1EvalVertex(p, rh, O, rdir, v, nullptr, emissive, &rs0,
                                  &rs1)) {
                  // Direct (NEE, shadow-tested) + the surface's share of the
                  // sky (albedo * E_sky, one-bounce ambient approximation).
                  const Vec3 sky = environmentRadiance(p, rdir);
                  L = Vec3{v.radiance.x + v.albedo.x * sky.x,
                           v.radiance.y + v.albedo.y * sky.y,
                           v.radiance.z + v.albedo.z * sky.z};
                }
              }
              if (haveF0) {
                // Fold the per-sample Fresnel into the VNDF weight (the
                // half-vector cosine is dot(wo, wm)).
                const Vec3 F = schlickF(F0pix, owm);
                sum.x += weight * F.x * L.x;
                sum.y += weight * F.y * L.y;
                sum.z += weight * F.z * L.z;
              } else {
                sum.x += weight * L.x;
                sum.y += weight * L.y;
                sum.z += weight * L.z;
              }
            }
            Espec[pix * 3 + 0] = sum.x * invSpp;
            Espec[pix * 3 + 1] = sum.y * invSpp;
            Espec[pix * 3 + 2] = sum.z * invSpp;
          }
        }
        if (prog)
          prog->addWork(
              static_cast<std::uint64_t>(r.rows().end() - r.rows().begin()) *
                  static_cast<std::uint64_t>(r.cols().end() -
                                             r.cols().begin()),
              static_cast<std::uint64_t>(W) * static_cast<std::uint64_t>(H));
      });
}

}  // namespace detail
}  // namespace umbreon
