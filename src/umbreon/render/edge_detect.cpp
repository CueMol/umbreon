#include "render/edge_detect.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

namespace umbreon {
namespace {

constexpr std::uint32_t kInvalidId = 0xFFFFFFFFu;

// Co-group test for object/material boundary suppression. The suppression table
// holds one 32-bit mask per group id 1..32 (index 0 = the default/unsectioned
// group): bit i set in suppress[selfGroup] means a boundary between selfGroup
// and group i is suppressed (the two sections are treated as one solid object /
// material, so no internal seam is drawn). Group ids outside [0,32] cannot be
// represented in the 33-entry table, so they are never co-grouped (boundary
// drawn). Symmetry is not assumed: the table is consulted from the center
// pixel's group, so a one-sided --edge override still suppresses consistently
// because the neighbor pixel runs the same test with its own group as center.
inline bool coGrouped(const std::array<std::uint32_t, 33>& suppress,
                      std::uint32_t selfGroup, std::uint32_t sampleGroup) {
  if (selfGroup > 32u || sampleGroup > 31u) return false;
  return (suppress[selfGroup] & (1u << sampleGroup)) != 0u;
}

// Linear "over" composite of a solid edge color onto an RGB pixel:
//   out = mix(dst, src, a) = dst*(1-a) + src*a,  a = strength * opacity.
// Alpha (channel 3) is left untouched, matching fog / assumed-gamma.
inline void compositeOver(float* rgba, const float color[3], float a) {
  if (a <= 0.0f) return;
  a = std::min(1.0f, a);
  const float ia = 1.0f - a;
  rgba[0] = rgba[0] * ia + color[0] * a;
  rgba[1] = rgba[1] * ia + color[1] * a;
  rgba[2] = rgba[2] * ia + color[2] * a;
}

// Camera projection basis + half-extents, recomputed here exactly as the
// renderer builds them (embree_renderer.cpp), so the depth-gap classes can
// recover per-pixel world-units-per-pixel (pixelSize) and the per-pixel view
// direction (toward camera) at the frame's CURRENT (hi-res) resolution. The
// edge pass runs before the box downsample, so width/height are the supersample
// dims and the normalized (u,v) mapping is ss-independent -- it reproduces the
// same primary-ray direction the renderer used.
struct ViewBasis {
  Vec3 dir;       // normalized forward (view) axis
  Vec3 right;     // normalized image-plane right axis
  Vec3 up;        // normalized image-plane up axis
  bool ortho;     // orthographic projection?
  float persHalfW;  // perspective half-width  at unit image-plane distance
  float persHalfH;  // perspective half-height at unit image-plane distance
  // World units per pixel at unit |viewZ|, used to perspective-normalize the
  // depth-gap threshold (orthographic: constant across the frame).
  float pxPerUnitZ;
};

ViewBasis makeViewBasis(const Camera& cam, int w, int h) {
  ViewBasis vb;
  vb.dir = normalize(cam.direction);
  vb.right = normalize(cross(vb.dir, cam.up));
  vb.up = normalize(cross(vb.right, vb.dir));
  vb.ortho = cam.orthographic;
  const float aspect = static_cast<float>(w) / static_cast<float>(h);
  vb.persHalfH = std::tan(radians(cam.fovy) * 0.5f);
  vb.persHalfW = vb.persHalfH * aspect;
  // pixelSize(viewZ) = (2 * tan(fovy/2) * |viewZ|) / heightPx for perspective;
  // for orthographic it is (cam.height / heightPx), independent of viewZ. Store
  // the per-|viewZ| slope (perspective) or the constant (orthographic, with a
  // sentinel handled by pixelSize()).
  vb.pxPerUnitZ =
      vb.ortho ? (cam.height / static_cast<float>(h))
               : (2.0f * vb.persHalfH / static_cast<float>(h));
  return vb;
}

// World units spanned by one pixel at the given linear view-z. Orthographic:
// constant; perspective: grows linearly with depth. Used to make the depth-gap
// thresholds resolution/perspective robust (Mol* getPixelSize analogue).
inline float pixelSize(const ViewBasis& vb, float viewZ) {
  return vb.ortho ? vb.pxPerUnitZ : (vb.pxPerUnitZ * std::fabs(viewZ));
}

// SCALE-INVARIANT curvature ratio used as the disconnected-face / crease veto.
//
// Given the center view-z and its four axial neighbors (a BACKGROUND or off-image
// neighbor must already be neutralised to selfZ by the caller, so no synthetic
// "2*far" depth leaks in), compute per axis the 2nd difference (slope change) and
// the larger adjacent 1st difference (slope), then return the dimensionless ratio
//   curv2nd / max(curv1st, pixelSize).
// At a genuine depth step the slope jumps, so 2nd and 1st differences are both ~
// the gap and the ratio is ~1; on a smoothly curved surface the slope changes
// slowly, so the ratio is well below 1. The pixelSize floor keeps the ratio at 0
// on a flat patch (both differences ~0) and damps sub-pixel-depth noise. Because
// both numerator and denominator are view-z lengths, the ratio is independent of
// scene scale / camera units (unlike a raw 2nd-difference vs an absolute gate).
inline float curvatureRatio(float selfZ, float zL, float zR, float zU, float zD,
                            float pxSize) {
  const float ddx = std::fabs(zL + zR - 2.0f * selfZ);
  const float ddy = std::fabs(zU + zD - 2.0f * selfZ);
  const float d1x = std::max(std::fabs(zR - selfZ), std::fabs(selfZ - zL));
  const float d1y = std::max(std::fabs(zD - selfZ), std::fabs(selfZ - zU));
  const float curv2nd = std::max(ddx, ddy);
  const float curv1st = std::max(d1x, d1y);
  return curv2nd / std::max(curv1st, pxSize);
}

// Unit view direction TOWARD the camera at pixel (x,y) (i.e. -rayDir). For
// orthographic this is constant (-forward); for perspective it is the per-pixel
// primary-ray reversed, reconstructed from the same (u,v) mapping the renderer
// uses. The crease grazing-angle bias keys on dot(faceNormal, viewDir).
inline Vec3 viewDir(const ViewBasis& vb, int w, int h, int x, int y) {
  if (vb.ortho) return vb.dir * -1.0f;  // toward camera
  const float u =
      2.0f * (static_cast<float>(x) + 0.5f) / static_cast<float>(w) - 1.0f;
  const float v =
      1.0f - 2.0f * (static_cast<float>(y) + 0.5f) / static_cast<float>(h);
  const Vec3 rd =
      normalize(vb.dir + vb.right * (u * vb.persHalfW) + vb.up * (v * vb.persHalfH));
  return rd * -1.0f;  // toward camera
}

// Class 1 (Silhouette): the center pixel is a surface (objectId != INVALID) and
// at least one 4-neighbor is the background sentinel. Edge of the molecule
// against empty space. Returns the per-pixel mask strength in [0,1] (binary at
// hi-res; the later box downsample yields the antialiasing).
inline float silhouetteStrength(const std::vector<std::uint32_t>& objectId,
                                int w, int h, int x, int y) {
  const std::size_t c = static_cast<std::size_t>(y) * w + x;
  if (objectId[c] == kInvalidId) return 0.0f;  // background pixel: no line
  // 4-neighborhood (clamped at the image border; off-image is treated as
  // surface-continuation so the frame edge does not draw a spurious outline).
  const bool bgL = (x > 0) && objectId[c - 1] == kInvalidId;
  const bool bgR = (x + 1 < w) && objectId[c + 1] == kInvalidId;
  const bool bgU = (y > 0) && objectId[c - w] == kInvalidId;
  const bool bgD = (y + 1 < h) && objectId[c + w] == kInvalidId;
  return (bgL || bgR || bgU || bgD) ? 1.0f : 0.0f;
}

// Class 5 (Crease / ridge-valley): a fold in the world-normal field within a
// single object. For each same-object 4-neighbor, the dot of the two
// face-forwarded world normals measures the local fold; the sharpest (minimum)
// dot across the kept neighbors drives the strength.
//
// MESH-ONLY. The crease class runs only on MESH pixels (kindBits == 0): a smooth
// CueMol SES/ribbon surface is the case it targets. Analytic primitives
// (sphere/cylinder) have exact smooth normals that nonetheless diverge sharply at
// a bond junction (two same-section capped cylinders meeting at an atom), where
// the object gate cannot separate them; their silhouettes are still drawn by the
// silhouette/object classes, so crease is restricted away from them.
//
// Four guards keep smooth curvature (spheres / SES) from creasing:
//   (1) Object gate. Only neighbors with the SAME objectId are considered.
//       Background and different-object neighbors are silhouettes / object
//       boundaries (classes 1 and 3); excluding them stops the crease class
//       from double-firing on those edges, where the normal also flips.
//   (2) Depth-gap veto. A same-object neighbor whose linear view-z differs by
//       more than (pixelSize * distanceThreshold) is a depth discontinuity
//       (the disconnected-face class 2), not a smooth fold; skip it so a
//       self-occluding silhouette does not register as a crease.
//   (3) Curvature veto. A smoothly but tightly curved mesh region has a
//       continuous normal field whose dot can still dip below the crease cosine.
//       The scale-invariant curvature ratio (2nd/1st view-z difference) is small
//       there (slope changes slowly) and ~1 at a genuine geometric crease, so a
//       sub-gate ratio cancels the line -- the doc-specified veto that keeps a
//       tightly-but-smoothly curved surface from creasing.
//   (4) Grazing-angle bias. The face-forwarded normal is always toward the
//       viewer, so dot(n, V) is ~1 head-on and ->0 at grazing. Interpolation /
//       discretization noise on the normal field is worst at grazing, so the
//       effective crease angle is SCALED UP there (require a sharper fold),
//       cutting false positives along the rim of a smooth surface.
//
// strength = clamp((cosT - dot) / (1 + cosT), 0, 1): 0 just past threshold,
// 1 at a full 180-degree fold (dot = -1); bounded and monotonic.
inline float creaseStrength(const std::vector<std::uint32_t>& objectId,
                            const std::vector<float>& normal,
                            const std::vector<float>& viewZ,
                            const ViewBasis& vb, float cosCrease,
                            float biasGain, float distanceThreshold,
                            float curvatureGate, int w, int h, int x, int y) {
  const std::size_t c = static_cast<std::size_t>(y) * w + x;
  const std::uint32_t selfId = objectId[c];
  if (selfId == kInvalidId) return 0.0f;  // background pixel: no crease
  if ((selfId & 3u) != 0u) return 0.0f;   // mesh-only: skip analytic primitives

  const Vec3 nC{normal[c * 3 + 0], normal[c * 3 + 1], normal[c * 3 + 2]};
  const float selfZ = viewZ[c];
  const float pxSize = pixelSize(vb, selfZ);

  // Grazing-angle bias: grazing in [0,1] (1 = head-on). Scale the crease angle
  // up as the surface turns edge-on, then convert back to a cosine threshold.
  const Vec3 V = viewDir(vb, w, h, x, y);
  const float grazing = std::max(0.0f, dot(nC, V));
  const float baseAngle = std::acos(std::min(1.0f, std::max(-1.0f, cosCrease)));
  float effAngle = baseAngle * (1.0f + biasGain * (1.0f - grazing));
  if (effAngle > 3.14159265f) effAngle = 3.14159265f;  // clamp to 180 deg
  const float cosT = std::cos(effAngle);

  // Depth-gap VETO threshold at this pixel (perspective/resolution robust).
  // NOTE this is intentionally LOOSER than the disconnected-face gap test: a
  // box-edge fold of a thin ribbon strand (a ~90-degree fold where the flat top
  // meets the thin side face) is seen nearly edge-on, so adjacent pixels straddle
  // a MODEST depth step that is still a genuine crease, not a strand-over-strand
  // cliff. Vetoing crease at the disc gap (distanceThreshold) erases exactly those
  // box edges. The veto here only rejects a LARGE step (kCreaseGapVetoScale x the
  // disc threshold) -- the true self-occlusion cliffs that the disconnected-face
  // class owns -- so the two classes still partition the work without crease
  // losing the thin-ribbon fold network.
  constexpr float kCreaseGapVetoScale = 8.0f;
  const float gapThresh = pxSize * distanceThreshold * kCreaseGapVetoScale;

  float minDot = 1.0f;  // 1 = perfectly flat (no fold)
  const int offs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (const auto& o : offs) {
    const int nx = x + o[0], ny = y + o[1];
    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
    const std::size_t s = static_cast<std::size_t>(ny) * w + nx;
    if (objectId[s] != selfId) continue;                 // guard (1)
    if (std::fabs(viewZ[s] - selfZ) > gapThresh) continue;  // guard (2)
    const Vec3 nN{normal[s * 3 + 0], normal[s * 3 + 1], normal[s * 3 + 2]};
    minDot = std::min(minDot, dot(nC, nN));
  }
  if (minDot >= cosT) return 0.0f;  // fold shallower than threshold

  // Guard (3): OPTIONAL curvature veto. A crease is a NORMAL-space event at
  // (near-)constant depth -- a box-edge fold has a small depth 2nd-difference, so
  // a depth-curvature gate over-suppresses it. The depth-gap veto (guard 2) above
  // already rejects genuine depth steps (those are the disconnected-face class),
  // so the curvature veto is DISABLED by default (curvatureGate <= 0). It is kept
  // as an opt-in knob (a positive gate) for scenes where a smoothly-but-tightly
  // curved mesh barrel creases spuriously; there the normal field is continuous
  // and the depth curvature ratio stays low, so a mild gate can cancel it.
  if (curvatureGate > 0.0f) {
    const bool hasL = x > 0, hasR = x + 1 < w, hasU = y > 0, hasD = y + 1 < h;
    auto sameZ = [&](bool has, int nx, int ny) -> float {
      if (!has) return selfZ;
      const std::size_t s = static_cast<std::size_t>(ny) * w + nx;
      return objectId[s] == selfId ? viewZ[s] : selfZ;
    };
    const float zL = sameZ(hasL, x - 1, y), zR = sameZ(hasR, x + 1, y);
    const float zU = sameZ(hasU, x, y - 1), zD = sameZ(hasD, x, y + 1);
    if (curvatureRatio(selfZ, zL, zR, zU, zD, pxSize) < curvatureGate)
      return 0.0f;  // smoothly curved, not a geometric crease
  }

  const float denom = 1.0f + cosT;
  const float strength = (denom > 1.0e-6f) ? (cosT - minDot) / denom : 1.0f;
  return std::min(1.0f, std::max(0.0f, strength));
}

// Linear view-z of a neighbor sample, with the background sentinel mapped to a
// depth far behind every surface (Mol* uses 2*far). umbreon has no far-clip
// plane, so a background neighbor is reported as selfZ + 2*|farViewZ|, where
// farViewZ is a per-frame "deep" reference (max surface view-z): the gap then
// always exceeds any pixelSize*distanceThreshold, matching the "2*far" intent
// without depending on a (nonexistent) far plane. Scene-scale independent.
inline float sampleViewZ(const std::vector<std::uint32_t>& objectId,
                         const std::vector<float>& viewZ, float selfZ,
                         float farViewZ, std::size_t s) {
  if (objectId[s] == kInvalidId) return selfZ + 2.0f * std::fabs(farViewZ);
  return viewZ[s];
}

// Classes 2 (Disconnected-face) + 3 (Object boundary): screen-space depth-gap
// edges. Both fire where a 4-neighbor's linear view-z differs from the center
// by more than (pixelSize * distanceThreshold) -- a depth discontinuity that is
// perspective/resolution robust because pixelSize scales the threshold with the
// per-pixel world-units-per-pixel.
//
//   Class 2 (Disconnected): neighbor has the SAME objectId (a fold/step WITHIN
//     one object, the signature Warabi line). After the gap test it must clear
//     TWO additional gates before drawing:
//       (a) CURVATURE VETO (scale-invariant). A smooth surface (sphere / SES) has
//           a slowly-changing depth slope, so curvatureRatio (2nd/1st view-z
//           difference) is small; a genuine step jumps the slope, so the ratio is
//           ~1. If the ratio is below curvatureGate the "gap" is smooth curvature,
//           not a discontinuity, and the candidate is cancelled.
//       (b) NORMAL-CONSISTENCY GATE. A smooth SES self-occlusion fold can have a
//           large depth 2nd-derivative IDENTICAL to a real disconnected face, so
//           the curvature veto alone over-inks dense molecular SES. The line
//           therefore fires only when the WORLD NORMALS across the gap also
//           DISAGREE: dot(n_center, n_sample) < cos(discNormalAngle). A smooth
//           fold (normals continuous) is NOT inked; a true face-to-face step
//           (normals discontinuous) still inks.
//
//   Class 3 (Object boundary): neighbor has a DIFFERENT objectId (including the
//     background, via sampleViewZ()). No curvature/normal veto -- different
//     objects ARE a discontinuity. (Background neighbors are also the Silhouette
//     class 1, which has higher precedence and composites over this.) A
//     different-object neighbor whose group is CO-GROUPED with the center
//     (objectSuppress) is skipped, so two sections marked as one object draw no
//     internal seam; the background (group >> bounds) is never co-grouped and
//     still draws.
//
// Both write their per-pixel strength into the matching out-parameter
// (0 = no edge). Computing them together avoids re-reading the neighborhood and
// re-deriving pixelSize twice.
inline void depthGapStrength(const std::vector<std::uint32_t>& objectId,
                             const std::vector<float>& viewZ,
                             const std::vector<float>& normal,
                             const ViewBasis& vb, float distanceThreshold,
                             float curvatureGate, float cosDiscNormal,
                             float farViewZ,
                             const std::array<std::uint32_t, 33>& objectSuppress,
                             int w, int h, int x, int y, bool wantDisc,
                             bool wantObj, float& discOut, float& objOut) {
  discOut = 0.0f;
  objOut = 0.0f;
  const std::size_t c = static_cast<std::size_t>(y) * w + x;
  const std::uint32_t selfId = objectId[c];
  if (selfId == kInvalidId) return;  // background pixel: no depth-gap line here

  const float selfZ = viewZ[c];
  const float pxSize = pixelSize(vb, selfZ);
  const float gapThresh = pxSize * distanceThreshold;

  // 4-neighborhood samples (clamped at the border; off-image neighbors are
  // skipped so the frame edge does not draw a spurious line). For curvature the
  // missing opposite sample is treated as a mirror of the center (zero 2nd
  // difference contribution), which is the no-edge default.
  const bool hasL = x > 0, hasR = x + 1 < w, hasU = y > 0, hasD = y + 1 < h;
  const std::size_t iL = c - 1, iR = c + 1, iU = c - w, iD = c + w;

  // Center world normal, used by the disconnected-face normal-consistency gate.
  const Vec3 nC{normal.empty() ? 0.0f : normal[c * 3 + 0],
                normal.empty() ? 0.0f : normal[c * 3 + 1],
                normal.empty() ? 0.0f : normal[c * 3 + 2]};

  bool discGap = false, objGap = false;
  const struct {
    bool ok;
    std::size_t s;
  } nbrs[4] = {{hasL, iL}, {hasR, iR}, {hasU, iU}, {hasD, iD}};
  for (const auto& n : nbrs) {
    if (!n.ok) continue;
    const float nz = sampleViewZ(objectId, viewZ, selfZ, farViewZ, n.s);
    if (std::fabs(nz - selfZ) <= gapThresh) continue;  // no gap to this neighbor
    if (objectId[n.s] == selfId) {
      // Same object => disconnected-face candidate, but ONLY when the world
      // normals across the gap also DISAGREE (a true face-to-face step). A
      // smooth SES self-occlusion fold keeps near-continuous normals across the
      // gap and is rejected here, which is what stops the SES over-inking.
      if (wantDisc) {
        const Vec3 nN{normal[n.s * 3 + 0], normal[n.s * 3 + 1],
                      normal[n.s * 3 + 2]};
        if (dot(nC, nN) < cosDiscNormal) discGap = true;
      }
    } else if (wantObj) {
      // Different object => object boundary, unless the two sections are
      // co-grouped (suppressed internal seam). Background neighbors have an
      // out-of-range group and are never co-grouped, so silhouette-adjacent
      // gaps still register here.
      const std::uint32_t selfGroup = selfId >> 2;
      const std::uint32_t sampleGroup = objectId[n.s] >> 2;
      if (!coGrouped(objectSuppress, selfGroup, sampleGroup)) objGap = true;
    }
  }

  if (objGap) objOut = 1.0f;

  if (discGap) {
    // Scale-invariant curvature veto: cancel the disconnected-face line where the
    // depth field is merely smoothly curved (the slope changes slowly). The
    // differences are taken on the ACTUAL surface view-z; a BACKGROUND neighbor
    // (or off-image) has no depth, so it is neutralised to selfZ -- the synthetic
    // "2*far" depth must NOT leak in here, or every disconnected face one pixel
    // from a silhouette would get a huge ratio and defeat the veto.
    auto curvZ = [&](bool has, std::size_t s) {
      return (has && objectId[s] != kInvalidId) ? viewZ[s] : selfZ;
    };
    const float zL = curvZ(hasL, iL), zR = curvZ(hasR, iR);
    const float zU = curvZ(hasU, iU), zD = curvZ(hasD, iD);
    if (curvatureRatio(selfZ, zL, zR, zU, zD, pxSize) >= curvatureGate)
      discOut = 1.0f;  // genuine step survives
  }
}

// Class 4 (Material boundary): a change in the global materialId across a
// 4-neighbor, within the visible surface. Unlike the object boundary this needs
// no depth gap -- two differently-materialed regions on ONE continuous surface
// (a co-planar seam) is exactly the case to draw, so a gap gate would erase the
// most-wanted lines. The center must be a surface (objectId != INVALID); a
// background neighbor (materialId == INVALID) is the Silhouette / Object class,
// not a material change, so it is skipped here. A neighbor whose group is
// co-grouped with the center via materialSuppress is suppressed (treat the two
// sections as one material, no internal seam). Binary at hi-res; the box
// downsample provides the antialiasing.
inline float materialBoundaryStrength(
    const std::vector<std::uint32_t>& objectId,
    const std::vector<std::uint32_t>& materialId,
    const std::array<std::uint32_t, 33>& materialSuppress, int w, int h, int x,
    int y) {
  const std::size_t c = static_cast<std::size_t>(y) * w + x;
  const std::uint32_t selfObj = objectId[c];
  if (selfObj == kInvalidId) return 0.0f;  // background pixel: no material line
  const std::uint32_t selfMat = materialId[c];
  const std::uint32_t selfGroup = selfObj >> 2;

  const int offs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (const auto& o : offs) {
    const int nx = x + o[0], ny = y + o[1];
    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
    const std::size_t s = static_cast<std::size_t>(ny) * w + nx;
    if (objectId[s] == kInvalidId) continue;  // background: not a material change
    if (materialId[s] == selfMat) continue;   // same material: no seam
    const std::uint32_t sampleGroup = objectId[s] >> 2;
    if (coGrouped(materialSuppress, selfGroup, sampleGroup)) continue;
    return 1.0f;  // first qualifying material change draws the line
  }
  return 0.0f;
}

// Per-section style lookup for one pixel's group (= objectId >> 2). Uses the
// per-group table when present and in range, else the global defaultStyle. The
// edge "belongs to" the surface it is drawn on, so the CENTER pixel's section
// supplies the color/opacity/width (a one-sided --edge override still styles its
// own side consistently). Background pixels never reach here (no edge drawn).
inline const EdgeStyle& styleForGroup(const std::vector<EdgeStyle>& table,
                                      const EdgeStyle& fallback,
                                      std::uint32_t group) {
  if (group < table.size()) return table[group];
  return fallback;
}

// One accumulated edge mask for a single class, at hi-res. Per pixel it stores
// the detected strength plus the (per-section) style that produced it, so the
// dilation pass can paint each pixel's disk in ITS OWN section color/opacity and
// radius. width < 1 leaves only the source pixel (a 1px hairline).
struct ClassMask {
  std::vector<float> strength;   // w*h, 0 = no edge at this pixel
  std::vector<float> color;      // w*h*3, linear RGB of the producing section
  std::vector<float> opacity;    // w*h
  std::vector<float> width;      // w*h, dilation radius in hi-res px
  float maxWidth = 0.0f;         // max radius over the frame (gather extent)
  void alloc(std::size_t n) {
    strength.assign(n, 0.0f);
    color.assign(n * 3, 0.0f);
    opacity.assign(n, 0.0f);
    width.assign(n, 0.0f);
  }
};

// Record one detected edge at pixel index i with the given style.
inline void writeMask(ClassMask& m, std::size_t i, float strength,
                      const EdgeClassStyle& cs) {
  m.strength[i] = strength;
  m.color[i * 3 + 0] = cs.color[0];
  m.color[i * 3 + 1] = cs.color[1];
  m.color[i * 3 + 2] = cs.color[2];
  m.opacity[i] = cs.opacity;
  m.width[i] = cs.width;
}

// Dilate one class mask by its per-pixel disk radius and over-composite the
// result onto frame.color. For each output pixel P we GATHER the source pixels
// within the class's max radius and keep the one whose disk covers P with the
// largest coverage (strength); P then draws in that source's color/opacity. A
// gather (not a scatter) keeps the pass data-parallel over output rows and order
// independent. The disk is the inclusive set x*x+y*y <= width*width (Mol*
// getOutline), and the source pixel itself always passes, so width < 1 leaves a
// 1px source-only line and width >= 1 thickens it (the box downsample then
// antialiases the hi-res result).
void compositeClassMask(const ClassMask& m, std::vector<float>& outColor, int w,
                        int h) {
  const int rad = static_cast<int>(std::ceil(m.maxWidth));
  tbb::parallel_for(
      tbb::blocked_range<int>(0, h),
      [&](const tbb::blocked_range<int>& rows) {
        for (int py = rows.begin(); py != rows.end(); ++py) {
          for (int px = 0; px < w; ++px) {
            float bestS = 0.0f;
            const float* bestColor = nullptr;
            float bestOpacity = 0.0f;
            const int x0 = std::max(0, px - rad), x1 = std::min(w - 1, px + rad);
            const int y0 = std::max(0, py - rad), y1 = std::min(h - 1, py + rad);
            for (int sy = y0; sy <= y1; ++sy) {
              for (int sx = x0; sx <= x1; ++sx) {
                const std::size_t s =
                    static_cast<std::size_t>(sy) * w + sx;
                const float ss = m.strength[s];
                if (ss <= 0.0f) continue;
                const float wr = m.width[s];
                const int dx = px - sx, dy = py - sy;
                const float d2 = static_cast<float>(dx * dx + dy * dy);
                // width<=0 paints only the source pixel; otherwise the disk
                // x*x+y*y <= width*width (inclusive). The center always passes.
                const float r2 = wr * wr;
                if (d2 > r2 && (dx != 0 || dy != 0)) continue;
                if (ss > bestS) {
                  bestS = ss;
                  bestColor = &m.color[s * 3];
                  bestOpacity = m.opacity[s];
                }
              }
            }
            if (bestColor)
              compositeOver(&outColor[(static_cast<std::size_t>(py) * w + px) * 4],
                            bestColor, bestS * bestOpacity);
          }
        }
      });
}

}  // namespace

void applyEdges(FrameResult& frame, const Scene& scene,
                const RenderOptions& opt) {
  // Requires the Stage A G-buffer; a no-op without it (e.g. edges disabled).
  if (frame.objectId.empty()) return;

  const int w = frame.width, h = frame.height;
  if (w <= 0 || h <= 0) return;

  // Per-class "active anywhere" = enabled in the global defaultStyle OR in any
  // per-section override, AND the AOVs the class needs are present. A class can
  // be enabled in one section's style and disabled in another, so the decision
  // to run a class's detection at all considers every style. Per pixel, the
  // CENTER pixel's section style then decides whether the class actually draws
  // there and in what color/opacity/width.
  const std::vector<EdgeStyle>& table = scene.groupEdgeStyle;
  const EdgeStyle& fallback = opt.edges.defaultStyle;
  bool classAny[static_cast<int>(EdgeClass::Count)] = {false, false, false,
                                                       false, false};
  float classMaxWidth[static_cast<int>(EdgeClass::Count)] = {0, 0, 0, 0, 0};
  for (int c = 0; c < static_cast<int>(EdgeClass::Count); ++c) {
    if (fallback.cls[c].enabled) {
      classAny[c] = true;
      classMaxWidth[c] = std::max(classMaxWidth[c], fallback.cls[c].width);
    }
    for (const EdgeStyle& s : table) {
      if (s.cls[c].enabled) {
        classAny[c] = true;
        classMaxWidth[c] = std::max(classMaxWidth[c], s.cls[c].width);
      }
    }
  }

  const int kSil = static_cast<int>(EdgeClass::Silhouette);
  const int kDisc = static_cast<int>(EdgeClass::Disconnected);
  const int kObj = static_cast<int>(EdgeClass::Object);
  const int kMat = static_cast<int>(EdgeClass::Material);
  const int kCrease = static_cast<int>(EdgeClass::Crease);

  // Gate each class on the AOVs it needs (absent => cannot run even if enabled).
  const bool doSil = classAny[kSil];
  // Disconnected needs the normal AOV too (its normal-consistency gate).
  const bool doDisc =
      classAny[kDisc] && !frame.viewZ.empty() && !frame.normal.empty();
  const bool doObj = classAny[kObj] && !frame.viewZ.empty();
  const bool doMat = classAny[kMat] && !frame.materialId.empty();
  const bool doCrease =
      classAny[kCrease] && !frame.normal.empty() && !frame.viewZ.empty();
  if (!doSil && !doDisc && !doObj && !doMat && !doCrease) return;

  // Camera/projection basis: recovers per-pixel pixelSize (depth-gap threshold
  // scaling) and the grazing-angle view direction (crease).
  const ViewBasis vb = makeViewBasis(scene.camera, w, h);
  const float cosCrease = std::cos(radians(opt.edges.creaseAngleDeg));
  // Disconnected-face normal-consistency threshold: the class-2 line fires only
  // when the world normals across the depth gap diverge past this angle.
  const float cosDiscNormal = std::cos(radians(opt.edges.discNormalAngleDeg));

  // "Far" reference for background neighbors of the depth-gap classes. umbreon
  // has no far-clip plane, so use the deepest surface view-z in the frame: a
  // background neighbor is then reported 2*|farViewZ| behind the center, always
  // exceeding the gap threshold (the Mol* "2*far" intent, scene-scale free).
  // Only scanned when a depth-gap class is active.
  float farViewZ = 0.0f;
  if (doDisc || doObj) {
    const std::size_t n = static_cast<std::size_t>(w) * h;
    farViewZ = tbb::parallel_reduce(
        tbb::blocked_range<std::size_t>(0, n), 0.0f,
        [&](const tbb::blocked_range<std::size_t>& r, float acc) {
          for (std::size_t i = r.begin(); i != r.end(); ++i)
            if (frame.objectId[i] != kInvalidId)
              acc = std::max(acc, std::fabs(frame.viewZ[i]));
          return acc;
        },
        [](float a, float b) { return std::max(a, b); });
  }

  // Stage B: accumulate one per-class mask, then Stage C dilates + composites.
  // Width (dilation radius) is per section, so masks must run before the
  // composite; collecting all five first also lets each class draw in its own
  // section's color/opacity at dilation time.
  const std::size_t np = static_cast<std::size_t>(w) * h;
  ClassMask mSil, mDisc, mObj, mMat, mCrease;
  if (doSil) { mSil.alloc(np); mSil.maxWidth = classMaxWidth[kSil]; }
  if (doDisc) { mDisc.alloc(np); mDisc.maxWidth = classMaxWidth[kDisc]; }
  if (doObj) { mObj.alloc(np); mObj.maxWidth = classMaxWidth[kObj]; }
  if (doMat) { mMat.alloc(np); mMat.maxWidth = classMaxWidth[kMat]; }
  if (doCrease) { mCrease.alloc(np); mCrease.maxWidth = classMaxWidth[kCrease]; }

  // Detection pass (row-parallel over the immutable AOVs). For each pixel and
  // each active class, compute the geometric strength, then consult the CENTER
  // pixel's section style: the class draws here only if that section enables it,
  // in that section's color/opacity/width.
  tbb::parallel_for(
      tbb::blocked_range<int>(0, h),
      [&](const tbb::blocked_range<int>& rows) {
        for (int y = rows.begin(); y != rows.end(); ++y) {
          for (int x = 0; x < w; ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * w + x;
            const std::uint32_t oid = frame.objectId[i];
            if (oid == kInvalidId) continue;  // background: no edge originates
            const EdgeStyle& st = styleForGroup(table, fallback, oid >> 2);

            if (doSil && st.cls[kSil].enabled) {
              const float s = silhouetteStrength(frame.objectId, w, h, x, y);
              if (s > 0.0f) writeMask(mSil, i, s, st.cls[kSil]);
            }
            if (doCrease && st.cls[kCrease].enabled) {
              const float s = creaseStrength(
                  frame.objectId, frame.normal, frame.viewZ, vb, cosCrease,
                  opt.edges.creaseGrazingBias, opt.edges.distanceThreshold,
                  opt.edges.creaseCurvatureGate, w, h, x, y);
              if (s > 0.0f) writeMask(mCrease, i, s, st.cls[kCrease]);
            }
            // Disconnected / Object share one neighborhood scan; each writes its
            // own mask only when its class is enabled for this section.
            const bool wantDisc = doDisc && st.cls[kDisc].enabled;
            const bool wantObj = doObj && st.cls[kObj].enabled;
            if (wantDisc || wantObj) {
              float sDisc = 0.0f, sObj = 0.0f;
              depthGapStrength(frame.objectId, frame.viewZ, frame.normal, vb,
                               opt.edges.distanceThreshold,
                               opt.edges.curvatureGate, cosDiscNormal, farViewZ,
                               opt.edges.objectSuppress, w, h, x, y, wantDisc,
                               wantObj, sDisc, sObj);
              if (wantDisc && sDisc > 0.0f)
                writeMask(mDisc, i, sDisc, st.cls[kDisc]);
              if (wantObj && sObj > 0.0f)
                writeMask(mObj, i, sObj, st.cls[kObj]);
            }
            if (doMat && st.cls[kMat].enabled) {
              const float s = materialBoundaryStrength(
                  frame.objectId, frame.materialId, opt.edges.materialSuppress,
                  w, h, x, y);
              if (s > 0.0f) writeMask(mMat, i, s, st.cls[kMat]);
            }
          }
        }
      });

  // Stage C: dilate each class mask by its per-pixel disk and over-composite in
  // precedence order, LOWEST first so higher-precedence classes draw OVER them:
  // Crease < Disconnected < Material < Object < Silhouette.
  if (doCrease) compositeClassMask(mCrease, frame.color, w, h);
  if (doDisc) compositeClassMask(mDisc, frame.color, w, h);
  if (doMat) compositeClassMask(mMat, frame.color, w, h);
  if (doObj) compositeClassMask(mObj, frame.color, w, h);
  if (doSil) compositeClassMask(mSil, frame.color, w, h);
}

}  // namespace umbreon
