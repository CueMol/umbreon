#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <cstdlib>
#include <cstdint>
#include <string>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "edges/stroke_render.hpp"

namespace umbreon {
namespace {

constexpr std::uint32_t kBackground = 0xFFFFFFFFu;

// World units spanned by one pixel at linear view-z `vz` (identity in vz under
// ortho). Uses the vertical extent; the renderer's pixels are square.
inline float pixelSizeAt(const ScreenProj& sp, float vz) {
  return sp.ortho ? (2.0f * sp.halfH / static_cast<float>(sp.H))
                  : (2.0f * sp.persHalfH * vz / static_cast<float>(sp.H));
}

// One-sided slope of the viewZ field at pixel `a` looking away from the crack
// (toward `outer`), clamped to +-clampS. Background / off-image outer neighbors
// contribute zero slope (flat extrapolation).
inline float sideSlope(const float* viewZ, const std::uint32_t* objectId,
                       int idxA, int idxOuter, bool outerValid, float clampS) {
  if (!outerValid || objectId[idxOuter] == kBackground) return 0.0f;
  const float s = viewZ[idxA] - viewZ[idxOuter];
  return std::max(-clampS, std::min(clampS, s));
}

// |n . viewdir| / |n| at pixel `idx`, i.e. how front-facing the surface is
// (1 = facing the camera, 0 = edge-on / at its silhouette). Zero for a
// degenerate normal.
inline float facingCos(const float* normal, const ScreenProj& sp, int idx) {
  const float* n = normal + 3 * static_cast<std::size_t>(idx);
  const float len2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
  if (len2 <= 1.0e-12f) return 0.0f;
  const float d = n[0] * sp.dir.x + n[1] * sp.dir.y + n[2] * sp.dir.z;
  return std::fabs(d) / std::sqrt(len2);
}

// One-sided slope for the contact veto on ID-keyed boundaries. Differs from
// sideSlope in two ways. (1) An outer neighbor with a DIFFERENT objectId is
// invalid (flat extrapolation): near an intersection contour / occlusion
// boundary the outer straight-line neighbor can belong to the other primitive
// or object, and its depth must not leak into this surface's extrapolation.
// (2) The slope is credited only while the side FACES the viewer (facingCos
// >= grazeCos). A grazing rim (a sphere/cylinder curling away toward its own
// silhouette in front of a farther surface) has a steep slope whose tangent
// extrapolation can coincidentally land on the far surface and fake a
// contact; but its shading normal is edge-on, so the slope degrades to flat
// there and the |vzA - vzB| occlusion step stays inked. A genuinely
// continuous surface (tilted or intersecting) still faces the viewer at the
// crack, so its slope keeps extrapolating.
inline float contactSideSlope(const float* viewZ,
                              const std::uint32_t* objectId,
                              const float* normal, const ScreenProj& sp,
                              int idxA, int idxOuter, bool outerValid,
                              float clampS, float grazeCos) {
  if (!outerValid || objectId[idxOuter] != objectId[idxA]) return 0.0f;
  if (normal && facingCos(normal, sp, idxA) < grazeCos)
    return 0.0f;  // grazing rim
  const float s = viewZ[idxA] - viewZ[idxOuter];
  return std::max(-clampS, std::min(clampS, s));
}

// True when any pixel within Chebyshev distance `r` of (x,y) is background.
// Used to suppress DepthGap cracks hugging the outline: a grazing surface
// (tube rim, near-edge-on facet) piles huge depth slopes into the last few
// pixels before the silhouette, indistinguishable there from an occlusion
// step -- and the silhouette class already inks that boundary.
inline bool nearBackground(const std::uint32_t* objectId, int W, int H, int x,
                           int y, int r) {
  const int x0 = std::max(0, x - r), x1 = std::min(W - 1, x + r);
  const int y0 = std::max(0, y - r), y1 = std::min(H - 1, y + r);
  for (int yy = y0; yy <= y1; ++yy)
    for (int xx = x0; xx <= x1; ++xx)
      if (objectId[yy * W + xx] == kBackground) return true;
  return false;
}

// True when background lies within `r` steps of the crack ALONG the crack's
// own direction (a right crack is a vertical lattice segment, a down crack a
// horizontal one): the strip spans both pair pixels across the crack and
// +-r along it. A weak DepthGap crack with background in this strip is the
// terminal piece of a contour running INTO the outline and survives the
// clearance kill; rim noise runs parallel to the outline and does not reach
// it along its own direction.
inline bool bgAlongCrack(const std::uint32_t* objectId, int W, int H, int x,
                         int y, bool rightCrack, int r) {
  const int x0 = rightCrack ? x : std::max(0, x - r);
  const int x1 = rightCrack ? std::min(W - 1, x + 1) : std::min(W - 1, x + r);
  const int y0 = rightCrack ? std::max(0, y - r) : y;
  const int y1 = rightCrack ? std::min(H - 1, y + r) : std::min(H - 1, y + 1);
  for (int yy = y0; yy <= y1; ++yy)
    for (int xx = x0; xx <= x1; ++xx)
      if (objectId[yy * W + xx] == kBackground) return true;
  return false;
}

// Wide-baseline recession slope of a crack's NEAR side (world units per
// pixel): walk up to 6 pixels away from the crack along the pair axis,
// staying on the near pixel's objectId, and return the steepest secant
// |vz[k] - vz[0]| / k. This is how fast the near surface itself recedes at
// the crack; the step-dominance gate compares the raw step against it.
// Returns a negative value when not even one same-id sample exists (caller
// treats the crack as not judgeable, i.e. not strong).
inline float nearSideRecession(const float* viewZ,
                               const std::uint32_t* objectId, int W, int H,
                               int ia, int ib) {
  const int d = ib - ia;  // +1 (right crack) or +W (down crack)
  const int near = viewZ[ia] <= viewZ[ib] ? ia : ib;
  const int dn = near == ia ? -d : d;
  const std::uint32_t id = objectId[near];
  const float vz0 = viewZ[near];
  float s = -1.0f;
  for (int k = 1; k <= 6; ++k) {
    const int j = near + dn * k;
    if (d == 1) {
      const int x = near % W + (dn < 0 ? -k : k);
      if (x < 0 || x >= W) break;
    } else if (j < 0 || j >= W * H) {
      break;
    }
    if (objectId[j] != id) break;
    s = std::max(s, std::fabs(viewZ[j] - vz0) / static_cast<float>(k));
  }
  return s;
}

// Classify ONE crack between pixel indices ia (first: left/top) and ib
// (second: right/bottom). iOutA / iOutB are the outer straight-line neighbors
// (a's far side, b's far side) with validity flags. Returns the packed crack
// byte (0 = no edge). `dbg`, when non-null, receives the same-id DepthGap
// branch diagnostics at cell `dbgCell` (dump path only).
inline std::uint8_t classifyPair(const float* viewZ,
                                 const std::uint32_t* objectId,
                                 const float* normal, int W, int H, int ia,
                                 int ib, int iOutA,
                                 bool outAValid, int iOutB, bool outBValid,
                                 const ScreenProj& sp, float cosCreaseBase,
                                 const ScreenClassifyParams& p,
                                 ScreenCrackDebugPlane* dbg = nullptr,
                                 std::size_t dbgCell = 0) {
  const bool bgA = objectId[ia] == kBackground;
  const bool bgB = objectId[ib] == kBackground;
  if (bgA && bgB) return 0;

  // 1. Silhouette: exactly one side background; the foreground pixel owns.
  if (bgA != bgB) {
    if (!p.silhouette) return 0;
    const std::uint8_t owner = bgA ? kCrackOwnerBit : 0;
    return static_cast<std::uint8_t>(CrackClass::Silhouette) | owner;
  }

  // 2. ID-keyed boundary: both foreground, ids differ. Covers BOTH a
  // cross-section boundary (different CueMol sections) and a same-section
  // mixed-kind boundary (a sphere, cylinder and mesh mixed in one section;
  // objectId == (group << 2) | kind, so equal high bits mean same section).
  // Either way the ink decision is the same: surfaces in CONTACT (a
  // sphere/cylinder penetrating a mesh, a bond embedded in an atom) have
  // CONTINUOUS viewZ across the crack at the 3D intersection contour and are
  // never inked; only a genuine occlusion step draws. The two cases differ
  // only in class/gate: cross-section inks as ObjectId under the border
  // toggle, same-section as DepthGap under the silhouette toggle (it is a
  // self-occlusion, exactly like the same-id depth gap below -- do NOT fall
  // through to it, though: its plain sideSlope would leak the other
  // primitive's depth across the kind boundary).
  //
  // Contact veto: same slope-adaptive one-sided-extrapolation form and
  // depthGapPx threshold as the DepthGap test below, so a grazing surface
  // meeting a flat one still reads as contact: the flatter side's
  // extrapolation predicts the far pixel within threshold. Taking the min of
  // the two one-sided gaps means one well-behaved side suffices, which
  // handles the grazing-vs-flat asymmetry that a naive |vzA - vzB| test
  // fails. The slope of a side is credited only while that side FACES the
  // viewer (see contactSideSlope); a rim curling toward its own silhouette in
  // front of a farther surface must not tangent-extrapolate onto it and fake
  // a contact.
  if (objectId[ia] != objectId[ib]) {
    const bool sameSection = (objectId[ia] >> 2) == (objectId[ib] >> 2);
    if (sameSection ? !p.silhouette : !p.objectBoundary) return 0;
    const float vzA = viewZ[ia], vzB = viewZ[ib];
    const float px = pixelSizeAt(sp, std::min(vzA, vzB));
    const float clampS = p.slopeClampPx * px;
    const float tol = p.depthGapPx * px;
    const float gapA = std::fabs(
        vzB - (vzA + contactSideSlope(viewZ, objectId, normal, sp, ia, iOutA,
                                      outAValid, clampS, p.borderGrazeCos)));
    const float gapB = std::fabs(
        vzA - (vzB + contactSideSlope(viewZ, objectId, normal, sp, ib, iOutB,
                                      outBValid, clampS, p.borderGrazeCos)));
    if (std::min(gapA, gapB) <= tol) return 0;  // contact
    const std::uint8_t owner = vzA <= vzB ? 0 : kCrackOwnerBit;
    return static_cast<std::uint8_t>(sameSection ? CrackClass::DepthGap
                                                 : CrackClass::ObjectId) |
           owner;
  }

  // 3. DepthGap: same id, both one-sided planar extrapolations miss the far
  // pixel (slope-adaptive second-derivative test; see header), AND the raw
  // |dvz| is the LOCAL MAX among the three parallel pixel pairs (non-maximum
  // suppression perpendicular to the crack). Without the NMS a grazing rim --
  // where the surface depth rises tangentially over a 1-2 px annulus (e.g. a
  // tube edge) -- fires a dense band of cracks that T-junctions the silhouette
  // into confetti; with it only the strongest pair on the profile fires, so
  // the boundary is one crack thin. A parallel pair that is itself a fg/bg
  // boundary counts as infinitely strong (the silhouette class owns the
  // profile there), which kills the rim annulus next to the outline.
  const float vzA = viewZ[ia], vzB = viewZ[ib];
  if (p.silhouette) {
    const float vzNear = std::min(vzA, vzB);
    const float px = pixelSizeAt(sp, vzNear);
    const float clampS = p.slopeClampPx * px;
    const float sA = sideSlope(viewZ, objectId, ia, iOutA, outAValid, clampS);
    const float sB = sideSlope(viewZ, objectId, ib, iOutB, outBValid, clampS);
    const float predA = vzA + sA;
    const float predB = vzB + sB;
    const float gapA = std::fabs(vzB - predA);
    const float gapB = std::fabs(vzA - predB);
    if (dbg) {
      dbg->gapA[dbgCell] = gapA;
      dbg->gapB[dbgCell] = gapB;
      dbg->sA[dbgCell] = sA;
      dbg->sB[dbgCell] = sB;
      dbg->g0[dbgCell] = std::fabs(vzB - vzA);
      dbg->reason[dbgCell] = ScreenCrackDebug::kSubThreshold;
    }
    const float weakRatio = std::max(0.0f, std::min(1.0f, p.weakGapRatio));
    if (std::min(gapA, gapB) > weakRatio * p.depthGapPx * px) {
      const float g0 = std::fabs(vzB - vzA);
      // Parallel-pair strength on a's far side (pair outA-a) and b's far side
      // (pair b-outB): bg neighbor => that pair is a silhouette boundary =>
      // +inf (suppress); off-image => no pair => 0. STRICT on the a side to
      // break plateau ties (a near-edge-on facet gives a run of equal-gap
      // pairs; >= on both sides would fire the whole run as a band).
      const float inf = std::numeric_limits<float>::infinity();
      float gLeft = 0.0f, gRight = 0.0f;
      if (outAValid)
        gLeft = objectId[iOutA] == kBackground ? inf
                                               : std::fabs(vzA - viewZ[iOutA]);
      if (outBValid)
        gRight = objectId[iOutB] == kBackground ? inf
                                                : std::fabs(viewZ[iOutB] - vzB);
      if (g0 > gLeft && g0 >= gRight) {
        const std::uint8_t owner = vzA <= vzB ? 0 : kCrackOwnerBit;
        // STRONG: full absolute threshold + step dominance (the raw step must
        // dwarf the near side's own recession; see nearSideRecession).
        bool strong = std::min(gapA, gapB) > p.depthGapPx * px;
        if (strong && p.stepDominanceK > 0.0f) {
          const float rec = nearSideRecession(viewZ, objectId, W, H, ia, ib);
          strong = rec >= 0.0f && g0 > p.stepDominanceK * std::max(rec, px);
          // Normal-difference rescue: the dominance gate exists to kill
          // facet-horizon slivers, which land on the SAME grazing ramp
          // (matching normals). A step onto a surface with a clearly
          // different normal is a genuine occlusion contour even when the
          // near side recedes fast (e.g. a ribbon fold in front of another
          // strand), so it stays strong.
          if (!strong && normal && p.strongNdelta > 0.0f) {
            const float* nA = normal + 3 * static_cast<std::size_t>(ia);
            const float* nB = normal + 3 * static_cast<std::size_t>(ib);
            const float lA = std::sqrt(nA[0] * nA[0] + nA[1] * nA[1] +
                                       nA[2] * nA[2]);
            const float lB = std::sqrt(nB[0] * nB[0] + nB[1] * nB[1] +
                                       nB[2] * nB[2]);
            if (lA > 1.0e-6f && lB > 1.0e-6f) {
              const float nd = 1.0f - (nA[0] * nB[0] + nA[1] * nB[1] +
                                       nA[2] * nB[2]) /
                                          (lA * lB);
              strong = nd > p.strongNdelta;
            }
          }
        }
        if (strong) {
          if (dbg) dbg->reason[dbgCell] = ScreenCrackDebug::kInked;
          return static_cast<std::uint8_t>(CrackClass::DepthGap) | owner |
                 kCrackStrongBit;
        }
        // WEAK: silhouette-clearance kill (within bgClearancePx of the
        // outline the depth signal is grazing-dominated and the silhouette
        // class already inks the boundary), EXCEPT when the crack runs into
        // the outline along its own direction -- the terminal piece of a
        // contour landing on the silhouette must reach it.
        const int ax = ia % W, ay = ia / W;
        const int bx = ib % W, by = ib / W;
        const bool rightCrack = (ib - ia) == 1;
        if (p.bgClearancePx <= 0 ||
            (!nearBackground(objectId, W, H, ax, ay, p.bgClearancePx) &&
             !nearBackground(objectId, W, H, bx, by, p.bgClearancePx)) ||
            bgAlongCrack(objectId, W, H, ax, ay, rightCrack,
                         p.bgClearancePx)) {
          if (dbg) dbg->reason[dbgCell] = ScreenCrackDebug::kInkedWeak;
          return static_cast<std::uint8_t>(CrackClass::DepthGap) | owner;
        }
        if (dbg) dbg->reason[dbgCell] = ScreenCrackDebug::kBgKilled;
      } else if (dbg) {
        dbg->reason[dbgCell] = ScreenCrackDebug::kNmsSuppressed;
      }
    }
  }

  // 4. Crease: shading-normal fold, angle widened at grazing incidence. The
  // view axis V is the camera forward (exact under ortho; a per-pixel ray
  // direction refinement is a later option -- CueMol scenes are mostly ortho).
  if (p.crease) {
    const float* nA = normal + 3 * static_cast<std::size_t>(ia);
    const float* nB = normal + 3 * static_cast<std::size_t>(ib);
    const float lA = nA[0] * nA[0] + nA[1] * nA[1] + nA[2] * nA[2];
    const float lB = nB[0] * nB[0] + nB[1] * nB[1] + nB[2] * nB[2];
    if (lA > 1.0e-12f && lB > 1.0e-12f) {
      const float d = nA[0] * nB[0] + nA[1] * nB[1] + nA[2] * nB[2];
      float cosT = cosCreaseBase;
      if (p.grazeK > 0.0f) {
        const float nvA = std::fabs(nA[0] * sp.dir.x + nA[1] * sp.dir.y +
                                    nA[2] * sp.dir.z);
        const float nvB = std::fabs(nB[0] * sp.dir.x + nB[1] * sp.dir.y +
                                    nB[2] * sp.dir.z);
        const float widen = 1.0f + p.grazeK * (1.0f - std::min(nvA, nvB));
        const float deg = std::min(179.0f, p.creaseAngleDeg * widen);
        cosT = std::cos(deg * 3.14159265358979323846f / 180.0f);
      }
      if (d < cosT) {
        const std::uint8_t owner = vzA <= vzB ? 0 : kCrackOwnerBit;
        return static_cast<std::uint8_t>(CrackClass::Crease) | owner;
      }
    }
  }
  return 0;
}

}  // namespace

CrackField classifyCracks(int W, int H, const float* viewZ,
                          const std::uint32_t* objectId, const float* normal,
                          const ScreenProj& sp,
                          const ScreenClassifyParams& params,
                          ScreenCrackDebug* dbg) {
  CrackField cf;
  cf.W = W;
  cf.H = H;
  if (W <= 0 || H <= 0) return cf;
  const std::size_t cells = static_cast<std::size_t>(W) * H;
  cf.right.assign(cells, 0);
  cf.down.assign(cells, 0);
  if (dbg) {
    for (ScreenCrackDebugPlane* pl : {&dbg->right, &dbg->down}) {
      pl->gapA.assign(cells, 0.0f);
      pl->gapB.assign(cells, 0.0f);
      pl->sA.assign(cells, 0.0f);
      pl->sB.assign(cells, 0.0f);
      pl->g0.assign(cells, 0.0f);
      pl->reason.assign(cells, ScreenCrackDebug::kNotEvaluated);
    }
  }
  if (W < 2 && H < 2) return cf;

  const float cosCreaseBase =
      std::cos(params.creaseAngleDeg * 3.14159265358979323846f / 180.0f);

  // Row-parallel: row y writes right[y*W+..] and down[y*W+..] only (disjoint);
  // reads of rows y-1 .. y+2 are read-only.
  tbb::parallel_for(
      tbb::blocked_range<int>(0, H),
      [&](const tbb::blocked_range<int>& rows) {
        for (int y = rows.begin(); y != rows.end(); ++y) {
          const int row = y * W;
          for (int x = 0; x < W; ++x) {
            const int ia = row + x;
            const std::size_t cell = static_cast<std::size_t>(ia);
            if (x + 1 < W) {  // right crack (x,y)-(x+1,y)
              cf.right[cell] = classifyPair(
                  viewZ, objectId, normal, W, H, ia, ia + 1, ia - 1,
                  x - 1 >= 0, ia + 2, x + 2 < W, sp, cosCreaseBase, params,
                  dbg ? &dbg->right : nullptr, cell);
            }
            if (y + 1 < H) {  // down crack (x,y)-(x,y+1)
              cf.down[cell] = classifyPair(
                  viewZ, objectId, normal, W, H, ia, ia + W, ia - W,
                  y - 1 >= 0, ia + 2 * W, y + 2 < H, sp, cosCreaseBase,
                  params, dbg ? &dbg->down : nullptr, cell);
            }
          }
        }
      });
  return cf;
}

namespace {

// ---------------------------------------------------------------------------
// Stage 2 tracer internals. Corners are lattice nodes (cx,cy), cx in [0..W],
// cy in [0..H]. The four incident lattice edges of a corner map to cracks:
//   E: corners (cx,cy)-(cx+1,cy)   == down [(cy-1)*W + cx]
//   S: corners (cx,cy)-(cx,cy+1)   == right[ cy   *W + (cx-1)]
//   W: corners (cx-1,cy)-(cx,cy)   == down [(cy-1)*W + (cx-1)]
//   N: corners (cx,cy-1)-(cx,cy)   == right[(cy-1)*W + (cx-1)]
// (right[y*W+x] joins corners (x+1,y)-(x+1,y+1); down[y*W+x] joins
// (x,y+1)-(x+1,y+1).)

// One incident lattice edge of a corner: the crack cell it maps to (plane +
// index), and the far corner reached by walking it.
struct CornerEdge {
  bool valid = false;
  bool isRight = false;  // true: cf.right, false: cf.down
  int cell = -1;         // index into the plane
  int farCx = 0, farCy = 0;
};

// The fixed E, S, W, N direction order (the tracer's determinism contract).
inline CornerEdge cornerEdge(const CrackField& cf, int cx, int cy, int dir) {
  CornerEdge e;
  const int W = cf.W, H = cf.H;
  switch (dir) {
    case 0:  // E
      if (cy >= 1 && cy <= H - 1 && cx >= 0 && cx <= W - 1) {
        e.valid = true;
        e.isRight = false;
        e.cell = (cy - 1) * W + cx;
        e.farCx = cx + 1;
        e.farCy = cy;
      }
      break;
    case 1:  // S
      if (cx >= 1 && cx <= W - 1 && cy >= 0 && cy <= H - 1) {
        e.valid = true;
        e.isRight = true;
        e.cell = cy * W + (cx - 1);
        e.farCx = cx;
        e.farCy = cy + 1;
      }
      break;
    case 2:  // W
      if (cy >= 1 && cy <= H - 1 && cx >= 1 && cx <= W) {
        e.valid = true;
        e.isRight = false;
        e.cell = (cy - 1) * W + (cx - 1);
        e.farCx = cx - 1;
        e.farCy = cy;
      }
      break;
    default:  // 3: N
      if (cx >= 1 && cx <= W - 1 && cy >= 1 && cy <= H) {
        e.valid = true;
        e.isRight = true;
        e.cell = (cy - 1) * W + (cx - 1);
        e.farCx = cx;
        e.farCy = cy - 1;
      }
      break;
  }
  return e;
}

inline std::uint8_t crackByte(const CrackField& cf, const CornerEdge& e) {
  return e.isRight ? cf.right[static_cast<std::size_t>(e.cell)]
                   : cf.down[static_cast<std::size_t>(e.cell)];
}

inline void markConsumed(CrackField& cf, const CornerEdge& e) {
  if (e.isRight)
    cf.right[static_cast<std::size_t>(e.cell)] |= kCrackConsumedBit;
  else
    cf.down[static_cast<std::size_t>(e.cell)] |= kCrackConsumedBit;
}

// Active-crack degree of a corner (consumed bit ignored -- stable during the
// trace).
inline int cornerDegree(const CrackField& cf, int cx, int cy) {
  int deg = 0;
  for (int dir = 0; dir < 4; ++dir) {
    const CornerEdge e = cornerEdge(cf, cx, cy, dir);
    if (e.valid && (crackByte(cf, e) & kCrackClassMask)) ++deg;
  }
  return deg;
}

// Owner PIXEL index of a crack (the side flagged by the owner bit).
inline int crackOwnerPixel(const CrackField& cf, const CornerEdge& e,
                           std::uint8_t byte) {
  const int W = cf.W;
  const int x = e.cell % W, y = e.cell / W;
  const bool second = (byte & kCrackOwnerBit) != 0;
  if (e.isRight) return y * W + (second ? x + 1 : x);  // (x,y) vs (x+1,y)
  return (second ? y + 1 : y) * W + x;                 // (x,y) vs (x,y+1)
}

// Walk one chain from `cx,cy` through starting edge `e0`, consuming cracks and
// collecting vertices/edgel attributes. Stops when the far corner is a
// TERMINAL (degree != 2) or, for `loopSeed` >= 0, when the far corner id
// equals loopSeed (closed loop). Returns the finished chain.
ScreenChain walkChain(CrackField& cf, int cx, int cy, CornerEdge e0,
                      long loopSeed, const float* viewZ,
                      const std::uint32_t* objectId,
                      const float* surfAlpha) {
  ScreenChain ch;
  std::vector<float> edgeVz;  // per-edgel owner view-z (attribution below)
  std::vector<float> edgeA;   // per-edgel owner surface alpha
  auto pushCorner = [&](int px, int py) {
    ch.pts.push_back({static_cast<float>(px) - 0.5f,
                      static_cast<float>(py) - 0.5f, 0.0f, 1.0f});
  };
  pushCorner(cx, cy);

  CornerEdge e = e0;
  for (;;) {
    const std::uint8_t byte = crackByte(cf, e);
    markConsumed(cf, e);
    ch.edgeClass.push_back(byte & kCrackClassMask);
    ch.edgeFlags.push_back((byte & kCrackStrongBit) ? 1 : 0);
    const int owner = crackOwnerPixel(cf, e, byte);
    ch.edgeGroup.push_back(
        objectId ? static_cast<std::uint16_t>(objectId[owner] >> 2) : 0);
    edgeVz.push_back(viewZ ? viewZ[owner] : 0.0f);
    edgeA.push_back(surfAlpha ? surfAlpha[owner] : 1.0f);

    cx = e.farCx;
    cy = e.farCy;
    pushCorner(cx, cy);
    const long cid = static_cast<long>(cy) * (cf.W + 1) + cx;
    if (cid == loopSeed) {
      ch.closed = true;
      break;
    }
    if (loopSeed < 0 && cornerDegree(cf, cx, cy) != 2) break;
    // Continue through the unique other unconsumed crack at this corner.
    CornerEdge next;
    bool found = false;
    for (int dir = 0; dir < 4 && !found; ++dir) {
      const CornerEdge cand = cornerEdge(cf, cx, cy, dir);
      if (!cand.valid) continue;
      const std::uint8_t b = crackByte(cf, cand);
      if ((b & kCrackClassMask) && !(b & kCrackConsumedBit)) {
        next = cand;
        found = true;
      }
    }
    if (!found) break;  // defensive: dead end (all consumed)
    e = next;
  }

  // Vertex view-z / surface alpha = mean of the adjacent edgels' owner
  // values; a closed loop's duplicated seed vertex averages the last and
  // first edgel.
  const std::size_t nE = edgeVz.size();
  for (std::size_t vi = 0; vi < ch.pts.size(); ++vi) {
    float vz, a;
    if (nE == 0) {
      vz = 0.0f;
      a = 1.0f;
    } else if (ch.closed && (vi == 0 || vi == ch.pts.size() - 1)) {
      vz = 0.5f * (edgeVz[0] + edgeVz[nE - 1]);
      a = 0.5f * (edgeA[0] + edgeA[nE - 1]);
    } else if (vi == 0) {
      vz = edgeVz[0];
      a = edgeA[0];
    } else if (vi >= nE) {
      vz = edgeVz[nE - 1];
      a = edgeA[nE - 1];
    } else {
      vz = 0.5f * (edgeVz[vi - 1] + edgeVz[vi]);
      a = 0.5f * (edgeA[vi - 1] + edgeA[vi]);
    }
    ch.pts[vi].vz = vz;
    ch.pts[vi].alpha = a;
  }
  return ch;
}

}  // namespace

std::vector<ScreenChain> traceCrackChains(CrackField& cf, const float* viewZ,
                                          const std::uint32_t* objectId,
                                          const float* surfAlpha) {
  std::vector<ScreenChain> chains;
  const int W = cf.W, H = cf.H;
  if (W <= 0 || H <= 0) return chains;

  // Pass 1: open chains, seeded at TERMINAL corners (degree 1, 3 or 4) in
  // row-major corner order, each walking its unconsumed incident cracks in the
  // fixed E, S, W, N order. Junction-to-junction chains are maximal and every
  // crack is emitted exactly once.
  for (int cy = 0; cy <= H; ++cy) {
    for (int cx = 0; cx <= W; ++cx) {
      const int deg = cornerDegree(cf, cx, cy);
      if (deg == 0 || deg == 2) continue;
      for (int dir = 0; dir < 4; ++dir) {
        const CornerEdge e = cornerEdge(cf, cx, cy, dir);
        if (!e.valid) continue;
        const std::uint8_t b = crackByte(cf, e);
        if (!(b & kCrackClassMask) || (b & kCrackConsumedBit)) continue;
        ScreenChain ch = walkChain(cf, cx, cy, e, -1, viewZ, objectId,
                                   surfAlpha);
        ch.deg0 = deg;
        const int lx = static_cast<int>(ch.pts.back().x + 0.5f);
        const int ly = static_cast<int>(ch.pts.back().y + 0.5f);
        ch.deg1 = cornerDegree(cf, lx, ly);
        chains.push_back(std::move(ch));
      }
    }
  }

  // Pass 2: closed loops. Every remaining unconsumed crack lies on a pure
  // degree-2 cycle; scan the right plane then the down plane in array order
  // and walk each loop from the crack's first corner back to itself.
  for (int pass = 0; pass < 2; ++pass) {
    const std::vector<std::uint8_t>& plane = pass == 0 ? cf.right : cf.down;
    for (std::size_t i = 0; i < plane.size(); ++i) {
      const std::uint8_t b = plane[i];
      if (!(b & kCrackClassMask) || (b & kCrackConsumedBit)) continue;
      const int x = static_cast<int>(i) % W, y = static_cast<int>(i) / W;
      // Seed corner and the walk direction along this crack: a right crack
      // starts at (x+1,y) walking S; a down crack starts at (x,y+1) walking E.
      const int cx = pass == 0 ? x + 1 : x;
      const int cy = pass == 0 ? y : y + 1;
      const CornerEdge e = cornerEdge(cf, cx, cy, pass == 0 ? 1 : 0);
      const long seed = static_cast<long>(cy) * (W + 1) + cx;
      ScreenChain ch = walkChain(cf, cx, cy, e, seed, viewZ, objectId,
                                 surfAlpha);
      ch.deg0 = ch.deg1 = 2;
      chains.push_back(std::move(ch));
    }
  }
  return chains;
}

bool keepScreenChain(const ScreenChain& ch, int minStrong) {
  int strong = 0;
  for (std::size_t i = 0; i < ch.edgeClass.size(); ++i) {
    if (ch.edgeClass[i] !=
        static_cast<std::uint8_t>(CrackClass::DepthGap))
      return true;
    if (i < ch.edgeFlags.size() && (ch.edgeFlags[i] & 1) &&
        ++strong >= std::max(1, minStrong))
      return true;
  }
  return false;
}

void eraseChainCracks(CrackField& cf, const ScreenChain& ch, std::size_t e0,
                      std::size_t e1) {
  // Consecutive corner pairs map back to lattice cells (see the cornerEdge
  // mapping): a horizontal lattice edge (cx,cy)-(cx+1,cy) is
  // down[(cy-1)*W + cx], a vertical one (cx,cy)-(cx,cy+1) is
  // right[cy*W + (cx-1)]. Chain points store corner - 0.5; edgel i spans
  // pts[i] -> pts[i+1].
  if (ch.pts.size() < 2) return;
  e1 = std::min(e1, ch.pts.size() - 1);
  for (std::size_t i = e0; i < e1; ++i) {
    const int cx0 = static_cast<int>(std::lround(ch.pts[i].x + 0.5f));
    const int cy0 = static_cast<int>(std::lround(ch.pts[i].y + 0.5f));
    const int cx1 = static_cast<int>(std::lround(ch.pts[i + 1].x + 0.5f));
    const int cy1 = static_cast<int>(std::lround(ch.pts[i + 1].y + 0.5f));
    if (cy0 == cy1)
      cf.down[static_cast<std::size_t>(cy0 - 1) * cf.W + std::min(cx0, cx1)] =
          0;
    else
      cf.right[static_cast<std::size_t>(std::min(cy0, cy1)) * cf.W +
               (cx0 - 1)] = 0;
  }
}

void eraseChainCracks(CrackField& cf, const ScreenChain& ch) {
  if (ch.pts.size() < 2) return;
  eraseChainCracks(cf, ch, 0, ch.pts.size() - 1);
}

namespace {

// Lattice corner id of a chain endpoint vertex (pts store corner - 0.5).
inline long cornerIdOf(const ScreenChainVert& v, int W) {
  const long cx = std::lround(v.x + 0.5f);
  const long cy = std::lround(v.y + 0.5f);
  return cy * (W + 1) + cx;
}

}  // namespace

std::vector<ScreenChain> pruneWeakChains(CrackField& cf,
                                         std::vector<ScreenChain> traced,
                                         const float* viewZ,
                                         const std::uint32_t* objectId,
                                         int minStrong,
                                         const float* surfAlpha) {
  // Outer loop: prune, retrace, re-evaluate. Bounded: every round erases at
  // least one chain's cracks for good; the cap only bounds the cost of
  // pathological peeling cascades (leftovers are then kept, not lost).
  for (int round = 0; round < 8; ++round) {
    const std::size_t n = traced.size();
    std::vector<char> kept(n, 0), interior(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
      kept[i] = keepScreenChain(traced[i], minStrong) ? 1 : 0;
      // INTERIOR support = a non-outline feature: strong DepthGap, ObjectId
      // or Crease. A pure-Silhouette chain supports weak neighbors only as
      // an anchor, never on its own: otherwise any weak line whose both ends
      // happen to land on the outline (the grazing rim band running parallel
      // to it) would survive.
      const ScreenChain& ch = traced[i];
      for (std::size_t e = 0; e < ch.edgeClass.size() && !interior[i]; ++e) {
        const std::uint8_t c = ch.edgeClass[e];
        if (c == static_cast<std::uint8_t>(CrackClass::ObjectId) ||
            c == static_cast<std::uint8_t>(CrackClass::Crease) ||
            (c == static_cast<std::uint8_t>(CrackClass::DepthGap) &&
             e < ch.edgeFlags.size() && (ch.edgeFlags[e] & 1)))
          interior[i] = 1;
      }
    }

    // Support propagation to fixpoint: a pure-weak OPEN chain survives when
    // BOTH endpoint corners junction into an already-kept chain AND at least
    // one of those junctions carries INTERIOR support (directly or through a
    // previously rescued weak chain, so a contour's weak tail can extend
    // across several junction-chopped fragments). Junction corners are
    // endpoints of every chain meeting there (the tracer splits at degree
    // != 2), so endpoint-to-endpoint matching is exhaustive.
    std::vector<long> end0(n, -1), end1(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
      if (traced[i].pts.empty() || traced[i].closed) continue;
      end0[i] = cornerIdOf(traced[i].pts.front(), cf.W);
      end1[i] = cornerIdOf(traced[i].pts.back(), cf.W);
    }
    for (;;) {
      std::vector<long> keptEnds, interiorEnds;
      for (std::size_t i = 0; i < n; ++i) {
        if (!kept[i] || end0[i] < 0) continue;
        keptEnds.push_back(end0[i]);
        keptEnds.push_back(end1[i]);
        if (interior[i]) {
          interiorEnds.push_back(end0[i]);
          interiorEnds.push_back(end1[i]);
        }
      }
      std::sort(keptEnds.begin(), keptEnds.end());
      std::sort(interiorEnds.begin(), interiorEnds.end());
      auto touches = [](const std::vector<long>& v, long cid) {
        return std::binary_search(v.begin(), v.end(), cid);
      };
      bool changed = false;
      for (std::size_t i = 0; i < n; ++i) {
        if (kept[i] || end0[i] < 0) continue;
        if (touches(keptEnds, end0[i]) && touches(keptEnds, end1[i]) &&
            (touches(interiorEnds, end0[i]) ||
             touches(interiorEnds, end1[i]))) {
          kept[i] = 1;
          interior[i] = 1;  // a rescued weak fragment extends the contour
          changed = true;
        }
      }
      if (!changed) break;
    }

    bool dropped = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (!kept[i]) {
        eraseChainCracks(cf, traced[i]);
        dropped = true;
      }
    }

    // RUN-LEVEL weak-tail trim on the kept OPEN chains WITHOUT strong self-
    // support. Chain-level support from a NON-DepthGap class must not extend
    // to a weak DepthGap run dangling toward an unsupported endpoint: without
    // a junction at the class transition, a weak sliver FUSED to a supported
    // run (e.g. a stick's cross-section ObjectId border continuing straight
    // into the mesh strand's same-id grazing-fade line) would ride the
    // whole-chain keep, while the SAME cracks are pruned as a free-end spur
    // when a junction separates them. Mirror the pure-weak-chain rule at run
    // granularity: a leading/trailing weak run survives only when its outer
    // endpoint corner junctions into another kept chain (its interior side
    // is bracket-supported by the adjacent non-weak run by construction).
    //
    // A chain carrying >= minStrong STRONG DepthGap edgels of its own is
    // EXEMPT: its weak end runs are the genuine tapering tails of a real
    // occlusion contour (classic hysteresis -- strong evidence extends the
    // connected weak cracks), and such contours routinely END FREE mid-
    // surface at a cusp over another surface behind, with no junction to
    // support them. Trimming those dashed the ribbon fold contours of
    // edge_ribbon2/stick3. The transp1 leak this trim exists for has ZERO
    // strong edgels (it rode an ObjectId run's keep), so it stays trimmed.
    //
    // Interior weak runs are always bracketed; closed chains are cyclically
    // bracketed; a whole-weak open chain kept here passed the both-ends
    // rescue above, so both its ends are supported.
    std::vector<long> keptEnds;
    for (std::size_t i = 0; i < n; ++i) {
      if (!kept[i] || end0[i] < 0) continue;
      keptEnds.push_back(end0[i]);
      keptEnds.push_back(end1[i]);
    }
    std::sort(keptEnds.begin(), keptEnds.end());
    // Endpoint corner `cid` of chain i is supported iff some OTHER kept
    // chain also ends there (subtract chain i's own contributions).
    auto supportedEnd = [&](std::size_t i, long cid) {
      const auto pr = std::equal_range(keptEnds.begin(), keptEnds.end(), cid);
      const int own = (end0[i] == cid ? 1 : 0) + (end1[i] == cid ? 1 : 0);
      return static_cast<int>(pr.second - pr.first) - own > 0;
    };
    for (std::size_t i = 0; i < n; ++i) {
      if (!kept[i] || traced[i].closed || end0[i] < 0) continue;
      const ScreenChain& ch = traced[i];
      const std::size_t nE = ch.edgeClass.size();
      // Strong self-support exemption: a chain that would be kept on its own
      // strong DepthGap evidence keeps its weak end runs too.
      {
        int strong = 0;
        bool exempt = false;
        for (std::size_t e = 0; e < ch.edgeFlags.size(); ++e) {
          if (ch.edgeClass[e] ==
                  static_cast<std::uint8_t>(CrackClass::DepthGap) &&
              (ch.edgeFlags[e] & 1) && ++strong >= std::max(1, minStrong)) {
            exempt = true;
            break;
          }
        }
        if (exempt) continue;
      }
      auto weakAt = [&](std::size_t e) {
        return ch.edgeClass[e] ==
                   static_cast<std::uint8_t>(CrackClass::DepthGap) &&
               !(e < ch.edgeFlags.size() && (ch.edgeFlags[e] & 1));
      };
      std::size_t lead = 0;
      while (lead < nE && weakAt(lead)) ++lead;
      if (lead == nE) continue;  // whole-weak: rescued, both ends supported
      if (lead > 0 && !supportedEnd(i, end0[i])) {
        eraseChainCracks(cf, ch, 0, lead);
        dropped = true;
      }
      std::size_t tail = nE;
      while (tail > 0 && weakAt(tail - 1)) --tail;
      if (tail < nE && !supportedEnd(i, end1[i])) {
        eraseChainCracks(cf, ch, tail, nE);
        dropped = true;
      }
    }
    if (!dropped) break;
    for (std::vector<std::uint8_t>* plane : {&cf.right, &cf.down})
      for (std::uint8_t& b : *plane)
        b &= static_cast<std::uint8_t>(~kCrackConsumedBit);
    traced = traceCrackChains(cf, viewZ, objectId, surfAlpha);
  }
  return traced;
}

// ---------------------------------------------------------------------------
// Stage 3: geometry cleanup.

float polylineLength2d(const std::vector<ScreenChainVert>& pts) {
  float len = 0.0f;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const float dx = pts[i].x - pts[i - 1].x;
    const float dy = pts[i].y - pts[i - 1].y;
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}

void collapseCollinear(std::vector<ScreenChainVert>& pts, bool /*closed*/) {
  if (pts.size() < 3) return;
  std::vector<ScreenChainVert> out;
  out.reserve(pts.size());
  out.push_back(pts.front());
  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    const ScreenChainVert& a = out.back();
    const ScreenChainVert& b = pts[i];
    const ScreenChainVert& c = pts[i + 1];
    const float abx = b.x - a.x, aby = b.y - a.y;
    const float bcx = c.x - b.x, bcy = c.y - b.y;
    const float cross = abx * bcy - aby * bcx;
    const float dot = abx * bcx + aby * bcy;
    if (cross == 0.0f && dot > 0.0f) continue;  // same direction: drop b
    out.push_back(b);
  }
  out.push_back(pts.back());
  pts.swap(out);
}

namespace {

inline ScreenChainVert lerpVert(const ScreenChainVert& a,
                                const ScreenChainVert& b, float t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
          a.vz + (b.vz - a.vz) * t, a.alpha + (b.alpha - a.alpha) * t};
}

}  // namespace

void chaikinSmooth(std::vector<ScreenChainVert>& pts, bool closed, int iters) {
  for (int it = 0; it < iters; ++it) {
    const std::size_t n = pts.size();
    if (n < 3) return;
    std::vector<ScreenChainVert> out;
    if (closed) {
      // Cut every cyclic segment of the n-1 unique vertices, then
      // re-duplicate the new seam.
      const std::size_t m = n - 1;  // unique vertices (front()==back())
      out.reserve(2 * m + 1);
      for (std::size_t i = 0; i < m; ++i) {
        const ScreenChainVert& a = pts[i];
        const ScreenChainVert& b = pts[(i + 1) % m];
        out.push_back(lerpVert(a, b, 0.25f));
        out.push_back(lerpVert(a, b, 0.75f));
      }
      out.push_back(out.front());
    } else {
      // Endpoints pinned (junction continuity across separate chains).
      out.reserve(2 * n);
      out.push_back(pts.front());
      for (std::size_t i = 0; i + 1 < n; ++i) {
        const ScreenChainVert& a = pts[i];
        const ScreenChainVert& b = pts[i + 1];
        out.push_back(lerpVert(a, b, 0.25f));
        out.push_back(lerpVert(a, b, 0.75f));
      }
      out.push_back(pts.back());
    }
    pts.swap(out);
  }
}

namespace {

// Iterative Douglas-Peucker on the OPEN range [first, last] of pts; marks kept
// vertices. Perpendicular distance to the chord (degenerate chord: distance to
// the endpoint).
void rdpMark(const std::vector<ScreenChainVert>& pts, std::size_t first,
             std::size_t last, float eps, std::vector<char>& keep) {
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  stack.push_back({first, last});
  while (!stack.empty()) {
    const auto [i0, i1] = stack.back();
    stack.pop_back();
    if (i1 <= i0 + 1) continue;
    const float ax = pts[i0].x, ay = pts[i0].y;
    const float bx = pts[i1].x, by = pts[i1].y;
    const float dx = bx - ax, dy = by - ay;
    const float len2 = dx * dx + dy * dy;
    float dMax = -1.0f;
    std::size_t iMax = i0;
    for (std::size_t i = i0 + 1; i < i1; ++i) {
      float d;
      if (len2 <= 0.0f) {
        const float ex = pts[i].x - ax, ey = pts[i].y - ay;
        d = std::sqrt(ex * ex + ey * ey);
      } else {
        d = std::fabs(dy * pts[i].x - dx * pts[i].y + bx * ay - by * ax) /
            std::sqrt(len2);
      }
      if (d > dMax) {
        dMax = d;
        iMax = i;
      }
    }
    if (dMax > eps) {
      keep[iMax] = 1;
      stack.push_back({i0, iMax});
      stack.push_back({iMax, i1});
    }
  }
}

}  // namespace

void simplifyRdp(std::vector<ScreenChainVert>& pts, bool closed, float eps) {
  if (eps <= 0.0f || pts.size() < 3) return;
  if (!closed) {
    std::vector<char> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    rdpMark(pts, 0, pts.size() - 1, eps, keep);
    std::vector<ScreenChainVert> out;
    for (std::size_t i = 0; i < pts.size(); ++i)
      if (keep[i]) out.push_back(pts[i]);
    pts.swap(out);
    return;
  }
  // Closed: rotate so the polyline starts at vertex A of an approximate-
  // diameter pair (two farthest sweeps from vertex 0 -- deterministic), split
  // at B, simplify both halves, rejoin with the seam duplicated.
  const std::size_t m = pts.size() - 1;  // unique vertices
  if (m < 3) return;
  auto farthestFrom = [&](std::size_t s) {
    std::size_t best = s;
    float bd = -1.0f;
    for (std::size_t i = 0; i < m; ++i) {
      const float dx = pts[i].x - pts[s].x, dy = pts[i].y - pts[s].y;
      const float d = dx * dx + dy * dy;
      if (d > bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  };
  const std::size_t a = farthestFrom(0);
  const std::size_t b = farthestFrom(a);
  const std::size_t lo = std::min(a, b), hi = std::max(a, b);
  if (lo == hi) return;
  // Rotate the unique ring so it starts at lo; B lands at index (hi - lo).
  std::vector<ScreenChainVert> ring;
  ring.reserve(m + 1);
  for (std::size_t i = 0; i < m; ++i) ring.push_back(pts[(lo + i) % m]);
  ring.push_back(ring.front());  // duplicated seam at the rotated start
  const std::size_t split = hi - lo;
  std::vector<char> keep(ring.size(), 0);
  keep.front() = keep.back() = 1;
  keep[split] = 1;
  rdpMark(ring, 0, split, eps, keep);
  rdpMark(ring, split, ring.size() - 1, eps, keep);
  std::vector<ScreenChainVert> out;
  for (std::size_t i = 0; i < ring.size(); ++i)
    if (keep[i]) out.push_back(ring[i]);
  pts.swap(out);
}

// ---------------------------------------------------------------------------
// Stage 4: class runs -> style slots -> shared draw stage.

namespace {

// Style slot (EdgeStyle::cls[] index) and paint precedence for a crack class.
// The screen analogue of the mesh path's natureStyleSlot/naturePrecedence:
// Silhouette and DepthGap paint on top (precedence 2), the object-id boundary
// like the mesh Border (1), Crease underneath (0). DepthGap draws with the
// Disconnected slot -- unreachable from the mesh path -- and applyScreen
// VectorEdges falls back to the Silhouette slot when a section leaves
// Disconnected unconfigured.
inline int classStyleSlot(CrackClass c) {
  switch (c) {
    case CrackClass::ObjectId:
      return static_cast<int>(EdgeClass::Object);
    case CrackClass::DepthGap:
      return static_cast<int>(EdgeClass::Disconnected);
    case CrackClass::Crease:
      return static_cast<int>(EdgeClass::Crease);
    default:
      return static_cast<int>(EdgeClass::Silhouette);
  }
}

inline int classPrecedence(CrackClass c) {
  switch (c) {
    case CrackClass::Crease:
      return 0;
    case CrackClass::ObjectId:
      return 1;
    default:
      return 2;  // Silhouette, DepthGap
  }
}

// Relabel a class run shorter than minLen edgels when bracketed by two runs
// of one same class (style-flicker suppression along a boundary whose
// classification alternates, e.g. silhouette <-> depth gap where an object
// edge grazes the background). Geometry is untouched -- only the labels move,
// so the continuity guarantee is intact. Linear scan; a closed chain's
// seam-straddling run pair is left as-is (harmless: one extra style split).
void mergeShortClassRuns(std::vector<std::uint8_t>& cls, int minLen) {
  if (minLen <= 1 || cls.size() < 3) return;
  std::size_t i = 0;
  while (i < cls.size()) {
    std::size_t j = i;
    while (j < cls.size() && cls[j] == cls[i]) ++j;
    const std::size_t runLen = j - i;
    if (i > 0 && j < cls.size() && runLen < static_cast<std::size_t>(minLen) &&
        cls[i - 1] == cls[j]) {
      for (std::size_t k = i; k < j; ++k) cls[k] = cls[i - 1];
      // Re-scan from the previous run start would be quadratic; extending the
      // left run and continuing forward keeps the pass linear and the result
      // deterministic.
    }
    i = j;
  }
}

// ---------------------------------------------------------------------------
// Debug dump (UMBREON_SCREEN_EDGE_DUMP=<prefix>): writes <prefix>_cracks.ppm
// (crack lattice colorized by class / kill reason over a viewZ gray base) and
// <prefix>_cracks.csv (per-crack DepthGap diagnostics, px-normalized).
// Optional UMBREON_SCREEN_EDGE_DUMP_ROI=x0,y0,x1,y1 restricts both outputs
// to a hi-res pixel rectangle. Diagnostic-only; never on in normal runs.

// Crack color by class byte, or by kill reason for un-inked candidates whose
// min gap exceeds `noiseFloor` (world units at that crack). Returns false for
// "draw nothing".
inline bool crackColor(std::uint8_t byte, std::uint8_t reason, float gapA,
                       float gapB, float noiseFloor, std::uint8_t rgb[3]) {
  switch (static_cast<CrackClass>(byte & kCrackClassMask)) {
    case CrackClass::Silhouette:
      rgb[0] = rgb[1] = rgb[2] = 255;  // white
      return true;
    case CrackClass::ObjectId:
      rgb[0] = 255, rgb[1] = 160, rgb[2] = 0;  // orange
      return true;
    case CrackClass::DepthGap:
      if (byte & kCrackStrongBit) {
        rgb[0] = 255, rgb[1] = 0, rgb[2] = 0;  // strong: red
      } else {
        rgb[0] = 80, rgb[1] = 120, rgb[2] = 255;  // weak: blue
      }
      return true;
    case CrackClass::Crease:
      rgb[0] = 160, rgb[1] = 0, rgb[2] = 255;  // violet
      return true;
    default:
      break;
  }
  if (std::min(gapA, gapB) <= noiseFloor) return false;
  switch (reason) {
    case ScreenCrackDebug::kNmsSuppressed:
      rgb[0] = 0, rgb[1] = 255, rgb[2] = 255;  // cyan
      return true;
    case ScreenCrackDebug::kBgKilled:
      rgb[0] = 255, rgb[1] = 255, rgb[2] = 0;  // yellow
      return true;
    case ScreenCrackDebug::kSubThreshold:
      rgb[0] = 0, rgb[1] = 160, rgb[2] = 0;  // green
      return true;
    default:
      return false;
  }
}

void writeCrackDump(const char* prefix, const CrackField& cf,
                    const ScreenCrackDebug& dbg, const float* viewZ,
                    const std::uint32_t* objectId, const float* normal,
                    const ScreenProj& sp, const ScreenClassifyParams& p) {
  const int W = cf.W, H = cf.H;
  int x0 = 0, y0 = 0, x1 = W - 1, y1 = H - 1;
  if (const char* roi = std::getenv("UMBREON_SCREEN_EDGE_DUMP_ROI")) {
    int rx0, ry0, rx1, ry1;
    if (std::sscanf(roi, "%d,%d,%d,%d", &rx0, &ry0, &rx1, &ry1) == 4) {
      x0 = std::max(0, rx0);
      y0 = std::max(0, ry0);
      x1 = std::min(W - 1, rx1);
      y1 = std::min(H - 1, ry1);
    }
  }
  if (x1 < x0 || y1 < y0) return;

  // viewZ gray range over the ROI's foreground pixels.
  float vzMin = std::numeric_limits<float>::max(), vzMax = -vzMin;
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x) {
      const int i = y * W + x;
      if (objectId[i] == kBackground) continue;
      vzMin = std::min(vzMin, viewZ[i]);
      vzMax = std::max(vzMax, viewZ[i]);
    }
  const float vzSpan = vzMax > vzMin ? vzMax - vzMin : 1.0f;

  const std::string base = std::string(prefix) + "_cracks";

  // PPM: pixel (x,y) cell at (2(x-x0)+1, 2(y-y0)+1); right crack of (x,y) at
  // (+1,0) from its cell, down crack at (0,+1); corner lattice nodes stay
  // black.
  {
    const int PW = 2 * (x1 - x0 + 1) + 1, PH = 2 * (y1 - y0 + 1) + 1;
    std::vector<std::uint8_t> img(static_cast<std::size_t>(PW) * PH * 3, 0);
    auto put = [&](int ix, int iy, std::uint8_t r, std::uint8_t g,
                   std::uint8_t b) {
      std::uint8_t* q =
          &img[(static_cast<std::size_t>(iy) * PW + ix) * 3];
      q[0] = r, q[1] = g, q[2] = b;
    };
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const int i = y * W + x;
        const int ix = 2 * (x - x0) + 1, iy = 2 * (y - y0) + 1;
        if (objectId[i] != kBackground) {
          const float t = (viewZ[i] - vzMin) / vzSpan;
          const std::uint8_t g =
              static_cast<std::uint8_t>(64.0f + 144.0f * (1.0f - t));
          put(ix, iy, g, g, g);
        }
        const float px =
            pixelSizeAt(sp, viewZ[i] > 0.0f ? viewZ[i] : vzMin);
        const float noiseFloor = 0.25f * p.depthGapPx * px;
        std::uint8_t rgb[3];
        const std::size_t cell = static_cast<std::size_t>(i);
        if (x + 1 < W &&
            crackColor(cf.right[cell], dbg.right.reason[cell],
                       dbg.right.gapA[cell], dbg.right.gapB[cell], noiseFloor,
                       rgb) &&
            ix + 1 < PW)
          put(ix + 1, iy, rgb[0], rgb[1], rgb[2]);
        if (y + 1 < H &&
            crackColor(cf.down[cell], dbg.down.reason[cell],
                       dbg.down.gapA[cell], dbg.down.gapB[cell], noiseFloor,
                       rgb) &&
            iy + 1 < PH)
          put(ix, iy + 1, rgb[0], rgb[1], rgb[2]);
      }
    }
    const std::string path = base + ".ppm";
    if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
      std::fprintf(f, "P6\n%d %d\n255\n", PW, PH);
      std::fwrite(img.data(), 1, img.size(), f);
      std::fclose(f);
      std::fprintf(stderr, "[screen-edges] dumped %s (%dx%d)\n", path.c_str(),
                   PW, PH);
    }
  }

  // Raw AOV planes (full frame, not ROI-cropped): viewZ as float32 and
  // objectId as uint32, for offline ground-truth analysis.
  {
    const std::size_t n = static_cast<std::size_t>(W) * H;
    const std::string zPath = base + "_viewz.f32";
    if (std::FILE* f = std::fopen(zPath.c_str(), "wb")) {
      std::fwrite(viewZ, sizeof(float), n, f);
      std::fclose(f);
    }
    const std::string idPath = base + "_objid.u32";
    if (std::FILE* f = std::fopen(idPath.c_str(), "wb")) {
      std::fwrite(objectId, sizeof(std::uint32_t), n, f);
      std::fclose(f);
    }
  }

  // CSV: every evaluated same-id DepthGap candidate above the noise floor.
  // Values normalized to px units at the pair's near depth; ndotv is the
  // facing cosine per side, ndelta = 1 - dot(unit normals).
  {
    const std::string path = base + ".csv";
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
                 "plane,x,y,reason,gapA_px,gapB_px,sA_px,sB_px,g0_px,"
                 "slopesum_px,ndotvA,ndotvB,ndelta\n");
    std::size_t rows = 0;
    for (int plane = 0; plane < 2; ++plane) {
      const ScreenCrackDebugPlane& d = plane == 0 ? dbg.right : dbg.down;
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          const std::size_t cell = static_cast<std::size_t>(y) * W + x;
          if (d.reason[cell] == ScreenCrackDebug::kNotEvaluated) continue;
          const int ib = plane == 0 ? static_cast<int>(cell) + 1
                                    : static_cast<int>(cell) + W;
          const float vzNear = std::min(viewZ[cell], viewZ[ib]);
          const float px = pixelSizeAt(sp, vzNear);
          if (px <= 0.0f) continue;
          if (std::min(d.gapA[cell], d.gapB[cell]) <=
              0.25f * p.depthGapPx * px)
            continue;
          float nvA = 0.0f, nvB = 0.0f, nd = 0.0f;
          if (normal) {
            nvA = facingCos(normal, sp, static_cast<int>(cell));
            nvB = facingCos(normal, sp, ib);
            const float* na = normal + 3 * cell;
            const float* nb = normal + 3 * static_cast<std::size_t>(ib);
            const float la = std::sqrt(na[0] * na[0] + na[1] * na[1] +
                                       na[2] * na[2]);
            const float lb = std::sqrt(nb[0] * nb[0] + nb[1] * nb[1] +
                                       nb[2] * nb[2]);
            if (la > 1.0e-6f && lb > 1.0e-6f)
              nd = 1.0f - (na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2]) /
                              (la * lb);
          }
          std::fprintf(f, "%c,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,"
                          "%.4f,%.5f\n",
                       plane == 0 ? 'r' : 'd', x, y, d.reason[cell],
                       d.gapA[cell] / px, d.gapB[cell] / px, d.sA[cell] / px,
                       d.sB[cell] / px, d.g0[cell] / px,
                       std::fabs(d.sA[cell] + d.sB[cell]) / px, nvA, nvB, nd);
          ++rows;
        }
      }
    }
    std::fclose(f);
    std::fprintf(stderr, "[screen-edges] dumped %s (%zu rows)\n", path.c_str(),
                 rows);
  }
}

}  // namespace

void applyScreenVectorEdges(FrameResult& frame, const Scene& scene,
                            const RenderOptions& opt) {
  const StrokeEdgeOptions& se = opt.strokeEdges;
  const int W = frame.width, H = frame.height;
  if (W <= 0 || H <= 0) return;
  if (frame.viewZ.empty() || frame.objectId.empty()) return;

  const ScreenProj sp = makeScreenProj(scene.camera, W, H);
  const float ssScale = static_cast<float>(std::max(1, opt.supersample));

  // Stage 1: classify. The nature master toggles gate the classes here (the
  // shared draw stage applies only the per-section style table).
  ScreenClassifyParams cp;
  cp.silhouette = se.silhouette;
  cp.objectBoundary = se.border;
  cp.crease = se.crease;
  cp.depthGapPx = se.screenDepthGapPx;
  cp.slopeClampPx = se.screenSlopeClampPx;
  cp.creaseAngleDeg = se.creaseAngleDeg;
  cp.grazeK = se.screenGrazeK;
  cp.bgClearancePx = static_cast<int>(std::lround(ssScale));
  const float* normalPtr = frame.normal.empty() ? nullptr : frame.normal.data();
  if (cp.crease && !normalPtr) cp.crease = false;
  const char* dumpPrefix = std::getenv("UMBREON_SCREEN_EDGE_DUMP");
  ScreenCrackDebug dbg;
  CrackField cf = classifyCracks(W, H, frame.viewZ.data(),
                                 frame.objectId.data(), normalPtr, sp, cp,
                                 dumpPrefix ? &dbg : nullptr);
  if (dumpPrefix)
    writeCrackDump(dumpPrefix, cf, dbg, frame.viewZ.data(),
                   frame.objectId.data(), normalPtr, sp, cp);

  // Stage 2: trace, then Stage 2.5: hysteresis prune + retrace. The optional
  // surfAlpha AOV attributes per-vertex surface opacity so transparent
  // sections / alpha-graded fragments fade their edges accordingly.
  const float* surfAlphaPtr =
      frame.surfAlpha.empty() ? nullptr : frame.surfAlpha.data();
  std::vector<ScreenChain> traced =
      traceCrackChains(cf, frame.viewZ.data(), frame.objectId.data(),
                       surfAlphaPtr);
  const std::size_t tracedRaw = traced.size();
  // Self-support needs ~2 FINAL px of strong evidence so a lone borderline
  // crack cannot resurrect an isolated sliver as a dash.
  const int minStrong = std::max(1, static_cast<int>(std::lround(
                                        2.0f * ssScale)));
  traced = pruneWeakChains(cf, std::move(traced), frame.viewZ.data(),
                           frame.objectId.data(), minStrong, surfAlphaPtr);

  // Stage 3+4 per chain: speck filter (whole chain), class-run relabel +
  // split, per-run geometry cleanup, slot mapping.
  const float minChainLen = se.screenMinLenPx * ssScale;
  const int mergeLen = std::max(
      0, static_cast<int>(std::lround(se.screenClassMergeLen * ssScale)));
  const float rdpEps = se.screenSimplifyPx * ssScale;
  const bool perSection = !scene.groupEdgeStyle.empty();

  std::vector<StrokeChainInput> drawChains;
  for (const ScreenChain& ch : traced) {
    if (ch.pts.size() < 2 || ch.edgeClass.empty()) continue;
    // Speck filter on the RAW chain: every edgel is one hi-res px long, so the
    // edgel count IS the arc length. JUNCTION-AWARE: a short chain whose ends
    // are both junctions (degree >= 3) is a piece of a larger boundary chopped
    // by side-branches (e.g. grazing-rim depth-gap spurs T-ing into the
    // silhouette) and is KEPT -- dropping it would dash the outline. Only a
    // short chain with a free end (a spur) or a tiny closed loop is an
    // isolated speckle and is dropped.
    if (minChainLen > 0.0f &&
        static_cast<float>(ch.edgeClass.size()) < minChainLen &&
        !(ch.deg0 >= 3 && ch.deg1 >= 3))
      continue;

    // Class-run relabel (labels only), then split into maximal same-class
    // runs. Adjacent runs share their boundary vertex, so the drawn geometry
    // stays continuous across a style change.
    std::vector<std::uint8_t> cls = ch.edgeClass;
    mergeShortClassRuns(cls, mergeLen);

    std::size_t e0 = 0;
    while (e0 < cls.size()) {
      std::size_t e1 = e0;
      while (e1 < cls.size() && cls[e1] == cls[e0]) ++e1;
      const CrackClass runClass = static_cast<CrackClass>(cls[e0]);
      // A run spanning the whole closed loop keeps the cyclic treatment.
      const bool runClosed = ch.closed && e0 == 0 && e1 == cls.size();

      StrokeChainInput in;
      in.group = ch.edgeGroup[e0];
      in.precedence = classPrecedence(runClass);
      in.styleSlot = classStyleSlot(runClass);
      // DepthGap falls back to the Silhouette slot when the section never
      // configured the Disconnected class (the default style table ships all
      // slots disabled except those the CLI enables; without the fallback a
      // same-id occlusion boundary would silently vanish).
      if (perSection &&
          runClass == CrackClass::DepthGap) {
        float h, c[3], o;
        if (!resolveStrokeStyle(scene, se, ssScale, in.styleSlot, in.group, h,
                                c, o))
          in.styleSlot = static_cast<int>(EdgeClass::Silhouette);
      }

      // Geometry cleanup on the run's vertex slice [e0, e1].
      std::vector<ScreenChainVert> pts(ch.pts.begin() + e0,
                                       ch.pts.begin() + e1 + 1);
      collapseCollinear(pts, runClosed);
      chaikinSmooth(pts, runClosed, se.screenSmoothIters);
      simplifyRdp(pts, runClosed, rdpEps);
      if (pts.size() < 2) {
        e0 = e1;
        continue;
      }
      in.pts.reserve(pts.size());
      for (const ScreenChainVert& v : pts)
        in.pts.push_back({v.x, v.y, v.vz, v.alpha, true});
      drawChains.push_back(std::move(in));
      e0 = e1;
    }
  }

  // Tuning aid (env-gated, zero-cost when unset): one stats line per frame;
  // a value of 2+ also lists every kept chain (bbox, class mix, strong
  // count) for artifact hunting.
  const char* dbgEnv = std::getenv("UMBREON_SCREEN_EDGE_DEBUG");
  if (dbgEnv && std::atoi(dbgEnv) >= 2) {
    for (std::size_t ci = 0; ci < traced.size(); ++ci) {
      const ScreenChain& ch = traced[ci];
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      for (const ScreenChainVert& v : ch.pts) {
        x0 = std::min(x0, v.x);
        y0 = std::min(y0, v.y);
        x1 = std::max(x1, v.x);
        y1 = std::max(y1, v.y);
      }
      int cc[5] = {0, 0, 0, 0, 0};
      std::size_t st = 0;
      for (std::uint8_t c : ch.edgeClass)
        if (c < 5) ++cc[c];
      for (std::uint8_t f : ch.edgeFlags) st += (f & 1);
      std::fprintf(stderr,
                   "[screen-edges]   chain %zu bbox=(%.0f,%.0f)-(%.0f,%.0f) "
                   "edgels=%zu sil=%d obj=%d gap=%d crease=%d strong=%zu "
                   "closed=%d deg=%d/%d\n",
                   ci, x0, y0, x1, y1, ch.edgeClass.size(), cc[1], cc[2],
                   cc[3], cc[4], st, ch.closed ? 1 : 0, ch.deg0, ch.deg1);
    }
  }
  if (dbgEnv) {
    std::size_t nEdgels = 0, nPts = 0, nStrong = 0;
    int clsCount[5] = {0, 0, 0, 0, 0};
    for (const ScreenChain& ch : traced) {
      nEdgels += ch.edgeClass.size();
      for (std::uint8_t c : ch.edgeClass)
        if (c < 5) ++clsCount[c];
      for (std::uint8_t f : ch.edgeFlags) nStrong += (f & 1);
    }
    for (const StrokeChainInput& in : drawChains) nPts += in.pts.size();
    std::fprintf(stderr,
                 "[screen-edges] traced=%zu kept=%zu edgels=%zu (sil=%d "
                 "obj=%d gap=%d crease=%d) gapStrong=%zu drawn=%zu pts=%zu\n",
                 tracedRaw, traced.size(), nEdgels, clsCount[1], clsCount[2],
                 clsCount[3], clsCount[4], nStrong, drawChains.size(), nPts);
  }

  renderStrokeChains(frame, scene, opt, drawChains);
}

}  // namespace umbreon
