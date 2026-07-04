// SCREEN-SPACE VECTOR edges, Stage 1: pixel-pair crack classification.
// See screen_vector_edges.hpp for the pipeline overview.
#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "edges/screen_edge_common.hpp"

namespace umbreon {
namespace {

using screen_edge::facingCos;
using screen_edge::kBackground;
using screen_edge::pixelSizeAt;

// One-sided slope of the viewZ field at pixel `a` looking away from the crack
// (toward `outer`), clamped to +-clampS. Background / off-image outer neighbors
// contribute zero slope (flat extrapolation).
inline float sideSlope(const float* viewZ, const std::uint32_t* objectId,
                       int idxA, int idxOuter, bool outerValid, float clampS) {
  if (!outerValid || objectId[idxOuter] == kBackground) return 0.0f;
  const float s = viewZ[idxA] - viewZ[idxOuter];
  return std::max(-clampS, std::min(clampS, s));
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

}  // namespace umbreon
