// Unit tests for the SCREEN-SPACE vector edge extraction, Stage 1 crack
// classification and Stage 2 chain tracing
// (src/umbreon/edges/screen_vector_edges.{hpp,cpp}).
//
// Stage 1 locks the per-pixel-pair classification contract on synthetic AOV
// buffers (no renderer needed): silhouette fires exactly on the foreground/
// background perimeter; a tilted plane of ANY in-clamp slope never fires
// (slope adaptivity); a smooth spherical cap never fires in the interior (the
// curvature veto by one-sided extrapolation); a same-id view-z step fires
// DepthGap with the nearer side as owner; abutting ids fire ObjectId; the
// class gates switch each class off.
//
// Stage 2 locks the continuity-by-construction contract: an isolated region
// boundary traces to exactly ONE closed loop (front()==back()); overlapping
// regions split chains exactly at the T-junction corners (open-chain
// endpoints have lattice degree != 2); every active crack is consumed exactly
// once; the trace is deterministic.
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <vector>

#include "edges/screen_vector_edges.hpp"
#include "edges/stroke_render.hpp"
#include "test_util.hpp"

namespace {

using umbreon::CrackClass;
using umbreon::CrackField;
using umbreon::kCrackClassMask;
using umbreon::kCrackOwnerBit;
using umbreon::kCrackStrongBit;
using umbreon::ScreenChain;
using umbreon::ScreenClassifyParams;
using umbreon::ScreenProj;

constexpr std::uint32_t kBg = 0xFFFFFFFFu;

// Synthetic AOV triplet with helpers to fill regions.
struct Buffers {
  int W, H;
  std::vector<float> viewZ;
  std::vector<std::uint32_t> objectId;
  std::vector<float> normal;
  Buffers(int w, int h)
      : W(w),
        H(h),
        viewZ(static_cast<std::size_t>(w) * h, 0.0f),
        objectId(static_cast<std::size_t>(w) * h, kBg),
        normal(static_cast<std::size_t>(w) * h * 3, 0.0f) {}
  std::size_t idx(int x, int y) const {
    return static_cast<std::size_t>(y) * W + x;
  }
  void set(int x, int y, std::uint32_t id, float vz, float nx = 0.0f,
           float ny = 0.0f, float nz = 1.0f) {
    const std::size_t i = idx(x, y);
    objectId[i] = id;
    viewZ[i] = vz;
    normal[3 * i] = nx;
    normal[3 * i + 1] = ny;
    normal[3 * i + 2] = nz;
  }
};

// Ortho projection with pixelSize == 1 world unit (halfH = H/2). The full
// basis is filled so the fold probe's screenToWorld reconstruction works
// (viewZ maps to world -z).
ScreenProj unitProj(int W, int H) {
  ScreenProj sp;
  sp.ortho = true;
  sp.W = W;
  sp.H = H;
  sp.halfW = static_cast<float>(W) * 0.5f;
  sp.halfH = static_cast<float>(H) * 0.5f;
  sp.dir = {0.0f, 0.0f, -1.0f};
  sp.right = {1.0f, 0.0f, 0.0f};
  sp.up = {0.0f, 1.0f, 0.0f};
  return sp;
}

CrackField classify(const Buffers& b, const ScreenClassifyParams& p) {
  return umbreon::classifyCracks(b.W, b.H, b.viewZ.data(), b.objectId.data(),
                                 b.normal.data(), unitProj(b.W, b.H), p);
}

// Count cracks of a given class over the whole field.
int countClass(const CrackField& cf, CrackClass c) {
  int n = 0;
  for (std::uint8_t v : cf.right)
    if ((v & kCrackClassMask) == static_cast<std::uint8_t>(c)) ++n;
  for (std::uint8_t v : cf.down)
    if ((v & kCrackClassMask) == static_cast<std::uint8_t>(c)) ++n;
  return n;
}

int countActive(const CrackField& cf) {
  int n = 0;
  for (std::uint8_t v : cf.right)
    if (v & kCrackClassMask) ++n;
  for (std::uint8_t v : cf.down)
    if (v & kCrackClassMask) ++n;
  return n;
}

}  // namespace

int main() {
  umbreon::test::Suite s("screen_vector_edges");
  const ScreenClassifyParams defaults;

  // ---- (1) square vs background: silhouette on exactly the perimeter ------
  {
    Buffers b(16, 16);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x) b.set(x, y, 7, 10.0f);
    const CrackField cf = classify(b, defaults);
    // An 8x8 square has 4*8 = 32 boundary pixel pairs.
    s.check_eq("square: 32 silhouette cracks",
               countClass(cf, CrackClass::Silhouette), 32);
    s.check_eq("square: nothing else fires", countActive(cf), 32);
    // Owner is the foreground side: left boundary crack (3,y)-(4,y) has the
    // fg pixel second -> owner bit set; right boundary (11,y)-(12,y) has the
    // fg pixel first -> owner bit clear.
    const std::uint8_t left = cf.right[b.idx(3, 8)];
    const std::uint8_t rightC = cf.right[b.idx(11, 8)];
    s.check("square: left-boundary owner is the fg (second) pixel",
            (left & kCrackOwnerBit) != 0);
    s.check("square: right-boundary owner is the fg (first) pixel",
            (rightC & kCrackOwnerBit) == 0);
    // Gate: silhouette off => nothing at all.
    ScreenClassifyParams off = defaults;
    off.silhouette = false;
    s.check_eq("square: silhouette gate off => 0 cracks",
               countActive(classify(b, off)), 0);
  }

  // ---- (2) tilted plane: steep but in-clamp slope never fires -------------
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, 3, 100.0f + 5.0f * x + 2.0f * y);  // slope 5 px > gap 2 px
    const CrackField cf = classify(b, defaults);
    s.check_eq("tilted plane: no cracks (slope-adaptive)", countActive(cf), 0);
  }

  // ---- (3) spherical cap: smooth curvature never fires in the interior ----
  {
    Buffers b(48, 48);
    const float r = 20.0f, cx = 23.5f, cy = 23.5f;
    const float rSample = 16.0f;  // sample out to 80% radius (gentle curvature)
    for (int y = 0; y < 48; ++y)
      for (int x = 0; x < 48; ++x) {
        const float dx = static_cast<float>(x) - cx;
        const float dy = static_cast<float>(y) - cy;
        const float d2 = dx * dx + dy * dy;
        if (d2 <= rSample * rSample)
          b.set(x, y, 5, 100.0f - std::sqrt(r * r - d2));
      }
    const CrackField cf = classify(b, defaults);
    // Every active crack must be Silhouette (the cap rim); no interior
    // DepthGap despite the curved depth field.
    s.check_eq("cap: no interior DepthGap (curvature tolerated)",
               countClass(cf, CrackClass::DepthGap), 0);
    s.check("cap: rim silhouette present",
            countClass(cf, CrackClass::Silhouette) > 0);
  }

  // ---- (4) same-id view-z step: DepthGap with nearer owner ----------------
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, 9, x < 8 ? 10.0f : 60.0f);  // step of 50 > gap 12
    const CrackField cf = classify(b, defaults);
    s.check_eq("z-step: one DepthGap crack per row",
               countClass(cf, CrackClass::DepthGap), 16);
    s.check_eq("z-step: nothing else fires", countActive(cf), 16);
    // Boundary crack (7,y)-(8,y): first pixel vz 10 (nearer) -> owner bit 0.
    s.check("z-step: nearer (first) side owns",
            (cf.right[b.idx(7, 5)] & kCrackOwnerBit) == 0);
  }

  // ---- (4b) facet kink: a pure slope change never fires --------------------
  // Piecewise-linear depth (flat, then a steep ramp) models the facet boundary
  // of a coarse mesh seen at grazing incidence: the steep side's one-sided
  // extrapolation predicts the flat side's edge pixel exactly, so DepthGap
  // stays silent no matter how steep the ramp (only a true DISCONTINUITY, not
  // a slope change, is an occlusion boundary).
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, 9, x < 8 ? 100.0f : 100.0f + 30.0f * (x - 7));
    const CrackField cf = classify(b, defaults);
    s.check_eq("facet kink: slope change never fires", countActive(cf), 0);
  }

  // ---- (4c) DepthGap NMS: a smeared step fires once, not as a band --------
  {
    Buffers b(16, 16);
    // Depth 10 -> 50 -> 60: a big jump followed by a smaller one (a step
    // smeared over two pixel pairs). Only the strongest pair may fire per row
    // (non-maximum suppression keeps the boundary one crack thin).
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, 9, x < 8 ? 10.0f : (x == 8 ? 50.0f : 60.0f));
    const CrackField cf = classify(b, defaults);
    s.check_eq("z-step NMS: exactly one crack per row",
               countClass(cf, CrackClass::DepthGap), 16);
  }

  // ---- (5) abutting sections at EQUAL depth: contact, not inked -----------
  {
    Buffers b(16, 16);
    // Two DIFFERENT sections (groups 1 and 2). objectId == (group << 2) | kind,
    // so use (1<<2) and (2<<2). At equal depth the two surfaces are in contact
    // (a tangential touch), NOT an occlusion step, so the cross-section border
    // is deliberately seamless -- only a genuine depth discontinuity inks.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) b.set(x, y, x < 8 ? (1u << 2) : (2u << 2),
                                         10.0f);
    const CrackField cf = classify(b, defaults);
    s.check_eq("section contact (equal depth): no ObjectId crack",
               countClass(cf, CrackClass::ObjectId), 0);
    s.check_eq("section contact (equal depth): nothing fires",
               countActive(cf), 0);
    ScreenClassifyParams off = defaults;
    off.objectBoundary = false;
    s.check_eq("section boundary: border gate off => 0 cracks",
               countActive(classify(b, off)), 0);
  }

  // ---- (5e) cross-section DEPTH STEP: ObjectId fires (occlusion) -----------
  {
    Buffers b(16, 16);
    // Section 1 in front (vz 10) of section 2 (vz 60): a real occlusion step
    // (50 > gap 12), so the between-section border is inked with the nearer
    // side as owner.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, x < 8 ? (1u << 2) : (2u << 2), x < 8 ? 10.0f : 60.0f);
    const CrackField cf = classify(b, defaults);
    s.check_eq("cross-section step: one ObjectId crack per row",
               countClass(cf, CrackClass::ObjectId), 16);
    s.check_eq("cross-section step: nothing else fires", countActive(cf), 16);
    // Boundary crack (7,y)-(8,y): first pixel vz 10 (nearer) -> owner bit 0.
    s.check("cross-section step: nearer (first) side owns",
            (cf.right[b.idx(7, 5)] & kCrackOwnerBit) == 0);
  }

  // ---- (5f) slope-adaptive contact: a grazing surface meeting a flat one ---
  // The two sections MEET (viewZ continuous across the crack) but at a slope:
  // an intersection contour, not an occlusion. The one-sided extrapolation of
  // the ramping side predicts the flat side within threshold, so it reads as
  // contact and is NOT inked -- for any ramp steepness. A naive |vzA - vzB|
  // test would ink the steep case; taking the min of the two one-sided gaps
  // does not.
  {
    for (float slope : {5.0f, 30.0f}) {
      Buffers b(16, 16);
      for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
          b.set(x, y, x < 8 ? (1u << 2) : (2u << 2),
                x < 8 ? 100.0f : 100.0f + slope * (x - 7));
      const CrackField cf = classify(b, defaults);
      s.check_eq("cross-section grazing contact: no ObjectId crack",
                 countClass(cf, CrackClass::ObjectId), 0);
      s.check_eq("cross-section grazing contact: nothing fires",
                 countActive(cf), 0);
    }
  }

  // ---- (5h) grazing rim in front of a farther section: still inks ---------
  // Section 1's surface curls away toward its own silhouette (slope grows
  // 4 -> 12 -> 48 like a sphere rim); section 2 is a flat plane behind at
  // exactly the depth the rim's TANGENT extrapolation lands on (164 + 48 =
  // 212). With a viewer-facing normal at the rim pixel the veto follows the
  // tangent and calls it contact (control case: this is how a genuinely
  // continuous steep surface must behave). With the physically correct
  // EDGE-ON normal there, the facing gate degrades the rim side to flat
  // extrapolation (|212 - 164| = 48 > 12), the sheet side predicts its own
  // continuation (gap 48), and the occlusion border inks.
  {
    for (bool grazingRim : {false, true}) {
      Buffers b(16, 16);
      for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x) {
          float vz;
          if (x <= 4) vz = 100.0f;
          else if (x == 5) vz = 104.0f;
          else if (x == 6) vz = 116.0f;
          else if (x == 7) vz = 164.0f;
          else vz = 212.0f;
          if (x == 7 && grazingRim)
            b.set(x, y, 1u << 2, vz, 1.0f, 0.0f, 0.1f);  // edge-on normal
          else
            b.set(x, y, x < 8 ? (1u << 2) : (2u << 2), vz);
        }
      const CrackField cf = classify(b, defaults);
      if (grazingRim) {
        s.check_eq("grazing rim occlusion: one ObjectId crack per row",
                   countClass(cf, CrackClass::ObjectId), 16);
        s.check_eq("grazing rim occlusion: nothing else fires",
                   countActive(cf), 16);
      } else {
        s.check_eq("facing steep surface: tangent reads as contact",
                   countActive(cf), 0);
      }
    }
  }

  // ---- (5g) cross-section outer-neighbor guard (regression) ---------------
  // Near a boundary the outer straight-line neighbor can belong to a THIRD
  // section at a depth whose (bogus) one-sided slope makes the extrapolation
  // land exactly on the far pixel -- a FALSE contact that would suppress a real
  // occlusion border. sideSlopeSameSection zeroes cross-section outer slopes,
  // degrading to a flat |vzA - vzB| step. Layout per row: grp3 vz110 (x<=6) |
  // grp2 vz60 (x==7) | grp1 vz10 (x>=8), two real occlusion steps. Both
  // between-section cracks must ink; without the guard each side's extrapolation
  // (grp3 through grp2, or grp1 through grp2) predicts the far pixel exactly and
  // both would be wrongly vetoed as contact.
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        std::uint32_t id = x <= 6 ? (3u << 2) : (x == 7 ? (2u << 2) : (1u << 2));
        float vz = x <= 6 ? 110.0f : (x == 7 ? 60.0f : 10.0f);
        b.set(x, y, id, vz);
      }
    const CrackField cf = classify(b, defaults);
    // Two cross-section boundaries per row (grp3|grp2 at x6-7, grp2|grp1 at
    // x7-8), both genuine steps.
    s.check_eq("outer-neighbor guard: both occlusion borders ink",
               countClass(cf, CrackClass::ObjectId), 32);
    s.check_eq("outer-neighbor guard: nothing else fires", countActive(cf), 32);
  }

  // ---- (5b) same section, mixed primitive kind: no internal edge ----------
  {
    Buffers b(16, 16);
    // Group 5, kind Sphere(1) vs Cylinder(2): objectId (5<<2)|1 vs (5<<2)|2.
    // A ball and a stick embedded in one CueMol section are in CONTACT at
    // their equal-depth boundary, which must NOT ink.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, (5u << 2) | (x < 8 ? 1u : 2u), 10.0f);
    s.check_eq("same section, mixed kind, equal depth: no cracks",
               countActive(classify(b, defaults)), 0);
  }

  // ---- (5c) same section, mixed kind, depth step: self-occlusion inks -----
  {
    Buffers b(16, 16);
    // A genuine view-z step at the kind boundary is a SELF-OCCLUSION (e.g. a
    // sphere of one residue in front of a distant cylinder of the same stick
    // section) and inks as DepthGap; only depth-continuous contact (5b) is
    // seamless.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, (5u << 2) | (x < 8 ? 1u : 2u), x < 8 ? 10.0f : 60.0f);
    const CrackField cf = classify(b, defaults);
    s.check_eq("same section, mixed kind, depth step: DepthGap per row",
               countClass(cf, CrackClass::DepthGap), 16);
    s.check_eq("same section, mixed kind, depth step: nothing else",
               countActive(cf), 16);
  }

  // ---- (5d) same section AND same kind: depth step still inks (unchanged) --
  {
    Buffers b(16, 16);
    // Two primitives of the SAME kind in one section share objectId entirely
    // (no per-primitive id in screen space), so a genuine occlusion step still
    // fires DepthGap -- same as mesh self-occlusion, preserved by this change.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, (5u << 2) | 1u, x < 8 ? 10.0f : 60.0f);
    s.check("same section, same kind, depth step: DepthGap fires",
            countClass(classify(b, defaults), CrackClass::DepthGap) > 0);
  }

  // ---- (5o) Outline mode: same-section self-occlusion suppressed ----------
  // SilhouetteMode::Outline draws only the section union's outer contour:
  // the same-id DepthGap and the same-section mixed-kind DepthGap are
  // suppressed at classification, while the fg/bg Silhouette and the
  // cross-section ObjectId boundaries are unaffected.
  {
    // (a) fg square over background with an internal same-id depth step:
    // Full inks the interior step, Outline keeps only the perimeter.
    Buffers b(16, 16);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x)
        b.set(x, y, (2u << 2) | 1u, x < 8 ? 10.0f : 60.0f);
    const CrackField full = classify(b, defaults);
    s.check_eq("outline (a): Full keeps the perimeter silhouette",
               countClass(full, CrackClass::Silhouette), 32);
    s.check("outline (a): Full inks the interior step",
            countClass(full, CrackClass::DepthGap) > 0);
    ScreenClassifyParams p = defaults;
    std::vector<umbreon::SilhouetteMode> mode(3, umbreon::SilhouetteMode::Full);
    mode[2] = umbreon::SilhouetteMode::Outline;
    p.groupSilhMode = mode.data();
    p.groupSilhModeCount = mode.size();
    const CrackField cf = classify(b, p);
    s.check_eq("outline (a): interior step suppressed",
               countClass(cf, CrackClass::DepthGap), 0);
    s.check_eq("outline (a): perimeter silhouette unchanged",
               countClass(cf, CrackClass::Silhouette), 32);
    s.check_eq("outline (a): nothing else fires", countActive(cf), 32);
  }
  {
    // (b) same-section mixed-kind step (the (5c) layout) is suppressed too.
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, (5u << 2) | (x < 8 ? 1u : 2u), x < 8 ? 10.0f : 60.0f);
    ScreenClassifyParams p = defaults;
    std::vector<umbreon::SilhouetteMode> mode(6, umbreon::SilhouetteMode::Full);
    mode[5] = umbreon::SilhouetteMode::Outline;
    p.groupSilhMode = mode.data();
    p.groupSilhModeCount = mode.size();
    s.check_eq("outline (b): mixed-kind step suppressed",
               countActive(classify(b, p)), 0);
  }
  {
    // (c) mixed-mode frame: group 1 (Full) still inks its own step, group 2
    // (Outline) does not. The cross-section boundary at (11,y)-(12,y) has the
    // Outline group 2 on the NEAR side (vz 10 vs 60), so it is promoted from
    // ObjectId to Silhouette (owner-side outline promotion).
    Buffers b(24, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 24; ++x) {
        const std::uint32_t id = x < 12 ? (1u << 2) : (2u << 2);
        const float vz = (x < 6 || (x >= 12 && x < 18)) ? 10.0f : 60.0f;
        b.set(x, y, id, vz);
      }
    ScreenClassifyParams p = defaults;
    std::vector<umbreon::SilhouetteMode> mode(3, umbreon::SilhouetteMode::Full);
    mode[2] = umbreon::SilhouetteMode::Outline;
    p.groupSilhMode = mode.data();
    p.groupSilhModeCount = mode.size();
    const CrackField cf = classify(b, p);
    s.check_eq("outline (c): Full group's step inks per row",
               countClass(cf, CrackClass::DepthGap), 16);
    s.check_eq("outline (c): cross-section boundary promotes to Silhouette",
               countClass(cf, CrackClass::Silhouette), 16);
    s.check_eq("outline (c): no ObjectId remains",
               countClass(cf, CrackClass::ObjectId), 0);
    s.check_eq("outline (c): Outline group's step suppressed (nothing else)",
               countActive(cf), 32);
    // Full group's step is at (5,y)-(6,y); the Outline group's would-be step
    // at (17,y)-(18,y) must be silent.
    s.check("outline (c): DepthGap sits on the Full group's step",
            (cf.right[b.idx(5, 8)] & kCrackClassMask) ==
                static_cast<std::uint8_t>(CrackClass::DepthGap));
    s.check_eq("outline (c): Outline group's step crack byte is 0",
               static_cast<int>(cf.right[b.idx(17, 8)]), 0);
    // The promoted crack's owner is the SECOND pixel (x=12, the nearer
    // Outline group), so the owner bit is set.
    const std::uint8_t promoted = cf.right[b.idx(11, 8)];
    s.check("outline (c): promoted crack is Silhouette class",
            (promoted & kCrackClassMask) ==
                static_cast<std::uint8_t>(CrackClass::Silhouette));
    s.check("outline (c): promoted crack owned by the near Outline pixel",
            (promoted & kCrackOwnerBit) != 0);
  }
  {
    // (d) default-mode fallback: a null table (and a too-short table) uses
    // silhModeDefault for every (out-of-range) group.
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, (2u << 2) | 1u, x < 8 ? 10.0f : 60.0f);
    ScreenClassifyParams p = defaults;
    p.silhModeDefault = umbreon::SilhouetteMode::Outline;
    s.check_eq("outline (d): null table + Outline default suppresses",
               countActive(classify(b, p)), 0);
    std::vector<umbreon::SilhouetteMode> mode(1, umbreon::SilhouetteMode::Full);
    p.groupSilhMode = mode.data();
    p.groupSilhModeCount = mode.size();  // group 2 out of range -> default
    s.check_eq("outline (d): out-of-range group falls back to Outline default",
               countActive(classify(b, p)), 0);
  }
  {
    // (e) Outline must not kill creases: the same-id DepthGap block is
    // skipped but the pair still falls through to the crease test.
    Buffers b(16, 16);
    const float c45 = std::sqrt(0.5f);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        if (x < 8)
          b.set(x, y, 1u << 2, 10.0f, -c45, 0.0f, c45);
        else
          b.set(x, y, 1u << 2, 10.0f, c45, 0.0f, c45);
      }
    ScreenClassifyParams p = defaults;
    p.crease = true;
    std::vector<umbreon::SilhouetteMode> mode(2, umbreon::SilhouetteMode::Full);
    mode[1] = umbreon::SilhouetteMode::Outline;
    p.groupSilhMode = mode.data();
    p.groupSilhModeCount = mode.size();
    s.check_eq("outline (e): crease still fires on an Outline section",
               countClass(classify(b, p), CrackClass::Crease), 16);
  }
  {
    // (f) owner-side outline promotion contract on a cross-section step.
    // Left half group 1 at vz 10 (near), right half group 2 at vz 60 (far);
    // the boundary (7,y)-(8,y) fires once per row. The promotion applies only
    // when the NEAR side is an Outline section: the crack becomes Silhouette
    // (gated by p.silhouette, styled from the sil slot) so the outer contour
    // survives even with the obj slot disabled and objects behind the group.
    Buffers b(16, 16);
    auto fill = [&](float vzRight) {
      for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
          b.set(x, y, x < 8 ? (1u << 2) : (2u << 2), x < 8 ? 10.0f : vzRight);
    };
    fill(60.0f);
    std::vector<umbreon::SilhouetteMode> mode(3, umbreon::SilhouetteMode::Full);
    ScreenClassifyParams p = defaults;
    p.groupSilhMode = mode.data();
    p.groupSilhModeCount = mode.size();

    // Outline on the FAR side only: no promotion, classic ObjectId.
    mode[2] = umbreon::SilhouetteMode::Outline;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("outline (f): far-side Outline keeps ObjectId",
                 countClass(cf, CrackClass::ObjectId), 16);
      s.check_eq("outline (f): far-side Outline, nothing else",
                 countActive(cf), 16);
    }

    // Outline on the NEAR side: promoted to Silhouette, owner bit clear
    // (the FIRST pixel x=7 is the nearer Outline side).
    mode[1] = umbreon::SilhouetteMode::Outline;
    mode[2] = umbreon::SilhouetteMode::Full;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("outline (f): near-side Outline promotes to Silhouette",
                 countClass(cf, CrackClass::Silhouette), 16);
      s.check_eq("outline (f): near-side Outline, nothing else",
                 countActive(cf), 16);
      s.check("outline (f): promoted crack owned by the first pixel",
              (cf.right[b.idx(7, 8)] & kCrackOwnerBit) == 0);
    }

    // The promotion is gated by p.silhouette, not p.objectBoundary.
    p.objectBoundary = false;
    s.check_eq("outline (f): promotion survives objectBoundary off",
               countClass(classify(b, p), CrackClass::Silhouette), 16);
    p.silhouette = false;
    p.objectBoundary = true;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("outline (f): silhouette off falls back to ObjectId",
                 countClass(cf, CrackClass::ObjectId), 16);
      s.check_eq("outline (f): fallback has no Silhouette",
                 countClass(cf, CrackClass::Silhouette), 0);
    }
    p.silhouette = true;

    // Both sides Outline: the near side wins, still Silhouette.
    mode[2] = umbreon::SilhouetteMode::Outline;
    s.check_eq("outline (f): both Outline promotes to Silhouette",
               countClass(classify(b, p), CrackClass::Silhouette), 16);

    // Depth-continuous contact is still vetoed in Outline mode.
    fill(10.0f);
    s.check_eq("outline (f): equal-depth contact stays silent",
               countActive(classify(b, p)), 0);
  }

  // ---- (5p) cross-section contact lines under contactBoundary -------------
  // p.contactBoundary inks the depth-continuous cross-section boundary the
  // contact veto normally suppresses (the intersection contour where one
  // group's primitive plunges into another group's mesh). Ownership must be
  // DETERMINISTIC because the near side is numerical noise at a contact: a
  // single Outline-mode side owns (Silhouette, its outer contour); otherwise
  // the smaller group id owns (ObjectId under the border gate, Silhouette
  // when both sides are Outline). Same-section contact and the occlusion-step
  // owner rule are untouched.
  {
    Buffers b(16, 16);
    auto fill = [&](float vzRight) {
      for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
          b.set(x, y, x < 8 ? (1u << 2) : (2u << 2), x < 8 ? 10.0f : vzRight);
    };
    fill(10.0f);  // equal depth: every boundary pair is a contact
    std::vector<umbreon::SilhouetteMode> mode(3, umbreon::SilhouetteMode::Full);
    ScreenClassifyParams p = defaults;
    p.groupSilhMode = mode.data();
    p.groupSilhModeCount = mode.size();

    // Default off: the veto stands.
    s.check_eq("contact (5p): flag off keeps the veto",
               countActive(classify(b, p)), 0);

    p.contactBoundary = true;

    // Both Full: ObjectId owned by the smaller group id (first pixel).
    {
      const CrackField cf = classify(b, p);
      s.check_eq("contact (5p): both Full inks ObjectId",
                 countClass(cf, CrackClass::ObjectId), 16);
      s.check_eq("contact (5p): both Full, nothing else", countActive(cf), 16);
      s.check("contact (5p): both Full owned by the smaller group id",
              (cf.right[b.idx(7, 8)] & kCrackOwnerBit) == 0);
    }

    // Both Full with the border gate off: nothing inks.
    p.objectBoundary = false;
    s.check_eq("contact (5p): both Full needs the border gate",
               countActive(classify(b, p)), 0);
    p.objectBoundary = true;

    // One Outline side owns regardless of id order: group 2 (the LARGER id,
    // second pixel) is Outline, so the owner bit is SET -- the Outline
    // preference beats the smaller-id tiebreak.
    mode[2] = umbreon::SilhouetteMode::Outline;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("contact (5p): Outline side inks Silhouette",
                 countClass(cf, CrackClass::Silhouette), 16);
      s.check_eq("contact (5p): Outline side, nothing else", countActive(cf),
                 16);
      s.check("contact (5p): Outline side owns over the smaller id",
              (cf.right[b.idx(7, 8)] & kCrackOwnerBit) != 0);
    }

    // Outline on the smaller-id side: owner bit clear.
    mode[1] = umbreon::SilhouetteMode::Outline;
    mode[2] = umbreon::SilhouetteMode::Full;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("contact (5p): Outline group 1 inks Silhouette",
                 countClass(cf, CrackClass::Silhouette), 16);
      s.check("contact (5p): Outline group 1 owns (first pixel)",
              (cf.right[b.idx(7, 8)] & kCrackOwnerBit) == 0);
    }

    // Silhouette gate off: the Outline preference is moot; smaller id owns
    // an ObjectId line under the border gate.
    mode[1] = umbreon::SilhouetteMode::Full;
    mode[2] = umbreon::SilhouetteMode::Outline;
    p.silhouette = false;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("contact (5p): silhouette off falls back to ObjectId",
                 countClass(cf, CrackClass::ObjectId), 16);
      s.check("contact (5p): silhouette off owned by the smaller id",
              (cf.right[b.idx(7, 8)] & kCrackOwnerBit) == 0);
    }
    p.silhouette = true;

    // Both Outline: Silhouette owned by the smaller id.
    mode[1] = umbreon::SilhouetteMode::Outline;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("contact (5p): both Outline inks Silhouette",
                 countClass(cf, CrackClass::Silhouette), 16);
      s.check("contact (5p): both Outline owned by the smaller id",
              (cf.right[b.idx(7, 8)] & kCrackOwnerBit) == 0);
    }
    mode[1] = mode[2] = umbreon::SilhouetteMode::Full;

    // Same-section mixed-kind contact stays silent: the flag is
    // cross-section only (a bond embedded in an atom draws no seam).
    {
      Buffers bs(16, 16);
      for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
          bs.set(x, y, (5u << 2) | (x < 8 ? 1u : 2u), 10.0f);
      s.check_eq("contact (5p): same-section contact stays silent",
                 countActive(classify(bs, p)), 0);
    }

    // A genuine occlusion step keeps the near-side owner rule even with the
    // flag on: far-side Outline does not steal the crack (no promotion, no
    // contact ownership).
    fill(60.0f);
    mode[2] = umbreon::SilhouetteMode::Outline;
    {
      const CrackField cf = classify(b, p);
      s.check_eq("contact (5p): occlusion step stays ObjectId",
                 countClass(cf, CrackClass::ObjectId), 16);
      s.check("contact (5p): occlusion step owned by the near side",
              (cf.right[b.idx(7, 8)] & kCrackOwnerBit) == 0);
    }
  }

  // ---- tracer helpers ------------------------------------------------------
  using umbreon::ScreenChain;
  auto totalEdgels = [](const std::vector<ScreenChain>& chains) {
    std::size_t n = 0;
    for (const ScreenChain& c : chains) n += c.edgeClass.size();
    return n;
  };
  auto serialize = [](const std::vector<ScreenChain>& chains) {
    std::vector<float> out;
    for (const ScreenChain& c : chains) {
      out.push_back(c.closed ? 1.0f : 0.0f);
      for (const auto& v : c.pts) {
        out.push_back(v.x);
        out.push_back(v.y);
        out.push_back(v.vz);
      }
      for (std::uint8_t e : c.edgeClass) out.push_back(e);
      for (std::uint16_t g : c.edgeGroup) out.push_back(g);
    }
    return out;
  };

  // ---- (T1) single square: exactly one closed loop -------------------------
  {
    Buffers b(16, 16);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x) b.set(x, y, 7 << 2, 10.0f);
    CrackField cf = classify(b, defaults);
    const int active = countActive(cf);
    auto chains = umbreon::traceCrackChains(cf, b.viewZ.data(),
                                            b.objectId.data());
    s.check_eq("square trace: exactly one chain", chains.size(),
               static_cast<std::size_t>(1));
    const ScreenChain& c = chains[0];
    s.check("square trace: closed", c.closed);
    s.check_eq("square trace: 32 edgels == active cracks",
               static_cast<int>(c.edgeClass.size()), active);
    s.check_eq("square trace: pts = edgels + 1", c.pts.size(),
               c.edgeClass.size() + 1);
    s.check("square trace: front == back",
            c.pts.front().x == c.pts.back().x &&
                c.pts.front().y == c.pts.back().y);
    // Geometry: the loop hugs the square boundary (stroke coords 3.5 .. 11.5).
    bool onBoundary = true;
    for (const auto& v : c.pts) {
      const bool xEdge = v.x == 3.5f || v.x == 11.5f;
      const bool yEdge = v.y == 3.5f || v.y == 11.5f;
      const bool inRange = v.x >= 3.5f && v.x <= 11.5f && v.y >= 3.5f &&
                           v.y <= 11.5f;
      if (!inRange || !(xEdge || yEdge)) onBoundary = false;
    }
    s.check("square trace: every vertex on the region boundary", onBoundary);
    // Attributes: all edgels Silhouette, group = objectId >> 2 = 7, vz = 10.
    bool attrs = true;
    for (std::size_t i = 0; i < c.edgeClass.size(); ++i)
      if (c.edgeClass[i] !=
              static_cast<std::uint8_t>(CrackClass::Silhouette) ||
          c.edgeGroup[i] != 7)
        attrs = false;
    for (const auto& v : c.pts)
      if (v.vz != 10.0f) attrs = false;
    s.check("square trace: silhouette class, group 7, vz 10 throughout",
            attrs);
    // Per-edgel owner vz is stored on the chain (Stage 4 re-attributes the
    // per-run vertex vz from it, exactly like edgeAlpha).
    bool vzStored = c.edgeVz.size() == c.edgeClass.size();
    for (float v : c.edgeVz)
      if (v != 10.0f) vzStored = false;
    s.check("square trace: per-edgel owner vz stored (edgeVz)", vzStored);
  }

  // ---- (T2) overlapping squares: T-junction split + full consumption ------
  {
    Buffers b(20, 20);
    // Far square B first, then near square A paints over the overlap (the AOV
    // keeps the first hit == nearer surface). The depth step across the A|B
    // border (10 vs 40) exceeds the contact threshold, so it inks as an
    // occlusion border and the T-junctions split the chains.
    for (int y = 6; y < 17; ++y)
      for (int x = 6; x < 17; ++x) b.set(x, y, 2 << 2, 40.0f);
    for (int y = 2; y < 10; ++y)
      for (int x = 2; x < 10; ++x) b.set(x, y, 1 << 2, 10.0f);
    CrackField cfCount = classify(b, defaults);
    const int active = countActive(cfCount);
    CrackField cf = classify(b, defaults);
    auto chains = umbreon::traceCrackChains(cf, b.viewZ.data(),
                                            b.objectId.data());
    s.check("overlap trace: several chains (split at junctions)",
            chains.size() >= 3);
    s.check_eq("overlap trace: every active crack consumed exactly once",
               totalEdgels(chains), static_cast<std::size_t>(active));
    // Open-chain endpoints sit on junction corners: their lattice degree in a
    // FRESH field is 1, 3 or 4 (never 2).
    CrackField fresh = classify(b, defaults);
    bool endpointsAtTerminals = true;
    for (const ScreenChain& c : chains) {
      if (c.closed) continue;
      for (const auto* v : {&c.pts.front(), &c.pts.back()}) {
        const int cx = static_cast<int>(v->x + 0.5f) + 0;
        const int cy = static_cast<int>(v->y + 0.5f) + 0;
        // stroke coord -> corner: (cx-0.5, cy-0.5) => corner = coord + 0.5
        int deg = 0;
        for (int dir = 0; dir < 4; ++dir) {
          // recompute degree via a classify-fresh field walk-around: count
          // active cracks incident to corner (cx+? ) -- use the public
          // mapping: E=down[(cy-1)W+cx], S=right[cyW+cx-1], W=down[(cy-1)W+cx-1],
          // N=right[(cy-1)W+cx-1].
          const int W = fresh.W, H = fresh.H;
          int cell = -1;
          bool isRight = false;
          if (dir == 0 && cy >= 1 && cy <= H - 1 && cx <= W - 1) {
            cell = (cy - 1) * W + cx;
          } else if (dir == 1 && cx >= 1 && cx <= W - 1 && cy <= H - 1) {
            cell = cy * W + (cx - 1);
            isRight = true;
          } else if (dir == 2 && cy >= 1 && cy <= H - 1 && cx >= 1) {
            cell = (cy - 1) * W + (cx - 1);
          } else if (dir == 3 && cx >= 1 && cx <= W - 1 && cy >= 1) {
            cell = (cy - 1) * W + (cx - 1);
            isRight = true;
          }
          if (cell < 0) continue;
          const std::uint8_t byte =
              isRight ? fresh.right[static_cast<std::size_t>(cell)]
                      : fresh.down[static_cast<std::size_t>(cell)];
          if (byte & kCrackClassMask) ++deg;
        }
        if (deg == 2 || deg == 0) endpointsAtTerminals = false;
      }
    }
    s.check("overlap trace: open-chain endpoints are junction corners",
            endpointsAtTerminals);
    // Determinism: an identical second run serializes identically.
    CrackField cf2 = classify(b, defaults);
    auto chains2 = umbreon::traceCrackChains(cf2, b.viewZ.data(),
                                             b.objectId.data());
    s.check("overlap trace: deterministic",
            serialize(chains) == serialize(chains2));
  }

  // ---- (T3) 1x1 region: minimal 4-edgel loop -------------------------------
  {
    Buffers b(8, 8);
    b.set(4, 4, 3 << 2, 5.0f);
    CrackField cf = classify(b, defaults);
    auto chains = umbreon::traceCrackChains(cf, b.viewZ.data(),
                                            b.objectId.data());
    s.check_eq("1x1 trace: one chain", chains.size(),
               static_cast<std::size_t>(1));
    s.check("1x1 trace: closed 4-edgel loop",
            chains[0].closed && chains[0].edgeClass.size() == 4 &&
                chains[0].pts.size() == 5);
  }

  // ---- (C1) collinear collapse: staircase runs merge exactly --------------
  {
    using umbreon::ScreenChainVert;
    std::vector<ScreenChainVert> pts = {{0, 0, 1}, {1, 0, 2}, {2, 0, 3},
                                        {2, 1, 4}, {2, 2, 5}};
    umbreon::collapseCollinear(pts, false);
    s.check("collapse: 5 -> 3 vertices",
            pts.size() == 3 && pts[0].x == 0 && pts[1].x == 2 &&
                pts[1].y == 0 && pts[2].y == 2);
  }

  // ---- (C2) Chaikin: staircase converges to the diagonal, endpoints exact -
  {
    using umbreon::ScreenChainVert;
    std::vector<ScreenChainVert> pts;
    // Unit staircase from (0,0) to (4,4): E,S alternating.
    float vz = 0.0f;
    pts.push_back({0, 0, vz});
    for (int i = 0; i < 4; ++i) {
      pts.push_back({static_cast<float>(i + 1), static_cast<float>(i),
                     vz += 1.0f});
      pts.push_back({static_cast<float>(i + 1), static_cast<float>(i + 1),
                     vz += 1.0f});
    }
    umbreon::chaikinSmooth(pts, false, 2);
    s.check("chaikin: endpoints pinned",
            pts.front().x == 0.0f && pts.front().y == 0.0f &&
                pts.back().x == 4.0f && pts.back().y == 4.0f);
    // The unit staircase oscillates symmetrically (+-0.354) around its true
    // boundary MIDLINE y = x - 0.5, not around y = x. Chaikin shrinks the
    // interior ripple well below the original amplitude; the pinned endpoints
    // (0.354 off the midline by construction) are excluded.
    float dMax = 0.0f;
    bool vzMonotone = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      if (i >= 3 && i + 3 < pts.size())
        dMax = std::max(dMax, std::fabs(pts[i].x - pts[i].y - 0.5f) /
                                  std::sqrt(2.0f));
      if (i > 0 && pts[i].vz < pts[i - 1].vz) vzMonotone = false;
    }
    s.check("chaikin: interior ripple shrinks toward the boundary midline",
            dMax < 0.25f);
    s.check("chaikin: monotone vz stays monotone", vzMonotone);
  }

  // ---- (C3) Chaikin closed: seam kept, corners cut within the box ---------
  {
    using umbreon::ScreenChainVert;
    std::vector<ScreenChainVert> pts = {
        {0, 0, 1}, {4, 0, 1}, {4, 4, 1}, {0, 4, 1}, {0, 0, 1}};
    umbreon::chaikinSmooth(pts, true, 2);
    bool inBox = true;
    for (const auto& v : pts)
      if (v.x < 0 || v.x > 4 || v.y < 0 || v.y > 4) inBox = false;
    s.check("chaikin closed: seam duplicated and inside the hull",
            pts.front().x == pts.back().x && pts.front().y == pts.back().y &&
                inBox);
    bool cornersCut = true;
    for (const auto& v : pts) {
      const bool atCorner = (v.x == 0 || v.x == 4) && (v.y == 0 || v.y == 4);
      if (atCorner) cornersCut = false;
    }
    s.check("chaikin closed: sharp corners removed", cornersCut);
  }

  // ---- (C4) RDP: near-collinear collapses, endpoints/vz preserved ---------
  {
    using umbreon::ScreenChainVert;
    std::vector<ScreenChainVert> pts = {
        {0, 0, 7}, {1, 0.1f, 8}, {2, -0.1f, 9}, {3, 0.05f, 10}, {4, 0, 11}};
    std::vector<ScreenChainVert> loose = pts;
    umbreon::simplifyRdp(loose, false, 0.4f);
    s.check("rdp: eps 0.4 collapses the wiggle to the chord",
            loose.size() == 2 && loose.front().vz == 7 &&
                loose.back().vz == 11);
    std::vector<ScreenChainVert> tight = pts;
    umbreon::simplifyRdp(tight, false, 0.05f);
    s.check("rdp: eps 0.05 keeps interior detail", tight.size() > 2);
  }

  // ---- (C5) RDP closed: square survives with its 4 corners ----------------
  {
    using umbreon::ScreenChainVert;
    // Square ring with collinear mid-edge vertices.
    std::vector<ScreenChainVert> pts;
    auto edge = [&](float x0, float y0, float x1, float y1) {
      for (int k = 0; k < 4; ++k) {
        const float t = static_cast<float>(k) / 4.0f;
        pts.push_back({x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, 1.0f});
      }
    };
    edge(0, 0, 4, 0);
    edge(4, 0, 4, 4);
    edge(4, 4, 0, 4);
    edge(0, 4, 0, 0);
    pts.push_back(pts.front());  // seam
    umbreon::simplifyRdp(pts, true, 0.3f);
    s.check("rdp closed: seam kept",
            pts.front().x == pts.back().x && pts.front().y == pts.back().y);
    // 4 corners + duplicated seam = 5 vertices.
    s.check_eq("rdp closed: square reduces to its corners", pts.size(),
               static_cast<std::size_t>(5));
    int corners = 0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
      const auto& v = pts[i];
      if ((v.x == 0 || v.x == 4) && (v.y == 0 || v.y == 4)) ++corners;
    }
    s.check_eq("rdp closed: all 4 corners survive", corners, 4);
  }

  // ---- (6) crease: normal fold fires only with the crease gate ------------
  {
    Buffers b(16, 16);
    const float c45 = std::sqrt(0.5f);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        // Two half-planes, same id and depth, normals 90 degrees apart
        // (both 45 degrees off the view axis => equal grazing widening).
        if (x < 8)
          b.set(x, y, 4, 10.0f, -c45, 0.0f, c45);
        else
          b.set(x, y, 4, 10.0f, c45, 0.0f, c45);
      }
    s.check_eq("crease: default (gate off) => 0 cracks",
               countActive(classify(b, defaults)), 0);
    ScreenClassifyParams on = defaults;
    on.crease = true;
    const CrackField cf = classify(b, on);
    s.check_eq("crease: 90-degree fold fires per row",
               countClass(cf, CrackClass::Crease), 16);
  }

  // ---- (7) step dominance: sliver on a grazing ramp weak, flat step strong -
  // A coarse mesh at grazing incidence throws off facet-horizon slivers: a
  // sight line skims a facet edge and lands a few pixels' worth of the SAME
  // grazing ramp deeper. The step is real and above the absolute threshold,
  // but it does not dominate the near side's own recession, so it must stay
  // WEAK (drawable only with chain support). A same-magnitude-class step on a
  // flat surface is a true occlusion contour and is STRONG.
  {
    Buffers b(24, 8);
    for (int y = 0; y < 8; ++y)
      for (int x = 0; x < 24; ++x) {
        float vz = 100.0f + 10.0f * x;  // grazing ramp, 10 per px
        if (x >= 12) vz += 45.0f;       // sliver: ~4.5 px of ramp
        b.set(x, y, 9, vz);
      }
    const CrackField cf = classify(b, defaults);
    const std::uint8_t byte = cf.right[b.idx(11, 4)];
    s.check("sliver on ramp: DepthGap fires",
            (byte & kCrackClassMask) ==
                static_cast<std::uint8_t>(CrackClass::DepthGap));
    s.check("sliver on ramp: weak (step does not dominate the ramp)",
            (byte & kCrackStrongBit) == 0);
  }
  {
    Buffers b(16, 8);
    for (int y = 0; y < 8; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, 9, x < 8 ? 100.0f : 500.0f);  // step 400 over flat
    const CrackField cf = classify(b, defaults);
    const std::uint8_t byte = cf.right[b.idx(7, 4)];
    s.check("flat step: DepthGap strong",
            (byte & kCrackClassMask) ==
                    static_cast<std::uint8_t>(CrackClass::DepthGap) &&
                (byte & kCrackStrongBit) != 0);
  }

  // ---- (8) Stage 2.5: weak survives only when bridging interior support ---
  // Two strong vertical steps with (a) a weak horizontal connector between
  // them (both ends junction into strong chains -> KEPT: this is the
  // near-cusp tail / chopped-fragment case) and (b) a weak line from the
  // right step to the image border (a free end -> pruned sliver). After
  // pruneWeakChains the connector's cracks survive, the spur's are erased.
  {
    Buffers b(20, 20);
    for (int y = 0; y < 20; ++y)
      for (int x = 0; x < 20; ++x) {
        float vz;
        if (x < 7)
          vz = 100.0f;
        else if (x < 14)
          vz = 600.0f + (y >= 10 ? 30.0f : 0.0f);   // weak connector at y 9|10
        else
          vz = 1500.0f + (y >= 15 ? 30.0f : 0.0f);  // weak spur at y 14|15
        b.set(x, y, 9, vz);
      }
    CrackField cf = classify(b, defaults);
    std::vector<ScreenChain> chains =
        umbreon::traceCrackChains(cf, b.viewZ.data(), b.objectId.data());
    chains = umbreon::pruneWeakChains(cf, std::move(chains), b.viewZ.data(),
                                      b.objectId.data());
    int weakKept = 0, strongKept = 0;
    for (const ScreenChain& ch : chains)
      for (std::size_t i = 0; i < ch.edgeClass.size(); ++i) {
        if (ch.edgeClass[i] !=
            static_cast<std::uint8_t>(CrackClass::DepthGap))
          continue;
        if (ch.edgeFlags[i] & 1)
          ++strongKept;
        else
          ++weakKept;
      }
    s.check_eq("prune: both strong steps fully kept", strongKept, 40);
    s.check_eq("prune: bridging weak connector kept, free-end spur pruned",
               weakKept, 7);
    // Determinism: prune of an identically classified field gives the same
    // chain set.
    CrackField cf2 = classify(b, defaults);
    std::vector<ScreenChain> chains2 =
        umbreon::traceCrackChains(cf2, b.viewZ.data(), b.objectId.data());
    chains2 = umbreon::pruneWeakChains(cf2, std::move(chains2),
                                       b.viewZ.data(), b.objectId.data());
    bool same = chains2.size() == chains.size();
    for (std::size_t i = 0; same && i < chains.size(); ++i) {
      same = chains[i].pts.size() == chains2[i].pts.size() &&
             chains[i].edgeClass == chains2[i].edgeClass;
      for (std::size_t v = 0; same && v < chains[i].pts.size(); ++v)
        same = chains[i].pts[v].x == chains2[i].pts[v].x &&
               chains[i].pts[v].y == chains2[i].pts[v].y;
    }
    s.check("prune: deterministic", same);
  }

  // ---- (8b) run-level weak-tail trim: a weak run fused to a supported ----
  // run must not ride the whole-chain keep to a free end. Left surface (one
  // section) against a farther right surface: the upper right half is
  // ANOTHER section (ObjectId border), the lower right half the SAME section
  // (weak same-id step, dominance-gated). No junction separates the two runs
  // (the id transition inside the right surface is depth-continuous), so
  // they trace as ONE chain; the weak run ends at a free image-border corner
  // and must be trimmed while the ObjectId run stays.
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        if (x < 8)
          b.set(x, y, 5, 100.0f);       // near surface, section 1
        else if (y < 8)
          b.set(x, y, 9, 130.0f);       // far surface, section 2 -> ObjectId
        else
          b.set(x, y, 5, 130.0f);       // far surface, section 1 -> weak gap
      }
    CrackField cf = classify(b, defaults);
    std::vector<ScreenChain> chains =
        umbreon::traceCrackChains(cf, b.viewZ.data(), b.objectId.data());
    // Sanity: one fused chain, 8 ObjectId + 8 weak DepthGap edgels.
    int objRaw = 0, weakRaw = 0;
    for (const ScreenChain& ch : chains)
      for (std::size_t e = 0; e < ch.edgeClass.size(); ++e) {
        if (ch.edgeClass[e] == static_cast<std::uint8_t>(CrackClass::ObjectId))
          ++objRaw;
        if (ch.edgeClass[e] == static_cast<std::uint8_t>(CrackClass::DepthGap))
          ++weakRaw;
      }
    s.check_eq("weak-tail trim: fused into one chain", chains.size(),
               static_cast<std::size_t>(1));
    s.check_eq("weak-tail trim: raw ObjectId edgels", objRaw, 8);
    s.check_eq("weak-tail trim: raw weak edgels", weakRaw, 8);
    chains = umbreon::pruneWeakChains(cf, std::move(chains), b.viewZ.data(),
                                      b.objectId.data());
    int objKept = 0, weakKept = 0;
    for (const ScreenChain& ch : chains)
      for (std::size_t e = 0; e < ch.edgeClass.size(); ++e) {
        if (ch.edgeClass[e] == static_cast<std::uint8_t>(CrackClass::ObjectId))
          ++objKept;
        if (ch.edgeClass[e] == static_cast<std::uint8_t>(CrackClass::DepthGap))
          ++weakKept;
      }
    s.check_eq("weak-tail trim: ObjectId run kept", objKept, 8);
    s.check_eq("weak-tail trim: free-end weak tail erased", weakKept, 0);
  }
  // Bracketed counterpart: the same fused chain, but the weak run lands on
  // the silhouette outline (background rows below). Its outer end junctions
  // into the kept silhouette chains, so the weak run SURVIVES -- the
  // contour-terminal case the trim must not break.
  {
    Buffers b(16, 16);
    for (int y = 0; y < 12; ++y)
      for (int x = 0; x < 16; ++x) {
        if (x < 8)
          b.set(x, y, 5, 100.0f);
        else if (y < 8)
          b.set(x, y, 9, 130.0f);
        else
          b.set(x, y, 5, 130.0f);
      }
    CrackField cf = classify(b, defaults);
    std::vector<ScreenChain> chains =
        umbreon::traceCrackChains(cf, b.viewZ.data(), b.objectId.data());
    chains = umbreon::pruneWeakChains(cf, std::move(chains), b.viewZ.data(),
                                      b.objectId.data());
    int objKept = 0, weakKept = 0;
    for (const ScreenChain& ch : chains)
      for (std::size_t e = 0; e < ch.edgeClass.size(); ++e) {
        if (ch.edgeClass[e] == static_cast<std::uint8_t>(CrackClass::ObjectId))
          ++objKept;
        if (ch.edgeClass[e] == static_cast<std::uint8_t>(CrackClass::DepthGap))
          ++weakKept;
      }
    s.check_eq("weak-tail trim: ObjectId run kept (bracketed)", objKept, 8);
    s.check_eq("weak-tail trim: outline-landing weak run survives", weakKept,
               4);
  }
  // Strong self-support exemption: a free-ended (deg 1/1) contour chain of
  // weak-strong-weak composition -- the tapering fold contour over another
  // surface behind (the edge_ribbon2 regression) -- keeps its weak end runs.
  // One vertical same-id step whose magnitude varies smoothly along y (linear
  // ramps, so no horizontal cracks fire): 30 (weak) at both ends, 510
  // (strong: > stepDominanceK * px) in the middle.
  {
    Buffers b(16, 16);
    const float f[16] = {30, 30, 30, 30, 30, 150, 270, 390, 510, 510, 510,
                         510, 390, 270, 150, 30};
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, 9, x < 8 ? 100.0f : 100.0f + f[y]);
    CrackField cf = classify(b, defaults);
    std::vector<ScreenChain> chains =
        umbreon::traceCrackChains(cf, b.viewZ.data(), b.objectId.data());
    s.check_eq("strong exemption: one free-ended chain", chains.size(),
               static_cast<std::size_t>(1));
    int strongRaw = 0, weakRaw = 0;
    for (const ScreenChain& ch : chains)
      for (std::size_t e = 0; e < ch.edgeClass.size(); ++e)
        if (ch.edgeClass[e] ==
            static_cast<std::uint8_t>(CrackClass::DepthGap)) {
          if (ch.edgeFlags[e] & 1)
            ++strongRaw;
          else
            ++weakRaw;
        }
    s.check_eq("strong exemption: raw strong edgels", strongRaw, 8);
    s.check_eq("strong exemption: raw weak edgels", weakRaw, 8);
    chains = umbreon::pruneWeakChains(cf, std::move(chains), b.viewZ.data(),
                                      b.objectId.data());
    int strongKept = 0, weakKept = 0;
    for (const ScreenChain& ch : chains)
      for (std::size_t e = 0; e < ch.edgeClass.size(); ++e)
        if (ch.edgeClass[e] ==
            static_cast<std::uint8_t>(CrackClass::DepthGap)) {
          if (ch.edgeFlags[e] & 1)
            ++strongKept;
          else
            ++weakKept;
        }
    s.check_eq("strong exemption: strong body kept", strongKept, 8);
    s.check_eq("strong exemption: free-end weak tails kept (not trimmed)",
               weakKept, 8);
  }

  // ---- (9) bg clearance: terminal weak cracks reach the outline -----------
  // A weak step line PERPENDICULAR to the outline is a contour terminal: its
  // cracks inside the clearance radius have background in their along-crack
  // strip and survive. A weak line PARALLEL to the outline at a 2 px offset
  // is grazing rim noise: its interior cracks are killed (its ends, which do
  // land on the side outline, survive as terminals).
  {
    Buffers b(16, 16);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x)
        b.set(x, y, 9, x >= 8 ? 130.0f : 100.0f);  // step 30 into the outline
    const CrackField cf = classify(b, defaults);
    s.check_eq("perpendicular weak line: every crack reaches the outline",
               countClass(cf, CrackClass::DepthGap), 8);
  }
  {
    Buffers b(16, 16);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x)
        b.set(x, y, 9, y >= 6 ? 130.0f : 100.0f);  // line 2 px below outline
    const CrackField cf = classify(b, defaults);
    s.check("parallel weak line: interior crack killed by clearance",
            (cf.down[b.idx(7, 5)] & kCrackClassMask) == 0);
    s.check("parallel weak line: terminal crack at the side outline survives",
            (cf.down[b.idx(4, 5)] & kCrackClassMask) ==
                static_cast<std::uint8_t>(CrackClass::DepthGap));
  }

  // ---- (10) surfAlpha attribution: chain vertices carry the owner-pixel ---
  // surface opacity (mean of the adjacent edgels), so a transparent section's
  // edge fades with it. Null buffer keeps every vertex opaque (alpha == 1).
  {
    Buffers b(16, 16);
    std::vector<float> surfA(static_cast<std::size_t>(b.W) * b.H, 1.0f);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x) {
        b.set(x, y, 7, 10.0f);
        surfA[b.idx(x, y)] = 0.4f;  // uniformly transparent section
      }
    CrackField cf = classify(b, defaults);
    std::vector<ScreenChain> chains = umbreon::traceCrackChains(
        cf, b.viewZ.data(), b.objectId.data(), surfA.data());
    bool all04 = !chains.empty();
    for (const ScreenChain& ch : chains)
      for (const umbreon::ScreenChainVert& v : ch.pts)
        if (v.alpha != 0.4f) all04 = false;
    s.check("uniform transparent section: every chain vertex alpha == 0.4",
            all04);

    CrackField cf2 = classify(b, defaults);
    std::vector<ScreenChain> chains2 =
        umbreon::traceCrackChains(cf2, b.viewZ.data(), b.objectId.data());
    bool all1 = !chains2.empty();
    for (const ScreenChain& ch : chains2)
      for (const umbreon::ScreenChainVert& v : ch.pts)
        if (v.alpha != 1.0f) all1 = false;
    s.check("no surfAlpha buffer: every chain vertex alpha == 1", all1);
  }

  // ---- (11) surfAlpha gradient: a fragment-alpha split inside one section -
  // yields per-vertex alphas following the owner pixels, with the transition
  // vertex averaging its two adjacent edgels (0.25 | 0.75 -> 0.5) -- the
  // linear-interpolation contract between differently-transparent fragments.
  {
    Buffers b(16, 16);
    std::vector<float> surfA(static_cast<std::size_t>(b.W) * b.H, 1.0f);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x) {
        b.set(x, y, 7, 10.0f);
        surfA[b.idx(x, y)] = x < 8 ? 0.25f : 0.75f;
      }
    CrackField cf = classify(b, defaults);
    std::vector<ScreenChain> chains = umbreon::traceCrackChains(
        cf, b.viewZ.data(), b.objectId.data(), surfA.data());
    float aMin = 1.0f, aMax = 0.0f;
    int nHalf = 0;
    bool inSet = !chains.empty();
    for (const ScreenChain& ch : chains)
      for (const umbreon::ScreenChainVert& v : ch.pts) {
        aMin = std::min(aMin, v.alpha);
        aMax = std::max(aMax, v.alpha);
        if (v.alpha == 0.5f) ++nHalf;
        if (v.alpha != 0.25f && v.alpha != 0.5f && v.alpha != 0.75f)
          inSet = false;
      }
    s.check("alpha split: vertex alphas are owner means only", inSet);
    s.check_eq("alpha split: min vertex alpha", aMin, 0.25f);
    s.check_eq("alpha split: max vertex alpha", aMax, 0.75f);
    // One averaged transition vertex on the top boundary, one on the bottom.
    s.check_eq("alpha split: two 0.5 transition vertices", nHalf, 2);
  }

  // ---- (12) draw stage: per-vertex alpha lerps along the stroke; uniform --
  // alpha takes the constant path. A horizontal 2 px stroke over a white
  // frame, alpha 0 -> 1: untouched at the start, ~half ink mid-span, full
  // ink at the end. A second chain with constant alpha 0.5 inks exactly 0.5.
  {
    const int W = 40, H = 12;
    umbreon::FrameResult frame;
    frame.width = W;
    frame.height = H;
    frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
    umbreon::Scene scene;  // groupEdgeStyle empty -> global stroke style
    umbreon::RenderOptions opt;
    opt.width = W;
    opt.height = H;
    opt.supersample = 1;
    opt.strokeEdges.enable = true;
    opt.strokeEdges.thickness = 2;  // black, opacity 1 (defaults)

    // Half-integer backbone coordinates, exactly like the tracer's chain
    // vertices (lattice corner - 0.5): pixel centers then sit strictly
    // inside the ribbon quads, never on a shared quad seam.
    std::vector<umbreon::StrokeChainInput> chains(2);
    chains[0].pts = {{4.5f, 3.5f, 10.0f, 0.0f, true},
                     {35.5f, 3.5f, 10.0f, 1.0f, true}};
    chains[1].pts = {{4.5f, 8.5f, 10.0f, 0.5f, true},
                     {35.5f, 8.5f, 10.0f, 0.5f, true}};
    umbreon::renderStrokeChains(frame, scene, opt, chains);

    auto lum = [&](int x, int y) {
      const std::size_t p = (static_cast<std::size_t>(y) * W + x) * 4;
      return frame.color[p];  // white base + black ink: R == G == B
    };
    s.check("gradient stroke: start stays (nearly) uninked",
            lum(5, 3) > 0.9f);
    s.check("gradient stroke: mid-span inks about half",
            lum(20, 3) > 0.3f && lum(20, 3) < 0.7f);
    s.check("gradient stroke: end inks (nearly) full", lum(34, 3) < 0.1f);
    // Constant-alpha chain: exactly style opacity (1) x surface alpha (0.5).
    s.check_eq("uniform 0.5 stroke: exact half ink", lum(20, 8), 0.5f);
  }

  // ---- (13) draw stage: DEPTH FOG fades edge lines with distance, keyed on the
  // per-vertex plane eye-z (vz), mirroring the surface fog post-process. A near-
  // to-far black stroke over a white frame with RED fog (start=10, end=30):
  //   * opaque background: the ink COLOR melts toward the fog color -> the near
  //     end stays black, the far end turns red (opacity untouched);
  //   * transparent background: the ink OPACITY fades -> the near end stays
  //     black, the far end drops out entirely (stays white; color NOT baked).
  // The two modes are told apart at the far end: opaque -> red, transparent ->
  // white. This locks the "edges fog like the 3D geometry under them" contract.
  {
    const int W = 40, H = 12;
    umbreon::Scene scene;
    scene.fog.enabled = true;
    scene.fog.start = 10.0f;
    scene.fog.end = 30.0f;
    scene.fog.color = {1.0f, 0.0f, 0.0f};  // red fog
    umbreon::RenderOptions opt;
    opt.width = W;
    opt.height = H;
    opt.supersample = 1;
    opt.strokeEdges.enable = true;
    opt.strokeEdges.thickness = 2;  // black, opacity 1 (defaults)

    // near end vz=10 (f=1, unfogged) -> far end vz=30 (f=0, full fog).
    std::vector<umbreon::StrokeChainInput> chain(1);
    chain[0].pts = {{4.5f, 3.5f, 10.0f, 1.0f, true},
                    {35.5f, 3.5f, 30.0f, 1.0f, true}};

    auto chanAt = [&](const umbreon::FrameResult& fr, int x, int y, int c) {
      return fr.color[(static_cast<std::size_t>(y) * W + x) * 4 + c];
    };

    // Opaque background: color melts toward red with depth.
    {
      umbreon::FrameResult fr;
      fr.width = W;
      fr.height = H;
      fr.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);  // white
      opt.transparentBackground = false;
      umbreon::renderStrokeChains(fr, scene, opt, chain);
      s.check("fog opaque: near end stays black (unfogged)",
              chanAt(fr, 5, 3, 0) < 0.1f && chanAt(fr, 5, 3, 1) < 0.1f);
      s.check("fog opaque: far end melts toward fog color (red)",
              chanAt(fr, 34, 3, 0) > 0.9f && chanAt(fr, 34, 3, 1) < 0.1f);
      s.check("fog opaque: mid-span is a partial mix",
              chanAt(fr, 20, 3, 0) > 0.3f && chanAt(fr, 20, 3, 0) < 0.7f &&
                  chanAt(fr, 20, 3, 1) < 0.2f);
    }

    // Transparent background: coverage fades, ink color NOT baked.
    {
      umbreon::FrameResult fr;
      fr.width = W;
      fr.height = H;
      fr.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);  // white
      opt.transparentBackground = true;
      umbreon::renderStrokeChains(fr, scene, opt, chain);
      s.check("fog transparent: near end stays black (full coverage)",
              chanAt(fr, 5, 3, 0) < 0.1f && chanAt(fr, 5, 3, 1) < 0.1f);
      // Far end drops out -> stays the white base (NOT red): the fog color is
      // never baked, distinguishing this mode from the opaque one above.
      s.check("fog transparent: far end fades out (stays white, not red)",
              chanAt(fr, 34, 3, 0) > 0.9f && chanAt(fr, 34, 3, 1) > 0.9f);
    }
  }

  // ---- (14) draw stage: with fog OFF the stroke is unaffected (control for the
  // gated fog path -- the enable flag is what gates it, nothing else changes).
  {
    const int W = 40, H = 12;
    umbreon::FrameResult fr;
    fr.width = W;
    fr.height = H;
    fr.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
    umbreon::Scene scene;  // fog disabled by default
    umbreon::RenderOptions opt;
    opt.width = W;
    opt.height = H;
    opt.supersample = 1;
    opt.strokeEdges.enable = true;
    opt.strokeEdges.thickness = 2;
    std::vector<umbreon::StrokeChainInput> chain(1);
    chain[0].pts = {{4.5f, 3.5f, 10.0f, 1.0f, true},
                    {35.5f, 3.5f, 30.0f, 1.0f, true}};
    umbreon::renderStrokeChains(fr, scene, opt, chain);
    auto lum = [&](int x, int y) {
      return fr.color[(static_cast<std::size_t>(y) * W + x) * 4];
    };
    s.check("fog off: near end fully inked", lum(5, 3) < 0.1f);
    s.check("fog off: far end fully inked (no depth fade)", lum(34, 3) < 0.1f);
  }

  // ---- (16) round caps and round joins (--stroke-cap/--stroke-join) ------
  // Butt/miter defaults are locked bit-identical elsewhere; here the round
  // geometry contract: a round cap inks the half-disk beyond the butt end,
  // and a round join replaces the miter spike with an arc of the stroke's
  // half-width radius.
  {
    auto renderChain = [&](std::vector<umbreon::StrokePoint> pts,
                           bool roundCap, bool roundJoin, int W, int H) {
      umbreon::FrameResult fr;
      fr.width = W;
      fr.height = H;
      fr.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
      umbreon::Scene scene;
      umbreon::RenderOptions opt;
      opt.width = W;
      opt.height = H;
      opt.supersample = 1;
      opt.strokeEdges.enable = true;
      opt.strokeEdges.thickness = 8;  // half-width 4
      opt.strokeEdges.roundCap = roundCap;
      opt.strokeEdges.roundJoin = roundJoin;
      std::vector<umbreon::StrokeChainInput> chain(1);
      chain[0].pts = std::move(pts);
      umbreon::renderStrokeChains(fr, scene, opt, chain);
      return fr;
    };
    auto lumAt = [](const umbreon::FrameResult& fr, int x, int y) {
      return fr.color[(static_cast<std::size_t>(y) * fr.width + x) * 4];
    };
    // Cap: horizontal stroke ending at x=10.5; pixel (9,10) lies beyond the
    // butt end (distance 1.6 from the endpoint, well inside radius 4).
    const std::vector<umbreon::StrokePoint> capPts = {
        {10.5f, 10.5f, 10.0f, 1.0f, true}, {30.5f, 10.5f, 10.0f, 1.0f, true}};
    {
      umbreon::FrameResult butt = renderChain(capPts, false, false, 40, 21);
      umbreon::FrameResult round = renderChain(capPts, true, false, 40, 21);
      s.check("round cap: butt leaves the end pixel blank",
              lumAt(butt, 9, 10) > 0.9f);
      s.check("round cap: half-disk inks beyond the end",
              lumAt(round, 9, 10) < 0.1f);
      s.check("round cap: outside the cap radius stays blank",
              lumAt(round, 5, 10) > 0.9f);
    }
    // Join: L-shaped 90-degree corner at (20.5,20.5); the miter tip reaches
    // (24.5,24.5) so pixel (24,24) inks under miter but lies OUTSIDE the
    // radius-4 round arc (distance ~4.9); pixel (22,22) (distance ~2.1) inks
    // under both.
    const std::vector<umbreon::StrokePoint> joinPts = {
        {5.5f, 20.5f, 10.0f, 1.0f, true},
        {20.5f, 20.5f, 10.0f, 1.0f, true},
        {20.5f, 5.5f, 10.0f, 1.0f, true}};
    {
      umbreon::FrameResult miter = renderChain(joinPts, false, false, 40, 40);
      umbreon::FrameResult round = renderChain(joinPts, false, true, 40, 40);
      s.check("round join: miter spike inks the far corner",
              lumAt(miter, 24, 24) < 0.1f);
      s.check("round join: arc removes the spike",
              lumAt(round, 24, 24) > 0.9f);
      s.check("round join: inside the arc still inked (miter)",
              lumAt(miter, 22, 22) < 0.1f);
      s.check("round join: inside the arc still inked (round)",
              lumAt(round, 22, 22) < 0.1f);
    }
  }

  // ---- (12) fold probe: edge-of-visible-surface test gates the rescue -----
  // A dominance-failing step whose sides have clearly different normals is a
  // strongNdelta-rescue candidate: either a genuine occlusion contour (empty
  // gap -> rescued STRONG) or a sharp same-surface fold (a wall connects the
  // depths -> veto, stays weak). The probe's answer decides; the segment must
  // sample the FAR-ANCHORED window [vzFar - max(frac*dz, 2px), vzFar - eps].
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        if (x < 8)
          b.set(x, y, 9, 100.0f);  // near, facing (0,0,1)
        else
          b.set(x, y, 9, 130.0f, 1.0f, 0.0f, 0.0f);  // far, edge-on normal
      }
    // Step 30 over flat sides: full threshold (30 > 12) passes, dominance
    // (30 > 250) fails, ndelta = 1 > 0.3 -> rescue candidate on every row.
    const ScreenProj sp = unitProj(16, 16);
    // classifyCracks runs TBB-parallel: the counter must be atomic.
    std::atomic<int> calls{0};
    float vz0 = 0.0f, vz1 = 0.0f;
    bool hit = true;  // fold: window occupied
    const umbreon::OcclusionQuery query = [&](const umbreon::Vec3& p,
                                              const umbreon::Vec3& q,
                                              const int*, int) {
      calls.fetch_add(1, std::memory_order_relaxed);
      // Every row probes the same geometry, so these race benignly to the
      // same values under TBB.
      vz0 = -p.z;  // depth along dir = (0,0,-1) from pos (0,0,0)
      vz1 = -q.z;
      return hit;
    };
    CrackField cf = umbreon::classifyCracks(
        16, 16, b.viewZ.data(), b.objectId.data(), b.normal.data(), sp,
        defaults, nullptr, &query);
    s.check_eq("fold probe: probed once per row", calls.load(), 16);
    s.check("fold probe: window start = vzFar - frac*dz",
            std::fabs(vz0 - (130.0f - 7.5f)) < 1.0e-3f);
    s.check("fold probe: window end just above the far surface",
            std::fabs(vz1 - (130.0f - 0.5f)) < 1.0e-3f);
    s.check_eq("fold probe: hit -> veto, weak DepthGap per row",
               countClass(cf, CrackClass::DepthGap), 16);
    int strongN = 0;
    for (std::uint8_t v : cf.right)
      if ((v & kCrackClassMask) ==
              static_cast<std::uint8_t>(CrackClass::DepthGap) &&
          (v & kCrackStrongBit))
        ++strongN;
    s.check_eq("fold probe: hit -> no strong crack", strongN, 0);

    // Same scene, empty window -> genuine contour, the rescue holds.
    hit = false;
    cf = umbreon::classifyCracks(16, 16, b.viewZ.data(), b.objectId.data(),
                                 b.normal.data(), sp, defaults, nullptr,
                                 &query);
    strongN = 0;
    for (std::uint8_t v : cf.right)
      if ((v & kCrackStrongBit)) ++strongN;
    s.check_eq("fold probe: miss -> rescued strong per row", strongN, 16);

    // No probe bound (tests / callers without a BVH): pre-probe behavior,
    // the rescue fires (fail-open).
    cf = umbreon::classifyCracks(16, 16, b.viewZ.data(), b.objectId.data(),
                                 b.normal.data(), sp, defaults);
    strongN = 0;
    for (std::uint8_t v : cf.right)
      if ((v & kCrackStrongBit)) ++strongN;
    s.check_eq("fold probe: unbound -> rescue fail-open", strongN, 16);
  }

  // ---- (13) junction-chop keep: entangled strong contour survives prune ---
  // A strong vertical contour crossed by weak rungs every 2 rows is chopped
  // into 2-edgel fragments, none reaching minStrong on its own. The interior
  // fragments (both ends junctions) must survive the prune, re-merge on the
  // retrace and pass the strong test as ONE chain; the pure-weak rungs and
  // the free-ended terminal stubs are erased. Without the junction-chop keep
  // the WHOLE continuously-strong line vanished.
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        // Strong step 400 over flat sides (dominates), plus a weak rung step
        // of 8 (above the weak threshold 6, below the full 12) every 2 rows.
        const int rung = y / 2;
        float vz = x < 8 ? 100.0f : 500.0f;
        vz += 8.0f * static_cast<float>(rung);
        b.set(x, y, 9, vz);
      }
    CrackField cf = classify(b, defaults);
    int strongRaw = 0;
    for (std::uint8_t v : cf.right)
      if (v & kCrackStrongBit) ++strongRaw;
    s.check_eq("junction chop: 16 strong cracks classified", strongRaw, 16);
    std::vector<ScreenChain> chains =
        umbreon::traceCrackChains(cf, b.viewZ.data(), b.objectId.data());
    chains = umbreon::pruneWeakChains(cf, std::move(chains), b.viewZ.data(),
                                      b.objectId.data(), 4);
    int strongKept = 0, weakKept = 0;
    std::size_t bestLen = 0;
    for (const ScreenChain& ch : chains) {
      std::size_t st = 0;
      for (std::size_t e = 0; e < ch.edgeClass.size(); ++e) {
        if (ch.edgeClass[e] !=
            static_cast<std::uint8_t>(CrackClass::DepthGap))
          continue;
        if (e < ch.edgeFlags.size() && (ch.edgeFlags[e] & 1)) {
          ++strongKept;
          ++st;
        } else {
          ++weakKept;
        }
      }
      bestLen = std::max(bestLen, st);
    }
    s.check("junction chop: interior strong body survives as one chain",
            bestLen >= 12);
    s.check_eq("junction chop: kept strong = merged interior", strongKept,
               static_cast<int>(bestLen));
    s.check_eq("junction chop: pure-weak rungs erased", weakKept, 0);
  }

  // ---- (14) clip-cut vetoes: cut boundaries classify as nothing ----------
  // Interior veto: a crack touching a clip-cut interior pixel (renderer
  // clipCut flag) never inks, regardless of class.
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, 9, x < 8 ? 100.0f : 500.0f);  // strong step
    std::vector<std::uint8_t> cut(16 * 16, 0);
    for (int y = 0; y < 16; ++y)
      for (int x = 8; x < 16; ++x) cut[b.idx(x, y)] = 1;  // deep side is cut
    umbreon::ScreenClipAovs clip;
    clip.cut = cut.data();
    CrackField cf = umbreon::classifyCracks(
        16, 16, b.viewZ.data(), b.objectId.data(), b.normal.data(),
        unitProj(16, 16), defaults, nullptr, nullptr, &clip);
    s.check_eq("clip interior: step crack fully vetoed", countActive(cf), 0);
    // Without the clip AOVs the same field inks the step (control).
    s.check_eq("clip interior: control still fires",
               countClass(classify(b, defaults), CrackClass::DepthGap), 16);
  }
  // Silhouette veto: the outline is a far-plane cut when the removed hit
  // recorded behind the bg pixel continues the fg surface's depth; an
  // unrelated removed hit keeps the outline.
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 8; ++x)
        b.set(x, y, 9, 100.0f + 5.0f * static_cast<float>(x));  // slope 5/px
    // fg edge pixel x=7: vz 135, one-sided slope 5 -> prediction 140.
    std::vector<float> farVz(16 * 16, 0.0f);
    for (int y = 0; y < 16; ++y)
      for (int x = 8; x < 16; ++x) farVz[b.idx(x, y)] = 140.0f;
    umbreon::ScreenClipAovs clip;
    clip.farVz = farVz.data();
    CrackField cf = umbreon::classifyCracks(
        16, 16, b.viewZ.data(), b.objectId.data(), b.normal.data(),
        unitProj(16, 16), defaults, nullptr, nullptr, &clip);
    s.check_eq("clip silhouette: depth-continuous cut edge vetoed",
               countClass(cf, CrackClass::Silhouette), 0);
    for (float& v : farVz) v = v > 0.0f ? 300.0f : 0.0f;  // unrelated object
    cf = umbreon::classifyCracks(16, 16, b.viewZ.data(), b.objectId.data(),
                                 b.normal.data(), unitProj(16, 16), defaults,
                                 nullptr, nullptr, &clip);
    s.check_eq("clip silhouette: unrelated removed hit keeps the outline",
               countClass(cf, CrackClass::Silhouette), 16);
  }
  // fg|fg far-plane veto: the DEEPER side's continuation exits the slab
  // across the crack -> slab artifact; with the plane off the step inks.
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        // near side flat 100; deep side 500 at the crack, receding away at
        // 10/px -> continuation across the crack = 510.
        const float vz =
            x < 8 ? 100.0f : 500.0f - 10.0f * static_cast<float>(x - 8);
        b.set(x, y, 9, vz);
      }
    std::vector<std::uint8_t> cut(16 * 16, 0);  // no interior flags
    umbreon::ScreenClipAovs clip;
    clip.cut = cut.data();
    ScreenClassifyParams p = defaults;
    p.clipFarVz = 505.0f;  // continuation 510 exits the slab
    CrackField cf = umbreon::classifyCracks(
        16, 16, b.viewZ.data(), b.objectId.data(), b.normal.data(),
        unitProj(16, 16), p, nullptr, nullptr, &clip);
    s.check_eq("clip fg|fg: far-cut step vetoed", countActive(cf), 0);
    p.clipFarVz = std::numeric_limits<float>::infinity();
    cf = umbreon::classifyCracks(16, 16, b.viewZ.data(), b.objectId.data(),
                                 b.normal.data(), unitProj(16, 16), p,
                                 nullptr, nullptr, &clip);
    s.check("clip fg|fg: plane off restores the contour",
            countClass(cf, CrackClass::DepthGap) > 0);
  }

  // ---- (15) ridge crease: never STRONG, weak carries the ridge flag ------
  // A convex ridge (both one-sided slopes fall away from the crack) is a
  // connected-surface crease: the depth step comes from the two faces'
  // slopes, not an occlusion. The crack profile is taken from the real case
  // (a sheet's edge fold): shallow flanks falling away on both sides, a
  // 17-unit step across the crease, normals ~90 deg apart so the ndelta
  // rescue WOULD fire -- the ridge test must block the promotion while the
  // crack still inks weak (hysteresis continuity) with edgeFlags bit 1 set
  // (the prune's strong-chain exemption does not keep ridge tails).
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) {
        if (x <= 7)
          b.set(x, y, 9, 200.0f + 0.81f * static_cast<float>(7 - x), 0.7f,
                0.0f, 0.72f);
        else
          b.set(x, y, 9, 217.0f + 1.04f * static_cast<float>(x - 8), -0.7f,
                0.0f, 0.72f);
      }
    CrackField cf = classify(b, defaults);
    int strongN = 0, ridgeWeak = 0;
    for (std::uint8_t v : cf.right) {
      if ((v & kCrackClassMask) !=
          static_cast<std::uint8_t>(CrackClass::DepthGap))
        continue;
      if (v & kCrackStrongBit) ++strongN;
      else if (v & umbreon::kCrackRidgeBit) ++ridgeWeak;
    }
    s.check_eq("ridge: never promotes to strong", strongN, 0);
    s.check_eq("ridge: weak cracks carry the ridge flag", ridgeWeak, 16);
    // Traced in isolation the pure-ridge chain has no support: pruned.
    std::vector<ScreenChain> chains =
        umbreon::traceCrackChains(cf, b.viewZ.data(), b.objectId.data());
    chains = umbreon::pruneWeakChains(cf, std::move(chains), b.viewZ.data(),
                                      b.objectId.data());
    s.check_eq("ridge: unsupported ridge chain pruned", chains.size(),
               static_cast<std::size_t>(0));
  }

  // ---- (16) short-run relabel operates on the (class, group) PAIR ---------
  // Flicker inside one section still fuses; a genuine section change never
  // does (fusing it would draw one section's contour with the other's
  // style), and sub-threshold owner-group jitter on an interpenetrating
  // boundary collapses to the bracketing group.
  {
    const std::uint8_t S = static_cast<std::uint8_t>(CrackClass::Silhouette);
    const std::uint8_t D = static_cast<std::uint8_t>(CrackClass::DepthGap);
    const std::uint8_t O = static_cast<std::uint8_t>(CrackClass::ObjectId);
    {
      std::vector<std::uint8_t> cls = {S, S, D, S, S};
      std::vector<std::uint16_t> grp = {7, 7, 7, 7, 7};
      umbreon::mergeShortClassRuns(cls, grp, 2);
      s.check("run merge: same-group class flicker fuses",
              cls == std::vector<std::uint8_t>({S, S, S, S, S}));
    }
    {
      std::vector<std::uint8_t> cls = {S, S, D, S, S};
      std::vector<std::uint16_t> grp = {1, 1, 1, 2, 2};
      const std::vector<std::uint8_t> cls0 = cls;
      const std::vector<std::uint16_t> grp0 = grp;
      umbreon::mergeShortClassRuns(cls, grp, 2);
      s.check("run merge: cross-group brackets refuse to fuse",
              cls == cls0 && grp == grp0);
    }
    {
      std::vector<std::uint8_t> cls = {O, O, O, O, O};
      std::vector<std::uint16_t> grp = {1, 1, 2, 1, 1};
      umbreon::mergeShortClassRuns(cls, grp, 2);
      s.check("run merge: owner-group jitter collapses",
              grp == std::vector<std::uint16_t>({1, 1, 1, 1, 1}));
    }
    {
      std::vector<std::uint8_t> cls = {S, D, S};
      std::vector<std::uint16_t> grp = {1, 2, 2};
      const std::vector<std::uint8_t> cls0 = cls;
      umbreon::mergeShortClassRuns(cls, grp, 2);
      s.check("run merge: class jitter across a group change refused",
              cls == cls0 && grp == std::vector<std::uint16_t>({1, 2, 2}));
    }
  }

  // ---- (17) per-(class, group) runs: each section draws its OWN style -----
  // Two same-depth touching rectangles of different sections: the boundary
  // between them is depth-continuous contact (never inked, locked by (5)),
  // so their shared outer silhouette traces as ONE closed loop whose owner
  // group flips at the degree-2 touch corners. The Stage-4 splitter must cut
  // the loop at the group changes so each section's rim draws with its own
  // color and width (previously the whole loop took the first edgel's group).
  {
    const int W = 48, H = 32;
    umbreon::FrameResult frame;
    frame.width = W;
    frame.height = H;
    frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
    frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
    frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
    for (int y = 8; y < 24; ++y)
      for (int x = 4; x < 44; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * W + x;
        frame.objectId[i] = (x < 24 ? 1u : 2u) << 2;
        frame.viewZ[i] = 50.0f;
      }
    umbreon::Scene scene;
    scene.camera.position = {0.0f, 0.0f, 100.0f};
    scene.camera.direction = {0.0f, 0.0f, -1.0f};
    scene.camera.up = {0.0f, 1.0f, 0.0f};
    scene.camera.orthographic = true;
    scene.camera.height = static_cast<float>(H);  // pixelSize == 1
    scene.background = {1.0f, 1.0f, 1.0f};
    scene.groupEdgeStyle.assign(3, umbreon::EdgeStyle{});
    auto seedSil = [&](int g, float r, float gc, float bc, float w) {
      umbreon::EdgeClassStyle& cs =
          scene.groupEdgeStyle[g]
              .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
      cs.enabled = true;
      cs.color[0] = r;
      cs.color[1] = gc;
      cs.color[2] = bc;
      cs.width = w;
    };
    seedSil(1, 1.0f, 0.0f, 0.0f, 2.0f);  // section 1: red, half-width 1
    seedSil(2, 0.0f, 0.0f, 1.0f, 6.0f);  // section 2: blue, half-width 3
    umbreon::RenderOptions opt;
    opt.width = W;
    opt.height = H;
    opt.supersample = 1;
    opt.strokeEdges.enable = true;
    opt.strokeEdges.edgesOnly = true;  // full-opacity ink over the blank bg
    umbreon::applyScreenVectorEdges(frame, scene, opt);

    auto chan = [&](int x, int y, int c) {
      return frame.color[(static_cast<std::size_t>(y) * W + x) * 4 + c];
    };
    // Window scans over each section's top rim (rim line y = 7.5).
    bool redOn1 = false, blueOn1 = false, blueOn2 = false, redOn2 = false;
    for (int y = 6; y <= 10; ++y)
      for (int x = 6; x <= 20; ++x) {
        if (chan(x, y, 0) - chan(x, y, 2) > 0.5f) redOn1 = true;
        if (chan(x, y, 2) - chan(x, y, 0) > 0.5f) blueOn1 = true;
      }
    for (int y = 6; y <= 10; ++y)
      for (int x = 28; x <= 42; ++x) {
        if (chan(x, y, 2) - chan(x, y, 0) > 0.5f) blueOn2 = true;
        if (chan(x, y, 0) - chan(x, y, 2) > 0.5f) redOn2 = true;
      }
    s.check("group split: section 1 rim inks red", redOn1);
    s.check("group split: no blue leaks onto section 1 rim", !blueOn1);
    s.check("group split: section 2 rim inks blue", blueOn2);
    s.check("group split: no red leaks onto section 2 rim", !redOn2);
    // Width: 2.5 px above the rim line only the wide (section 2) stroke inks.
    bool wideAbove = false, narrowAbove = false;
    for (int x = 28; x <= 42; ++x)
      if (chan(x, 5, 2) - chan(x, 5, 0) > 0.5f) wideAbove = true;
    for (int x = 6; x <= 20; ++x)
      if (chan(x, 5, 0) < 0.9f) narrowAbove = true;
    s.check("group split: wide section inks 2.5 px off the rim", wideAbove);
    s.check("group split: narrow section does not", !narrowAbove);
  }

  // ---- (18) per-run vz re-attribution: fog must not leak across a group ---
  // change. A vertical ObjectId boundary whose nearer-pixel owner flips
  // mid-line (top: section 1 at vz 100, heavily fogged; bottom: section 2 at
  // vz 20, unfogged). The far section's internal depth step is suppressed
  // with Outline mode so the flip corner stays degree 2 and the boundary
  // traces as ONE chain. Without the per-run split + vz re-attribution the
  // collapsed straight line lerps vz 100 -> 20 end to end and the near
  // (unfogged) half inks visibly fog-colored.
  {
    const int W = 32, H = 32;
    umbreon::FrameResult frame;
    frame.width = W;
    frame.height = H;
    frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
    frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
    frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * W + x;
        if (x < 16) {
          frame.objectId[i] = 1u << 2;
          frame.viewZ[i] = 100.0f;
        } else {
          frame.objectId[i] = 2u << 2;
          frame.viewZ[i] = y < 16 ? 500.0f : 20.0f;
        }
      }
    umbreon::Scene scene;
    scene.camera.position = {0.0f, 0.0f, 100.0f};
    scene.camera.direction = {0.0f, 0.0f, -1.0f};
    scene.camera.up = {0.0f, 1.0f, 0.0f};
    scene.camera.orthographic = true;
    scene.camera.height = static_cast<float>(H);  // pixelSize == 1
    scene.fog.enabled = true;
    scene.fog.color = {1.0f, 0.0f, 0.0f};  // red fog isolates the vz path
    scene.fog.start = 20.0f;               // vz 20 -> f=1 (pure ink)
    scene.fog.end = 110.0f;                // vz 100 -> f~0.11 (mostly fog)
    scene.groupEdgeStyle.assign(3, umbreon::EdgeStyle{});
    for (int g : {1, 2}) {
      umbreon::EdgeClassStyle& cs =
          scene.groupEdgeStyle[g]
              .cls[static_cast<int>(umbreon::EdgeClass::Object)];
      cs.enabled = true;  // black, opacity 1, width 2 (identical styles)
    }
    // Suppress section 2's internal 500|20 step so the owner-flip corner
    // stays degree 2 (one chain through it).
    scene.groupEdgeStyle[2].silhouetteMode = umbreon::SilhouetteMode::Outline;
    // Below the flip section 2 is the near side, so the boundary promotes to
    // its Silhouette class; enable that slot with the same default style.
    scene.groupEdgeStyle[2]
        .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)]
        .enabled = true;
    umbreon::RenderOptions opt;
    opt.width = W;
    opt.height = H;
    opt.supersample = 1;
    opt.strokeEdges.enable = true;  // NOT edgesOnly: FogShader must run
    umbreon::applyScreenVectorEdges(frame, scene, opt);

    auto chan = [&](int x, int y, int c) {
      return frame.color[(static_cast<std::size_t>(y) * W + x) * 4 + c];
    };
    // Below the flip (owner vz 20, f=1): every inked pixel stays pure black.
    bool inked = false, leaked = false;
    for (int y = 19; y <= 22; ++y)
      for (int x = 13; x <= 18; ++x) {
        if (chan(x, y, 1) > 0.5f) continue;  // uninked (white bg)
        inked = true;
        if (chan(x, y, 0) > 0.1f) leaked = true;  // red fog on the near run
      }
    s.check("vz split: near run inks below the flip", inked);
    s.check("vz split: no fog leak onto the near run", !leaked);
    // Above the flip (owner vz 100): the far run IS fog-tinted (sanity).
    bool fogged = false;
    for (int y = 8; y <= 12; ++y)
      for (int x = 13; x <= 18; ++x)
        if (chan(x, y, 1) < 0.5f && chan(x, y, 0) > 0.3f) fogged = true;
    s.check("vz split: far run carries the fog tint", fogged);
  }

  // ---- (19) owner view-z discontinuity splits a run WITHIN one section ----
  // Two touching rectangles of ONE section at very different depths, with
  // Outline mode suppressing the internal same-id step: the union silhouette
  // walks through the touch corner as one chain with uniform class AND
  // group, but the owner depth jumps near -> far there (a near strand's
  // contour continuing into a far strand's, e.g. a coil in front of a
  // fogged loop of the same CueMol section). The jump exceeds the slope
  // clamp (no surface slopes that fast), so the run must split: without the
  // split, collinear collapse reduces the straight top rim to two vertices
  // and the draw stage lerps the depth (and thus the fog color) across the
  // whole rim, fogging the near half.
  {
    const int W = 48, H = 32;
    umbreon::FrameResult frame;
    frame.width = W;
    frame.height = H;
    frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
    frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
    frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
    for (int y = 8; y < 24; ++y)
      for (int x = 4; x < 44; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * W + x;
        frame.objectId[i] = 1u << 2;              // ONE section throughout
        frame.viewZ[i] = x < 24 ? 20.0f : 500.0f;  // jump 480 > clamp 300
      }
    umbreon::Scene scene;
    scene.camera.position = {0.0f, 0.0f, 100.0f};
    scene.camera.direction = {0.0f, 0.0f, -1.0f};
    scene.camera.up = {0.0f, 1.0f, 0.0f};
    scene.camera.orthographic = true;
    scene.camera.height = static_cast<float>(H);  // pixelSize == 1
    scene.fog.enabled = true;
    scene.fog.color = {1.0f, 0.0f, 0.0f};  // red fog isolates the vz path
    scene.fog.start = 20.0f;               // near rim -> f=1 (pure ink)
    scene.fog.end = 110.0f;                // far rim (500) -> f=0 (all fog)
    scene.groupEdgeStyle.assign(2, umbreon::EdgeStyle{});
    umbreon::EdgeClassStyle& cs =
        scene.groupEdgeStyle[1]
            .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
    cs.enabled = true;  // black, opacity 1, width 2
    // Suppress the internal 20|500 same-id step so the touch corner stays
    // degree 2 (the silhouette loop walks through it).
    scene.groupEdgeStyle[1].silhouetteMode = umbreon::SilhouetteMode::Outline;
    umbreon::RenderOptions opt;
    opt.width = W;
    opt.height = H;
    opt.supersample = 1;
    opt.strokeEdges.enable = true;  // NOT edgesOnly: FogShader must run
    umbreon::applyScreenVectorEdges(frame, scene, opt);

    auto chan = [&](int x, int y, int c) {
      return frame.color[(static_cast<std::size_t>(y) * W + x) * 4 + c];
    };
    // Near half of the top rim: inked and pure black (no red fog leak).
    bool inked = false, leaked = false;
    for (int y = 6; y <= 9; ++y)
      for (int x = 6; x <= 18; ++x) {
        if (chan(x, y, 1) > 0.5f) continue;  // uninked
        inked = true;
        if (chan(x, y, 0) > 0.1f) leaked = true;
      }
    s.check("vz jump split: near rim inks", inked);
    s.check("vz jump split: no fog leak onto the near rim", !leaked);
    // Far half of the top rim: fog-tinted (sanity that fog is live).
    bool fogged = false;
    for (int y = 6; y <= 9; ++y)
      for (int x = 30; x <= 42; ++x)
        if (chan(x, y, 1) < 0.5f && chan(x, y, 0) > 0.3f) fogged = true;
    s.check("vz jump split: far rim carries the fog tint", fogged);
  }

  // ---- (20) outside stroke alignment (--stroke-align) ---------------------
  // StrokeAlign::Outside (the default) puts the full stroke width on the
  // outer (occluded / background) side of every occlusion contour --
  // Silhouette, ObjectId and DepthGap -- (a thin inner pad remains) so a
  // thick line never thins the object whose contour it draws; Center
  // restores the legacy symmetric ribbon. Per section via EdgeStyle::align;
  // contact runs and Crease always centered.

  // (20a) walkChain side bit: bit 3 of edgeFlags marks "outer side on the
  // walk-direction left", which for a convex region must equal the geometric
  // test "left normal points away from the region center" on every edgel.
  {
    Buffers b(16, 16);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x) b.set(x, y, 7, 10.0f);
    CrackField cf = classify(b, defaults);
    auto chains = umbreon::traceCrackChains(cf, b.viewZ.data(),
                                            b.objectId.data());
    bool one = chains.size() == 1 && chains[0].closed;
    s.check("outside side bit: square traces one closed loop", one);
    if (one) {
      const ScreenChain& ch = chains[0];
      bool sideOk = ch.edgeFlags.size() == ch.edgeClass.size();
      for (std::size_t k = 0; sideOk && k < ch.edgeFlags.size(); ++k) {
        const float dx = ch.pts[k + 1].x - ch.pts[k].x;
        const float dy = ch.pts[k + 1].y - ch.pts[k].y;
        const float mx = 0.5f * (ch.pts[k].x + ch.pts[k + 1].x) - 7.5f;
        const float my = 0.5f * (ch.pts[k].y + ch.pts[k + 1].y) - 7.5f;
        // left normal = orth(d) = (-dy, dx); outward iff it points away
        // from the square center (7.5, 7.5).
        const bool outLeft = (-dy) * mx + dx * my > 0.0f;
        if (((ch.edgeFlags[k] & 8) != 0) != outLeft) sideOk = false;
      }
      s.check("outside side bit: bit 3 matches the outward normal on every "
              "edgel",
              sideOk);
    }
  }

  // (20b) contact bit propagation: a depth-continuous cross-section contact
  // crack carries kCrackContactBit into edgeFlags bit 2 (the alignment vote
  // must skip these edgels -- no outer side exists at a contact).
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, x < 8 ? (1u << 2) : (2u << 2), 10.0f);
    ScreenClassifyParams p = defaults;
    p.contactBoundary = true;
    CrackField cf = classify(b, p);
    auto chains = umbreon::traceCrackChains(cf, b.viewZ.data(),
                                            b.objectId.data());
    bool allContact = !chains.empty();
    for (const ScreenChain& ch : chains)
      for (std::size_t k = 0; k < ch.edgeFlags.size(); ++k)
        if (!(ch.edgeFlags[k] & 4)) allContact = false;
    s.check("contact bit: every contact edgel carries edgeFlags bit 2",
            allContact);
    // The silhouette square from (20a) must NOT carry it.
    Buffers b2(16, 16);
    for (int y = 4; y < 12; ++y)
      for (int x = 4; x < 12; ++x) b2.set(x, y, 7, 10.0f);
    CrackField cf2 = classify(b2, defaults);
    auto chains2 = umbreon::traceCrackChains(cf2, b2.viewZ.data(),
                                             b2.objectId.data());
    bool noContact = !chains2.empty();
    for (const ScreenChain& ch : chains2)
      for (std::size_t k = 0; k < ch.edgeFlags.size(); ++k)
        if (ch.edgeFlags[k] & 4) noContact = false;
    s.check("contact bit: a background silhouette carries none", noContact);
  }

  // (20c) draw stage: a nonzero StrokeChainInput::outsideSide shifts the
  // resolved width to that side (left = +normal = raster +y for a +x chain),
  // keeping a thin pad on the other; the round cap survives the one-sided
  // width (the fan radius lerps outer -> pad, never zero).
  {
    auto renderAligned = [&](std::int8_t side, bool roundCap) {
      umbreon::FrameResult fr;
      fr.width = 40;
      fr.height = 32;
      fr.color.assign(static_cast<std::size_t>(40) * 32 * 4, 1.0f);
      umbreon::Scene scene;
      umbreon::RenderOptions opt;
      opt.width = 40;
      opt.height = 32;
      opt.supersample = 1;
      opt.strokeEdges.enable = true;
      opt.strokeEdges.thickness = 6;  // half-width 3
      opt.strokeEdges.roundCap = roundCap;
      std::vector<umbreon::StrokeChainInput> chain(1);
      chain[0].pts = {{8.5f, 16.5f, 10.0f, 1.0f, true},
                      {24.5f, 16.5f, 10.0f, 1.0f, true}};
      chain[0].outsideSide = side;
      umbreon::renderStrokeChains(fr, scene, opt, chain);
      return fr;
    };
    auto lumAt = [](const umbreon::FrameResult& fr, int x, int y) {
      return fr.color[(static_cast<std::size_t>(y) * fr.width + x) * 4];
    };
    // Centered (side 0): band y in [13.5, 19.5].
    const umbreon::FrameResult center = renderAligned(0, false);
    s.check("align draw: centered inks both sides",
            lumAt(center, 16, 14) < 0.1f && lumAt(center, 16, 19) < 0.1f);
    s.check("align draw: centered stays inside its band",
            lumAt(center, 16, 20) > 0.9f);
    // Left-outside (+1): band y in [16.0, 22.0] -- the +y (left) side.
    const umbreon::FrameResult left = renderAligned(1, false);
    s.check("align draw: outside-left inks the +normal side",
            lumAt(left, 16, 20) < 0.1f);
    s.check("align draw: outside-left leaves the -normal side blank",
            lumAt(left, 16, 14) > 0.9f);
    // Right-outside (-1): band y in [11.0, 17.0].
    const umbreon::FrameResult right = renderAligned(-1, false);
    s.check("align draw: outside-right inks the -normal side",
            lumAt(right, 16, 12) < 0.1f);
    s.check("align draw: outside-right leaves the +normal side blank",
            lumAt(right, 16, 20) > 0.9f);
    // Round cap: the one-sided stroke still fans beyond the butt end.
    const umbreon::FrameResult capped = renderAligned(1, true);
    bool beyond = false;
    for (int y = 10; y <= 23 && !beyond; ++y)
      for (int x = 26; x <= 30 && !beyond; ++x)
        if (lumAt(capped, x, y) < 0.1f) beyond = true;
    s.check("align draw: round cap survives the one-sided width", beyond);
    const umbreon::FrameResult butt = renderAligned(1, false);
    bool buttBeyond = false;
    for (int y = 10; y <= 23 && !buttBeyond; ++y)
      for (int x = 26; x <= 30 && !buttBeyond; ++x)
        if (lumAt(butt, x, y) < 0.1f) buttBeyond = true;
    s.check("align draw: butt cap still ends at the endpoint", !buttBeyond);
  }

  // (20g) junction taper + fold re-centering (draw stage). A flagged end
  // blends the offset band back to the symmetric ribbon over one stroke
  // width, so the ribbon arrives centered where it meets other lines; a
  // backbone that doubles back within a stroke width (a hairpin around a
  // narrow wedge / notch) re-centers around the fold, so the one-sided
  // band cannot spur out past the meeting lines.
  {
    auto render = [&](std::vector<umbreon::StrokePoint> pts, bool taperEnd) {
      umbreon::FrameResult fr;
      fr.width = 48;
      fr.height = 32;
      fr.color.assign(static_cast<std::size_t>(48) * 32 * 4, 1.0f);
      umbreon::Scene scene;
      umbreon::RenderOptions opt;
      opt.width = 48;
      opt.height = 32;
      opt.supersample = 1;
      opt.strokeEdges.enable = true;
      opt.strokeEdges.thickness = 6;  // half 3, pad 0.5, outer 5.5
      std::vector<umbreon::StrokeChainInput> chain(1);
      chain[0].pts = std::move(pts);
      chain[0].outsideSide = 1;
      chain[0].taperEnd = taperEnd;
      umbreon::renderStrokeChains(fr, scene, opt, chain);
      return fr;
    };
    auto lumAt = [](const umbreon::FrameResult& fr, int x, int y) {
      return fr.color[(static_cast<std::size_t>(y) * fr.width + x) * 4];
    };
    // Straight chain, taperEnd: mid-chain keeps the full offset band
    // [16.0, 22.0]; the flagged end arrives centered (~[13.5, 19.5]).
    const std::vector<umbreon::StrokePoint> line = {
        {8.5f, 16.5f, 10.0f, 1.0f, true}, {40.5f, 16.5f, 10.0f, 1.0f, true}};
    const umbreon::FrameResult tap = render(line, true);
    s.check("junction taper: mid-chain keeps the offset band",
            lumAt(tap, 20, 20) < 0.1f && lumAt(tap, 20, 14) > 0.9f);
    s.check("junction taper: flagged end re-centers (near side inks)",
            lumAt(tap, 40, 14) < 0.1f);
    s.check("junction taper: flagged end re-centers (offset side shrinks)",
            lumAt(tap, 40, 21) > 0.9f);
    const umbreon::FrameResult noTap = render(line, false);
    s.check("junction taper: unflagged end keeps the offset",
            lumAt(noTap, 40, 14) > 0.9f && lumAt(noTap, 40, 21) < 0.1f);
    // Hairpin: east along y=10.5, back west along y=12.5. The fold at
    // x=30.5 re-centers the band, so no spur inks east of the turn (an
    // un-centered one-sided band would miter/spike well past it); the legs
    // away from the fold keep their offset bands.
    const std::vector<umbreon::StrokePoint> hairpin = {
        {10.5f, 10.5f, 10.0f, 1.0f, true},
        {30.5f, 10.5f, 10.0f, 1.0f, true},
        {30.5f, 12.5f, 10.0f, 1.0f, true},
        {10.5f, 12.5f, 10.0f, 1.0f, true}};
    const umbreon::FrameResult fold = render(hairpin, false);
    bool spur = false;
    for (int y = 4; y <= 20 && !spur; ++y)
      for (int x = 38; x <= 46 && !spur; ++x)
        if (lumAt(fold, x, y) < 0.5f) spur = true;
    s.check("fold re-center: no spur past the hairpin", !spur);
    s.check("fold re-center: legs still ink away from the fold",
            lumAt(fold, 16, 12) < 0.1f && lumAt(fold, 16, 8) < 0.1f);
  }

  // (20h) junction quality under the outside alignment: notch excision, bar
  // continuity through a T junction, cap suppression at tapered ends, and
  // the free-end connection probe.
  {
    auto lumAt = [](const umbreon::FrameResult& fr, int x, int y) {
      return fr.color[(static_cast<std::size_t>(y) * fr.width + x) * 4];
    };
    auto makeScene = [&](int W, int H) {
      umbreon::Scene scene;
      scene.camera.position = {0.0f, 0.0f, 500.0f};
      scene.camera.direction = {0.0f, 0.0f, -1.0f};
      scene.camera.up = {0.0f, 1.0f, 0.0f};
      scene.camera.orthographic = true;
      scene.camera.height = static_cast<float>(H);  // pixelSize == 1
      scene.background = {1.0f, 1.0f, 1.0f};
      return scene;
    };
    auto makeOpt = [&](int W, int H, umbreon::StrokeAlign align) {
      umbreon::RenderOptions opt;
      opt.width = W;
      opt.height = H;
      opt.supersample = 1;
      opt.strokeEdges.enable = true;
      opt.strokeEdges.edgesOnly = true;
      opt.strokeEdges.thickness = 6;
      opt.strokeEdges.align = align;
      return opt;
    };

    // (20h-1) notch excision: a 2 px wide, 4 px deep background notch in a
    // rectangle's top rim. Outside: the sub-width detour is bridged, the
    // rim draws straight and the notch interior stays clean; Center keeps
    // the legacy detour (the gate holds).
    auto renderNotch = [&](umbreon::StrokeAlign align) {
      const int W = 32, H = 32;
      umbreon::FrameResult frame;
      frame.width = W;
      frame.height = H;
      frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
      frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
      frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
      for (int y = 8; y < 24; ++y)
        for (int x = 8; x < 24; ++x) {
          if (y < 12 && x >= 15 && x < 17) continue;  // the notch (bg)
          const std::size_t i = static_cast<std::size_t>(y) * W + x;
          frame.objectId[i] = 1u << 2;
          frame.viewZ[i] = 50.0f;
        }
      umbreon::Scene scene = makeScene(W, H);
      umbreon::RenderOptions opt = makeOpt(W, H, align);
      umbreon::applyScreenVectorEdges(frame, scene, opt);
      return frame;
    };
    const umbreon::FrameResult notchOut =
        renderNotch(umbreon::StrokeAlign::Outside);
    s.check("notch bridge: rim inks straight across the notch",
            lumAt(notchOut, 15, 3) < 0.1f && lumAt(notchOut, 16, 3) < 0.1f);
    s.check("notch bridge: the notch interior stays clean",
            lumAt(notchOut, 15, 10) > 0.9f && lumAt(notchOut, 16, 10) > 0.9f);
    const umbreon::FrameResult notchCen =
        renderNotch(umbreon::StrokeAlign::Center);
    s.check("notch bridge: center keeps the legacy detour",
            lumAt(notchCen, 16, 10) < 0.5f);

    // (20h-2) bar continuity: a stem T-ing into a straight rim must not
    // kink the rim -- the two rim chains meeting at the junction corner
    // continue each other (passThrough) and keep the full offset band.
    // Near square over a far same-section square (strong step): rim y=23.5,
    // stem x=15.5; outside band above the rim = [18.0, 24.0].
    {
      const int W = 56, H = 56;
      umbreon::FrameResult frame;
      frame.width = W;
      frame.height = H;
      frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
      frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
      frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
      auto put = [&](int x, int y, float vz) {
        const std::size_t i = static_cast<std::size_t>(y) * W + x;
        frame.objectId[i] = 1u << 2;
        frame.viewZ[i] = vz;
      };
      for (int y = 8; y < 32; ++y)
        for (int x = 16; x < 48; ++x) put(x, y, 400.0f);  // far
      for (int y = 24; y < 48; ++y)
        for (int x = 8; x < 40; ++x) put(x, y, 10.0f);  // near, on top
      umbreon::Scene scene = makeScene(W, H);
      umbreon::RenderOptions opt =
          makeOpt(W, H, umbreon::StrokeAlign::Outside);
      umbreon::applyScreenVectorEdges(frame, scene, opt);
      s.check("bar continuity: full offset band at the junction x",
              lumAt(frame, 15, 19) < 0.1f);
      s.check("bar continuity: no centered dip below the rim",
              lumAt(frame, 15, 26) > 0.9f);
      // The stem (the far square's left silhouette, band west of x=15.5)
      // is clipped at the rim's far ink edge: nothing pokes below the rim
      // even though the drawn stem extends into the junction.
      s.check("stem clip: no stem ink below the rim band",
              lumAt(frame, 12, 26) > 0.9f && lumAt(frame, 12, 28) > 0.9f);
    }

    // (20h-3) cap suppression: a round cap at a junction-tapered end would
    // poke past the line it meets; the tapered end draws a butt instead.
    {
      auto renderCap = [&](bool taperEnd) {
        umbreon::FrameResult fr;
        fr.width = 40;
        fr.height = 32;
        fr.color.assign(static_cast<std::size_t>(40) * 32 * 4, 1.0f);
        umbreon::Scene scene;
        umbreon::RenderOptions opt;
        opt.width = 40;
        opt.height = 32;
        opt.supersample = 1;
        opt.strokeEdges.enable = true;
        opt.strokeEdges.thickness = 6;
        opt.strokeEdges.roundCap = true;
        std::vector<umbreon::StrokeChainInput> chain(1);
        chain[0].pts = {{8.5f, 16.5f, 10.0f, 1.0f, true},
                        {24.5f, 16.5f, 10.0f, 1.0f, true}};
        chain[0].outsideSide = 1;
        chain[0].taperEnd = taperEnd;
        umbreon::renderStrokeChains(fr, scene, opt, chain);
        return fr;
      };
      auto anyBeyond = [&](const umbreon::FrameResult& fr) {
        for (int y = 10; y <= 23; ++y)
          for (int x = 26; x <= 30; ++x)
            if (lumAt(fr, x, y) < 0.5f) return true;
        return false;
      };
      s.check("cap suppression: tapered end draws no cap",
              !anyBeyond(renderCap(true)));
      s.check("cap suppression: untapered end keeps its cap",
              anyBeyond(renderCap(false)));
    }

    // (20h-4) free-end connection: a cross-section boundary whose top few
    // px are depth-CONTINUOUS (below the gap threshold -> contact veto, no
    // crack) ends FREE, short of the union's outer rim; the probe finds
    // the rim beyond the free end and extends the drawn stem to meet it
    // (legacy centered bands bridged this gap invisibly; an offset band
    // would leave a white gap). Section 2 ramps smoothly (8 world/px, well
    // under the DepthGap slope threshold) so no internal edge fires.
    {
      const int W = 32, H = 32;
      umbreon::FrameResult frame;
      frame.width = W;
      frame.height = H;
      frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
      frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
      frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
      for (int y = 8; y < 24; ++y)
        for (int x = 4; x < 28; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * W + x;
          frame.objectId[i] = (x < 16 ? 1u : 2u) << 2;
          frame.viewZ[i] =
              x < 16 ? 50.0f : 50.0f + 8.0f * static_cast<float>(y - 9);
        }
      umbreon::Scene scene = makeScene(W, H);
      umbreon::RenderOptions opt =
          makeOpt(W, H, umbreon::StrokeAlign::Outside);
      umbreon::applyScreenVectorEdges(frame, scene, opt);
      // The boundary crack fires only where |vz2 - vz1| = 8*(y-9) exceeds
      // the 12-per-px threshold, i.e. y >= 11: the stem's top end is a
      // free corner ~3.5 px below the rim line y = 7.5. With the probe it
      // extends up through the gap keeping its offset band (east of the
      // backbone x=15.5); without it these rows stay white.
      s.check("free-end connect: the stem reaches the rim",
              lumAt(frame, 17, 9) < 0.1f && lumAt(frame, 17, 10) < 0.1f);
    }
  }

  // (20d) end-to-end: with the default Outside alignment a thick silhouette
  // inks only OUTSIDE the object (interior pixels a centered ribbon would
  // cover stay clean); StrokeAlign::Center restores the symmetric band.
  // Square x,y in [8,24) of a 32x32 frame: top rim line y = 7.5, width 6
  // (half 3) -> Outside band [2.0, 8.0], Center band [4.5, 10.5].
  {
    auto renderSquare = [&](umbreon::StrokeAlign align) {
      const int W = 32, H = 32;
      umbreon::FrameResult frame;
      frame.width = W;
      frame.height = H;
      frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
      frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
      frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
      for (int y = 8; y < 24; ++y)
        for (int x = 8; x < 24; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * W + x;
          frame.objectId[i] = 1u << 2;
          frame.viewZ[i] = 50.0f;
        }
      umbreon::Scene scene;
      scene.camera.position = {0.0f, 0.0f, 100.0f};
      scene.camera.direction = {0.0f, 0.0f, -1.0f};
      scene.camera.up = {0.0f, 1.0f, 0.0f};
      scene.camera.orthographic = true;
      scene.camera.height = static_cast<float>(H);  // pixelSize == 1
      scene.background = {1.0f, 1.0f, 1.0f};
      umbreon::RenderOptions opt;
      opt.width = W;
      opt.height = H;
      opt.supersample = 1;
      opt.strokeEdges.enable = true;
      opt.strokeEdges.edgesOnly = true;
      opt.strokeEdges.thickness = 6;
      opt.strokeEdges.align = align;  // empty style table -> global align
      umbreon::applyScreenVectorEdges(frame, scene, opt);
      return frame;
    };
    auto lumAt = [](const umbreon::FrameResult& fr, int x, int y) {
      return fr.color[(static_cast<std::size_t>(y) * fr.width + x) * 4];
    };
    const umbreon::FrameResult outside =
        renderSquare(umbreon::StrokeAlign::Outside);
    s.check("align e2e: outside inks the full width off the rim",
            lumAt(outside, 16, 3) < 0.1f);
    s.check("align e2e: outside leaves the object interior clean",
            lumAt(outside, 16, 10) > 0.9f);
    const umbreon::FrameResult centered =
        renderSquare(umbreon::StrokeAlign::Center);
    s.check("align e2e: center covers the interior half-band",
            lumAt(centered, 16, 10) < 0.1f);
    s.check("align e2e: center stays inside its outer half-band",
            lumAt(centered, 16, 3) > 0.9f);
  }

  // (20f) occlusion boundaries shift to the FAR side: a near rectangle
  // occluding a far one draws the boundary line entirely on the FAR
  // (non-owner) side under Outside alignment -- for the same-section
  // DepthGap step and the cross-section ObjectId step alike -- so the near
  // surface never thins; Center keeps the legacy symmetric band. Boundary
  // line x = 15.5, width 6 (half 3) -> Outside band [15.0, 21.0] (east,
  // the far side), Center band [12.5, 18.5].
  {
    auto renderStep = [&](std::uint32_t farId, umbreon::StrokeAlign align) {
      const int W = 32, H = 32;
      umbreon::FrameResult frame;
      frame.width = W;
      frame.height = H;
      frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
      frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
      frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
      for (int y = 8; y < 24; ++y)
        for (int x = 4; x < 28; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * W + x;
          frame.objectId[i] = x < 16 ? (1u << 2) : farId;
          // Near west, far east. The 390 step keeps the same-id DepthGap
          // STRONG (raw step > stepDominanceK * px on flat sides); a weak
          // crack would need junction support and the prune would drop the
          // whole line.
          frame.viewZ[i] = x < 16 ? 10.0f : 400.0f;
        }
      umbreon::Scene scene;
      scene.camera.position = {0.0f, 0.0f, 100.0f};
      scene.camera.direction = {0.0f, 0.0f, -1.0f};
      scene.camera.up = {0.0f, 1.0f, 0.0f};
      scene.camera.orthographic = true;
      scene.camera.height = static_cast<float>(H);  // pixelSize == 1
      scene.background = {1.0f, 1.0f, 1.0f};
      umbreon::RenderOptions opt;
      opt.width = W;
      opt.height = H;
      opt.supersample = 1;
      opt.strokeEdges.enable = true;
      opt.strokeEdges.edgesOnly = true;
      opt.strokeEdges.thickness = 6;
      opt.strokeEdges.align = align;
      umbreon::applyScreenVectorEdges(frame, scene, opt);
      return frame;
    };
    auto lumAt = [](const umbreon::FrameResult& fr, int x, int y) {
      return fr.color[(static_cast<std::size_t>(y) * fr.width + x) * 4];
    };
    const std::uint32_t kSame = 1u << 2, kOther = 2u << 2;
    // Same-section DepthGap step.
    const umbreon::FrameResult dgOut =
        renderStep(kSame, umbreon::StrokeAlign::Outside);
    s.check("align DepthGap: outside inks the far side",
            lumAt(dgOut, 20, 16) < 0.1f);
    s.check("align DepthGap: outside leaves the near surface clean",
            lumAt(dgOut, 13, 16) > 0.9f);
    const umbreon::FrameResult dgCen =
        renderStep(kSame, umbreon::StrokeAlign::Center);
    s.check("align DepthGap: center covers the near half-band",
            lumAt(dgCen, 13, 16) < 0.1f);
    s.check("align DepthGap: center stays inside its far half-band",
            lumAt(dgCen, 20, 16) > 0.9f);
    // Cross-section ObjectId step.
    const umbreon::FrameResult obOut =
        renderStep(kOther, umbreon::StrokeAlign::Outside);
    s.check("align ObjectId: outside inks the far side",
            lumAt(obOut, 20, 16) < 0.1f);
    s.check("align ObjectId: outside leaves the near surface clean",
            lumAt(obOut, 13, 16) > 0.9f);
    const umbreon::FrameResult obCen =
        renderStep(kOther, umbreon::StrokeAlign::Center);
    s.check("align ObjectId: center covers the near half-band",
            lumAt(obCen, 13, 16) < 0.1f);
    s.check("align ObjectId: center stays inside its far half-band",
            lumAt(obCen, 20, 16) > 0.9f);
  }

  // (20e) per-section align + contact runs stay centered: two touching
  // same-depth sections (their shared outer silhouette splits into per-group
  // runs at the touch corners), section 1 overridden to Center, section 2 on
  // the Outside default; the depth-continuous contact boundary between them
  // (both Outline, contact on -> Silhouette class) must ink BOTH sides even
  // under Outside alignment (the vote abstains on contact edgels).
  {
    const int W = 48, H = 32;
    umbreon::FrameResult frame;
    frame.width = W;
    frame.height = H;
    frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);
    frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
    frame.objectId.assign(static_cast<std::size_t>(W) * H, kBg);
    for (int y = 8; y < 24; ++y)
      for (int x = 4; x < 44; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * W + x;
        frame.objectId[i] = (x < 24 ? 1u : 2u) << 2;
        frame.viewZ[i] = 50.0f;
      }
    umbreon::Scene scene;
    scene.camera.position = {0.0f, 0.0f, 100.0f};
    scene.camera.direction = {0.0f, 0.0f, -1.0f};
    scene.camera.up = {0.0f, 1.0f, 0.0f};
    scene.camera.orthographic = true;
    scene.camera.height = static_cast<float>(H);  // pixelSize == 1
    scene.background = {1.0f, 1.0f, 1.0f};
    scene.groupEdgeStyle.assign(3, umbreon::EdgeStyle{});
    for (int g = 1; g <= 2; ++g) {
      umbreon::EdgeClassStyle& cs =
          scene.groupEdgeStyle[g]
              .cls[static_cast<int>(umbreon::EdgeClass::Silhouette)];
      cs.enabled = true;
      cs.width = 6.0f;
      // Outline on both sides: the contact boundary inks as Silhouette and
      // the same-section gates stay quiet.
      scene.groupEdgeStyle[g].silhouetteMode = umbreon::SilhouetteMode::Outline;
    }
    scene.groupEdgeStyle[1].align = umbreon::StrokeAlign::Center;
    umbreon::RenderOptions opt;
    opt.width = W;
    opt.height = H;
    opt.supersample = 1;
    opt.strokeEdges.enable = true;
    opt.strokeEdges.edgesOnly = true;
    opt.strokeEdges.contact = true;
    umbreon::applyScreenVectorEdges(frame, scene, opt);

    auto lumAt = [&](int x, int y) {
      return frame.color[(static_cast<std::size_t>(y) * W + x) * 4];
    };
    // Section 1 top rim (y = 7.5, Center): interior half-band inks, nothing
    // 4.5 px above the rim.
    s.check("align per-section: Center section inks its interior half-band",
            lumAt(13, 9) < 0.1f);
    s.check("align per-section: Center section stays inside its band",
            lumAt(13, 3) > 0.9f);
    // Section 2 top rim (Outside default): full width above the rim, clean
    // interior.
    s.check("align per-section: Outside section inks off the rim",
            lumAt(35, 3) < 0.1f);
    s.check("align per-section: Outside section leaves its interior clean",
            lumAt(35, 10) > 0.9f);
    // Contact boundary x = 23.5 (width 6): centered band [20.5, 26.5] inks
    // both sides at mid-height.
    s.check("align contact: contact contour inks the owner (west) side",
            lumAt(21, 16) < 0.1f);
    s.check("align contact: contact contour inks the far (east) side",
            lumAt(26, 16) < 0.1f);
  }

  return s.report();
}
