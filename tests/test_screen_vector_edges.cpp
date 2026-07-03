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

// Ortho projection with pixelSize == 1 world unit (halfH = H/2).
ScreenProj unitProj(int W, int H) {
  ScreenProj sp;
  sp.ortho = true;
  sp.W = W;
  sp.H = H;
  sp.halfW = static_cast<float>(W) * 0.5f;
  sp.halfH = static_cast<float>(H) * 0.5f;
  sp.dir = {0.0f, 0.0f, -1.0f};
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

  return s.report();
}
