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
#include "test_util.hpp"

namespace {

using umbreon::CrackClass;
using umbreon::CrackField;
using umbreon::kCrackClassMask;
using umbreon::kCrackOwnerBit;
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

  // ---- (5) abutting sections at equal depth: ObjectId class ---------------
  {
    Buffers b(16, 16);
    // Two DIFFERENT sections (groups 1 and 2). objectId == (group << 2) | kind,
    // so use (1<<2) and (2<<2); a cross-section boundary still inks.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) b.set(x, y, x < 8 ? (1u << 2) : (2u << 2),
                                         10.0f);
    const CrackField cf = classify(b, defaults);
    s.check_eq("section boundary: one ObjectId crack per row",
               countClass(cf, CrackClass::ObjectId), 16);
    ScreenClassifyParams off = defaults;
    off.objectBoundary = false;
    s.check_eq("section boundary: border gate off => 0 cracks",
               countActive(classify(b, off)), 0);
  }

  // ---- (5b) same section, mixed primitive kind: no internal edge ----------
  {
    Buffers b(16, 16);
    // Group 5, kind Sphere(1) vs Cylinder(2): objectId (5<<2)|1 vs (5<<2)|2.
    // A ball and a stick embedded in one CueMol section join seamlessly, so
    // their equal-depth boundary must NOT ink.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, (5u << 2) | (x < 8 ? 1u : 2u), 10.0f);
    s.check_eq("same section, mixed kind, equal depth: no cracks",
               countActive(classify(b, defaults)), 0);
  }

  // ---- (5c) same section, mixed kind, depth step: still no edge -----------
  {
    Buffers b(16, 16);
    // A big view-z step at the kind boundary must NOT ink: same section means
    // seamless, and the mixed-kind pair never falls through to DepthGap.
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x)
        b.set(x, y, (5u << 2) | (x < 8 ? 1u : 2u), x < 8 ? 10.0f : 60.0f);
    s.check_eq("same section, mixed kind, depth step: no cracks",
               countActive(classify(b, defaults)), 0);
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
    // keeps the first hit == nearer surface).
    for (int y = 6; y < 17; ++y)
      for (int x = 6; x < 17; ++x) b.set(x, y, 2 << 2, 20.0f);
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

  return s.report();
}
