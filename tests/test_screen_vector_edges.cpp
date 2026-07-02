// Unit tests for the SCREEN-SPACE vector edge extraction, Stage 1 crack
// classification (src/umbreon/edges/screen_vector_edges.{hpp,cpp}).
//
// Locks the per-pixel-pair classification contract on synthetic AOV buffers
// (no renderer needed): silhouette fires exactly on the foreground/background
// perimeter; a tilted plane of ANY in-clamp slope never fires (slope
// adaptivity); a smooth spherical cap never fires in the interior (the
// curvature veto by one-sided extrapolation); a same-id view-z step fires
// DepthGap with the nearer side as owner; abutting ids fire ObjectId; the
// class gates switch each class off.
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
        b.set(x, y, 9, x < 8 ? 10.0f : 15.0f);  // step of 5 > gap 2
    const CrackField cf = classify(b, defaults);
    s.check_eq("z-step: one DepthGap crack per row",
               countClass(cf, CrackClass::DepthGap), 16);
    s.check_eq("z-step: nothing else fires", countActive(cf), 16);
    // Boundary crack (7,y)-(8,y): first pixel vz 10 (nearer) -> owner bit 0.
    s.check("z-step: nearer (first) side owns",
            (cf.right[b.idx(7, 5)] & kCrackOwnerBit) == 0);
  }

  // ---- (5) abutting ids at equal depth: ObjectId class --------------------
  {
    Buffers b(16, 16);
    for (int y = 0; y < 16; ++y)
      for (int x = 0; x < 16; ++x) b.set(x, y, x < 8 ? 1 : 2, 10.0f);
    const CrackField cf = classify(b, defaults);
    s.check_eq("id boundary: one ObjectId crack per row",
               countClass(cf, CrackClass::ObjectId), 16);
    ScreenClassifyParams off = defaults;
    off.objectBoundary = false;
    s.check_eq("id boundary: border gate off => 0 cracks",
               countActive(classify(b, off)), 0);
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
