// Integration tests for the screen-space NPR edge pass (applyEdges), covering
// all five Warabi classes plus the suppression tables and Stage-C styling:
//   - Silhouette (class 1): object-vs-background outline; an image-border
//     surface pixel must NOT spuriously outline.
//   - Disconnected-face (class 2): same-object depth step, gated by the
//     curvature veto AND the normal-consistency gate.
//   - Object boundary (class 3): different-object gap, with objectSuppress.
//   - Material boundary (class 4): co-planar materialId seam, with
//     materialSuppress.
//   - Crease (class 5): mesh-only normal fold, gated by the curvature veto, the
//     grazing-angle bias and the smooth-normal field (a smoothly curved mesh
//     region must NOT crease while a sharp ridge does).
//   - Stage-C styling: per-class width>1 disk dilation and per-section
//     groupEdgeStyle color routing.
//
// applyEdges() composites edge ink over frame.color in place. Most cases build a
// synthetic hi-res FrameResult (orthographic camera, so pixelSize is constant
// and viewZ is planar) with regions in the objectId/viewZ/normal/materialId
// AOVs, then assert what each class draws. The decisive properties under test
// are the false-positive guards: the curvature veto / normal-consistency gate
// keep smooth spheres/SES CLEAN while genuine steps/ridges still draw.
#include <cmath>
#include <cstdint>
#include <vector>

#include "render/edge_detect.hpp"
#include "test_util.hpp"

namespace {

constexpr std::uint32_t kInvalid = 0xFFFFFFFFu;

// A pixel is "inked" if the edge pass darkened it below the white background.
bool inked(const umbreon::FrameResult& f, int x, int y) {
  const std::size_t i = (static_cast<std::size_t>(y) * f.width + x) * 4;
  return f.color[i] < 0.5f;  // white (1) -> black (0) where an edge composited
}

// Count inked pixels inside [x0,x1) x [y0,y1).
int inkedCount(const umbreon::FrameResult& f, int x0, int y0, int x1, int y1) {
  int n = 0;
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x)
      if (inked(f, x, y)) ++n;
  return n;
}

// Enable exactly one class in opt.edges.defaultStyle (black, opacity 1).
void enableOnly(umbreon::RenderOptions& opt, umbreon::EdgeClass cls) {
  opt.edges = umbreon::EdgeOptions{};
  opt.edges.enable = true;
  opt.edges.defaultStyle.cls[static_cast<int>(cls)].enabled = true;
}

}  // namespace

int main() {
  umbreon::test::Suite s("edge_detect");

  // ---- Synthetic frame -----------------------------------------------------
  // Orthographic camera looking down -Z, image-plane height == H so the
  // per-pixel world size is exactly 1.0 (pixelSize == cam.height / H). Then the
  // default distanceThreshold(1.0) gives gapThresh == 1.0 world unit and the
  // default curvatureGate(0.75) gives a curvature veto at 0.75.
  const int W = 96, H = 64;
  umbreon::Scene scene;
  scene.camera.position = umbreon::Vec3{0, 0, 100};
  scene.camera.direction = umbreon::Vec3{0, 0, -1};
  scene.camera.up = umbreon::Vec3{0, 1, 0};
  scene.camera.orthographic = true;
  scene.camera.height = static_cast<float>(H);  // => pixelSize == 1.0

  umbreon::FrameResult frame;
  frame.width = W;
  frame.height = H;
  frame.color.assign(static_cast<std::size_t>(W) * H * 4, 1.0f);  // white
  frame.viewZ.assign(static_cast<std::size_t>(W) * H, 0.0f);
  frame.normal.assign(static_cast<std::size_t>(W) * H * 3, 0.0f);
  frame.objectId.assign(static_cast<std::size_t>(W) * H, kInvalid);
  frame.materialId.assign(static_cast<std::size_t>(W) * H, kInvalid);

  // Write a world normal into the AOV at pixel index i.
  auto setN = [&](std::size_t i, umbreon::Vec3 n) {
    frame.normal[i * 3 + 0] = n.x;
    frame.normal[i * 3 + 1] = n.y;
    frame.normal[i * 3 + 2] = n.z;
  };

  // OBJ_A/OBJ_B carry encoded groups (objectId = (group<<2)|kind): OBJ_A -> group
  // 2, OBJ_B -> group 5. The suppression tables key on these groups.
  const std::uint32_t OBJ_A = 10, OBJ_B = 20;  // groups 2 and 5
  const std::uint32_t GROUP_A = OBJ_A >> 2, GROUP_B = OBJ_B >> 2;
  // Material ids: a co-planar material split inside the FLAT object-B region (no
  // depth gap there) isolates the Material class from the depth-gap classes.
  const std::uint32_t MAT_A = 1, MAT_B1 = 2, MAT_B2 = 3;

  // Region layout (x columns). The world-normal AOV is set per region too, since
  // the disconnected-face class now also requires the normals to DISAGREE across
  // the gap (BLOCKER 2): a depth step alone is no longer enough.
  //   [4, 36)  : smooth sphere cap, object A. Depth = 50 - sqrt(R^2 - r^2);
  //              the true sphere normal (continuous) -> low curvature interior.
  //   [40, 60) : object A, FLAT at z=50, but column 50 onward steps to z=58 AND
  //              flips its facing normal (0,0,1)->(1,0,0): a genuine face-to-face
  //              disconnected face (gap 8 >> 1, normals disagree, survives both
  //              the curvature veto and the new normal-consistency gate).
  //   [64, 92) : object B, FLAT at z=58, butted against A across x=64 with a
  //              depth gap (A side z=50, B side z=58) -> object boundary.
  const float R = 30.0f;
  const float capCx = 20.0f, capCy = 32.0f;  // cap center in pixel coords
  const umbreon::Vec3 N_FRONT{0.0f, 0.0f, 1.0f};  // facing the camera (-Z look)
  const umbreon::Vec3 N_SIDE{1.0f, 0.0f, 0.0f};   // perpendicular face
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * W + x;
      if (x >= 4 && x < 36) {
        const float dx = (static_cast<float>(x) - capCx);
        const float dy = (static_cast<float>(y) - capCy);
        const float r2 = dx * dx + dy * dy;
        if (r2 <= (R - 1.0f) * (R - 1.0f)) {  // stay off the rim
          frame.objectId[i] = OBJ_A;
          frame.viewZ[i] = 50.0f - std::sqrt(R * R - r2);
          frame.materialId[i] = MAT_A;
          // True sphere normal (toward +Z / camera): smooth across the cap.
          setN(i, umbreon::normalize(umbreon::Vec3{
                      dx, dy, std::sqrt(std::max(0.0f, R * R - r2))}));
        }
      } else if (x >= 40 && x < 60 && y >= 8 && y < 56) {
        frame.objectId[i] = OBJ_A;
        frame.viewZ[i] = (x < 50) ? 50.0f : 58.0f;  // step at x=50
        frame.materialId[i] = MAT_A;
        setN(i, (x < 50) ? N_FRONT : N_SIDE);  // normals flip at the step
      } else if (x >= 64 && x < 92 && y >= 8 && y < 56) {
        frame.objectId[i] = OBJ_B;
        frame.viewZ[i] = 58.0f;  // butts A's flat z=50 across x=64 boundary
        // Co-planar material split at x=78 (both halves flat z=58, same object).
        frame.materialId[i] = (x < 78) ? MAT_B1 : MAT_B2;
        setN(i, N_FRONT);
      } else if (x >= 60 && x < 64 && y >= 8 && y < 56) {
        // A's flat patch extends to x=64 so B has an adjacent A neighbor.
        frame.objectId[i] = OBJ_A;
        frame.viewZ[i] = 50.0f;
        frame.materialId[i] = MAT_A;
        setN(i, N_FRONT);
      }
    }
  }

  // ---- Silhouette (class 1): object-vs-background outline -------------------
  // The decisive pair: a real object/background boundary INKS, but a surface
  // pixel that merely runs into the IMAGE BORDER does NOT (off-image neighbors
  // are treated as surface-continuation, so the frame edge draws no outline --
  // otherwise every full-bleed render would get a spurious 1px frame).
  {
    const int Ws = 32, Hs = 24;
    umbreon::Scene scS = scene;
    scS.camera.height = static_cast<float>(Hs);
    umbreon::FrameResult fr;
    fr.width = Ws;
    fr.height = Hs;
    fr.color.assign(static_cast<std::size_t>(Ws) * Hs * 4, 1.0f);
    fr.viewZ.assign(static_cast<std::size_t>(Ws) * Hs, 0.0f);
    fr.normal.assign(static_cast<std::size_t>(Ws) * Hs * 3, 0.0f);
    fr.objectId.assign(static_cast<std::size_t>(Ws) * Hs, kInvalid);
    fr.materialId.assign(static_cast<std::size_t>(Ws) * Hs, kInvalid);
    // A solid object slab spanning the LEFT half [0,16): it bleeds off the left
    // image border (x=0) and tops/bottoms (y=0, y=Hs-1), but has a real interior
    // boundary against background along x=16.
    for (int y = 0; y < Hs; ++y)
      for (int x = 0; x < 16; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * Ws + x;
        fr.objectId[i] = OBJ_A;
        fr.viewZ[i] = 50.0f;
        fr.materialId[i] = MAT_A;
        fr.normal[i * 3 + 0] = N_FRONT.x;
        fr.normal[i * 3 + 1] = N_FRONT.y;
        fr.normal[i * 3 + 2] = N_FRONT.z;
      }
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Silhouette);
    umbreon::applyEdges(fr, scS, opt);

    // The real object/background boundary at x=15 (surface) | x=16 (bg) inks.
    int realEdgeInk = 0;
    for (int y = 4; y < Hs - 4; ++y)
      if (inked(fr, 15, y)) ++realEdgeInk;
    s.check("sil: object-vs-background boundary inks", realEdgeInk > 10);

    // The left/top/bottom image-border surface pixels must NOT outline (off-image
    // neighbors are surface-continuation). Sample columns [0,14): clear of the
    // real right edge at x=15, whose width-1 dilation reaches x=14 -- counting
    // there would catch the legitimate boundary, not a spurious frame line.
    int borderInk = 0;
    for (int y = 0; y < Hs; ++y)
      if (inked(fr, 0, y)) ++borderInk;          // left border column
    for (int x = 0; x < 14; ++x)
      if (inked(fr, x, 0) || inked(fr, x, Hs - 1)) ++borderInk;  // top/bottom
    s.check("sil: image-border surface does not spuriously outline",
            borderInk == 0);

    // Background pixels themselves are never inked (no line ORIGINATES in bg).
    int bgInk = 0;
    for (int y = 0; y < Hs; ++y)
      for (int x = 17; x < Ws; ++x)
        if (inked(fr, x, y)) ++bgInk;
    s.check("sil: background interior stays clean", bgInk == 0);
  }

  // ---- Disconnected-face (class 2) -----------------------------------------
  {
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Disconnected);
    umbreon::FrameResult f = frame;  // fresh copy
    umbreon::applyEdges(f, scene, opt);

    // The genuine same-object step at x=50 must draw a line (curv is large).
    // Sample interior rows only, clear of the region's bg-bordered top/bottom.
    const int stepInk = inkedCount(f, 49, 12, 51, 52);
    s.check("disc: genuine same-object step draws", stepInk > 20);

    // The smooth sphere interior must stay CLEAN: sample a well-interior box
    // around the cap center, away from the rim. The curvature veto cancels the
    // sub-threshold depth curvature there.
    const int capInteriorInk =
        inkedCount(f, static_cast<int>(capCx) - 10, static_cast<int>(capCy) - 10,
                   static_cast<int>(capCx) + 10, static_cast<int>(capCy) + 10);
    s.check("disc: smooth sphere interior stays clean (curvature veto)",
            capInteriorInk == 0);

    // The object boundary (A vs B at x=64) is NOT a class-2 edge (different
    // objects), so class 2 alone must not ink the seam there (interior rows;
    // the very top/bottom border against bg is a separate silhouette case).
    const int objSeamInk = inkedCount(f, 63, 12, 65, 52);
    s.check("disc: does not fire on the object boundary", objSeamInk == 0);
  }

  // ---- Disconnected-face NORMAL-CONSISTENCY gate (BLOCKER 2) ----------------
  // A same-object depth step large enough to clear the gap AND the curvature veto
  // is NO LONGER sufficient: the world normals across the gap must also disagree.
  // This is what keeps a smooth SES self-occlusion fold (large depth 2nd-deriv but
  // continuous normals) from over-inking, while a true face-to-face step still
  // draws. Build a dedicated frame with two same-object steps differing ONLY in
  // their normal field: identical depth, identical curvature, opposite verdicts.
  {
    const int W2 = 64, H2 = 48;
    umbreon::Scene sc2 = scene;
    sc2.camera.height = static_cast<float>(H2);  // pixelSize == 1.0 again
    umbreon::FrameResult fr;
    fr.width = W2;
    fr.height = H2;
    fr.color.assign(static_cast<std::size_t>(W2) * H2 * 4, 1.0f);
    fr.viewZ.assign(static_cast<std::size_t>(W2) * H2, 0.0f);
    fr.normal.assign(static_cast<std::size_t>(W2) * H2 * 3, 0.0f);
    fr.objectId.assign(static_cast<std::size_t>(W2) * H2, kInvalid);
    fr.materialId.assign(static_cast<std::size_t>(W2) * H2, kInvalid);
    auto setN2 = [&](std::size_t i, umbreon::Vec3 n) {
      fr.normal[i * 3 + 0] = n.x;
      fr.normal[i * 3 + 1] = n.y;
      fr.normal[i * 3 + 2] = n.z;
    };
    // Two object-A patches, each a flat z=50 -> z=58 step at its mid-column:
    //   left  patch [8,24) , step at x=16 : normals FLIP (front -> side) -> ink
    //   right patch [40,56), step at x=48 : normals CONTINUOUS (front both
    //                                       sides) -> a smooth fold, NO ink.
    for (int y = 8; y < H2 - 8; ++y) {
      for (int xs = 0; xs < 2; ++xs) {
        const int x0 = xs == 0 ? 8 : 40;
        const int mid = x0 + 8;
        for (int x = x0; x < x0 + 16; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * W2 + x;
          fr.objectId[i] = OBJ_A;
          fr.materialId[i] = MAT_A;
          fr.viewZ[i] = (x < mid) ? 50.0f : 58.0f;
          const bool flip = (xs == 0);  // only the left patch flips its normal
          setN2(i, (flip && x >= mid) ? N_SIDE : N_FRONT);
        }
      }
    }
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Disconnected);
    umbreon::applyEdges(fr, sc2, opt);
    const int sharpInk = inkedCount(fr, 15, 12, 17, H2 - 12);
    const int smoothInk = inkedCount(fr, 47, 12, 49, H2 - 12);
    s.check("disc: face-to-face step (normals disagree) draws", sharpInk > 10);
    s.check("disc: smooth fold (normals continuous) is NOT inked",
            smoothInk == 0);
  }

  // ---- Object boundary (class 3) -------------------------------------------
  {
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Object);
    umbreon::FrameResult f = frame;
    umbreon::applyEdges(f, scene, opt);

    // The A/B boundary with a depth gap must draw.
    const int objSeamInk = inkedCount(f, 63, 12, 65, 52);
    s.check("obj: object boundary with depth gap draws", objSeamInk > 20);

    // The same-object internal step is NOT an object boundary; class 3 must not
    // ink it (the depth step is between two OBJ_A pixels). Interior rows only:
    // the region's top/bottom border IS an object-vs-background boundary, which
    // class 3 legitimately catches (silhouette composites over it in full runs).
    const int stepInk = inkedCount(f, 49, 12, 51, 52);
    s.check("obj: does not fire on same-object internal step", stepInk == 0);
  }

  // ---- Curvature veto is the discriminator ---------------------------------
  // Raising curvatureGate far above the step's 2nd difference must veto even the
  // genuine step (proves the veto path is the gate that decides). The smooth
  // interior stays clean regardless.
  {
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Disconnected);
    opt.edges.curvatureGate = 100.0f;  // gate >> step 2nd diff (8)
    umbreon::FrameResult f = frame;
    umbreon::applyEdges(f, scene, opt);
    const int stepInk = inkedCount(f, 49, 12, 51, 52);
    s.check("disc: huge curvatureGate vetoes even the genuine step",
            stepInk == 0);
  }

  // ---- Background neighbor (2*far semantics) does not crash / over-draw -----
  // A surface pixel adjacent to background is a silhouette (class 1), not a
  // disconnected face: class 2 must not ink the cap's outer ring against bg.
  {
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Disconnected);
    umbreon::FrameResult f = frame;
    umbreon::applyEdges(f, scene, opt);
    // Column x=4..5 is the cap's left edge against background; class 2 keys on
    // SAME object, so a background neighbor never makes it a disconnected face.
    int bgAdjInk = 0;
    for (int y = 20; y < 44; ++y)
      if (inked(f, 4, y)) ++bgAdjInk;
    s.check("disc: silhouette-against-bg is not a class-2 edge", bgAdjInk == 0);
  }

  // ---- Material boundary (class 4) -----------------------------------------
  // Fires on a materialId change across a 4-neighbor regardless of depth gap:
  // the co-planar split at x=78 inside the FLAT object-B region (both halves
  // z=58, same object) is the case to draw -- a depth-gap class would miss it.
  {
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Material);
    umbreon::FrameResult f = frame;
    umbreon::applyEdges(f, scene, opt);

    // The co-planar material seam at x=78 must draw despite zero depth gap.
    const int matSeamInk = inkedCount(f, 77, 12, 79, 52);
    s.check("mat: co-planar material seam draws (no depth gap)", matSeamInk > 20);

    // Object-B interior away from the seam shares one material -> stays clean.
    const int matInteriorInk = inkedCount(f, 67, 12, 75, 52);
    s.check("mat: same-material interior stays clean", matInteriorInk == 0);

    // Background pixels (materialId == INVALID) are silhouette/object cases, not
    // a material change: the cap's outer ring against bg must not ink.
    int bgAdjInk = 0;
    for (int y = 20; y < 44; ++y)
      if (inked(f, 4, y)) ++bgAdjInk;
    s.check("mat: surface-against-bg is not a class-4 edge", bgAdjInk == 0);
  }

  // ---- Material suppression (materialSuppress co-group) ---------------------
  // Marking object-B's group as co-grouped WITH ITSELF treats both halves of the
  // x=78 split as one material region, suppressing the internal seam.
  {
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Material);
    opt.edges.materialSuppress[GROUP_B] = (1u << GROUP_B);
    umbreon::FrameResult f = frame;
    umbreon::applyEdges(f, scene, opt);
    const int matSeamInk = inkedCount(f, 77, 12, 79, 52);
    s.check("mat: materialSuppress co-group removes the internal seam",
            matSeamInk == 0);
  }

  // ---- Object suppression (objectSuppress co-group) ------------------------
  // The A/B object boundary at x=64 normally draws (class 3, verified above).
  // Co-grouping A and B via objectSuppress (from BOTH sides, since the center
  // pixel's group drives the test) suppresses that internal seam.
  {
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Object);
    opt.edges.objectSuppress[GROUP_A] = (1u << GROUP_B);
    opt.edges.objectSuppress[GROUP_B] = (1u << GROUP_A);
    umbreon::FrameResult f = frame;
    umbreon::applyEdges(f, scene, opt);
    const int objSeamInk = inkedCount(f, 63, 12, 65, 52);
    s.check("obj: objectSuppress co-group removes the A/B boundary",
            objSeamInk == 0);

    // Suppression must NOT leak to the silhouette: the cap-vs-background rim is
    // still an object boundary (background group is out of table range, never
    // co-grouped). Sample the cap's left edge against bg.
    int rimInk = 0;
    for (int y = 20; y < 44; ++y)
      if (inked(f, 4, y)) ++rimInk;
    s.check("obj: suppression does not erase the background silhouette",
            rimInk > 0);
  }

  // ---- Crease (class 5): mesh-only + curvature veto (MAJOR 3, MAJOR 4) ------
  // Build a dedicated frame with a "roof ridge": the depth slope flips sign at a
  // mid-column (large curvature ratio) and the two roof-face normals fold ~90deg.
  // Three patches, identical normal fold, differing only in kind / depth:
  //   left  [8,24)  kind=Mesh(0)   , roof depth (slope flips) -> a real crease.
  //   mid   [28,44) kind=Sphere(1) , roof depth, SAME normal fold -> must NOT
  //                                  crease (mesh-only restriction, MAJOR 4).
  //   right [48,64) kind=Mesh(0)   , FLAT depth, SAME normal fold -> must NOT
  //                                  crease (curvature veto cancels it, MAJOR 3).
  {
    const int W3 = 72, H3 = 48;
    umbreon::Scene sc3 = scene;
    sc3.camera.height = static_cast<float>(H3);  // pixelSize == 1.0
    umbreon::FrameResult fr;
    fr.width = W3;
    fr.height = H3;
    fr.color.assign(static_cast<std::size_t>(W3) * H3 * 4, 1.0f);
    fr.viewZ.assign(static_cast<std::size_t>(W3) * H3, 0.0f);
    fr.normal.assign(static_cast<std::size_t>(W3) * H3 * 3, 0.0f);
    fr.objectId.assign(static_cast<std::size_t>(W3) * H3, kInvalid);
    fr.materialId.assign(static_cast<std::size_t>(W3) * H3, kInvalid);
    auto setN3 = [&](std::size_t i, umbreon::Vec3 n) {
      fr.normal[i * 3 + 0] = n.x;
      fr.normal[i * 3 + 1] = n.y;
      fr.normal[i * 3 + 2] = n.z;
    };
    // Steeply-tilted roof faces so the fold is sharp (dot ~ -0.83): the crease
    // strength then darkens the ridge well below the ink threshold.
    const umbreon::Vec3 nL = umbreon::normalize(umbreon::Vec3{-1, 0, 0.3f});
    const umbreon::Vec3 nR = umbreon::normalize(umbreon::Vec3{1, 0, 0.3f});
    const std::uint32_t MESH_A = (GROUP_A << 2) | 0u;   // kindBits 0 = Mesh
    const std::uint32_t SPH_A = (GROUP_A << 2) | 1u;    // kindBits 1 = Sphere
    struct Patch { int x0; std::uint32_t oid; bool roof; };
    const Patch patches[3] = {{8, MESH_A, true},    // real crease
                              {28, SPH_A, true},    // primitive: no crease
                              {48, MESH_A, false}}; // flat depth: vetoed
    for (int y = 6; y < H3 - 6; ++y) {
      for (const Patch& p : patches) {
        const int mid = p.x0 + 8;
        for (int x = p.x0; x < p.x0 + 16; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * W3 + x;
          fr.objectId[i] = p.oid;
          fr.materialId[i] = MAT_A;
          // Roof: z rises to the ridge then falls (slope flips -> high curvature
          // ratio). Slope 0.5/px stays under the depth-gap veto (gapThresh ~1)
          // so the ridge reads as a smooth fold, not a depth discontinuity.
          const float d = std::fabs(static_cast<float>(x - mid));
          fr.viewZ[i] = p.roof ? (50.0f - 0.5f * d) : 50.0f;
          setN3(i, (x < mid) ? nL : nR);  // ~90deg normal fold at the ridge
        }
      }
    }
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Crease);
    umbreon::applyEdges(fr, sc3, opt);
    const int meshRidge = inkedCount(fr, 15, 10, 17, H3 - 10);
    const int sphRidge = inkedCount(fr, 35, 10, 37, H3 - 10);
    const int flatRidge = inkedCount(fr, 55, 10, 57, H3 - 10);
    s.check("crease: genuine mesh ridge (normals fold + depth slope) draws",
            meshRidge > 10);
    s.check("crease: analytic primitive normal fold does NOT crease (mesh-only)",
            sphRidge == 0);
    s.check("crease: smooth-depth normal fold is vetoed (curvature veto)",
            flatRidge == 0);
  }

  // ---- Crease (class 5): smooth-normal field vs a sharp ridge ---------------
  // The headline crease guard on the actual surface signal: a smoothly CURVED
  // mesh region (a hemisphere cap, normals rotating gradually pixel-to-pixel so
  // no adjacent pair ever folds past creaseAngle) must NOT crease anywhere,
  // while a real sharp ridge (a one-pixel normal STEP across creaseAngle) on the
  // same object DOES. This is the SES/ribbon false-positive case from the design
  // doc (smooth surfaces stay clean); it differs from the block above, which
  // uses a sharp normal step on flat OR primitive geometry -- here the curved
  // region has a genuinely continuous normal field, the true SES analogue.
  {
    const int W4 = 80, H4 = 48;
    umbreon::Scene sc4 = scene;
    sc4.camera.height = static_cast<float>(H4);  // pixelSize == 1.0
    umbreon::FrameResult fr;
    fr.width = W4;
    fr.height = H4;
    fr.color.assign(static_cast<std::size_t>(W4) * H4 * 4, 1.0f);
    fr.viewZ.assign(static_cast<std::size_t>(W4) * H4, 0.0f);
    fr.normal.assign(static_cast<std::size_t>(W4) * H4 * 3, 0.0f);
    fr.objectId.assign(static_cast<std::size_t>(W4) * H4, kInvalid);
    fr.materialId.assign(static_cast<std::size_t>(W4) * H4, kInvalid);
    const std::uint32_t MESH_A = (GROUP_A << 2) | 0u;  // kindBits 0 = Mesh

    // LEFT [6,34): a smooth cylinder-cap mesh. Across the patch the surface
    // normal sweeps a half-turn in the x-z plane from facing +X to facing -X via
    // +Z, so every adjacent column differs by only ~6.5deg (< creaseAngle 30) --
    // a continuous normal field. Depth is the matching circular profile, so the
    // curvature ratio stays smooth too. Nothing here may crease.
    const float kPi = 3.14159265f;
    const int capX0 = 6, capW = 28;
    const float capR = 14.0f;
    for (int y = 6; y < H4 - 6; ++y) {
      for (int k = 0; k < capW; ++k) {
        const int x = capX0 + k;
        const std::size_t i = static_cast<std::size_t>(y) * W4 + x;
        // theta in (0, pi): 0 at the left rim (-X), pi at the right rim (+X).
        const float theta = kPi * (static_cast<float>(k) + 0.5f) / capW;
        fr.objectId[i] = MESH_A;
        fr.materialId[i] = MAT_A;
        fr.viewZ[i] = 50.0f - capR * std::sin(theta);  // bulge toward camera
        // Outward normal of the half-cylinder, face-forwarded toward +Z.
        fr.normal[i * 3 + 0] = -std::cos(theta);
        fr.normal[i * 3 + 1] = 0.0f;
        fr.normal[i * 3 + 2] = std::sin(theta);
      }
    }
    // RIGHT [48,72): a sharp roof ridge at x=60 -- the two roof faces fold ~146deg
    // (dot ~ -0.83, well past creaseAngle 30) with a matching depth slope kink so
    // the curvature veto does NOT cancel it. A genuine ridge that must crease.
    const umbreon::Vec3 nL = umbreon::normalize(umbreon::Vec3{-1.0f, 0.0f, 0.3f});
    const umbreon::Vec3 nR = umbreon::normalize(umbreon::Vec3{1.0f, 0.0f, 0.3f});
    const int ridgeMid = 60;
    for (int y = 6; y < H4 - 6; ++y) {
      for (int x = 48; x < 72; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * W4 + x;
        fr.objectId[i] = MESH_A;
        fr.materialId[i] = MAT_A;
        const float d = std::fabs(static_cast<float>(x - ridgeMid));
        fr.viewZ[i] = 50.0f - 0.5f * d;  // roof: slope flips at the ridge
        const umbreon::Vec3 n = (x < ridgeMid) ? nL : nR;
        fr.normal[i * 3 + 0] = n.x;
        fr.normal[i * 3 + 1] = n.y;
        fr.normal[i * 3 + 2] = n.z;
      }
    }
    umbreon::RenderOptions opt;
    enableOnly(opt, umbreon::EdgeClass::Crease);
    umbreon::applyEdges(fr, sc4, opt);
    // The whole smooth cap interior (away from the rim, which is a silhouette and
    // not under test here) must be free of crease ink.
    const int capInk = inkedCount(fr, capX0 + 2, 10, capX0 + capW - 2, H4 - 10);
    s.check("crease: smooth curved normal field does NOT crease", capInk == 0);
    const int ridgeInk = inkedCount(fr, ridgeMid - 1, 10, ridgeMid + 1, H4 - 10);
    s.check("crease: sharp ridge (normal step > creaseAngle) draws",
            ridgeInk > 10);
  }

  // ---- Stage-C styling: width dilation + per-section color routing ----------
  // Two properties of Stage C on a single silhouette boundary:
  //   (1) width>1 disk dilation WIDENS the inked band vs a 1px hairline, and
  //   (2) groupEdgeStyle routes each section's edge to its OWN color, so two
  //       differently-styled sections produce distinct ink colors on screen.
  // Build two stacked object slabs of DIFFERENT groups, each bleeding off to its
  // own background edge, and give each group a different --edge style.
  {
    const int Wc = 48, Hc = 40;
    umbreon::Scene scC = scene;
    scC.camera.height = static_cast<float>(Hc);  // pixelSize == 1.0
    auto makeTwoSlabFrame = [&]() {
      umbreon::FrameResult fr;
      fr.width = Wc;
      fr.height = Hc;
      fr.color.assign(static_cast<std::size_t>(Wc) * Hc * 4, 1.0f);  // white
      fr.viewZ.assign(static_cast<std::size_t>(Wc) * Hc, 0.0f);
      fr.normal.assign(static_cast<std::size_t>(Wc) * Hc * 3, 0.0f);
      fr.objectId.assign(static_cast<std::size_t>(Wc) * Hc, kInvalid);
      fr.materialId.assign(static_cast<std::size_t>(Wc) * Hc, kInvalid);
      // Top slab rows [4,16) = OBJ_A (group A); bottom slab rows [24,36) = OBJ_B
      // (group B). Each spans columns [8,40), so its right edge at x=40 is a clean
      // object-vs-background silhouette, well clear of the image border.
      for (int x = 8; x < 40; ++x) {
        for (int y = 4; y < 16; ++y) {
          const std::size_t i = static_cast<std::size_t>(y) * Wc + x;
          fr.objectId[i] = OBJ_A;
          fr.viewZ[i] = 50.0f;
          fr.materialId[i] = MAT_A;
          fr.normal[i * 3 + 2] = 1.0f;
        }
        for (int y = 24; y < 36; ++y) {
          const std::size_t i = static_cast<std::size_t>(y) * Wc + x;
          fr.objectId[i] = OBJ_B;
          fr.viewZ[i] = 50.0f;
          fr.materialId[i] = MAT_B1;
          fr.normal[i * 3 + 2] = 1.0f;
        }
      }
      return fr;
    };
    // Count pixels on row `yr` whose RGB is approximately `col` (and inked).
    auto countColorRow = [](const umbreon::FrameResult& f, int yr,
                            const float col[3]) {
      int n = 0;
      for (int x = 0; x < f.width; ++x) {
        const std::size_t i = (static_cast<std::size_t>(yr) * f.width + x) * 4;
        if (f.color[i] > 0.9f && f.color[i + 1] > 0.9f && f.color[i + 2] > 0.9f)
          continue;  // still white background, not inked
        if (std::fabs(f.color[i + 0] - col[0]) < 0.1f &&
            std::fabs(f.color[i + 1] - col[1]) < 0.1f &&
            std::fabs(f.color[i + 2] - col[2]) < 0.1f)
          ++n;
      }
      return n;
    };

    const float kRed[3] = {1.0f, 0.0f, 0.0f};
    const float kBlue[3] = {0.0f, 0.0f, 1.0f};

    // Per-section style table: group A silhouette = thick (width 3) RED, group B
    // silhouette = thin (width 1, a hairline) BLUE. groupEdgeStyle is indexed by
    // group id; size it to cover both groups.
    umbreon::RenderOptions opt;
    opt.edges = umbreon::EdgeOptions{};
    opt.edges.enable = true;
    scC.groupEdgeStyle.assign(GROUP_B + 1, umbreon::EdgeStyle{});
    {
      umbreon::EdgeClassStyle& a =
          scC.groupEdgeStyle[GROUP_A].cls[static_cast<int>(
              umbreon::EdgeClass::Silhouette)];
      a.enabled = true;
      a.color[0] = kRed[0]; a.color[1] = kRed[1]; a.color[2] = kRed[2];
      a.width = 3.0f;
      umbreon::EdgeClassStyle& b =
          scC.groupEdgeStyle[GROUP_B].cls[static_cast<int>(
              umbreon::EdgeClass::Silhouette)];
      b.enabled = true;
      b.color[0] = kBlue[0]; b.color[1] = kBlue[1]; b.color[2] = kBlue[2];
      b.width = 1.0f;
    }

    umbreon::FrameResult fr = makeTwoSlabFrame();
    umbreon::applyEdges(fr, scC, opt);

    // (1) Color routing: group A's right edge (row 10) inks RED, group B's right
    // edge (row 30) inks BLUE -- never the other color.
    const int aRed = countColorRow(fr, 10, kRed);
    const int aBlue = countColorRow(fr, 10, kBlue);
    const int bBlue = countColorRow(fr, 30, kBlue);
    const int bRed = countColorRow(fr, 30, kRed);
    s.check("style: group A silhouette inks its own red", aRed > 0 && aBlue == 0);
    s.check("style: group B silhouette inks its own blue", bBlue > 0 && bRed == 0);

    // (2) Width dilation: the thick (width 3) red band at x=40 is wider than the
    // thin (width 1) blue hairline. Count inked columns on each section's edge
    // row; the dilated band must span strictly more columns than the hairline.
    auto inkedColsOnRow = [&](const umbreon::FrameResult& f, int yr) {
      int n = 0;
      for (int x = 0; x < Wc; ++x) {
        const std::size_t i = (static_cast<std::size_t>(yr) * Wc + x) * 4;
        if (f.color[i] < 0.5f || f.color[i + 1] < 0.5f || f.color[i + 2] < 0.5f)
          ++n;
      }
      return n;
    };
    const int thickCols = inkedColsOnRow(fr, 10);  // group A, width 3
    const int thinCols = inkedColsOnRow(fr, 30);   // group B, width 1
    s.check("style: width>1 dilation widens the band vs a 1px hairline",
            thickCols > thinCols && thinCols >= 1);
  }

  return s.report();
}
