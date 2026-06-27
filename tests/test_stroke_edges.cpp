// Unit tests for the Freestyle-style STROKE edge pass
// (src/umbreon/render/stroke_edges.{hpp,cpp}).
//
// Guards the bidirectional chaining contract (ported from Freestyle
// Operators::bidirectionalChain + ChainSilhouetteIterator::traverse): every
// FeatureSeg is consumed by exactly one chain; an open path forms a single
// ordered polyline; a closed ring forms exactly one closed chain; a Y/T branch
// stops the chain at the junction (nextSeg requires EXACTLY one continuation);
// chaining never crosses natures.
#include <cmath>
#include <cstddef>
#include <set>
#include <vector>

#include "render/mesh_feature_edges.hpp"
#include "render/stroke_edges.hpp"
#include "scene.hpp"
#include "test_util.hpp"

namespace {

using umbreon::EdgeChain;
using umbreon::EdgeNature;
using umbreon::FeatureSeg;
using umbreon::OcclusionQuery;
using umbreon::ScreenProj;
using umbreon::Vec3;

// Build a silhouette feature segment between welded node ids a,b. Positions are
// synthetic (node id mapped onto the x axis) -- the chainer only uses v0/v1.
FeatureSeg seg(int a, int b, EdgeNature nat = EdgeNature::Silhouette) {
  FeatureSeg s;
  s.v0 = a;
  s.v1 = b;
  s.p0 = Vec3{static_cast<float>(a), 0, 0};
  s.p1 = Vec3{static_cast<float>(b), 0, 0};
  s.nature = nat;
  return s;
}

// Total segments referenced across all chains.
std::size_t totalSegs(const std::vector<EdgeChain>& chains) {
  std::size_t n = 0;
  for (const EdgeChain& c : chains) n += c.segs.size();
  return n;
}

// Every segment index appears at most once across all chains.
bool eachSegOnce(const std::vector<EdgeChain>& chains) {
  std::set<int> seen;
  for (const EdgeChain& c : chains)
    for (int si : c.segs)
      if (!seen.insert(si).second) return false;
  return true;
}

}  // namespace

int main() {
  umbreon::test::Suite s("stroke_edges");

  // ---- (1) open path: a chain of 4 segments -> one polyline ----------------
  {
    // 0-1-2-3-4 : four segments forming a single open polyline.
    std::vector<FeatureSeg> segs = {seg(0, 1), seg(1, 2), seg(2, 3), seg(3, 4)};
    auto chains = umbreon::chainFeatureSegs(segs, 5);
    s.check_eq("open path: exactly one chain", chains.size(),
               static_cast<std::size_t>(1));
    s.check("open path: all 4 segments consumed",
            totalSegs(chains) == segs.size());
    s.check("open path: each segment used once", eachSegOnce(chains));
    s.check("open path: chain not closed", !chains[0].closed);
    // 4 segments -> 5 ordered backbone points, monotonic 0..4.
    s.check_eq("open path: 5 backbone points", chains[0].pts.size(),
               static_cast<std::size_t>(5));
    bool ordered = chains[0].pts.size() == 5;
    for (std::size_t i = 0; ordered && i < chains[0].pts.size(); ++i)
      if (chains[0].pts[i].x != static_cast<float>(i)) ordered = false;
    s.check("open path: backbone points are in connected order", ordered);
  }

  // ---- (2) seed in the middle: forward+backward still one chain ------------
  {
    // Same path but seed at the middle segment (1-2). Forward grows to 4,
    // backward grows to 0; the result must still be one chain of all 4.
    std::vector<FeatureSeg> segs = {seg(1, 2), seg(2, 3), seg(3, 4), seg(0, 1)};
    auto chains = umbreon::chainFeatureSegs(segs, 5);
    s.check_eq("mid-seed: one chain", chains.size(),
               static_cast<std::size_t>(1));
    s.check("mid-seed: all 4 segments consumed once",
            totalSegs(chains) == 4 && eachSegOnce(chains));
    s.check_eq("mid-seed: 5 backbone points", chains[0].pts.size(),
               static_cast<std::size_t>(5));
  }

  // ---- (3) closed ring -> exactly one CLOSED chain, all consumed once ------
  {
    // 0-1-2-3-0 : a 4-segment closed loop.
    std::vector<FeatureSeg> segs = {seg(0, 1), seg(1, 2), seg(2, 3), seg(3, 0)};
    auto chains = umbreon::chainFeatureSegs(segs, 4);
    s.check_eq("ring: exactly one chain", chains.size(),
               static_cast<std::size_t>(1));
    s.check("ring: all 4 segments consumed once",
            totalSegs(chains) == 4 && eachSegOnce(chains));
    s.check("ring: chain marked closed", chains[0].closed);
    // A closed chain's first and last backbone points coincide.
    if (!chains[0].pts.empty()) {
      const Vec3 a = chains[0].pts.front();
      const Vec3 b = chains[0].pts.back();
      s.check("ring: backbone closes (front == back)",
              a.x == b.x && a.y == b.y && a.z == b.z);
    } else {
      s.check("ring: backbone closes (front == back)", false);
    }
  }

  // ---- (4) Y junction: the SEED chain stops at the branch (nextSeg cnt!=1) --
  {
    // Three fresh spokes meeting at node 0: 0-1, 0-2, 0-3. When the first seed
    // is grown, node 0 has THREE unconsumed candidates -> nextSeg returns -1
    // (branch), so the seed chain cannot grow past the junction and is a single
    // segment. (Freestyle is order-dependent: as spokes get consumed the node
    // degree drops, so the remaining two spokes may later chain through each
    // other -- the chain-once + branch-stop invariants are what matter here.)
    std::vector<FeatureSeg> segs = {seg(0, 1), seg(0, 2), seg(0, 3)};
    auto chains = umbreon::chainFeatureSegs(segs, 4);
    s.check("Y junction: all spokes consumed exactly once",
            totalSegs(chains) == 3 && eachSegOnce(chains));
    // The chain seeded first (containing seg index 0) must not cross the branch.
    bool seedChainSingle = false;
    for (const EdgeChain& c : chains)
      for (int si : c.segs)
        if (si == 0) seedChainSingle = (c.segs.size() == 1);
    s.check("Y junction: the seed chain stops at the branch (length 1)",
            seedChainSingle);
    s.check("Y junction: more than one chain (branch was not bridged in place)",
            chains.size() >= 2);
  }

  // ---- (5) plus junction: a degree-4 hub stops every fresh traversal -------
  {
    // Four spokes at node 0: with >=2 candidates at the hub on the FIRST and
    // SECOND traversals, the hub never bridges down to a unique pass-through
    // until only two spokes remain. The robust invariant: every segment is
    // consumed exactly once and the seed chain is a single segment.
    std::vector<FeatureSeg> segs = {seg(0, 1), seg(0, 2), seg(0, 3), seg(0, 4)};
    auto chains = umbreon::chainFeatureSegs(segs, 5);
    s.check("plus junction: every spoke consumed once",
            totalSegs(chains) == 4 && eachSegOnce(chains));
    bool seedChainSingle = false;
    for (const EdgeChain& c : chains)
      for (int si : c.segs)
        if (si == 0) seedChainSingle = (c.segs.size() == 1);
    s.check("plus junction: the seed chain stops at the branch (length 1)",
            seedChainSingle);
  }

  // ---- (6) nature isolation: same topology, different natures never merge --
  {
    // 0-1 silhouette, 1-2 crease: although they share node 1 they are different
    // natures, so they chain SEPARATELY (two chains, each one segment).
    std::vector<FeatureSeg> segs = {seg(0, 1, EdgeNature::Silhouette),
                                    seg(1, 2, EdgeNature::Crease)};
    auto chains = umbreon::chainFeatureSegs(segs, 3);
    s.check_eq("nature isolation: two single-nature chains", chains.size(),
               static_cast<std::size_t>(2));
    s.check("nature isolation: each segment consumed once",
            totalSegs(chains) == 2 && eachSegOnce(chains));
  }

  // ---- (7) ray-cast visibility: a vertex behind an occluding quad is hidden,
  //          a vertex in front is visible -----------------------------------
  {
    // Ortho camera at z=+10 looking down -Z. Build the same projection basis the
    // renderer uses (makeScreenProj just needs a Camera).
    umbreon::Camera cam;
    cam.position = {0, 0, 10};
    cam.direction = {0, 0, -1};
    cam.up = {0, 1, 0};
    cam.orthographic = true;
    cam.height = 4.0f;
    const ScreenProj sp = umbreon::makeScreenProj(cam, 64, 64);

    // Synthetic occluder: a quad in the plane z = 1, spanning |x|,|y| <= 2,
    // standing in for mesh "face 7". occluded(p,q,excl,n) is true iff the segment
    // p->q crosses z=1 within that extent, strictly between the endpoints, UNLESS
    // face 7 is in the exclude set (then the self-face is skipped, as Freestyle
    // does). The camera (q) is at z=10.
    constexpr int kQuadFace = 7;
    const OcclusionQuery occluded = [](const Vec3& p, const Vec3& q,
                                       const int* excl, int n) {
      for (int i = 0; i < n; ++i)
        if (excl[i] == kQuadFace) return false;  // self-face: not an occluder
      const float zPlane = 1.0f;
      // No crossing if both endpoints are on the same side of the plane.
      if ((p.z - zPlane) * (q.z - zPlane) >= 0.0f) return false;
      const float t = (zPlane - p.z) / (q.z - p.z);
      if (t <= 0.0f || t >= 1.0f) return false;  // strictly between
      const float x = p.x + (q.x - p.x) * t;
      const float y = p.y + (q.y - p.y) * t;
      return std::fabs(x) <= 2.0f && std::fabs(y) <= 2.0f;
    };

    // QI samples each segment's INTERIOR (Freestyle center3d), so visibility is
    // decided per SEGMENT then propagated to the two backbone vertices it borders.
    // A segment fully BEHIND the quad (both endpoints z<1, center z<1): the
    // center->camera ray crosses z=1 -> hidden; a segment fully in FRONT (z>1):
    // the ray never crosses -> visible. No incident faces carried (the quad is a
    // foreign occluder), so neither is excluded.
    EdgeChain behind;
    behind.pts = {Vec3{0, 0, 0}, Vec3{0, 0, 0.5f}};  // center z=0.25 < 1
    const std::vector<char> visB =
        umbreon::computeChainVisibility(behind, sp, occluded);
    s.check("visibility: a segment behind the quad hides both its vertices",
            visB.size() == 2 && !visB[0] && !visB[1]);

    EdgeChain front;
    front.pts = {Vec3{0, 0, 2}, Vec3{0, 0, 3}};  // center z=2.5 > 1
    const std::vector<char> visF =
        umbreon::computeChainVisibility(front, sp, occluded);
    s.check("visibility: a segment in front of the quad is visible",
            visF.size() == 2 && visF[0] && visF[1]);

    // FILTER-EXCLUSION (the mandatory QI fix): a segment lying ON its OWN surface
    // (face 7, the quad plane) must NOT count that surface as an occluder. The
    // segment carries face 7 as its incident face (segFaces), so
    // computeChainVisibility passes it to the query, which skips it (Freestyle
    // self/adjacent-face exclusion, now live via the Embree argument filter).
    // Without exclusion this grazing segment would self-occlude and dash the
    // stroke. The segment center is just behind the plane so the ray WOULD be
    // occluded by face 7 if it were not excluded.
    EdgeChain onPlane;
    onPlane.pts = {Vec3{0, 0, 0.9f}, Vec3{0, 0, 0.95f}};  // center z=0.925 < 1
    onPlane.segFaces = {{kQuadFace, -1}};
    const std::vector<char> visOn =
        umbreon::computeChainVisibility(onPlane, sp, occluded);
    s.check("visibility: a segment on its own surface is not self-occluded",
            visOn.size() == 2 && visOn[0] && visOn[1]);

    // Control: the SAME segment WITHOUT the self-face exclude set IS occluded by
    // the quad (confirms the exclusion above is load-bearing, not a no-op).
    EdgeChain onPlaneNoExcl;
    onPlaneNoExcl.pts = onPlane.pts;
    const std::vector<char> visNoExcl =
        umbreon::computeChainVisibility(onPlaneNoExcl, sp, occluded);
    s.check("visibility: without exclusion the grazing segment self-occludes",
            visNoExcl.size() == 2 && !visNoExcl[0] && !visNoExcl[1]);

    // Null query (no live BVH) => everything visible.
    const std::vector<char> visNone =
        umbreon::computeChainVisibility(behind, sp, OcclusionQuery{});
    s.check("visibility: empty query marks all visible",
            visNone.size() == 2 && visNone[0] && visNone[1]);
  }

  // ---- (7b) 2D image-space crossing depth-order (Freestyle CreateTVertex) ----
  // Two silhouette segments that CROSS in screen space at different eye-space
  // depths: the FARTHER one is hidden at the crossing; the nearer is untouched.
  {
    // Ortho camera at z=+10 looking down -Z; view-z == (10 - world z), so a
    // SMALLER world z is FARTHER (larger view-z).
    umbreon::Camera cam;
    cam.position = {0, 0, 10};
    cam.direction = {0, 0, -1};
    cam.up = {0, 1, 0};
    cam.orthographic = true;
    cam.height = 4.0f;
    const ScreenProj sp = umbreon::makeScreenProj(cam, 64, 64);

    // Chain 0: a horizontal silhouette segment at world z=5 (NEAR).
    // Chain 1: a vertical silhouette segment at world z=2 (FAR), crossing it at
    // screen center. They share no node, are different chains -> a real crossing.
    EdgeChain near;
    near.pts = {Vec3{-1, 0, 5}, Vec3{1, 0, 5}};
    near.segNature = {EdgeNature::Silhouette};
    EdgeChain far;
    far.pts = {Vec3{0, -1, 2}, Vec3{0, 1, 2}};
    far.segNature = {EdgeNature::Silhouette};
    std::vector<EdgeChain> chains = {near, far};

    const auto cross = umbreon::computeEdgeCrossings(chains, sp, /*zTol=*/0.1f);
    // Freestyle ComputeIntersections splits BOTH edges at the crossing (a pure
    // ViewEdge boundary; the QI majority, not the crossing, decides visibility).
    s.check_eq("crossing: both edges split (Freestyle SplitEdge-on-both)",
               cross.size(), static_cast<std::size_t>(2));
    bool gotNear = false, gotFar = false;
    for (const auto& c : cross) {
      const bool mid = c.segIdx == 0 && c.t > 0.4f && c.t < 0.6f;
      if (c.chainIdx == 0 && mid) gotNear = true;
      if (c.chainIdx == 1 && mid) gotFar = true;
    }
    s.check("crossing: both the near and far chain are split at the crossing",
            gotNear && gotFar);

    // Coincident depth is NOT a special case: the crossing still splits BOTH
    // edges (Freestyle splits unconditionally; the QI decides nothing is hidden).
    EdgeChain sameZ;
    sameZ.pts = {Vec3{0, -1, 5}, Vec3{0, 1, 5}};  // same z=5 as `near`
    sameZ.segNature = {EdgeNature::Silhouette};
    std::vector<EdgeChain> chains2 = {near, sameZ};
    const auto cross2 = umbreon::computeEdgeCrossings(chains2, sp, /*zTol=*/0.1f);
    s.check_eq("crossing: coincident-depth crossing still splits both edges",
               cross2.size(), static_cast<std::size_t>(2));

    // Crease-vs-crease crossing is skipped (silhouette_binary_rule).
    EdgeChain crA;
    crA.pts = {Vec3{-1, 0, 5}, Vec3{1, 0, 5}};
    crA.segNature = {EdgeNature::Crease};
    EdgeChain crB;
    crB.pts = {Vec3{0, -1, 2}, Vec3{0, 1, 2}};
    crB.segNature = {EdgeNature::Crease};
    std::vector<EdgeChain> chains3 = {crA, crB};
    const auto cross3 = umbreon::computeEdgeCrossings(chains3, sp, /*zTol=*/0.1f);
    s.check("crossing: crease-vs-crease crossing is not a T-vertex",
            cross3.empty());
  }

  // ---- (8) ribbon geometry: straight stroke -> ~2*halfThick rectangle; a
  //          right-angle corner miters within the spike clamp ----------------
  {
    using umbreon::Vec2f;
    const float halfThick = 3.0f;
    const float step = 1000.0f;  // huge step => no extra resample vertices

    // (8a) A straight horizontal 2-vertex stroke. The strip border vertices sit
    // halfThick above/below the backbone, so the band is 2*halfThick wide.
    {
      std::vector<Vec2f> bb = {{10.0f, 50.0f}, {90.0f, 50.0f}};
      auto strips = umbreon::buildRibbonStrips(bb, {}, halfThick, step);
      s.check_eq("ribbon straight: one strip", strips.size(),
                 static_cast<std::size_t>(1));
      bool widthOk = false, xOk = false;
      if (strips.size() == 1 && strips[0].size() == 4) {
        const auto& v = strips[0];
        // Pairs: [0]=left(+normal), [1]=right(-normal) at vertex 0; [2],[3] at 1.
        const float bandStart = std::fabs(v[0].y - v[1].y);
        const float bandEnd = std::fabs(v[2].y - v[3].y);
        widthOk = std::fabs(bandStart - 2.0f * halfThick) < 1e-3f &&
                  std::fabs(bandEnd - 2.0f * halfThick) < 1e-3f;
        // The offset is purely in y (normal is vertical for a horizontal line).
        xOk = std::fabs(v[0].x - 10.0f) < 1e-3f && std::fabs(v[2].x - 90.0f) < 1e-3f;
      }
      s.check("ribbon straight: band width == 2*halfThick", widthOk);
      s.check("ribbon straight: offsets are along the normal only", xOk);
    }

    // (8b) A right-angle corner (L shape). The interior miter vertex must stay
    // within the spike clamp: its distance from the corner is <=
    // MAX_RATIO_LENGTH_SINGU(2)*halfThick. A 90-degree miter sits at
    // sqrt(2)*halfThick (~4.24 for halfThick 3), well inside the 2x clamp.
    {
      std::vector<Vec2f> bb = {{10.0f, 10.0f}, {50.0f, 10.0f}, {50.0f, 50.0f}};
      auto strips = umbreon::buildRibbonStrips(bb, {}, halfThick, step);
      s.check_eq("ribbon corner: one strip", strips.size(),
                 static_cast<std::size_t>(1));
      bool miterOk = false;
      if (strips.size() == 1 && strips[0].size() == 6) {
        const auto& v = strips[0];
        // Interior backbone vertex (index 1) -> strip pair [2],[3]. Corner at
        // (50,10). Both miter vertices must be within the spike limit.
        const float limit = 2.0f * halfThick + 1e-3f;
        const float dL = std::sqrt((v[2].x - 50.0f) * (v[2].x - 50.0f) +
                                   (v[2].y - 10.0f) * (v[2].y - 10.0f));
        const float dR = std::sqrt((v[3].x - 50.0f) * (v[3].x - 50.0f) +
                                   (v[3].y - 10.0f) * (v[3].y - 10.0f));
        miterOk = dL <= limit && dR <= limit && dL > halfThick - 1e-3f;
      }
      s.check("ribbon corner: miter vertex within the spike clamp", miterOk);
    }

    // (8c) Visibility split: a 3-vertex backbone with the MIDDLE vertex hidden
    // yields zero strips (no run of >=2 consecutive visible vertices).
    {
      std::vector<Vec2f> bb = {{10.0f, 10.0f}, {50.0f, 10.0f}, {90.0f, 10.0f}};
      std::vector<char> vis = {1, 0, 1};
      auto strips = umbreon::buildRibbonStrips(bb, vis, halfThick, step);
      s.check("ribbon visibility split: a single hidden middle drops the strip",
              strips.empty());
    }
  }

  // ---- (9) closeVisibilityMask: grazing-silhouette dash suppression ---------
  // The primary connectivity fix. A short HIDDEN run bracketed by visible
  // vertices (a spurious self-occlusion flicker) is bridged; a long hidden run
  // (a genuine occlusion) and a run touching an OPEN end are NOT.
  {
    using umbreon::closeVisibilityMask;
    // (9a) a single spurious-hidden flicker is bridged (maxBridge 2, open).
    {
      std::vector<char> v = {1, 1, 0, 1, 1};
      closeVisibilityMask(v, 2, /*closed=*/false);
      s.check("visClose: isolated hidden flicker bridged",
              v == std::vector<char>({1, 1, 1, 1, 1}));
    }
    // (9b) a hidden run longer than maxBridge (genuine occlusion) survives.
    {
      std::vector<char> v = {1, 0, 0, 0, 1};
      closeVisibilityMask(v, 2, /*closed=*/false);
      s.check("visClose: long hidden run not bridged (occlusion kept)",
              v == std::vector<char>({1, 0, 0, 0, 1}));
    }
    // (9c) a hidden run touching an OPEN end has no visible bracket -> kept.
    {
      std::vector<char> v = {0, 0, 1, 1, 1};
      closeVisibilityMask(v, 2, /*closed=*/false);
      s.check("visClose: open-end hidden tail not bridged",
              v == std::vector<char>({0, 0, 1, 1, 1}));
    }
    // (9d) on a CLOSED loop a hidden run straddling the seam IS bridged: index
    // 0 and n-1 wrap, so the run {n-1, 0} is bracketed by visible neighbours.
    {
      std::vector<char> v = {0, 1, 1, 1, 0};  // seam run = {v[4], v[0]}
      closeVisibilityMask(v, 2, /*closed=*/true);
      s.check("visClose: closed-loop seam flicker bridged",
              v == std::vector<char>({1, 1, 1, 1, 1}));
    }
    // (9e) maxBridge 0 is a no-op.
    {
      std::vector<char> v = {1, 0, 1};
      closeVisibilityMask(v, 0, /*closed=*/false);
      s.check("visClose: maxBridge 0 is a no-op",
              v == std::vector<char>({1, 0, 1}));
    }
  }

  return s.report();
}
