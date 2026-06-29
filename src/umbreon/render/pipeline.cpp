#include "render/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "edges/object_space_edges.hpp"
#include "edges/stroke_edges.hpp"
#include "postprocess/fog.hpp"
#include "postprocess/image_ops.hpp"
#include "render/embree_renderer.hpp"

namespace umbreon {

// Freestyle TANGENTIAL-occluder rejection cosine for the stroke QI ray (detailed
// rationale at the binding site in renderFrame()). Defined at namespace scope --
// and so with internal linkage, no capture -- because MSVC rejects an implicitly-
// captured local constexpr in the QI lambda (C3493), which GCC/Clang treat as
// not-odr-used.
constexpr float kQiGrazeCosEps = 1.0e-4f;

FrameResult renderFrame(const Scene& sceneIn, const RenderOptions& opt) {
  // The two NPR edge methods both draw the silhouette and would double-ink if
  // run together (stroke ribbons over object-space edge cylinders); reject the
  // combination rather than silently picking one.
  if (opt.strokeEdges.enable && opt.objectSpaceEdges.enable)
    throw std::runtime_error(
        "umbreon::render: strokeEdges and objectSpaceEdges are mutually "
        "exclusive; enable at most one");

  // Method B (object-space edges): emit the analytic/mesh silhouette as "open"
  // edge cylinders that the tracer occludes for free. The pass mutates the scene
  // (appends to Scene::cylinders) and is camera dependent, so run it here -- on a
  // PRIVATE copy, keeping render()'s `const Scene&` contract -- before tracing.
  // The copy is paid only when the pass is enabled.
  Scene objEdgeScene;
  const Scene* scenePtr = &sceneIn;
  if (opt.objectSpaceEdges.enable) {
    objEdgeScene = sceneIn;
    generateObjectSpaceEdges(objEdgeScene, opt.objectSpaceEdges);
    scenePtr = &objEdgeScene;
  }
  const Scene& scene = *scenePtr;

  const int ss = std::max(1, opt.supersample);
  const int finalW = opt.width, finalH = opt.height;

  // Render at the supersampled resolution; the camera frames identically.
  RenderOptions hi = opt;
  hi.width = finalW * ss;
  hi.height = finalH * ss;

  EmbreeRenderer renderer;
  FrameResult frame = renderer.render(scene, hi);

  // OpenGL linear fog at full (supersampled) resolution, before downsampling, so
  // the box-average mirrors antialiased, fogged samples. Uses the plane eye-z
  // AOV (viewZ); transparent backgrounds fade coverage instead of baking fog.
  if (scene.fog.enabled && !frame.viewZ.empty()) {
    applyFog(scene.fog, frame.width, frame.height, 4, frame.color.data(),
             frame.viewZ.data(), opt.transparentBackground);
  }

  // Freestyle-style stroke edges (--edges, NEW): extract/chain/visibility/ribbon
  // composited over the color in LINEAR space, here -- BEFORE the downsample --
  // so the box-average antialiases them. The Embree scene is kept ALIVE in
  // `renderer` through this pass (see EmbreeRenderer) for ray-cast visibility.
  // Gated on the master flag; with edges off this is never entered, keeping the
  // default render path byte-identical. --edges drives the stroke pipeline.
  if (opt.strokeEdges.enable) {
    // Bind the ray-cast visibility query to the live BVH kept alive in
    // `renderer` (see EmbreeRenderer): occluded(P, target, selfFaces) is the QI
    // test, excluding the edge's own incident mesh faces (Freestyle self-face
    // exclusion). The renderer holds the mesh geomID needed to match those faces.
    // Freestyle TANGENTIAL-occluder rejection threshold: a QI ray hit on a face
    // grazed nearly edge-on (|dir . normalize(Ng)| <= this cosine) is a numerical
    // degeneracy, not a real occluder. Match Freestyle's literal value -- its
    // ComputeRayCastingVisibility counts a hit only when fabs(u*normal) > 0.0001
    // (already cited in embree_renderer.cpp). This is a pure DEGENERACY GUARD, NOT
    // an angular cull: the silhouette's OWN faces are dropped by face-ID
    // (excludeFaceFilter step 1) plus the unbiased true-surface QI origin (fix B,
    // camBias=0), NOT by an angle. The former 0.1 (~5.7deg) discarded a REAL front
    // occluder seen near its OWN silhouette (|dir.Ng|->0), so a back line just
    // inside that silhouette wrongly voted VISIBLE -- the hidden band. Relies on
    // fix B (true-surface origin) + fix D (face-ID self-exclude keeps each strand's
    // own grazing faces) so flat strands self-hide by id, not by this angle.
    // (kQiGrazeCosEps is defined at namespace scope; see the note there.)
    // Coincident-plane self-surface epsilon (Freestyle GeomUtils::COINCIDENT),
    // scaled to the mesh tessellation so it is unit-robust: a hit whose plane sits
    // within this perpendicular distance of the silhouette point is the point's
    // own surface (skip); a real occluder a fold-gap away (>= ~one face) is
    // counted. Derived from the mean triangle edge.
    double elsum = 0.0;
    const std::size_t nTri = scene.mesh.triangleCount();
    for (std::size_t t = 0; t < nTri; ++t) {
      const Vec3& a = scene.mesh.positions[scene.mesh.cornerVertex(t * 3 + 0)];
      const Vec3& b = scene.mesh.positions[scene.mesh.cornerVertex(t * 3 + 1)];
      const Vec3& c = scene.mesh.positions[scene.mesh.cornerVertex(t * 3 + 2)];
      elsum += length(b - a) + length(c - b) + length(a - c);
    }
    const float meanEdge =
        nTri ? static_cast<float>(elsum / (3.0 * nTri)) : 0.0f;
    // 0.2*meanEdge: the silhouette point's OWN surface deviates from its tangent
    // plane by only curvature*O(meanEdge^2) (perp distance ~0), so a small fraction
    // catches it; a real occluder a fold-gap away (>= ~one face) stays well above
    // it and is counted. Verified to protect the outer outline (no self-occlusion)
    // on both tube and ribbon while removing the self-fold leak.
    float coplanarEps = 0.2f * meanEdge;
    // Mesh-free (pure ball-and-stick) scene: meanEdge == 0 disables the mesh-
    // derived coplanar self-surface skip. Derive it instead from the analytic
    // primitive scale so an analytic silhouette point's OWN sphere/cylinder
    // surface (which the QI ray grazes tangentially) is still treated as self,
    // not as a real occluder -- belt-and-suspenders with the ring circumscription
    // in appendAnalyticFeatureSegs. A small fraction of the mean primitive radius:
    // far above the circumscribed chord's sub-sagitta deviation, far below the gap
    // to any genuine occluder (>= ~one radius away). A scene WITH a mesh keeps the
    // mesh-derived value unchanged (byte-identical).
    if (meanEdge == 0.0f) {
      double rsum = 0.0;
      std::size_t rn = 0;
      for (const Sphere& sp : scene.spheres)
        if (!sp.fromEdgeMacro) { rsum += sp.radius; ++rn; }
      for (const Cylinder& cy : scene.cylinders)
        if (!cy.fromEdgeMacro) { rsum += cy.radius; ++rn; }
      if (rn > 0)
        coplanarEps = 0.05f * static_cast<float>(rsum / static_cast<double>(rn));
    }
    const OcclusionQuery occluded = [&renderer, coplanarEps](
                                        const Vec3& p, const Vec3& q,
                                        const int* excludeFaces, int nExclude) {
      return renderer.occluded(p, q, excludeFaces, nExclude, 1.0e-4f,
                               kQiGrazeCosEps, coplanarEps);
    };
    // RAW visibility query (NO QI self-occlusion heuristics: no self-face exclude,
    // no grazing, no coplanar -- just whether any surface lies between the point and
    // the eye). Used by the production normal-lift QI (--edge-qi-lift, approach A:
    // the lifted sample is off its own surface so no heuristic is needed) and by the
    // --edge-qi-vertex-dots debug overlay. Built only when either is requested so
    // the legacy path is unaffected.
    OcclusionQuery occludedRaw;
    if (opt.strokeEdges.debugQiVertexDots || opt.strokeEdges.qiNormalLift > 0.0f) {
      occludedRaw = [&renderer](const Vec3& p, const Vec3& q, const int*, int) {
        return renderer.occluded(p, q, nullptr, 0, 1.0e-4f, 0.0f, 0.0f);
      };
    }
    applyStrokeEdges(frame, scene, opt, occluded, occludedRaw);
  }

  if (ss > 1) {
    frame.color = boxDownsample(frame.color, frame.width, frame.height, 4, ss);
    if (!frame.albedo.empty())
      frame.albedo =
          boxDownsample(frame.albedo, frame.width, frame.height, 3, ss);
    // Edge AOVs (normal/viewZ/objectId/materialId) are a hi-res set: the edge
    // pass runs at supersample resolution before this downsample, and box-
    // averaging integer ids is meaningless. So when edges are on, leave them at
    // hi-res; only the legacy normal AOV path downsamples. frame.width/height
    // below become the FINAL color dims.
    if (!frame.normal.empty() && !opt.strokeEdges.enable)
      frame.normal =
          boxDownsample(frame.normal, frame.width, frame.height, 3, ss);
    // AO AOVs are a continuous hi-res set: box-averaging them to the output
    // resolution is exactly the supersample denoise the AO relies on (more
    // effective samples per output pixel). Done before width/height become final.
    if (!frame.contactAo.empty()) {
      frame.contactAo =
          boxDownsample(frame.contactAo, frame.width, frame.height, 1, ss);
      frame.shapeAo =
          boxDownsample(frame.shapeAo, frame.width, frame.height, 1, ss);
      frame.avgHitDist =
          boxDownsample(frame.avgHitDist, frame.width, frame.height, 1, ss);
      frame.bentNormal =
          boxDownsample(frame.bentNormal, frame.width, frame.height, 3, ss);
    }
    // GI cache AOVs (continuous): downsample to the output resolution like the
    // other guide channels. position is world-space, so the box average is a
    // mild edge blend, acceptable for a debug/guide buffer.
    if (!frame.indirect.empty()) {
      frame.position =
          boxDownsample(frame.position, frame.width, frame.height, 3, ss);
      frame.indirect =
          boxDownsample(frame.indirect, frame.width, frame.height, 3, ss);
      frame.giRecordViz =
          boxDownsample(frame.giRecordViz, frame.width, frame.height, 3, ss);
      frame.giOcclusion =
          boxDownsample(frame.giOcclusion, frame.width, frame.height, 1, ss);
    }
    frame.width = finalW;
    frame.height = finalH;
  }

  applyAssumedGamma(frame, scene.assumedGamma);
  return frame;
}

}  // namespace umbreon
