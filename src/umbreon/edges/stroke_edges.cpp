// Freestyle-style stroke edge pass entry point (--edges). The extraction
// pipeline lives in the screen vector source (screen_vector_edges.cpp) and the
// shared draw stage in stroke_render.cpp; this TU only keeps the historical
// applyStrokeEdges signature.
#include "edges/stroke_edges.hpp"

#include "edges/screen_vector_edges.hpp"

namespace umbreon {

// STEP 4: variable-width ribbon strokes. Extraction is delegated to the screen
// vector source (screen_vector_edges.cpp:applyScreenVectorEdges): it classifies
// the per-pixel edge AOVs into cracks, traces them into continuous 2D polylines
// and hands them to the shared draw stage (stroke_render.hpp:renderStrokeChains)
// which builds a miter-joined offset RIBBON per run and hard-fills the triangle
// strips composited over frame.color in LINEAR space at hi-res (the box
// downsample antialiases). The default (edges off) path never reaches here, so
// the no-edge render stays byte-identical. The occluded/occludedRaw QI queries
// are unused by the screen source (its visibility is exact from the z-buffer)
// and are kept only for signature stability.
void applyStrokeEdges(FrameResult& frame, const Scene& scene,
                      const RenderOptions& opt, const OcclusionQuery& occluded,
                      const OcclusionQuery& occludedRaw) {
  (void)occluded; (void)occludedRaw;
  const StrokeEdgeOptions& se = opt.strokeEdges;
  if (!se.enable) return;
  applyScreenVectorEdges(frame, scene, opt);
}


}  // namespace umbreon
