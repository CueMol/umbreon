// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// SCREEN-SPACE VECTOR edge extraction (--stroke-source screen).
//
// Extracts stroke chains from the per-pixel edge G-buffer AOVs (viewZ /
// objectId / normal, FrameResult) instead of from mesh topology. Edges are
// detected per PIXEL PAIR as "cracks" on the dual lattice between adjacent
// pixels, then traced into polylines along the lattice. Because every
// boundary between two differently-classified pixel regions is a set of
// maximal paths/loops on that lattice, the chains are CONTINUOUS BY
// CONSTRUCTION (closed loops or junction-to-junction paths) -- no voting, no
// chaining tolerance, unlike the mesh-topology source. Visibility is exact
// and free (the AOVs come from the z-buffered first hit), so no QI ray cast
// runs under this source.
//
// The tessellation-independence is the point: a coarse tube mesh whose
// smooth-contour silhouette goes sparse/unstable at grazing folds under the
// mesh source still has a pixel-exact raster silhouette, which this module
// vectorizes. Trade-off: chains live at hi-res pixel resolution (Chaikin +
// Douglas-Peucker clean up the staircase; sub-pixel refinement from the
// viewZ/normal gradients is a possible later step).
//
// Pipeline (applyScreenVectorEdges):
//   Stage 1  classifyCracks    pixel-pair classification -> CrackField
//   Stage 2  traceCrackChains  deterministic lattice tracing -> ScreenChain[]
//   Stage 3  cleanup           collinear collapse + Chaikin + RDP + speck
//   Stage 4  class runs -> StrokeChainInput -> renderStrokeChains
//            (the shared draw stage, stroke_render.hpp)
#pragma once

#include <cstdint>
#include <vector>

#include "edges/mesh_feature_edges.hpp"

namespace umbreon {

// Per-crack classification, priority-ordered (a crack gets the FIRST matching
// class). The class also keys the styling slot the traced run draws with
// (EdgeStyle::cls[], see natureStyleSlot's screen analogue in the .cpp).
enum class CrackClass : std::uint8_t {
  None = 0,
  Silhouette = 1,  // exactly one side is background
  ObjectId = 2,    // both foreground, objectId differs
  DepthGap = 3,    // same objectId, slope-adaptive view-z discontinuity
  Crease = 4,      // same objectId, shading-normal fold
};

// Crack byte layout: [0..2] CrackClass, [3] owner side (0 = the FIRST pixel of
// the pair -- left for a right-crack, top for a down-crack -- is the nearer
// "owner" whose viewZ/group the edge carries; 1 = the second pixel), [4] a
// consumed scratch bit used by the Stage-2 tracer.
constexpr std::uint8_t kCrackClassMask = 0x07;
constexpr std::uint8_t kCrackOwnerBit = 0x08;
constexpr std::uint8_t kCrackConsumedBit = 0x10;

// The classified crack lattice of a W x H pixel buffer. right[y*W+x] is the
// crack between pixels (x,y) and (x+1,y) (valid for x < W-1; the x == W-1
// column is always 0). down[y*W+x] is the crack between (x,y) and (x,y+1)
// (valid for y < H-1). A zero byte means "no edge here".
struct CrackField {
  int W = 0, H = 0;
  std::vector<std::uint8_t> right;
  std::vector<std::uint8_t> down;
};

// Stage-1 parameters. The class gates mirror the stroke master nature toggles
// (silhouette gates Silhouette + DepthGap, objectBoundary gates ObjectId --
// wired from the border toggle -- and crease gates Crease). Thresholds are in
// the units noted; the depth thresholds scale with the per-pixel world size at
// the surface depth (pixelSize), so sensitivity is projection- and
// depth-independent.
struct ScreenClassifyParams {
  bool silhouette = true;
  bool objectBoundary = true;
  bool crease = false;
  // DepthGap: fire when BOTH one-sided planar extrapolations miss the far
  // pixel by more than depthGapPx * pixelSize (world units per lateral pixel;
  // this is the second-derivative form of the Mol*-style curvature veto -- a
  // smooth surface satisfies at least one extrapolation, a grazing plane is
  // predicted exactly, a true occlusion step fails both).
  float depthGapPx = 2.0f;
  // One-sided slope clamp in pixelSize units, so extreme grazing noise cannot
  // extrapolate across a genuine fold.
  float slopeClampPx = 30.0f;
  // Crease: fire when dot(nA, nB) < cos(effective angle), where the effective
  // angle widens at grazing incidence: creaseAngleDeg * (1 + grazeK * (1 -
  // min(|nA.V|, |nB.V|))) with V the view forward axis.
  float creaseAngleDeg = 30.0f;
  float grazeK = 1.0f;
};

// Stage 1: classify every 4-neighbor pixel-pair crack of the hi-res AOV
// buffers. viewZ is the linear view-z (0 / undefined for background pixels --
// background is keyed on objectId), objectId uses 0xFFFFFFFFu as the
// background sentinel, normal is the world-space per-pixel shading normal
// (3 floats per pixel; only read when params.crease). `sp` supplies the
// projection half-extents for pixelSize (build with makeScreenProj at the SAME
// W x H as the buffers). Buffers must not alias the returned field. Runs
// TBB-parallel over rows; the result is deterministic.
CrackField classifyCracks(int W, int H, const float* viewZ,
                          const std::uint32_t* objectId, const float* normal,
                          const ScreenProj& sp,
                          const ScreenClassifyParams& params);

// One traced chain vertex, in STROKE pixel coordinates: the pixel-corner
// lattice node (cx,cy), cx in [0..W], cy in [0..H], maps to (cx-0.5, cy-0.5)
// (pixel (x,y) center == stroke coordinate (x,y)). vz is the mean owner-pixel
// linear view-z of the vertex's adjacent edgels (0 when the tracer was given
// no viewZ buffer).
struct ScreenChainVert {
  float x = 0.0f, y = 0.0f;
  float vz = 0.0f;
};

// One traced chain: an ordered corner-lattice polyline. A closed loop
// duplicates the seed vertex at the end (pts.front() == pts.back()). Per
// EDGEL (pts.size()-1 entries): the CrackClass and the owner pixel's section
// group (objectId >> 2; 0 when the tracer was given no objectId buffer).
struct ScreenChain {
  std::vector<ScreenChainVert> pts;
  std::vector<std::uint8_t> edgeClass;
  std::vector<std::uint16_t> edgeGroup;
  bool closed = false;
};

// Stage 2: trace the classified cracks into maximal continuous chains,
// deterministically. Corners are lattice nodes of active-crack degree <= 4; a
// corner of degree 1, 3 or 4 is a TERMINAL (chain endpoint / junction), a
// degree-2 corner is interior. Pass 1 scans corners in row-major id order and
// walks every unconsumed crack incident to a terminal (fixed direction order
// E, S, W, N), producing maximal junction-to-junction open chains; pass 2
// scans the remaining cracks in array order (right plane, then down) -- all on
// pure degree-2 cycles -- and emits closed loops. Every active crack is
// consumed exactly once (the consumed bit in `cf` is set; class bits are
// preserved). viewZ / objectId are the SAME hi-res buffers given to
// classifyCracks, used to attribute per-edgel owner vz / group; either may be
// null (attributes then stay 0).
std::vector<ScreenChain> traceCrackChains(CrackField& cf,
                                          const float* viewZ = nullptr,
                                          const std::uint32_t* objectId =
                                              nullptr);

}  // namespace umbreon
