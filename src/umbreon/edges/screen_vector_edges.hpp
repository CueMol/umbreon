// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
// SCREEN-SPACE VECTOR edge extraction: the stroke edge pass chain source
// (--edges on).
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
#include "render/render_types.hpp"

namespace umbreon {

// Per-crack classification, priority-ordered (a crack gets the FIRST matching
// class). The class also keys the styling slot the traced run draws with
// (EdgeStyle::cls[], see natureStyleSlot's screen analogue in the .cpp).
enum class CrackClass : std::uint8_t {
  None = 0,
  Silhouette = 1,  // exactly one side is background
  ObjectId = 2,    // both foreground, cross-section objectId differs across a
                   // depth step; depth-continuous contact/intersection
                   // contours are suppressed (surface contact, not occlusion)
  DepthGap = 3,    // same-id slope-adaptive view-z discontinuity, or a
                   // same-section mixed-kind boundary across a depth step
                   // (self-occlusion between primitives of one section)
  Crease = 4,      // same objectId, shading-normal fold
};

// Crack byte layout: [0..2] CrackClass, [3] owner side (0 = the FIRST pixel of
// the pair -- left for a right-crack, top for a down-crack -- is the nearer
// "owner" whose viewZ/group the edge carries; 1 = the second pixel), [4] a
// consumed scratch bit used by the Stage-2 tracer, [5] the hysteresis
// STRONG bit (DepthGap only): the crack passed the full strong gate, not just
// the weak candidate threshold. A traced chain survives the Stage-2.5 prune
// only with strong (or non-DepthGap) support; see keepScreenChain.
constexpr std::uint8_t kCrackClassMask = 0x07;
constexpr std::uint8_t kCrackOwnerBit = 0x08;
constexpr std::uint8_t kCrackConsumedBit = 0x10;
constexpr std::uint8_t kCrackStrongBit = 0x20;

// The classified crack lattice of a W x H pixel buffer. right[y*W+x] is the
// crack between pixels (x,y) and (x+1,y) (valid for x < W-1; the x == W-1
// column is always 0). down[y*W+x] is the crack between (x,y) and (x,y+1)
// (valid for y < H-1). A zero byte means "no edge here".
struct CrackField {
  int W = 0, H = 0;
  std::vector<std::uint8_t> right;
  std::vector<std::uint8_t> down;
};

// Per-crack Stage-1 diagnostics for the same-id DepthGap branch, filled only
// when a debug sink is passed to classifyCracks (the UMBREON_SCREEN_EDGE_DUMP
// path in applyScreenVectorEdges). Plane layout mirrors CrackField (right /
// down, W*H cells each); values are in world units -- divide by the pair's
// pixelSize for px units. reason records the FIRST decision that applied.
struct ScreenCrackDebugPlane {
  std::vector<float> gapA, gapB, sA, sB, g0;
  std::vector<std::uint8_t> reason;
};
struct ScreenCrackDebug {
  enum Reason : std::uint8_t {
    kNotEvaluated = 0,   // other branch (bg / id boundary) or gate off
    kSubThreshold = 1,   // min(gapA, gapB) <= weak threshold
    kNmsSuppressed = 2,  // lost the perpendicular 3-pair NMS
    kBgKilled = 3,       // weak within bgClearancePx of background
    kInked = 4,          // classified DepthGap STRONG
    kInkedWeak = 5,      // classified DepthGap weak (hysteresis candidate)
  };
  ScreenCrackDebugPlane right, down;
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
  // predicted exactly, a true occlusion step fails both). Also thresholds the
  // ObjectId contact veto: a cross-section boundary within this depth
  // continuity is treated as surface contact (intersection contour) and not
  // inked; only a genuine depth step draws a border.
  float depthGapPx = 12.0f;
  // One-sided slope clamp in pixelSize units, so extreme grazing noise cannot
  // extrapolate across a genuine fold.
  float slopeClampPx = 300.0f;
  // DepthGap hysteresis: a crack is a WEAK candidate when min(gapA, gapB)
  // exceeds weakGapRatio * depthGapPx * pixelSize; it is STRONG when it also
  // clears the full depthGapPx threshold AND the step-dominance gate below.
  // Weak cracks trace like any crack but survive only in chains with strong
  // (or non-DepthGap) support (Stage 2.5, keepScreenChain). 1 = weak equals
  // strong threshold (hysteresis effectively off).
  float weakGapRatio = 0.5f;
  // Step-dominance gate for STRONG DepthGap cracks: the raw step |dvz| must
  // exceed stepDominanceK * max(recession, pixelSize), where recession is the
  // near side's wide-baseline (up to 6 px) one-sided depth slope per pixel.
  // Discriminates a true occlusion contour (step huge relative to how fast
  // the near surface itself recedes) from the facet-horizon slivers a coarse
  // mesh throws off at grazing incidence: a sight line skims a facet edge and
  // lands a few pixels' worth of the same grazing ramp deeper -- a real but
  // unwanted micro-occlusion (measured on mesh4: sliver ratios <= ~200,
  // contour ratios >= ~500). 0 disables the gate (every inked crack strong).
  float stepDominanceK = 250.0f;
  // ObjectId contact veto only: a side's slope is credited toward the
  // depth-continuity extrapolation only while its shading normal still faces
  // the viewer (|n.v|/|n| >= this). At a rim curling toward its own
  // silhouette |n.v| -> 0 and the steep tangent could land on a farther
  // object by coincidence, faking a contact; degrading such a side to flat
  // extrapolation keeps the occlusion border inked. A true contact stays
  // vetoed through the OTHER (continuing, viewer-facing) surface's
  // extrapolation. 0 disables the gate.
  float borderGrazeCos = 0.3f;
  // Suppress a WEAK DepthGap crack when either pixel lies within this
  // Chebyshev radius (hi-res px) of a background pixel: the last pixels
  // before the silhouette are grazing-dominated and the Silhouette class
  // already inks that boundary. STRONG cracks are exempt (a step-dominant
  // discontinuity is a real occlusion even at the rim), and so is a weak
  // crack whose ALONG-CRACK strip reaches the background (the terminal piece
  // of a contour landing on the outline runs into it; the rim noise this
  // kill targets hugs the outline side-on). 0 = off. The Stage-4 driver
  // scales this with the supersample factor.
  int bgClearancePx = 3;
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
// TBB-parallel over rows; the result is deterministic. `dbg`, when non-null,
// is resized to the planes and filled with per-crack DepthGap diagnostics
// (debug/dump path only; the normal path passes nullptr at zero cost).
CrackField classifyCracks(int W, int H, const float* viewZ,
                          const std::uint32_t* objectId, const float* normal,
                          const ScreenProj& sp,
                          const ScreenClassifyParams& params,
                          ScreenCrackDebug* dbg = nullptr);

// One traced chain vertex, in STROKE pixel coordinates: the pixel-corner
// lattice node (cx,cy), cx in [0..W], cy in [0..H], maps to (cx-0.5, cy-0.5)
// (pixel (x,y) center == stroke coordinate (x,y)). vz is the mean owner-pixel
// linear view-z of the vertex's adjacent edgels (0 when the tracer was given
// no viewZ buffer). alpha is the mean owner-pixel first-hit surface opacity
// of the adjacent edgels (1 when the tracer was given no surfAlpha buffer):
// the draw stage multiplies the stroke opacity by it, so an edge traced on a
// transparent surface inks with that surface's transparency.
struct ScreenChainVert {
  float x = 0.0f, y = 0.0f;
  float vz = 0.0f;
  float alpha = 1.0f;
};

// One traced chain: an ordered corner-lattice polyline. A closed loop
// duplicates the seed vertex at the end (pts.front() == pts.back()). Per
// EDGEL (pts.size()-1 entries): the CrackClass and the owner pixel's section
// group (objectId >> 2; 0 when the tracer was given no objectId buffer).
// deg0/deg1 are the lattice active-crack degrees of the first/last corner
// (2 for a closed loop): a short chain whose ends are BOTH junctions
// (degree >= 3) is a piece of a larger boundary chopped by side-branches and
// must survive the speck filter; a short chain with a free end (degree 1) or
// a tiny loop is an isolated speckle/spur.
struct ScreenChain {
  std::vector<ScreenChainVert> pts;
  std::vector<std::uint8_t> edgeClass;
  std::vector<std::uint16_t> edgeGroup;
  // Per edgel, bit 0 = the crack's kCrackStrongBit (DepthGap hysteresis).
  std::vector<std::uint8_t> edgeFlags;
  bool closed = false;
  int deg0 = 0, deg1 = 0;
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
// null (attributes then stay 0). surfAlpha is the optional first-hit surface
// opacity plane (same layout); null keeps every vertex alpha at 1 (opaque).
std::vector<ScreenChain> traceCrackChains(CrackField& cf,
                                          const float* viewZ = nullptr,
                                          const std::uint32_t* objectId =
                                              nullptr,
                                          const float* surfAlpha = nullptr);

// Stage 2.5 self-support predicate: true when the chain contains any
// non-DepthGap edgel or at least `minStrong` STRONG DepthGap edgels (a lone
// borderline-strong crack inside a weak sliver must not resurrect it as an
// isolated dash). Pure-weak chains have no support of their own;
// pruneWeakChains may still keep one when BOTH its endpoint corners junction
// into kept chains (support propagation).
bool keepScreenChain(const ScreenChain& ch, int minStrong = 1);

// Zero every crack cell traversed by `ch` in the field (class bits and all).
// Used by the Stage-2.5 prune: dropped chains stop chopping their neighbors
// into junction fragments on the next trace. The range overload erases only
// the edgels [e0, e1) (edgel i spans pts[i] -> pts[i+1]) -- the run-level
// weak-tail trim's partial erase.
void eraseChainCracks(CrackField& cf, const ScreenChain& ch);
void eraseChainCracks(CrackField& cf, const ScreenChain& ch, std::size_t e0,
                      std::size_t e1);

// Stage 2.5: hysteresis prune + retrace. Keeps every self-supported chain
// (keepScreenChain), then propagates support to pure-weak OPEN chains whose
// BOTH endpoint corners coincide with an endpoint of an already-kept chain
// (a weak fragment bridging kept boundaries -- e.g. the near-cusp tail of a
// contour between the strong contour body and the silhouette outline, or a
// piece of boundary chopped by side branches). Unsupported chains (isolated
// facet-horizon slivers, grazing speckles, pure-weak loops, and clusters of
// weak chains that only support each other) are erased from the field.
// RUN-LEVEL weak-tail trim: on a kept OPEN chain, a leading/trailing run of
// WEAK DepthGap edgels whose outer endpoint corner does NOT junction into
// another kept chain is erased too -- chain-level support (a non-DepthGap
// or strong edgel elsewhere in the chain) must not extend to a weak sliver
// dangling toward a free end. Without this, the SAME weak cracks would be
// kept or pruned depending on whether a junction happens to coincide with
// the class transition (e.g. a mesh strand's grazing-fade line rides a
// stick's cross-section ObjectId run when the stick touches the strand, but
// is pruned as a free-end spur when nothing splits it off). After erases the
// consumed bits are cleared and the field is retraced so survivors re-merge
// across the dissolved junctions; this repeats until stable. Deterministic.
// viewZ / objectId / surfAlpha are the same buffers given to traceCrackChains.
std::vector<ScreenChain> pruneWeakChains(CrackField& cf,
                                         std::vector<ScreenChain> traced,
                                         const float* viewZ,
                                         const std::uint32_t* objectId,
                                         int minStrong = 1,
                                         const float* surfAlpha = nullptr);

// ---------------------------------------------------------------------------
// Stage 3: geometry cleanup. These operate on a bare vertex polyline (the
// Stage-4 class-run split happens BEFORE cleanup, so a run is uniform in
// class/group and the per-edgel attribute arrays need no maintenance here).
// `closed` polylines carry the duplicated seam vertex (front()==back())
// throughout. vz is carried: interpolated by Chaikin, subset-kept by RDP.

// Total 2D arc length (the Stage-4 speck filter's key).
float polylineLength2d(const std::vector<ScreenChainVert>& pts);

// Merge consecutive collinear same-direction steps (exact staircase
// reduction; removes interior vertices only, so a closed seam vertex may
// survive mid-segment -- harmless).
void collapseCollinear(std::vector<ScreenChainVert>& pts, bool closed);

// Chaikin corner cutting, `iters` iterations. Open polylines pin both
// endpoints (junction continuity across separately-smoothed chains); closed
// polylines are cut cyclically and keep the duplicated seam.
void chaikinSmooth(std::vector<ScreenChainVert>& pts, bool closed, int iters);

// Douglas-Peucker simplification with tolerance `eps` (same pixel units as
// the points). Endpoints are always kept. A closed polyline is split at an
// approximate-diameter vertex pair (deterministic two-sweep pick), each half
// simplified, and the seam re-duplicated.
void simplifyRdp(std::vector<ScreenChainVert>& pts, bool closed, float eps);

// Stage 4 driver (--edges on): classify the frame's edge AOVs,
// trace the cracks, split each chain into same-class runs (with the short-run
// relabel filter), clean up each run's geometry (collapse + Chaikin + RDP;
// whole chains below the speck length are dropped first), map classes onto
// the EdgeStyle slots, and hand the chains to the shared draw stage
// (stroke_render.hpp:renderStrokeChains). Requires the edge AOVs (viewZ /
// objectId / normal) at the frame's (hi-res) resolution -- they are captured
// whenever strokeEdges.enable is on. No QI machinery runs under this source.
void applyScreenVectorEdges(FrameResult& frame, const Scene& scene,
                            const RenderOptions& opt);

}  // namespace umbreon
