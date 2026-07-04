// SCREEN-SPACE VECTOR edges, Stage 4 driver: split chains into class runs,
// map them onto the EdgeStyle slots and hand them to the shared draw stage
// (stroke_render.hpp:renderStrokeChains). Also the UMBREON_SCREEN_EDGE_DUMP
// debug sink (PPM/CSV crack dumps). Stages 1-3 live in screen_edge_*.cpp;
// see screen_vector_edges.hpp for the pipeline overview.
#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <cstdlib>
#include <cstdint>
#include <string>

#include "edges/screen_edge_common.hpp"
#include "edges/stroke_render.hpp"

namespace umbreon {

using screen_edge::facingCos;
using screen_edge::kBackground;
using screen_edge::pixelSizeAt;

// ---------------------------------------------------------------------------
// Stage 4: class runs -> style slots -> shared draw stage.

namespace {

// Style slot (EdgeStyle::cls[] index) and paint precedence for a crack class.
// The screen analogue of the mesh path's natureStyleSlot/naturePrecedence:
// Silhouette and DepthGap paint on top (precedence 2), the object-id boundary
// like the mesh Border (1), Crease underneath (0). DepthGap draws with the
// Disconnected slot -- unreachable from the mesh path -- and applyScreen
// VectorEdges falls back to the Silhouette slot when a section leaves
// Disconnected unconfigured.
inline int classStyleSlot(CrackClass c) {
  switch (c) {
    case CrackClass::ObjectId:
      return static_cast<int>(EdgeClass::Object);
    case CrackClass::DepthGap:
      return static_cast<int>(EdgeClass::Disconnected);
    case CrackClass::Crease:
      return static_cast<int>(EdgeClass::Crease);
    default:
      return static_cast<int>(EdgeClass::Silhouette);
  }
}

inline int classPrecedence(CrackClass c) {
  switch (c) {
    case CrackClass::Crease:
      return 0;
    case CrackClass::ObjectId:
      return 1;
    default:
      return 2;  // Silhouette, DepthGap
  }
}

// Relabel a class run shorter than minLen edgels when bracketed by two runs
// of one same class (style-flicker suppression along a boundary whose
// classification alternates, e.g. silhouette <-> depth gap where an object
// edge grazes the background). Geometry is untouched -- only the labels move,
// so the continuity guarantee is intact. Linear scan; a closed chain's
// seam-straddling run pair is left as-is (harmless: one extra style split).
void mergeShortClassRuns(std::vector<std::uint8_t>& cls, int minLen) {
  if (minLen <= 1 || cls.size() < 3) return;
  std::size_t i = 0;
  while (i < cls.size()) {
    std::size_t j = i;
    while (j < cls.size() && cls[j] == cls[i]) ++j;
    const std::size_t runLen = j - i;
    if (i > 0 && j < cls.size() && runLen < static_cast<std::size_t>(minLen) &&
        cls[i - 1] == cls[j]) {
      for (std::size_t k = i; k < j; ++k) cls[k] = cls[i - 1];
      // Re-scan from the previous run start would be quadratic; extending the
      // left run and continuing forward keeps the pass linear and the result
      // deterministic.
    }
    i = j;
  }
}

// ---------------------------------------------------------------------------
// Debug dump (UMBREON_SCREEN_EDGE_DUMP=<prefix>): writes <prefix>_cracks.ppm
// (crack lattice colorized by class / kill reason over a viewZ gray base) and
// <prefix>_cracks.csv (per-crack DepthGap diagnostics, px-normalized).
// Optional UMBREON_SCREEN_EDGE_DUMP_ROI=x0,y0,x1,y1 restricts both outputs
// to a hi-res pixel rectangle. Diagnostic-only; never on in normal runs.

// Crack color by class byte, or by kill reason for un-inked candidates whose
// min gap exceeds `noiseFloor` (world units at that crack). Returns false for
// "draw nothing".
inline bool crackColor(std::uint8_t byte, std::uint8_t reason, float gapA,
                       float gapB, float noiseFloor, std::uint8_t rgb[3]) {
  switch (static_cast<CrackClass>(byte & kCrackClassMask)) {
    case CrackClass::Silhouette:
      rgb[0] = rgb[1] = rgb[2] = 255;  // white
      return true;
    case CrackClass::ObjectId:
      rgb[0] = 255, rgb[1] = 160, rgb[2] = 0;  // orange
      return true;
    case CrackClass::DepthGap:
      if (byte & kCrackStrongBit) {
        rgb[0] = 255, rgb[1] = 0, rgb[2] = 0;  // strong: red
      } else {
        rgb[0] = 80, rgb[1] = 120, rgb[2] = 255;  // weak: blue
      }
      return true;
    case CrackClass::Crease:
      rgb[0] = 160, rgb[1] = 0, rgb[2] = 255;  // violet
      return true;
    default:
      break;
  }
  if (std::min(gapA, gapB) <= noiseFloor) return false;
  switch (reason) {
    case ScreenCrackDebug::kNmsSuppressed:
      rgb[0] = 0, rgb[1] = 255, rgb[2] = 255;  // cyan
      return true;
    case ScreenCrackDebug::kBgKilled:
      rgb[0] = 255, rgb[1] = 255, rgb[2] = 0;  // yellow
      return true;
    case ScreenCrackDebug::kSubThreshold:
      rgb[0] = 0, rgb[1] = 160, rgb[2] = 0;  // green
      return true;
    default:
      return false;
  }
}

void writeCrackDump(const char* prefix, const CrackField& cf,
                    const ScreenCrackDebug& dbg, const float* viewZ,
                    const std::uint32_t* objectId, const float* normal,
                    const ScreenProj& sp, const ScreenClassifyParams& p) {
  const int W = cf.W, H = cf.H;
  int x0 = 0, y0 = 0, x1 = W - 1, y1 = H - 1;
  if (const char* roi = std::getenv("UMBREON_SCREEN_EDGE_DUMP_ROI")) {
    int rx0, ry0, rx1, ry1;
    if (std::sscanf(roi, "%d,%d,%d,%d", &rx0, &ry0, &rx1, &ry1) == 4) {
      x0 = std::max(0, rx0);
      y0 = std::max(0, ry0);
      x1 = std::min(W - 1, rx1);
      y1 = std::min(H - 1, ry1);
    }
  }
  if (x1 < x0 || y1 < y0) return;

  // viewZ gray range over the ROI's foreground pixels.
  float vzMin = std::numeric_limits<float>::max(), vzMax = -vzMin;
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x) {
      const int i = y * W + x;
      if (objectId[i] == kBackground) continue;
      vzMin = std::min(vzMin, viewZ[i]);
      vzMax = std::max(vzMax, viewZ[i]);
    }
  const float vzSpan = vzMax > vzMin ? vzMax - vzMin : 1.0f;

  const std::string base = std::string(prefix) + "_cracks";

  // PPM: pixel (x,y) cell at (2(x-x0)+1, 2(y-y0)+1); right crack of (x,y) at
  // (+1,0) from its cell, down crack at (0,+1); corner lattice nodes stay
  // black.
  {
    const int PW = 2 * (x1 - x0 + 1) + 1, PH = 2 * (y1 - y0 + 1) + 1;
    std::vector<std::uint8_t> img(static_cast<std::size_t>(PW) * PH * 3, 0);
    auto put = [&](int ix, int iy, std::uint8_t r, std::uint8_t g,
                   std::uint8_t b) {
      std::uint8_t* q =
          &img[(static_cast<std::size_t>(iy) * PW + ix) * 3];
      q[0] = r, q[1] = g, q[2] = b;
    };
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const int i = y * W + x;
        const int ix = 2 * (x - x0) + 1, iy = 2 * (y - y0) + 1;
        if (objectId[i] != kBackground) {
          const float t = (viewZ[i] - vzMin) / vzSpan;
          const std::uint8_t g =
              static_cast<std::uint8_t>(64.0f + 144.0f * (1.0f - t));
          put(ix, iy, g, g, g);
        }
        const float px =
            pixelSizeAt(sp, viewZ[i] > 0.0f ? viewZ[i] : vzMin);
        const float noiseFloor = 0.25f * p.depthGapPx * px;
        std::uint8_t rgb[3];
        const std::size_t cell = static_cast<std::size_t>(i);
        if (x + 1 < W &&
            crackColor(cf.right[cell], dbg.right.reason[cell],
                       dbg.right.gapA[cell], dbg.right.gapB[cell], noiseFloor,
                       rgb) &&
            ix + 1 < PW)
          put(ix + 1, iy, rgb[0], rgb[1], rgb[2]);
        if (y + 1 < H &&
            crackColor(cf.down[cell], dbg.down.reason[cell],
                       dbg.down.gapA[cell], dbg.down.gapB[cell], noiseFloor,
                       rgb) &&
            iy + 1 < PH)
          put(ix, iy + 1, rgb[0], rgb[1], rgb[2]);
      }
    }
    const std::string path = base + ".ppm";
    if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
      std::fprintf(f, "P6\n%d %d\n255\n", PW, PH);
      std::fwrite(img.data(), 1, img.size(), f);
      std::fclose(f);
      std::fprintf(stderr, "[screen-edges] dumped %s (%dx%d)\n", path.c_str(),
                   PW, PH);
    }
  }

  // Raw AOV planes (full frame, not ROI-cropped): viewZ as float32 and
  // objectId as uint32, for offline ground-truth analysis.
  {
    const std::size_t n = static_cast<std::size_t>(W) * H;
    const std::string zPath = base + "_viewz.f32";
    if (std::FILE* f = std::fopen(zPath.c_str(), "wb")) {
      std::fwrite(viewZ, sizeof(float), n, f);
      std::fclose(f);
    }
    const std::string idPath = base + "_objid.u32";
    if (std::FILE* f = std::fopen(idPath.c_str(), "wb")) {
      std::fwrite(objectId, sizeof(std::uint32_t), n, f);
      std::fclose(f);
    }
  }

  // CSV: every evaluated same-id DepthGap candidate above the noise floor.
  // Values normalized to px units at the pair's near depth; ndotv is the
  // facing cosine per side, ndelta = 1 - dot(unit normals).
  {
    const std::string path = base + ".csv";
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
                 "plane,x,y,reason,gapA_px,gapB_px,sA_px,sB_px,g0_px,"
                 "slopesum_px,ndotvA,ndotvB,ndelta\n");
    std::size_t rows = 0;
    for (int plane = 0; plane < 2; ++plane) {
      const ScreenCrackDebugPlane& d = plane == 0 ? dbg.right : dbg.down;
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          const std::size_t cell = static_cast<std::size_t>(y) * W + x;
          if (d.reason[cell] == ScreenCrackDebug::kNotEvaluated) continue;
          const int ib = plane == 0 ? static_cast<int>(cell) + 1
                                    : static_cast<int>(cell) + W;
          const float vzNear = std::min(viewZ[cell], viewZ[ib]);
          const float px = pixelSizeAt(sp, vzNear);
          if (px <= 0.0f) continue;
          if (std::min(d.gapA[cell], d.gapB[cell]) <=
              0.25f * p.depthGapPx * px)
            continue;
          float nvA = 0.0f, nvB = 0.0f, nd = 0.0f;
          if (normal) {
            nvA = facingCos(normal, sp, static_cast<int>(cell));
            nvB = facingCos(normal, sp, ib);
            const float* na = normal + 3 * cell;
            const float* nb = normal + 3 * static_cast<std::size_t>(ib);
            const float la = std::sqrt(na[0] * na[0] + na[1] * na[1] +
                                       na[2] * na[2]);
            const float lb = std::sqrt(nb[0] * nb[0] + nb[1] * nb[1] +
                                       nb[2] * nb[2]);
            if (la > 1.0e-6f && lb > 1.0e-6f)
              nd = 1.0f - (na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2]) /
                              (la * lb);
          }
          std::fprintf(f, "%c,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,"
                          "%.4f,%.5f\n",
                       plane == 0 ? 'r' : 'd', x, y, d.reason[cell],
                       d.gapA[cell] / px, d.gapB[cell] / px, d.sA[cell] / px,
                       d.sB[cell] / px, d.g0[cell] / px,
                       std::fabs(d.sA[cell] + d.sB[cell]) / px, nvA, nvB, nd);
          ++rows;
        }
      }
    }
    std::fclose(f);
    std::fprintf(stderr, "[screen-edges] dumped %s (%zu rows)\n", path.c_str(),
                 rows);
  }
}

}  // namespace

void applyScreenVectorEdges(FrameResult& frame, const Scene& scene,
                            const RenderOptions& opt) {
  const StrokeEdgeOptions& se = opt.strokeEdges;
  const int W = frame.width, H = frame.height;
  if (W <= 0 || H <= 0) return;
  if (frame.viewZ.empty() || frame.objectId.empty()) return;

  const ScreenProj sp = makeScreenProj(scene.camera, W, H);
  const float ssScale = static_cast<float>(std::max(1, opt.supersample));

  // Stage 1: classify. The nature master toggles gate the classes here (the
  // shared draw stage applies only the per-section style table).
  ScreenClassifyParams cp;
  cp.silhouette = se.silhouette;
  cp.objectBoundary = se.border;
  cp.crease = se.crease;
  cp.depthGapPx = se.screenDepthGapPx;
  cp.slopeClampPx = se.screenSlopeClampPx;
  cp.creaseAngleDeg = se.creaseAngleDeg;
  cp.grazeK = se.screenGrazeK;
  cp.bgClearancePx = static_cast<int>(std::lround(ssScale));
  const float* normalPtr = frame.normal.empty() ? nullptr : frame.normal.data();
  if (cp.crease && !normalPtr) cp.crease = false;
  const char* dumpPrefix = std::getenv("UMBREON_SCREEN_EDGE_DUMP");
  ScreenCrackDebug dbg;
  CrackField cf = classifyCracks(W, H, frame.viewZ.data(),
                                 frame.objectId.data(), normalPtr, sp, cp,
                                 dumpPrefix ? &dbg : nullptr);
  if (dumpPrefix)
    writeCrackDump(dumpPrefix, cf, dbg, frame.viewZ.data(),
                   frame.objectId.data(), normalPtr, sp, cp);

  // Stage 2: trace, then Stage 2.5: hysteresis prune + retrace. The optional
  // surfAlpha AOV attributes per-vertex surface opacity so transparent
  // sections / alpha-graded fragments fade their edges accordingly.
  const float* surfAlphaPtr =
      frame.surfAlpha.empty() ? nullptr : frame.surfAlpha.data();
  std::vector<ScreenChain> traced =
      traceCrackChains(cf, frame.viewZ.data(), frame.objectId.data(),
                       surfAlphaPtr);
  const std::size_t tracedRaw = traced.size();
  // Self-support needs ~2 FINAL px of strong evidence so a lone borderline
  // crack cannot resurrect an isolated sliver as a dash.
  const int minStrong = std::max(1, static_cast<int>(std::lround(
                                        2.0f * ssScale)));
  traced = pruneWeakChains(cf, std::move(traced), frame.viewZ.data(),
                           frame.objectId.data(), minStrong, surfAlphaPtr);

  // Stage 3+4 per chain: speck filter (whole chain), class-run relabel +
  // split, per-run geometry cleanup, slot mapping.
  const float minChainLen = se.screenMinLenPx * ssScale;
  const int mergeLen = std::max(
      0, static_cast<int>(std::lround(se.screenClassMergeLen * ssScale)));
  const float rdpEps = se.screenSimplifyPx * ssScale;
  const bool perSection = !scene.groupEdgeStyle.empty();

  std::vector<StrokeChainInput> drawChains;
  for (const ScreenChain& ch : traced) {
    if (ch.pts.size() < 2 || ch.edgeClass.empty()) continue;
    // Speck filter on the RAW chain: every edgel is one hi-res px long, so the
    // edgel count IS the arc length. JUNCTION-AWARE: a short chain whose ends
    // are both junctions (degree >= 3) is a piece of a larger boundary chopped
    // by side-branches (e.g. grazing-rim depth-gap spurs T-ing into the
    // silhouette) and is KEPT -- dropping it would dash the outline. Only a
    // short chain with a free end (a spur) or a tiny closed loop is an
    // isolated speckle and is dropped.
    if (minChainLen > 0.0f &&
        static_cast<float>(ch.edgeClass.size()) < minChainLen &&
        !(ch.deg0 >= 3 && ch.deg1 >= 3))
      continue;

    // Class-run relabel (labels only), then split into maximal same-class
    // runs. Adjacent runs share their boundary vertex, so the drawn geometry
    // stays continuous across a style change.
    std::vector<std::uint8_t> cls = ch.edgeClass;
    mergeShortClassRuns(cls, mergeLen);

    std::size_t e0 = 0;
    while (e0 < cls.size()) {
      std::size_t e1 = e0;
      while (e1 < cls.size() && cls[e1] == cls[e0]) ++e1;
      const CrackClass runClass = static_cast<CrackClass>(cls[e0]);
      // A run spanning the whole closed loop keeps the cyclic treatment.
      const bool runClosed = ch.closed && e0 == 0 && e1 == cls.size();

      StrokeChainInput in;
      in.group = ch.edgeGroup[e0];
      in.precedence = classPrecedence(runClass);
      in.styleSlot = classStyleSlot(runClass);
      // DepthGap falls back to the Silhouette slot when the section never
      // configured the Disconnected class (the default style table ships all
      // slots disabled except those the CLI enables; without the fallback a
      // same-id occlusion boundary would silently vanish).
      if (perSection &&
          runClass == CrackClass::DepthGap) {
        float h, c[3], o;
        if (!resolveStrokeStyle(scene, se, ssScale, in.styleSlot, in.group, h,
                                c, o))
          in.styleSlot = static_cast<int>(EdgeClass::Silhouette);
      }

      // Geometry cleanup on the run's vertex slice [e0, e1].
      std::vector<ScreenChainVert> pts(ch.pts.begin() + e0,
                                       ch.pts.begin() + e1 + 1);
      // Re-attribute vertex alpha from the run's OWN edgels. The chain-level
      // vertex alpha blends the two adjacent edgels regardless of run
      // membership, so a run-boundary vertex inherits half of the
      // neighboring run's owner opacity (e.g. an opaque stick border
      // junctioning into an edge on a fully transparent surface pushed that
      // endpoint to 0.5, and the draw stage lerped the leak across the whole
      // segment). Within the run, interior vertices still average their two
      // in-run edgels; the endpoints take their single in-run edgel, so the
      // run's opacity is a function of its own surface only.
      if (!ch.edgeAlpha.empty()) {
        const std::size_t nV = pts.size();
        for (std::size_t k = 0; k < nV; ++k) {
          float a;
          if (runClosed && (k == 0 || k == nV - 1)) {
            a = 0.5f * (ch.edgeAlpha[e0] + ch.edgeAlpha[e1 - 1]);
          } else if (k == 0) {
            a = ch.edgeAlpha[e0];
          } else if (k == nV - 1) {
            a = ch.edgeAlpha[e1 - 1];
          } else {
            a = 0.5f * (ch.edgeAlpha[e0 + k - 1] + ch.edgeAlpha[e0 + k]);
          }
          pts[k].alpha = a;
        }
      }
      collapseCollinear(pts, runClosed);
      chaikinSmooth(pts, runClosed, se.screenSmoothIters);
      simplifyRdp(pts, runClosed, rdpEps);
      if (pts.size() < 2) {
        e0 = e1;
        continue;
      }
      in.pts.reserve(pts.size());
      for (const ScreenChainVert& v : pts)
        in.pts.push_back({v.x, v.y, v.vz, v.alpha, true});
      drawChains.push_back(std::move(in));
      e0 = e1;
    }
  }

  // Tuning aid (env-gated, zero-cost when unset): one stats line per frame;
  // a value of 2+ also lists every kept chain (bbox, class mix, strong
  // count) for artifact hunting.
  const char* dbgEnv = std::getenv("UMBREON_SCREEN_EDGE_DEBUG");
  if (dbgEnv && std::atoi(dbgEnv) >= 2) {
    for (std::size_t ci = 0; ci < traced.size(); ++ci) {
      const ScreenChain& ch = traced[ci];
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      for (const ScreenChainVert& v : ch.pts) {
        x0 = std::min(x0, v.x);
        y0 = std::min(y0, v.y);
        x1 = std::max(x1, v.x);
        y1 = std::max(y1, v.y);
      }
      int cc[5] = {0, 0, 0, 0, 0};
      std::size_t st = 0;
      for (std::uint8_t c : ch.edgeClass)
        if (c < 5) ++cc[c];
      for (std::uint8_t f : ch.edgeFlags) st += (f & 1);
      std::fprintf(stderr,
                   "[screen-edges]   chain %zu bbox=(%.0f,%.0f)-(%.0f,%.0f) "
                   "edgels=%zu sil=%d obj=%d gap=%d crease=%d strong=%zu "
                   "closed=%d deg=%d/%d\n",
                   ci, x0, y0, x1, y1, ch.edgeClass.size(), cc[1], cc[2],
                   cc[3], cc[4], st, ch.closed ? 1 : 0, ch.deg0, ch.deg1);
    }
  }
  if (dbgEnv) {
    std::size_t nEdgels = 0, nPts = 0, nStrong = 0;
    int clsCount[5] = {0, 0, 0, 0, 0};
    for (const ScreenChain& ch : traced) {
      nEdgels += ch.edgeClass.size();
      for (std::uint8_t c : ch.edgeClass)
        if (c < 5) ++clsCount[c];
      for (std::uint8_t f : ch.edgeFlags) nStrong += (f & 1);
    }
    for (const StrokeChainInput& in : drawChains) nPts += in.pts.size();
    std::fprintf(stderr,
                 "[screen-edges] traced=%zu kept=%zu edgels=%zu (sil=%d "
                 "obj=%d gap=%d crease=%d) gapStrong=%zu drawn=%zu pts=%zu\n",
                 tracedRaw, traced.size(), nEdgels, clsCount[1], clsCount[2],
                 clsCount[3], clsCount[4], nStrong, drawChains.size(), nPts);
  }

  renderStrokeChains(frame, scene, opt, drawChains);
}

}  // namespace umbreon
