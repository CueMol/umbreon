// SCREEN-SPACE VECTOR edges, Stage 4 driver: split chains into class runs,
// map them onto the EdgeStyle slots and hand them to the shared draw stage
// (stroke_render.hpp:renderStrokeChains). Also the UMBREON_SCREEN_EDGE_DUMP
// debug sink (PPM/CSV crack dumps). Stages 1-3 live in screen_edge_*.cpp;
// see screen_vector_edges.hpp for the pipeline overview.
#include "../log.hpp"
#include "edges/screen_vector_edges.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

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

// ---------------------------------------------------------------------------
// Debug dump (UMBREON_SCREEN_EDGE_DUMP=<prefix>): writes <prefix>_cracks.ppm
// (crack lattice colorized by class / kill reason over a viewZ gray base) and
// <prefix>_cracks.csv (per-crack DepthGap diagnostics, px-normalized).
// Optional UMBREON_SCREEN_EDGE_DUMP_ROI=x0,y0,x1,y1 restricts both outputs
// to a hi-res pixel rectangle. Diagnostic-only; never on in normal runs.

// Crack color by class byte, or by kill reason for un-inked candidates whose
// min gap exceeds `noiseFloor` (world units at that crack). Returns false for
// "draw nothing". `probeVal` is the fold-probe outcome (ScreenCrackDebugPlane
// ::probe): a weak crack whose rescue the probe vetoed draws magenta.
inline bool crackColor(std::uint8_t byte, std::uint8_t reason, float gapA,
                       float gapB, float noiseFloor, std::uint8_t probeVal,
                       std::uint8_t rgb[3]) {
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
      } else if (probeVal == 2) {
        rgb[0] = 255, rgb[1] = 0, rgb[2] = 255;  // probe-vetoed fold: magenta
      } else if (byte & kCrackRidgeBit) {
        rgb[0] = 0, rgb[1] = 128, rgb[2] = 96;  // ridge crease (weak): teal
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
  // Clip-cut veto has no DepthGap gaps to clear the noise floor: color it
  // unconditionally (brown) so cut boundaries are visible in the dump.
  if (reason == ScreenCrackDebug::kClipVeto) {
    rgb[0] = 160, rgb[1] = 96, rgb[2] = 0;  // brown
    return true;
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
                       dbg.right.probe[cell], rgb) &&
            ix + 1 < PW)
          put(ix + 1, iy, rgb[0], rgb[1], rgb[2]);
        if (y + 1 < H &&
            crackColor(cf.down[cell], dbg.down.reason[cell],
                       dbg.down.gapA[cell], dbg.down.gapB[cell], noiseFloor,
                       dbg.down.probe[cell], rgb) &&
            iy + 1 < PH)
          put(ix, iy + 1, rgb[0], rgb[1], rgb[2]);
      }
    }
    const std::string path = base + ".ppm";
    if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
      std::fprintf(f, "P6\n%d %d\n255\n", PW, PH);
      std::fwrite(img.data(), 1, img.size(), f);
      std::fclose(f);
      umbreon::logMessage(umbreon::LogLevel::Info, "[screen-edges] dumped %s (%dx%d)", path.c_str(),
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
                 "slopesum_px,ndotvA,ndotvB,ndelta,probe\n");
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
                          "%.4f,%.5f,%d\n",
                       plane == 0 ? 'r' : 'd', x, y, d.reason[cell],
                       d.gapA[cell] / px, d.gapB[cell] / px, d.sA[cell] / px,
                       d.sB[cell] / px, d.g0[cell] / px,
                       std::fabs(d.sA[cell] + d.sB[cell]) / px, nvA, nvB, nd,
                       d.probe[cell]);
          ++rows;
        }
      }
    }
    std::fclose(f);
    umbreon::logMessage(umbreon::LogLevel::Info, "[screen-edges] dumped %s (%zu rows)", path.c_str(),
                 rows);
  }
}

}  // namespace

// See the header for the contract. The run key is the (class, group) PAIR:
// a short middle run relabels (both arrays) only when the bracketing runs
// share one identical key, so flicker inside one section still fuses while
// a genuine section change never does. Linear scan; extending the left run
// and continuing forward keeps the pass linear and the result deterministic
// (a re-scan from the previous run start would be quadratic). A closed
// chain's seam-straddling run pair is left as-is (harmless: one extra style
// split).
void mergeShortClassRuns(std::vector<std::uint8_t>& cls,
                         std::vector<std::uint16_t>& grp, int minLen) {
  if (minLen <= 1 || cls.size() < 3 || grp.size() != cls.size()) return;
  std::size_t i = 0;
  while (i < cls.size()) {
    std::size_t j = i;
    while (j < cls.size() && cls[j] == cls[i] && grp[j] == grp[i]) ++j;
    const std::size_t runLen = j - i;
    if (i > 0 && j < cls.size() && runLen < static_cast<std::size_t>(minLen) &&
        cls[i - 1] == cls[j] && grp[i - 1] == grp[j]) {
      for (std::size_t k = i; k < j; ++k) {
        cls[k] = cls[i - 1];
        grp[k] = grp[i - 1];
      }
    }
    i = j;
  }
}

void applyScreenVectorEdges(FrameResult& frame, const Scene& scene,
                            const RenderOptions& opt,
                            const OcclusionQuery& occluded) {
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
  cp.contactBoundary = se.contact;
  cp.crease = se.crease;
  cp.depthGapPx = se.screenDepthGapPx;
  cp.slopeClampPx = se.screenSlopeClampPx;
  cp.creaseAngleDeg = se.creaseAngleDeg;
  cp.grazeK = se.screenGrazeK;
  cp.bgClearancePx = static_cast<int>(std::lround(ssScale));
  // Per-section silhouette mode table (indexed by group id); classification
  // reads it via objectId >> 2. Kept alive across the classifyCracks call.
  std::vector<SilhouetteMode> groupMode;
  groupMode.reserve(scene.groupEdgeStyle.size());
  for (const EdgeStyle& es : scene.groupEdgeStyle)
    groupMode.push_back(es.silhouetteMode);
  cp.groupSilhMode = groupMode.empty() ? nullptr : groupMode.data();
  cp.groupSilhModeCount = groupMode.size();
  cp.silhModeDefault = se.defaultStyle.silhouetteMode;
  const float* normalPtr = frame.normal.empty() ? nullptr : frame.normal.data();
  if (cp.crease && !normalPtr) cp.crease = false;
  const char* dumpPrefix = std::getenv("UMBREON_SCREEN_EDGE_DUMP");
  ScreenCrackDebug dbg;
  // Clip-cut G-buffer planes: present only when the scene's view-clip planes
  // are set (see FrameResult); boundaries the planes cut stay line-free.
  // The interior flag is DILATED (Chebyshev, one output pixel) before use:
  // along a cut rim the sampling can land on 1-2 hi-res px slivers of the
  // object's own near-edge-on SIDE WALL -- frontface hits that carry no
  // clipCut flag but are as much a slab artifact as the interior they hug.
  // Every such sliver lies within a pixel of flagged interior, so the
  // dilated mask absorbs it (and any crack it forms) into the veto.
  ScreenClipAovs clipAovs;
  const bool hasClip = !frame.clipCut.empty();
  std::vector<std::uint8_t> cutDilated;
  if (hasClip) {
    const int r = static_cast<int>(std::lround(ssScale));
    const std::uint8_t* src = frame.clipCut.data();
    cutDilated.assign(static_cast<std::size_t>(W) * H, 0);
    std::vector<std::uint8_t> tmp(static_cast<std::size_t>(W) * H, 0);
    for (int y = 0; y < H; ++y) {  // horizontal max filter
      const std::size_t row = static_cast<std::size_t>(y) * W;
      for (int x = 0; x < W; ++x) {
        std::uint8_t v = 0;
        for (int k = std::max(0, x - r); k <= std::min(W - 1, x + r); ++k)
          v |= src[row + k];
        tmp[row + x] = v;
      }
    }
    for (int y = 0; y < H; ++y) {  // vertical max filter
      for (int x = 0; x < W; ++x) {
        std::uint8_t v = 0;
        for (int k = std::max(0, y - r); k <= std::min(H - 1, y + r); ++k)
          v |= tmp[static_cast<std::size_t>(k) * W + x];
        cutDilated[static_cast<std::size_t>(y) * W + x] = v;
      }
    }
    clipAovs.cut = cutDilated.data();
    clipAovs.nearVz = frame.clipNearVz.data();
    clipAovs.farVz = frame.clipFarVz.data();
    cp.clipNearVz = scene.clipNear;
    cp.clipFarVz = scene.clipFar;
  }
  CrackField cf = classifyCracks(W, H, frame.viewZ.data(),
                                 frame.objectId.data(), normalPtr, sp, cp,
                                 dumpPrefix ? &dbg : nullptr,
                                 occluded ? &occluded : nullptr,
                                 hasClip ? &clipAovs : nullptr);
  if (dumpPrefix) {
    writeCrackDump(dumpPrefix, cf, dbg, frame.viewZ.data(),
                   frame.objectId.data(), normalPtr, sp, cp);
    // Raw clip-cut planes for offline analysis (full frame, debug only).
    if (hasClip) {
      const std::size_t n = static_cast<std::size_t>(W) * H;
      const std::string base = std::string(dumpPrefix) + "_clip";
      if (std::FILE* f = std::fopen((base + "_cut.u8").c_str(), "wb")) {
        std::fwrite(clipAovs.cut, 1, n, f);
        std::fclose(f);
      }
      if (std::FILE* f = std::fopen((base + "_nearvz.f32").c_str(), "wb")) {
        std::fwrite(clipAovs.nearVz, sizeof(float), n, f);
        std::fclose(f);
      }
      if (std::FILE* f = std::fopen((base + "_farvz.f32").c_str(), "wb")) {
        std::fwrite(clipAovs.farVz, sizeof(float), n, f);
        std::fclose(f);
      }
    }
  }

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

  // Debug level 3+: one line per drawn run (side / taper / clip wiring);
  // level 4+ additionally dumps every drawn polyline's node coordinates.
  const int dbgLevel = [] {
    const char* e = std::getenv("UMBREON_SCREEN_EDGE_DEBUG");
    return e ? std::atoi(e) : 0;
  }();
  const bool dbgRuns = dbgLevel >= 3;
  const bool dbgPts = dbgLevel >= 4;

  // Helpers shared by the junction stages and Stage 4 below.
  const bool perSection = !scene.groupEdgeStyle.empty();
  // Per-section alignment (same two-tier rule as resolveStrokeStyle).
  auto alignFor = [&](std::uint16_t g) {
    return scene.groupEdgeStyle.empty()
               ? se.align
               : (g < scene.groupEdgeStyle.size() ? scene.groupEdgeStyle[g]
                                                  : se.defaultStyle)
                     .align;
  };
  // Resolved draw style of a (class, group) run, DepthGap -> sil slot
  // fallback included; ok == false when the slot is disabled (such a run is
  // skipped at draw time anyway).
  struct RunStyle {
    bool ok = false;
    int slot = 0;
    float half = 0.0f;
    float color[3] = {0.0f, 0.0f, 0.0f};
    float opacity = 1.0f;
  };
  auto styleFor = [&](CrackClass c, std::uint16_t g) {
    RunStyle rs;
    rs.slot = classStyleSlot(c);
    rs.ok = resolveStrokeStyle(scene, se, ssScale, rs.slot, g, rs.half,
                               rs.color, rs.opacity);
    if (!rs.ok && perSection && c == CrackClass::DepthGap) {
      rs.slot = static_cast<int>(EdgeClass::Silhouette);
      rs.ok = resolveStrokeStyle(scene, se, ssScale, rs.slot, g, rs.half,
                                 rs.color, rs.opacity);
    }
    return rs;
  };
  // Resolved HALF width alone (0 when disabled).
  auto halfFor = [&](CrackClass c, std::uint16_t g) {
    return styleFor(c, g).half;
  };

  // ---- Stage 3.5: JUNCTION WEAVING (topology reconstruction) --------------
  // The tracer splits chains at every lattice corner of degree >= 3, so the
  // BAR of a T junction arrives as two chains. Drawn separately, the halves
  // smooth independently (endpoints pinned at the corner) and the joint
  // shows: as a lateral step of the offset band under the outside
  // alignment, and as a kink/node pair under center. Weave the split back
  // together: at each corner where chain ends meet, the pair(s) of ends
  // that continue each other nearly straight merge into ONE chain
  // (reversing as needed; the walk-relative outer-side bits flip with the
  // reversal), so the bar draws as a single stroke -- one smoothing pass,
  // one unbroken band -- whether or not stems attach. Unpaired ends are
  // true STEMS; each junction's woven bar direction / band side / style key
  // are recorded (BarInfo) for the outside alignment's stem clip in
  // Stage 4. This stage is TOPOLOGY, not styling: it runs for every
  // alignment, so center and outside share one extracted line structure
  // and differ only in band placement.
  struct BarInfo {
    float dx = 0.0f, dy = 0.0f;    // bar local direction (unit, sign free)
    float bnx = 0.0f, bny = 0.0f;  // unit normal toward the band (0 = unknown)
    CrackClass cls = CrackClass::Silhouette;  // style key of the bar
    std::uint16_t grp = 0;
  };
  std::unordered_map<long, BarInfo> barAt;
  if (!traced.empty()) {
    const long cornerW = static_cast<long>(cf.W) + 1;
    auto cornerIdOf = [&](const ScreenChain& c, int end) {
      const ScreenChainVert& v = end == 0 ? c.pts.front() : c.pts.back();
      return static_cast<long>(std::lround(v.y + 0.5f)) * cornerW +
             static_cast<long>(std::lround(v.x + 0.5f));
    };
    auto dirOf = [&](const ScreenChain& c, int end) {
      // Direction from the end corner INTO the chain over an ~8 px window.
      const std::size_t n = c.pts.size();
      const std::size_t k = std::min<std::size_t>(8, n - 1);
      float dx, dy;
      if (end == 0) {
        dx = c.pts[k].x - c.pts.front().x;
        dy = c.pts[k].y - c.pts.front().y;
      } else {
        dx = c.pts[n - 1 - k].x - c.pts.back().x;
        dy = c.pts[n - 1 - k].y - c.pts.back().y;
      }
      const float l = std::sqrt(dx * dx + dy * dy);
      return l > 1.0e-5f ? std::array<float, 2>{dx / l, dy / l}
                         : std::array<float, 2>{0.0f, 0.0f};
    };
    // Walk-relative outer-side vote over the ~8 edgels at an end: +1 = the
    // outer side is walk-LEFT there, 0 = unknown/contact.
    auto sideOf = [&](const ScreenChain& c, int end) -> int {
      if (c.edgeFlags.size() != c.edgeClass.size()) return 0;
      const std::size_t n = c.edgeFlags.size();
      const std::size_t k = std::min<std::size_t>(8, n);
      long vote = 0;
      for (std::size_t i = 0; i < k; ++i) {
        const std::uint8_t f = c.edgeFlags[end == 0 ? i : n - 1 - i];
        if (f & 4) continue;
        vote += (f & 8) ? 1 : -1;
      }
      return vote > 0 ? 1 : (vote < 0 ? -1 : 0);
    };
    // Orientation helper: a merged chain is built end0 -> end1; a segment
    // entered at its end1 is reversed (flipping the walk-relative
    // outer-side bit 3 of edgeFlags).
    auto oriented = [&](const ScreenChain& src, bool reverse) {
      ScreenChain c = src;
      if (reverse) {
        std::reverse(c.pts.begin(), c.pts.end());
        std::reverse(c.edgeClass.begin(), c.edgeClass.end());
        std::reverse(c.edgeGroup.begin(), c.edgeGroup.end());
        if (c.edgeVz.size() == c.edgeClass.size())
          std::reverse(c.edgeVz.begin(), c.edgeVz.end());
        if (c.edgeAlpha.size() == c.edgeClass.size())
          std::reverse(c.edgeAlpha.begin(), c.edgeAlpha.end());
        if (c.edgeFlags.size() == c.edgeClass.size()) {
          std::reverse(c.edgeFlags.begin(), c.edgeFlags.end());
          for (std::uint8_t& f : c.edgeFlags) f ^= 8;
        }
        std::swap(c.deg0, c.deg1);
      }
      return c;
    };

    // ---- Junction RE-WIRING (through-chain splitting) --------------------
    // A stem whose lattice cracks die a few px short of a junction (the
    // depth-continuity veto near a contact) leaves that junction as a
    // degree-2 corner: the tracer walks the other two branches straight
    // through as ONE chain even when they turn hard there, because they
    // bound the same REGION -- e.g. the background rim hopping off the
    // front object's contour onto the far object's. The drawn bar then
    // bends off sideways at the junction and the stem's end leaves a wedge
    // gap at the turn. Re-wire such junctions by GEOMETRY: probe each free
    // end; when it lands on another chain's INTERIOR and continues one of
    // its halves nearly straight -- clearly straighter than that chain's
    // own turn -- split the chain there and splice the stem onto the
    // continuing half (bridging the few-px crack gap). The other half
    // becomes a stem ending at the junction; the re-wired bar's BarInfo is
    // recorded so that stem clips against it in Stage 4. One re-wire per
    // chain per round (the splice invalidates the indices), iterated to a
    // fixed point so a long chain with several junctions handles them all.
    for (int rewireRound = 0; rewireRound < 4; ++rewireRound) {
      // Crack cell -> (chain, edgel) over all traced chains (unit steps).
      const long planeStride = static_cast<long>(cf.W) * cf.H;
      std::unordered_map<long, std::pair<std::uint32_t, std::uint32_t>>
          crackOwner;
      for (std::size_t i = 0; i < traced.size(); ++i) {
        const ScreenChain& c = traced[i];
        for (std::size_t e = 0; e + 1 < c.pts.size(); ++e) {
          const int c0x = static_cast<int>(std::lround(c.pts[e].x + 0.5f));
          const int c0y = static_cast<int>(std::lround(c.pts[e].y + 0.5f));
          const int c1x =
              static_cast<int>(std::lround(c.pts[e + 1].x + 0.5f));
          const int c1y =
              static_cast<int>(std::lround(c.pts[e + 1].y + 0.5f));
          long key = -1;
          if (c0x == c1x) {
            const int cy2 = std::min(c0y, c1y);
            if (c0x - 1 >= 0 && c0x - 1 < cf.W && cy2 >= 0 && cy2 < cf.H)
              key = static_cast<long>(cy2) * cf.W + (c0x - 1);
          } else if (c0y == c1y) {
            const int cx2 = std::min(c0x, c1x);
            if (cx2 >= 0 && cx2 < cf.W && c0y - 1 >= 0 && c0y - 1 < cf.H)
              key = planeStride + static_cast<long>(c0y - 1) * cf.W + cx2;
          }
          if (key >= 0)
            crackOwner[key] = {static_cast<std::uint32_t>(i),
                               static_cast<std::uint32_t>(e)};
        }
      }

      struct Rewire {
        std::size_t stem;
        int stemEnd;
        std::size_t target;
        std::size_t j;      // target vertex index at the junction
        bool continueHigh;  // splice with the [j..] half
      };
      std::vector<Rewire> cands;
      const std::size_t K = 8;
      for (std::size_t i = 0; i < traced.size(); ++i) {
        const ScreenChain& c = traced[i];
        if (c.closed || c.pts.size() < 6 || c.edgeClass.empty()) continue;
        for (int end = 0; end < 2; ++end) {
          if ((end == 0 ? c.deg0 : c.deg1) > 1) continue;
          const std::array<float, 2> din = dirOf(c, end);
          const float ox = -din[0], oy = -din[1];  // stem walk, outward
          if (ox == 0.0f && oy == 0.0f) continue;
          const std::size_t ei0 = end == 0 ? 0 : c.edgeClass.size() - 1;
          const float half = halfFor(
              static_cast<CrackClass>(c.edgeClass[ei0]),
              c.edgeGroup.size() == c.edgeClass.size() ? c.edgeGroup[ei0]
                                                       : 0);
          const int R = std::min(8L, std::max(2L, std::lround(half) + 2));
          const ScreenChainVert& ev = end == 0 ? c.pts.front() : c.pts.back();
          const float ecx = ev.x + 0.5f, ecy = ev.y + 0.5f;
          const int icx = static_cast<int>(std::lround(ecx));
          const int icy = static_cast<int>(std::lround(ecy));
          float bestD = -1.0f;
          std::size_t bestChain = 0, bestEdgel = 0;
          for (int yy = icy - R; yy <= icy + R; ++yy) {
            if (yy < 0 || yy >= cf.H) continue;
            for (int xx = icx - R; xx <= icx + R; ++xx) {
              if (xx < 0 || xx >= cf.W) continue;
              const std::size_t cell =
                  static_cast<std::size_t>(yy) * cf.W + xx;
              for (int plane = 0; plane < 2; ++plane) {
                const std::uint8_t byte =
                    plane == 0 ? cf.right[cell] : cf.down[cell];
                if (!(byte & kCrackClassMask)) continue;
                const auto it = crackOwner.find(
                    plane == 0 ? static_cast<long>(cell)
                               : planeStride + static_cast<long>(cell));
                if (it == crackOwner.end() || it->second.first == i)
                  continue;
                const float mx = plane == 0 ? xx + 1.0f : xx + 0.5f;
                const float my = plane == 0 ? yy + 0.5f : yy + 1.0f;
                const float vx = mx - ecx, vy = my - ecy;
                const float d = std::sqrt(vx * vx + vy * vy);
                if (d <= 0.5f || d > static_cast<float>(R) + 0.5f) continue;
                if (vx * ox + vy * oy < 0.7071f * d) continue;
                if (bestD < 0.0f || d < bestD) {
                  bestD = d;
                  bestChain = it->second.first;
                  bestEdgel = it->second.second;
                }
              }
            }
          }
          if (bestD < 0.0f) continue;
          const ScreenChain& X = traced[bestChain];
          if (X.closed) continue;  // splitting a loop: out of scope
          auto unitTo = [&](std::size_t a, std::size_t b) {
            const float dx = X.pts[b].x - X.pts[a].x;
            const float dy = X.pts[b].y - X.pts[a].y;
            const float l = std::sqrt(dx * dx + dy * dy);
            return l > 1.0e-5f ? std::array<float, 2>{dx / l, dy / l}
                               : std::array<float, 2>{0.0f, 0.0f};
          };
          auto throughAt = [&](std::size_t v) {
            const std::array<float, 2> a = unitTo(v, v - K);
            const std::array<float, 2> b = unitTo(v, v + K);
            return -(a[0] * b[0] + a[1] * b[1]);
          };
          // The nearest crack usually lands a few px up one LEG of the
          // junction, where the chain looks locally straight; anchor at the
          // SHARPEST corner of the chain near the hit instead (minimum
          // through-straightness) -- that is the junction the stem points
          // at.
          std::size_t j = 0;
          {
            const std::size_t lo = std::max<std::size_t>(
                K, bestEdgel > static_cast<std::size_t>(2 * R)
                       ? bestEdgel - 2 * R
                       : 0);
            const std::size_t hi = std::min(
                bestEdgel + 2 * static_cast<std::size_t>(R),
                X.pts.size() >= K + 1 ? X.pts.size() - 1 - K : 0);
            if (lo > hi) continue;
            float minThrough = std::numeric_limits<float>::max();
            for (std::size_t v = lo; v <= hi; ++v) {
              const float t = throughAt(v);
              if (t < minThrough) {
                minThrough = t;
                j = v;
              }
            }
          }
          if (j < K || j + K >= X.pts.size()) continue;  // near an end:
                                                         // corner territory
          const std::array<float, 2> dirB = unitTo(j, j + K);
          const std::array<float, 2> dirA = unitTo(j, j - K);
          const float contB = ox * dirB[0] + oy * dirB[1];
          const float contA = ox * dirA[0] + oy * dirA[1];
          const float through = -(dirA[0] * dirB[0] + dirA[1] * dirB[1]);
          const bool useHigh = contB >= contA;
          const float cont = useHigh ? contB : contA;
          if (dbgLevel >= 3)
            std::fprintf(stderr,
                         "[screen-edges]   rewire? stem=%zu end=%d at "
                         "(%.1f,%.1f) -> ch=%zu j=%zu cont=%.2f "
                         "through=%.2f high=%d\n",
                         i, end, ecx - 0.5f, ecy - 0.5f, bestChain, j, cont,
                         through, useHigh ? 1 : 0);
          // Re-wire only at a GENUINE corner of the met chain (it does not
          // continue straight itself) that the stem continues clearly
          // straighter.
          if (cont < 0.82f || through >= 0.82f || cont < through + 0.1f)
            continue;
          cands.push_back({i, end, bestChain, j, useHigh});
        }
      }

      std::vector<char> touched(traced.size(), 0);
      std::size_t applied = 0;
      for (const Rewire& rw : cands) {
        if (touched[rw.stem] || touched[rw.target]) continue;
        touched[rw.stem] = touched[rw.target] = 1;
        ++applied;
        const ScreenChain X = std::move(traced[rw.target]);
        const bool sized = X.edgeGroup.size() == X.edgeClass.size();
        auto slice = [&](std::size_t v0, std::size_t v1) {  // vertices
          ScreenChain part;
          part.pts.assign(X.pts.begin() + v0, X.pts.begin() + v1 + 1);
          part.edgeClass.assign(X.edgeClass.begin() + v0,
                                X.edgeClass.begin() + v1);
          if (sized)
            part.edgeGroup.assign(X.edgeGroup.begin() + v0,
                                  X.edgeGroup.begin() + v1);
          if (X.edgeVz.size() == X.edgeClass.size())
            part.edgeVz.assign(X.edgeVz.begin() + v0, X.edgeVz.begin() + v1);
          if (X.edgeAlpha.size() == X.edgeClass.size())
            part.edgeAlpha.assign(X.edgeAlpha.begin() + v0,
                                  X.edgeAlpha.begin() + v1);
          if (X.edgeFlags.size() == X.edgeClass.size())
            part.edgeFlags.assign(X.edgeFlags.begin() + v0,
                                  X.edgeFlags.begin() + v1);
          return part;
        };
        ScreenChain low = slice(0, rw.j);
        low.deg0 = X.deg0;
        low.deg1 = 3;
        ScreenChain high = slice(rw.j, X.pts.size() - 1);
        high.deg0 = 3;
        high.deg1 = X.deg1;
        ScreenChain M = oriented(traced[rw.stem], rw.stemEnd == 0);
        const int mSide = sideOf(M, 1);
        const std::size_t mLast = M.edgeClass.size() - 1;
        const std::array<float, 2> mDir = dirOf(M, 1);
        // BarInfo at the junction corner: the re-wired bar runs along the
        // stem's walk direction; band = walk-left * side.
        {
          const long cornerId =
              static_cast<long>(std::lround(X.pts[rw.j].y + 0.5f)) * cornerW +
              static_cast<long>(std::lround(X.pts[rw.j].x + 0.5f));
          BarInfo bar;
          bar.dx = -mDir[0];
          bar.dy = -mDir[1];
          if (mSide != 0) {
            bar.bnx = static_cast<float>(mSide) * mDir[1];
            bar.bny = static_cast<float>(mSide) * -mDir[0];
          }
          bar.cls = static_cast<CrackClass>(M.edgeClass[mLast]);
          bar.grp = M.edgeGroup.size() == M.edgeClass.size()
                        ? M.edgeGroup[mLast]
                        : 0;
          barAt[cornerId] = bar;
        }
        // Bridge edgel (the few px the depth-continuity veto killed):
        // duplicate the stem's last edgel attributes.
        M.edgeClass.push_back(M.edgeClass[mLast]);
        if (M.edgeGroup.size() == M.edgeClass.size() - 1)
          M.edgeGroup.push_back(M.edgeGroup[mLast]);
        if (M.edgeVz.size() == M.edgeClass.size() - 1)
          M.edgeVz.push_back(M.edgeVz[mLast]);
        if (M.edgeAlpha.size() == M.edgeClass.size() - 1)
          M.edgeAlpha.push_back(M.edgeAlpha[mLast]);
        if (M.edgeFlags.size() == M.edgeClass.size() - 1)
          M.edgeFlags.push_back(M.edgeFlags[mLast]);
        const ScreenChain C2 =
            rw.continueHigh ? std::move(high) : oriented(low, true);
        ScreenChain other = rw.continueHigh ? std::move(low) : std::move(high);
        M.pts.insert(M.pts.end(), C2.pts.begin(), C2.pts.end());
        M.edgeClass.insert(M.edgeClass.end(), C2.edgeClass.begin(),
                           C2.edgeClass.end());
        auto appendArr = [&](auto& dst, const auto& srcArr) {
          if (!dst.empty() && srcArr.size() == C2.edgeClass.size())
            dst.insert(dst.end(), srcArr.begin(), srcArr.end());
          else
            dst.clear();
        };
        appendArr(M.edgeGroup, C2.edgeGroup);
        appendArr(M.edgeVz, C2.edgeVz);
        appendArr(M.edgeAlpha, C2.edgeAlpha);
        appendArr(M.edgeFlags, C2.edgeFlags);
        M.deg1 = C2.deg1;
        traced[rw.stem] = std::move(M);
        traced[rw.target] = std::move(other);
      }
      if (applied == 0) break;
    }

    struct WeaveEnd {
      std::size_t chain;
      int end;  // 0 = start, 1 = end
    };
    std::unordered_map<long, std::vector<WeaveEnd>> byCorner;
    for (std::size_t i = 0; i < traced.size(); ++i) {
      const ScreenChain& c = traced[i];
      if (c.closed || c.pts.size() < 2 || c.edgeClass.empty()) continue;
      byCorner[cornerIdOf(c, 0)].push_back({i, 0});
      byCorner[cornerIdOf(c, 1)].push_back({i, 1});
    }
    // CLUSTER corners within a 2 px Chebyshev radius (union-find): a
    // junction between three surfaces is often spread over two or three
    // adjacent lattice corners, each of degree <= 3, and pairing per
    // EXACT corner cannot see the straight continuation whose two ends
    // sit on different corners of the same junction -- the leftovers then
    // pair among themselves and the bar hops onto the wrong line.
    std::vector<long> cids;
    cids.reserve(byCorner.size());
    for (const auto& kv : byCorner) cids.push_back(kv.first);
    std::unordered_map<long, std::size_t> cidIdx;
    for (std::size_t i = 0; i < cids.size(); ++i) cidIdx[cids[i]] = i;
    std::vector<std::size_t> ufParent(cids.size());
    for (std::size_t i = 0; i < cids.size(); ++i) ufParent[i] = i;
    auto findRoot = [&](std::size_t x) {
      while (ufParent[x] != x) {
        ufParent[x] = ufParent[ufParent[x]];
        x = ufParent[x];
      }
      return x;
    };
    for (std::size_t i = 0; i < cids.size(); ++i) {
      const long cy = cids[i] / cornerW, cx = cids[i] % cornerW;
      for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const auto it = cidIdx.find((cy + dy) * cornerW + (cx + dx));
          if (it == cidIdx.end()) continue;
          const std::size_t ra = findRoot(i), rb = findRoot(it->second);
          if (ra != rb) ufParent[rb] = ra;
        }
    }
    std::unordered_map<std::size_t, std::vector<WeaveEnd>> clusters;
    std::unordered_map<std::size_t, std::vector<long>> clusterCorners;
    for (std::size_t i = 0; i < cids.size(); ++i) {
      const std::size_t r = findRoot(i);
      const auto& srcEnds = byCorner[cids[i]];
      auto& dst = clusters[r];
      dst.insert(dst.end(), srcEnds.begin(), srcEnds.end());
      clusterCorners[r].push_back(cids[i]);
    }

    // Greedy best-opposite pairing per junction CLUSTER; partner[chain][end].
    constexpr float kContinueCos = -0.82f;  // ~145 deg or straighter
    std::vector<std::array<int, 2>> partnerChain(
        traced.size(), {-1, -1});
    std::vector<std::array<int, 2>> partnerEnd(traced.size(), {-1, -1});
    for (auto& kv : clusters) {
      std::vector<WeaveEnd>& ends = kv.second;
      if (ends.size() < 2) continue;
      std::vector<char> used(ends.size(), 0);
      bool haveBar = false;
      for (;;) {
        float best = kContinueCos;
        int bi = -1, bj = -1;
        for (std::size_t i = 0; i < ends.size(); ++i) {
          if (used[i]) continue;
          const std::array<float, 2> di =
              dirOf(traced[ends[i].chain], ends[i].end);
          for (std::size_t j = i + 1; j < ends.size(); ++j) {
            if (used[j]) continue;
            const std::array<float, 2> dj =
                dirOf(traced[ends[j].chain], ends[j].end);
            const float cosT = di[0] * dj[0] + di[1] * dj[1];
            if (cosT <= best) {
              best = cosT;
              bi = static_cast<int>(i);
              bj = static_cast<int>(j);
            }
          }
        }
        if (bi < 0) break;
        used[bi] = used[bj] = 1;
        const WeaveEnd& A = ends[bi];
        const WeaveEnd& B = ends[bj];
        if (dbgLevel >= 3) {
          const ScreenChainVert& va = A.end == 0 ? traced[A.chain].pts.front()
                                                 : traced[A.chain].pts.back();
          std::fprintf(stderr,
                       "[screen-edges]   weave pair at (%.1f,%.1f): "
                       "ch%zu.e%d + ch%zu.e%d cos=%.3f (of %zu ends)\n",
                       va.x, va.y, A.chain, A.end, B.chain, B.end, best,
                       ends.size());
        }
        partnerChain[A.chain][A.end] = static_cast<int>(B.chain);
        partnerEnd[A.chain][A.end] = B.end;
        partnerChain[B.chain][B.end] = static_cast<int>(A.chain);
        partnerEnd[B.chain][B.end] = A.end;
        if (!haveBar) {
          // Record the woven bar for this junction's stems: direction is
          // the mean of the two (opposed) end windows; the band normal
          // follows A's walk-relative side at the corner.
          haveBar = true;
          BarInfo bar;
          const std::array<float, 2> da = dirOf(traced[A.chain], A.end);
          const std::array<float, 2> db = dirOf(traced[B.chain], B.end);
          float dx = da[0] - db[0], dy = da[1] - db[1];
          const float l = std::sqrt(dx * dx + dy * dy);
          if (l > 1.0e-5f) {
            bar.dx = dx / l;
            bar.dy = dy / l;
            const int s = sideOf(traced[A.chain], A.end);
            if (s != 0) {
              // Walk direction at the corner: A.end == 0 leaves along da,
              // A.end == 1 arrives along -da; band = walk-left * side.
              const float wx = A.end == 0 ? da[0] : -da[0];
              const float wy = A.end == 0 ? da[1] : -da[1];
              bar.bnx = static_cast<float>(s) * -wy;
              bar.bny = static_cast<float>(s) * wx;
            }
            const ScreenChain& ca = traced[A.chain];
            const std::size_t ei =
                A.end == 0 ? 0 : ca.edgeClass.size() - 1;
            bar.cls = static_cast<CrackClass>(ca.edgeClass[ei]);
            bar.grp = ca.edgeGroup.size() == ca.edgeClass.size()
                          ? ca.edgeGroup[ei]
                          : 0;
            for (const long cid : clusterCorners[kv.first]) barAt[cid] = bar;
          }
        }
      }
    }

    // Concatenate along the pairings (orientation handling in `oriented`
    // above).
    std::vector<ScreenChain> woven;
    woven.reserve(traced.size());
    std::vector<char> consumed(traced.size(), 0);
    for (std::size_t i = 0; i < traced.size(); ++i) {
      if (consumed[i]) continue;
      const ScreenChain& c0 = traced[i];
      if (c0.closed || c0.pts.size() < 2 || c0.edgeClass.empty()) {
        woven.push_back(std::move(traced[i]));
        consumed[i] = 1;
        continue;
      }
      if (partnerChain[i][0] < 0 && partnerChain[i][1] < 0) {
        woven.push_back(std::move(traced[i]));
        consumed[i] = 1;
        continue;
      }
      // Find a free (unpartnered) entry end; a fully partnered component is
      // a cycle -- start anywhere and close the loop.
      std::size_t start = i;
      int entry = partnerChain[i][0] < 0 ? 0 : 1;
      bool cycle = false;
      {
        std::size_t cur = i;
        int in = 0;  // walk backward through end-0 partners to a free end
        std::size_t guard = 0;
        while (partnerChain[cur][in] >= 0) {
          const std::size_t nxt =
              static_cast<std::size_t>(partnerChain[cur][in]);
          const int nin = partnerEnd[cur][in];
          cur = nxt;
          in = 1 - nin;  // continue out the far end of the neighbor
          if (cur == i && ++guard > 0) {
            cycle = true;
            break;
          }
          if (guard++ > traced.size()) {
            cycle = true;
            break;
          }
        }
        if (!cycle) {
          start = cur;
          entry = in;
        }
      }
      ScreenChain merged;
      std::size_t cur = start;
      int in = entry;
      bool first = true;
      for (;;) {
        consumed[cur] = 1;
        const ScreenChain piece = oriented(traced[cur], in == 1);
        if (first) {
          merged = piece;
          first = false;
        } else {
          // Same corner: drop the piece's duplicated first point. Cluster
          // pairs can join ends on ADJACENT corners (1-2 px apart): keep
          // both vertices and add a BRIDGE edgel carrying the previous
          // edgel's attributes.
          const float gx = piece.pts.front().x - merged.pts.back().x;
          const float gy = piece.pts.front().y - merged.pts.back().y;
          const bool gapJoin = gx * gx + gy * gy > 0.56f;
          if (gapJoin && !merged.edgeClass.empty()) {
            const std::size_t ml = merged.edgeClass.size() - 1;
            merged.edgeClass.push_back(merged.edgeClass[ml]);
            if (merged.edgeGroup.size() == merged.edgeClass.size() - 1)
              merged.edgeGroup.push_back(merged.edgeGroup[ml]);
            if (merged.edgeVz.size() == merged.edgeClass.size() - 1)
              merged.edgeVz.push_back(merged.edgeVz[ml]);
            if (merged.edgeAlpha.size() == merged.edgeClass.size() - 1)
              merged.edgeAlpha.push_back(merged.edgeAlpha[ml]);
            if (merged.edgeFlags.size() == merged.edgeClass.size() - 1)
              merged.edgeFlags.push_back(merged.edgeFlags[ml]);
          }
          merged.pts.insert(merged.pts.end(),
                            piece.pts.begin() + (gapJoin ? 0 : 1),
                            piece.pts.end());
          merged.edgeClass.insert(merged.edgeClass.end(),
                                  piece.edgeClass.begin(),
                                  piece.edgeClass.end());
          merged.edgeGroup.insert(merged.edgeGroup.end(),
                                  piece.edgeGroup.begin(),
                                  piece.edgeGroup.end());
          auto appendOpt = [&](std::vector<float>& dst,
                               const std::vector<float>& srcv) {
            if (!dst.empty() && !srcv.empty())
              dst.insert(dst.end(), srcv.begin(), srcv.end());
            else
              dst.clear();
          };
          appendOpt(merged.edgeVz, piece.edgeVz);
          appendOpt(merged.edgeAlpha, piece.edgeAlpha);
          if (!merged.edgeFlags.empty() && !piece.edgeFlags.empty())
            merged.edgeFlags.insert(merged.edgeFlags.end(),
                                    piece.edgeFlags.begin(),
                                    piece.edgeFlags.end());
          else
            merged.edgeFlags.clear();
          merged.deg1 = piece.deg1;
        }
        const int out = 1 - in;
        const int nxtChain = partnerChain[cur][out];
        if (nxtChain < 0) break;
        const int nxtEnd = partnerEnd[cur][out];
        cur = static_cast<std::size_t>(nxtChain);
        in = nxtEnd;
        if (consumed[cur]) {
          merged.closed = true;
          break;
        }
      }
      // A cycle closed across a cluster gap: restore the closed-chain
      // convention (pts.front() == pts.back()) with a bridge segment.
      if (merged.closed && merged.pts.size() >= 2) {
        const float gx = merged.pts.front().x - merged.pts.back().x;
        const float gy = merged.pts.front().y - merged.pts.back().y;
        if (gx * gx + gy * gy > 1.0e-4f) {
          const std::size_t ml = merged.edgeClass.size() - 1;
          merged.edgeClass.push_back(merged.edgeClass[ml]);
          if (merged.edgeGroup.size() == merged.edgeClass.size() - 1)
            merged.edgeGroup.push_back(merged.edgeGroup[ml]);
          if (merged.edgeVz.size() == merged.edgeClass.size() - 1)
            merged.edgeVz.push_back(merged.edgeVz[ml]);
          if (merged.edgeAlpha.size() == merged.edgeClass.size() - 1)
            merged.edgeAlpha.push_back(merged.edgeAlpha[ml]);
          if (merged.edgeFlags.size() == merged.edgeClass.size() - 1)
            merged.edgeFlags.push_back(merged.edgeFlags[ml]);
          merged.pts.push_back(merged.pts.front());
        }
      }
      woven.push_back(std::move(merged));
    }
    traced.swap(woven);
  }

  // Stage 3+4 per chain: speck filter (whole chain), class-run relabel +
  // split, per-run geometry cleanup, slot mapping.
  const float minChainLen = se.screenMinLenPx * ssScale;
  const int mergeLen = std::max(
      0, static_cast<int>(std::lround(se.screenClassMergeLen * ssScale)));
  const float rdpEps = se.screenSimplifyPx * ssScale;
  // The Stage-4 chain work runs in TWO passes: PASS 1 cleans each chain's
  // geometry (the outside-alignment notch bridge), splits it into
  // (class, group, vz) runs and votes each run's outer side; PASS 2 builds
  // the draw chains. The split exists because a junction end's taper
  // decision needs OTHER chains' endpoint sides (a bar passing straight
  // through a junction is split into two chains there, and neither half may
  // taper, or the bar visibly kinks at every junction).
  struct ChainWork {
    std::size_t chIdx = 0;
    // Cleaned copies; the bridge excision splices all of them in step.
    std::vector<ScreenChainVert> pts;
    std::vector<std::uint8_t> cls;
    std::vector<std::uint16_t> grp;
    std::vector<std::uint8_t> flg;  // edgeFlags (empty when absent)
    std::vector<float> vz, alp;     // per-edgel owner attribution
    struct Span {
      std::size_t e0, e1;
    };
    std::vector<Span> runs;
    std::vector<std::int8_t> side;
  };

  // ---- PASS 1 -------------------------------------------------------------
  std::vector<ChainWork> works;
  works.reserve(traced.size());
  for (std::size_t chIdx = 0; chIdx < traced.size(); ++chIdx) {
    const ScreenChain& ch = traced[chIdx];
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

    ChainWork w;
    w.chIdx = chIdx;
    w.pts = ch.pts;
    // (class, group)-run relabel (labels only; the chain's stored arrays
    // keep the physical owner attribution): a chain that walks across a
    // section change (e.g. the shared outer silhouette of two touching
    // sections, or an ObjectId boundary whose nearer-pixel owner flips)
    // must not draw the other section's contour with this section's style.
    w.cls = ch.edgeClass;
    w.grp = ch.edgeGroup;
    w.flg = ch.edgeFlags.size() == ch.edgeClass.size()
                ? ch.edgeFlags
                : std::vector<std::uint8_t>();
    w.vz = ch.edgeVz;
    w.alp = ch.edgeAlpha;
    mergeShortClassRuns(w.cls, w.grp, mergeLen);

    // NOTCH BRIDGE (topology, every alignment): where the backbone doubles
    // back on itself within a stroke width -- the boundary detouring
    // around a few-px background notch where two surfaces almost touch --
    // excise the detour and bridge it with a straight segment. An offset
    // band would paint the detour's full outer width as a spur poking out
    // of the meeting lines; a centered band shows it as a nub. Fold zones
    // DEEPER than a stroke width (a real hairpin around a wedge, with long
    // legs) are left alone; the outside re-centering taper handles them.
    // Straightness = chord/arc over a +-width window; the depth test (max
    // distance of the detour to its bridging chord) separates the two.
    if (w.pts.size() >= 5) {
      float wMax = 0.0f;
      {
        std::uint8_t pc = 0xFF;
        std::uint16_t pg = 0xFFFF;
        for (std::size_t e = 0; e < w.cls.size(); ++e) {
          if (w.cls[e] == pc && w.grp[e] == pg) continue;
          pc = w.cls[e];
          pg = w.grp[e];
          wMax = std::max(wMax,
                          2.0f * halfFor(static_cast<CrackClass>(pc), pg));
        }
      }
      const std::size_t win = static_cast<std::size_t>(
          std::max(2L, std::lround(wMax)));
      const std::size_t nV = w.pts.size();
      // Pre-bridge edgels are unit steps, so the index span IS the arc.
      std::vector<char> folded(nV, 0);
      for (std::size_t i = 1; i + 1 < nV; ++i) {
        const std::size_t a = i > win ? i - win : 0;
        const std::size_t b = std::min(i + win, nV - 1);
        const float dx = w.pts[b].x - w.pts[a].x;
        const float dy = w.pts[b].y - w.pts[a].y;
        folded[i] = std::sqrt(dx * dx + dy * dy) <
                    0.55f * static_cast<float>(b - a);
      }
      struct Zone {
        std::size_t a, b;
      };
      std::vector<Zone> zones;
      for (std::size_t i = 1; i + 1 < nV;) {
        if (!folded[i]) {
          ++i;
          continue;
        }
        std::size_t j = i;
        while (j + 2 < nV && folded[j + 1]) ++j;
        const std::size_t a = i > win ? i - win : 0;
        const std::size_t b = std::min(j + win, nV - 1);
        if (!zones.empty() && a <= zones.back().b)
          zones.back().b = b;
        else
          zones.push_back({a, b});
        i = j + 1;
      }
      for (std::size_t zi = zones.size(); zi-- > 0;) {
        const std::size_t a = zones[zi].a, b = zones[zi].b;
        if (b <= a + 1) continue;
        const float ax = w.pts[a].x, ay = w.pts[a].y;
        const float ex = w.pts[b].x - ax, ey = w.pts[b].y - ay;
        const float el2 = ex * ex + ey * ey;
        float depth = 0.0f;
        for (std::size_t k = a + 1; k < b; ++k) {
          const float px2 = w.pts[k].x - ax, py2 = w.pts[k].y - ay;
          float t = el2 > 1.0e-6f ? (px2 * ex + py2 * ey) / el2 : 0.0f;
          t = std::max(0.0f, std::min(1.0f, t));
          const float qx = px2 - t * ex, qy = py2 - t * ey;
          depth = std::max(depth, std::sqrt(qx * qx + qy * qy));
        }
        if (depth > wMax) continue;  // deep fold: a real hairpin, keep it
        // A NOTCH returns to the direction it came from -- the boundary
        // steps aside and back, so the trend before and after the zone is
        // the same line. A sharp CUSP (two long legs of a wide background
        // wedge meeting at a point) also reads as a shallow local fold,
        // but the trend TURNS there; excising it would let the drawn line
        // cut the chord across the wedge tip, leaving a triangular gap
        // between the line and the object. Keep the zone whenever the
        // surrounding directions differ by more than ~45 degrees.
        {
          const std::size_t m = std::max<std::size_t>(win, 2);
          const std::size_t ia = a > m ? a - m : 0;
          const std::size_t ib = std::min(b + m, nV - 1);
          const float inx = w.pts[a].x - w.pts[ia].x;
          const float iny = w.pts[a].y - w.pts[ia].y;
          const float outx = w.pts[ib].x - w.pts[b].x;
          const float outy = w.pts[ib].y - w.pts[b].y;
          const float li = std::sqrt(inx * inx + iny * iny);
          const float lo = std::sqrt(outx * outx + outy * outy);
          if (li > 1.0e-5f && lo > 1.0e-5f &&
              (inx * outx + iny * outy) / (li * lo) < 0.7071f)
            continue;  // a turn, not a notch
        }
        w.pts.erase(w.pts.begin() + a + 1, w.pts.begin() + b);
        auto cut = [&](auto& arr) {
          if (arr.size() >= b)
            arr.erase(arr.begin() + a + 1, arr.begin() + b);
        };
        cut(w.cls);
        cut(w.grp);
        if (!w.flg.empty()) cut(w.flg);
        if (!w.vz.empty()) cut(w.vz);
        if (!w.alp.empty()) cut(w.alp);
      }
    }

    // Run split: a run also ends at an owner view-z DISCONTINUITY --
    // consecutive edgels whose owner depths differ by more than the slope
    // clamp cannot lie on one surface, so the walk hopped to a different
    // part of the SAME section across a degree-2 corner. Without the split,
    // collinear collapse + the draw stage's per-vertex lerp smear the depth
    // jump along the line (fog leak, depth-sort drift).
    const bool hasVzArr = w.vz.size() == w.cls.size();
    auto vzContinuous = [&](std::size_t ea, std::size_t eb) {
      if (!hasVzArr) return true;
      const float a = w.vz[ea], b2 = w.vz[eb];
      const float px = pixelSizeAt(sp, std::min(a, b2));
      return std::fabs(b2 - a) <= se.screenSlopeClampPx * px;
    };
    for (std::size_t r0 = 0; r0 < w.cls.size();) {
      std::size_t r1 = r0;
      while (r1 < w.cls.size() && w.cls[r1] == w.cls[r0] &&
             w.grp[r1] == w.grp[r0] && (r1 == r0 || vzContinuous(r1 - 1, r1)))
        ++r1;
      w.runs.push_back({r0, r1});
      r0 = r1;
    }

    // Outside stroke alignment: every OCCLUSION contour -- Silhouette,
    // ObjectId and DepthGap alike -- shifts its ink to the far (non-owner)
    // side, so the nearer surface whose contour it is never thins under a
    // thick line; only Crease stays centered (a surface fold has no
    // occluded side). Keyed on the run class, NOT styleSlot (the
    // DepthGap->sil slot fallback is a style lookup, not a class change).
    // The run's group is the OWNER (nearer) section, so a section's align
    // governs its own contours. The outer side is voted per run over the
    // edgel side bits: a majority absorbs the few edgels whose owner
    // flipped (owner jitter, mergeShortClassRuns relabels). Contact edgels
    // (bit 2) have no defined outer side and abstain; an all-contact run
    // (or a tie) stays centered.
    w.side.assign(w.runs.size(), 0);
    for (std::size_t ri = 0; ri < w.runs.size(); ++ri) {
      const CrackClass rc = static_cast<CrackClass>(w.cls[w.runs[ri].e0]);
      const std::uint16_t g = w.grp[w.runs[ri].e0];
      if (alignFor(g) != StrokeAlign::Outside || rc == CrackClass::Crease ||
          w.flg.size() != w.cls.size())
        continue;
      long vote = 0;
      for (std::size_t e = w.runs[ri].e0; e < w.runs[ri].e1; ++e) {
        const std::uint8_t f = w.flg[e];
        if (f & 4) continue;
        vote += (f & 8) ? 1 : -1;
      }
      w.side[ri] = vote > 0 ? 1 : (vote < 0 ? -1 : 0);
    }
    works.push_back(std::move(w));
  }

  const long cornerW = static_cast<long>(cf.W) + 1;
  auto endCorner = [&](const ChainWork& w2, int end) {
    const ScreenChainVert& v = end == 0 ? w2.pts.front() : w2.pts.back();
    return static_cast<long>(std::lround(v.y + 0.5f)) * cornerW +
           static_cast<long>(std::lround(v.x + 0.5f));
  };
  auto endDirIn = [&](const ChainWork& w2, int end) {
    // Direction from the endpoint INTO the chain, over an ~8 px window.
    const std::size_t n = w2.pts.size();
    const std::size_t k = std::min<std::size_t>(8, n - 1);
    float dx, dy;
    if (end == 0) {
      dx = w2.pts[k].x - w2.pts.front().x;
      dy = w2.pts[k].y - w2.pts.front().y;
    } else {
      dx = w2.pts[n - 1 - k].x - w2.pts.back().x;
      dy = w2.pts[n - 1 - k].y - w2.pts.back().y;
    }
    const float l = std::sqrt(dx * dx + dy * dy);
    return l > 1.0e-5f ? std::array<float, 2>{dx / l, dy / l}
                       : std::array<float, 2>{0.0f, 0.0f};
  };

  // Build the end clip for a stem terminating on a known bar line (Stage 3.5
  // BarInfo or the free-end probe's fit): cull this chain's ink beyond the
  // bar-ink boundary FARTHEST from the stem, so the stem keeps its offset
  // band and stops flush at the bar's far edge -- clipping replaces the
  // earlier re-centering taper, which visibly necked shallow junctions.
  // Coordinates are STROKE coords. bandN = unit normal toward the bar's
  // band; (0, 0) = unknown -> a small slack past the backbone (the bar's
  // ink covers any overshoot when its band faces the stem; otherwise the
  // sub-px slack is invisible).
  auto stemClip = [&](float barPx, float barPy, float barDx, float barDy,
                      float bandNx, float bandNy, float outX, float outY,
                      CrackClass barCls, std::uint16_t barGrp, float stemHalf) {
    StrokeEndClip clip;
    float nx = -barDy, ny = barDx;
    if (nx * outX + ny * outY < 0.0f) {
      nx = -nx;
      ny = -ny;
    }
    const float halfBar = halfFor(barCls, barGrp);
    const float pad = std::min(halfBar, 0.5f * ssScale);
    float extent;
    if (bandNx == 0.0f && bandNy == 0.0f)
      extent = 0.5f * ssScale;
    else if (bandNx * nx + bandNy * ny > 0.0f)
      extent = std::max(0.5f * ssScale, 2.0f * halfBar - pad);
    else
      extent = pad;
    clip.enabled = true;
    clip.px = barPx + nx * extent;
    clip.py = barPy + ny * extent;
    clip.nx = nx;
    clip.ny = ny;
    clip.radius = 4.0f * std::max(halfBar, stemHalf) + 2.0f * ssScale;
    return clip;
  };

  // FREE-END probe: the prune's weak-tail trim and the classifier's
  // bg-clearance kill leave a stem's lattice end 1-3 px short of the line
  // it visually T's into; the legacy centered bands bridged that gap
  // invisibly, the offset band does not. Probe the crack field in a 45-deg
  // cone beyond the end (excluding the chain's own tail cracks); when
  // another chain's cracks sit within reach, return the nearest distance
  // (so the drawn backbone can be EXTENDED to touch the line) plus the line
  // FITTED through the hit midpoints (PCA), for the end clip. dist < 0 =
  // nothing in reach.
  struct ProbeHit {
    float dist = -1.0f;            // to the nearest foreign crack
    float px = 0.0f, py = 0.0f;    // anchor on the fitted line, STROKE coords
    float dx = 0.0f, dy = 0.0f;    // fitted line direction (unit)
    float bnx = 0.0f, bny = 0.0f;  // met line's outer-band normal (0 = n/a)
    CrackClass cls = CrackClass::Silhouette;  // met line's style key
    std::uint16_t grp = 0;
  };
  auto probeFreeEnd = [&](std::size_t wi, int end) -> ProbeHit {
    const ChainWork& w2 = works[wi];
    const ScreenChain& ch = traced[w2.chIdx];
    const float half =
        halfFor(static_cast<CrackClass>(
                    w2.cls[end == 0 ? w2.runs.front().e0 : w2.runs.back().e0]),
                w2.grp[end == 0 ? w2.runs.front().e0 : w2.runs.back().e0]);
    const int R = std::min(8L, std::max(2L, std::lround(half) + 2));
    // Own tail crack cells, reconstructed from the ORIGINAL (unit-step)
    // lattice polyline -- the bridged copy may hold long segments.
    std::vector<std::pair<int, std::size_t>> own;
    {
      const std::size_t n = ch.pts.size();
      const std::size_t K = std::min<std::size_t>(6, n - 1);
      for (std::size_t s = 0; s < K; ++s) {
        const std::size_t i0 = end == 0 ? s : n - 2 - s;
        const int c0x = static_cast<int>(std::lround(ch.pts[i0].x + 0.5f));
        const int c0y = static_cast<int>(std::lround(ch.pts[i0].y + 0.5f));
        const int c1x = static_cast<int>(std::lround(ch.pts[i0 + 1].x + 0.5f));
        const int c1y = static_cast<int>(std::lround(ch.pts[i0 + 1].y + 0.5f));
        if (c0x == c1x) {  // vertical corner step -> right-plane crack
          const int cy2 = std::min(c0y, c1y);
          if (c0x - 1 >= 0 && c0x - 1 < cf.W && cy2 >= 0 && cy2 < cf.H)
            own.push_back(
                {0, static_cast<std::size_t>(cy2) * cf.W + (c0x - 1)});
        } else if (c0y == c1y) {  // horizontal -> down-plane crack
          const int cx2 = std::min(c0x, c1x);
          if (cx2 >= 0 && cx2 < cf.W && c0y - 1 >= 0 && c0y - 1 < cf.H)
            own.push_back(
                {1, static_cast<std::size_t>(c0y - 1) * cf.W + cx2});
        }
      }
    }
    const std::array<float, 2> din = endDirIn(w2, end);
    const float ox = -din[0], oy = -din[1];  // outward, away from the chain
    ProbeHit hit;
    if (ox == 0.0f && oy == 0.0f) return hit;
    const ScreenChainVert& ev = end == 0 ? w2.pts.front() : w2.pts.back();
    const float ecx = ev.x + 0.5f, ecy = ev.y + 0.5f;
    const int icx = static_cast<int>(std::lround(ecx));
    const int icy = static_cast<int>(std::lround(ecy));
    std::vector<std::array<float, 2>> mids;  // cone hits, corner coords
    float bnxAcc = 0.0f, bnyAcc = 0.0f;      // met line's outer-side estimate
    for (int yy = icy - R; yy <= icy + R; ++yy) {
      if (yy < 0 || yy >= cf.H) continue;
      for (int xx = icx - R; xx <= icx + R; ++xx) {
        if (xx < 0 || xx >= cf.W) continue;
        const std::size_t cell = static_cast<std::size_t>(yy) * cf.W + xx;
        for (int plane = 0; plane < 2; ++plane) {
          const std::uint8_t byte =
              plane == 0 ? cf.right[cell] : cf.down[cell];
          if (!(byte & kCrackClassMask)) continue;
          bool isOwn = false;
          for (const auto& oc : own)
            if (oc.first == plane && oc.second == cell) {
              isOwn = true;
              break;
            }
          if (isOwn) continue;
          // Crack midpoints in corner coords: right (x+1, y+0.5),
          // down (x+0.5, y+1).
          const float mx = plane == 0 ? xx + 1.0f : xx + 0.5f;
          const float my = plane == 0 ? yy + 0.5f : yy + 1.0f;
          const float vx = mx - ecx, vy = my - ecy;
          const float d = std::sqrt(vx * vx + vy * vy);
          if (d <= 0.5f || d > static_cast<float>(R) + 0.5f) continue;
          if (vx * ox + vy * oy < 0.7071f * d) continue;  // 45-deg cone
          mids.push_back({mx, my});
          const bool second = (byte & kCrackOwnerBit) != 0;
          if (!(byte & kCrackContactBit)) {
            // The met line's band (outer side) points toward the NON-owner
            // pixel of its cracks; contact cracks abstain (arbitrary owner).
            const float toOuter = second ? -1.0f : 1.0f;
            if (plane == 0)
              bnxAcc += toOuter;
            else
              bnyAcc += toOuter;
          }
          if (hit.dist < 0.0f || d < hit.dist) {
            hit.dist = d;
            // Style key of the met line, from the nearest crack's owner px.
            const int ownPix =
                plane == 0
                    ? yy * cf.W + (second ? xx + 1 : xx)
                    : (second ? yy + 1 : yy) * cf.W + xx;
            hit.cls = static_cast<CrackClass>(byte & kCrackClassMask);
            hit.grp = static_cast<std::uint16_t>(
                frame.objectId[static_cast<std::size_t>(ownPix)] >> 2);
          }
        }
      }
    }
    if (hit.dist < 0.0f) return hit;
    {
      const float bl = std::sqrt(bnxAcc * bnxAcc + bnyAcc * bnyAcc);
      if (bl > 0.5f) {
        hit.bnx = bnxAcc / bl;
        hit.bny = bnyAcc / bl;
      }
    }
    // Fit the met line through the cone hits (PCA); with a single hit fall
    // back to "perpendicular to the stem".
    float cx3 = 0.0f, cy3 = 0.0f;
    for (const auto& m : mids) {
      cx3 += m[0];
      cy3 += m[1];
    }
    cx3 /= static_cast<float>(mids.size());
    cy3 /= static_cast<float>(mids.size());
    float sxx = 0.0f, sxy = 0.0f, syy = 0.0f;
    for (const auto& m : mids) {
      const float dx = m[0] - cx3, dy = m[1] - cy3;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }
    if (mids.size() >= 2 && (sxx + syy) > 1.0e-5f) {
      const float th = 0.5f * std::atan2(2.0f * sxy, sxx - syy);
      hit.dx = std::cos(th);
      hit.dy = std::sin(th);
    } else {
      hit.dx = -oy;  // single hit: assume the met line crosses the stem
      hit.dy = ox;
    }
    hit.px = cx3 - 0.5f;  // corner -> STROKE coords
    hit.py = cy3 - 0.5f;
    return hit;
  };

  // ---- PASS 2: build the draw chains -------------------------------------
  std::vector<StrokeChainInput> drawChains;
  for (std::size_t wi = 0; wi < works.size(); ++wi) {
    const ChainWork& w = works[wi];
    const ScreenChain& ch = traced[w.chIdx];
    const bool hasVzArr = w.vz.size() == w.cls.size();
    auto vzContinuous = [&](std::size_t ea, std::size_t eb) {
      if (!hasVzArr) return true;
      const float a = w.vz[ea], b2 = w.vz[eb];
      const float px = pixelSizeAt(sp, std::min(a, b2));
      return std::fabs(b2 - a) <= se.screenSlopeClampPx * px;
    };
    // Drawn-backbone extension past each chain end, in hi-res px (< 0 =
    // none). A tapered junction end pushes slightly INTO the junction: the
    // bar's drawn backbone deviates from the lattice corner by up to the
    // RDP epsilon (plus the Chaikin cut), and with its band offset to the
    // far side only the thin inner pad remains to cover that deviation -- a
    // ~1 px pinhole opens between the stem's butt and the bar's band. The
    // overlap closes it; ink over ink is invisible, and an overshoot on the
    // bandless side is under a px after downsampling.
    const float extJunction = 0.5f * ssScale + 0.5f * rdpEps;
    float extendStart = -1.0f, extendEnd = -1.0f;

    // DRAW-SPAN coalescing: the (class, group, vz) run split is an
    // ATTRIBUTION construct, but where two adjacent runs resolve to the
    // SAME drawn style (e.g. a rim switching between Silhouette and the
    // DepthGap->sil fallback at a T junction), share the voted side and
    // the paint precedence, and the boundary carries no VISIBLE depth
    // change, keeping the split would still cut the drawn polyline in two:
    // each half smooths independently, a mid-contour end-node pair appears
    // where nothing visibly changes, and the two butt ends leave the outer
    // wedge of any turn at the boundary unfilled (a triangular pinhole a
    // single stroke's miter would cover). Coalesce such runs into one
    // drawn span, so the bar of a T stays ONE polyline whether or not
    // stems attach. A vz jump is "visible" only through the fog fade (and
    // it stays a split whenever it could shift the fade noticeably); with
    // fog off it only orders overlap compositing, which same-style ink
    // cannot expose. Topology, every alignment: the sides must merely be
    // EQUAL (center's are all 0), so both modes share one polyline
    // structure.
    auto vzDrawMergeable = [&](std::size_t ea, std::size_t eb) {
      if (!hasVzArr) return true;
      if (vzContinuous(ea, eb)) return true;
      if (!scene.fog.enabled) return true;
      const float span = scene.fog.end - scene.fog.start;
      return span > 0.0f && std::fabs(w.vz[eb] - w.vz[ea]) <= 0.15f * span;
    };
    struct DrawSpan {
      std::size_t r0, r1;  // inclusive run index range
    };
    std::vector<DrawSpan> spans;
    for (std::size_t ri = 0; ri < w.runs.size();) {
      std::size_t rj = ri;
      while (rj + 1 < w.runs.size()) {
        if (w.side[rj + 1] != w.side[rj]) break;
        const CrackClass ca = static_cast<CrackClass>(w.cls[w.runs[rj].e0]);
        const CrackClass cb =
            static_cast<CrackClass>(w.cls[w.runs[rj + 1].e0]);
        const RunStyle sa = styleFor(ca, w.grp[w.runs[rj].e0]);
        const RunStyle sb = styleFor(cb, w.grp[w.runs[rj + 1].e0]);
        if (!sa.ok || !sb.ok || sa.half != sb.half ||
            sa.opacity != sb.opacity || sa.color[0] != sb.color[0] ||
            sa.color[1] != sb.color[1] || sa.color[2] != sb.color[2])
          break;
        if (!vzDrawMergeable(w.runs[rj + 1].e0 - 1, w.runs[rj + 1].e0))
          break;
        ++rj;
      }
      spans.push_back({ri, rj});
      ri = rj + 1;
    }

    for (std::size_t si = 0; si < spans.size(); ++si) {
      const std::size_t e0 = w.runs[spans[si].r0].e0;
      const std::size_t e1 = w.runs[spans[si].r1].e1;
      const CrackClass runClass = static_cast<CrackClass>(w.cls[e0]);
      // A span covering the whole closed loop keeps the cyclic treatment --
      // unless the loop seam itself hides a depth jump that could visibly
      // shift the fog fade (the duplicated seam vertex would average the
      // near and far owner depths); same bar as the span coalescing above.
      const bool runClosed = ch.closed && e0 == 0 && e1 == w.cls.size() &&
                             vzDrawMergeable(w.cls.size() - 1, 0);

      StrokeChainInput in;
      in.group = w.grp[e0];
      // A span may coalesce runs of different classes (identical styles);
      // paint with the highest precedence among them. Same-style ink
      // composites identically in any order, so this only decides
      // exact-depth ties against other strokes.
      in.precedence = classPrecedence(runClass);
      for (std::size_t r = spans[si].r0 + 1; r <= spans[si].r1; ++r)
        in.precedence = std::max(
            in.precedence, classPrecedence(static_cast<CrackClass>(
                               w.cls[w.runs[r].e0])));
      in.styleSlot = classStyleSlot(runClass);
      in.outsideSide = w.side[spans[si].r0];
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
      // Junction end handling. After the Stage-3.5 weaving, a chain end
      // still sitting at a junction is a true STEM -- the bar it meets was
      // woven into one chain and never ends here. The stem keeps its offset
      // band and is CLIPPED against the woven bar's ink (stemClip; extended
      // into the bar so smoothing deviations cannot open a pinhole). A
      // degree-1 free end left short of a line by the weak-tail trims is
      // connected the same way via the crack-field probe's fitted line.
      // The re-centering TAPER remains for the cases with no met line to
      // clip against: a run boundary whose neighbor's voted side differs, a
      // deep fold at a run boundary (only real hairpins remain after the
      // PASS-1 notch bridge), a junction with no woven bar (e.g. a Y of
      // three stems), and the closed-chain seam wrap.
      if (in.outsideSide != 0 && !runClosed) {
        const bool wrap = ch.closed && spans.size() > 1;
        // The run's resolved half-width sizes the fold window.
        const float rh = halfFor(runClass, in.group);
        const float winPx = std::max(2.0f, 2.0f * rh);
        // Straightness across a run boundary, by TRUE arc length (bridged
        // segments are longer than one px).
        auto foldAt = [&](std::size_t eb) {
          const std::size_t last = w.pts.size() - 1;
          std::size_t a = eb;
          float arcA = 0.0f;
          while (a > 0 && arcA < winPx) {
            const float dx = w.pts[a].x - w.pts[a - 1].x;
            const float dy = w.pts[a].y - w.pts[a - 1].y;
            arcA += std::sqrt(dx * dx + dy * dy);
            --a;
          }
          std::size_t b = eb;
          float arcB = 0.0f;
          while (b < last && arcB < winPx) {
            const float dx = w.pts[b + 1].x - w.pts[b].x;
            const float dy = w.pts[b + 1].y - w.pts[b].y;
            arcB += std::sqrt(dx * dx + dy * dy);
            ++b;
          }
          if (b <= a || arcA + arcB <= 1.0e-5f) return false;
          const float cx2 = w.pts[b].x - w.pts[a].x;
          const float cy2 = w.pts[b].y - w.pts[a].y;
          return std::sqrt(cx2 * cx2 + cy2 * cy2) <
                 0.55f * (arcA + arcB);
        };
        auto continues = [&](std::size_t nb) {  // nb = span index
          return w.side[spans[nb].r0] == in.outsideSide;
        };
        // Resolve one chain end: bar clip (junction / probed free end) or
        // the taper fallback. The endpoint NODE lands exactly ON the met
        // line -- a junction corner already sits on the woven bar's lattice
        // line, and a probed free end is extended precisely to the ray/
        // fitted-line intersection -- so the vector data forms a true T
        // (SVG-ready); the sub-px raster seam against the met line's
        // smoothed backbone is covered by the DRAW stage's pad extension on
        // clipped ends, which the end clip bounds.
        auto resolveEnd = [&](int end, bool& taper, StrokeEndClip& clip,
                              float& extend) {
          const int deg = end == 0 ? ch.deg0 : ch.deg1;
          const std::array<float, 2> din = endDirIn(w, end);
          if (deg >= 3) {
            const auto bar = barAt.find(endCorner(w, end));
            if (bar != barAt.end() &&
                !(din[0] == 0.0f && din[1] == 0.0f)) {
              const ScreenChainVert& v =
                  end == 0 ? w.pts.front() : w.pts.back();
              clip = stemClip(v.x, v.y, bar->second.dx, bar->second.dy,
                              bar->second.bnx, bar->second.bny, -din[0],
                              -din[1], bar->second.cls, bar->second.grp, rh);
            } else {
              taper = true;
              extend = extJunction;
            }
          } else if (deg <= 1) {
            const auto h = probeFreeEnd(wi, end);
            if (h.dist >= 0.0f) {
              clip = stemClip(h.px, h.py, h.dx, h.dy, h.bnx, h.bny, -din[0],
                              -din[1], h.cls, h.grp, rh);
              // Extend exactly to the intersection of the outward ray with
              // the fitted met line (near-parallel: fall back to the probe
              // reach; clamp against runaway grazing intersections).
              const float ox = -din[0], oy = -din[1];
              const float nx = -h.dy, ny = h.dx;
              const float denom = ox * nx + oy * ny;
              const ScreenChainVert& ev =
                  end == 0 ? w.pts.front() : w.pts.back();
              float t = h.dist;
              if (std::fabs(denom) > 0.2f)
                t = ((h.px - ev.x) * nx + (h.py - ev.y) * ny) / denom;
              extend = std::min(std::max(t, 0.0f),
                                2.0f * h.dist + 4.0f * ssScale);
            }
          }
        };
        if (si > 0) {
          in.taperStart = !continues(si - 1) || foldAt(e0);
        } else if (wrap) {
          in.taperStart = !continues(spans.size() - 1);
        } else {
          resolveEnd(0, in.taperStart, in.clipStart, extendStart);
        }
        if (si + 1 < spans.size()) {
          in.taperEnd = !continues(si + 1) || foldAt(e1);
        } else if (wrap) {
          in.taperEnd = !continues(0);
        } else {
          resolveEnd(1, in.taperEnd, in.clipEnd, extendEnd);
        }
      }

      // Geometry cleanup on the run's vertex slice [e0, e1].
      std::vector<ScreenChainVert> pts(w.pts.begin() + e0,
                                       w.pts.begin() + e1 + 1);
      // Re-attribute vertex alpha AND view-z from the run's OWN edgels. The
      // chain-level vertex values blend the two adjacent edgels regardless
      // of run membership, so a run-boundary vertex inherits half of the
      // neighboring run's owner attribution: for alpha, an opaque stick
      // border junctioning into an edge on a fully transparent surface
      // pushed that endpoint to 0.5 and the draw stage lerped the leak
      // across the whole segment; for vz, a near section's silhouette
      // junctioning into a far section's inherited half the far depth, so
      // the FogShader faded the near line's ink toward the fog color and
      // the strip depth sort key was pulled off its surface. Within the
      // run, interior vertices still average their two in-run edgels; the
      // endpoints take their single in-run edgel, so the run's opacity, fog
      // fade and paint depth are functions of its own surface only.
      {
        const std::size_t nV = pts.size();
        const bool hasA = w.alp.size() == w.cls.size();
        for (std::size_t k = 0; k < nV && (hasA || hasVzArr); ++k) {
          std::size_t ea, eb;  // the edgel(s) attributed to vertex k
          if (runClosed && (k == 0 || k == nV - 1)) {
            ea = e0;
            eb = e1 - 1;
          } else if (k == 0) {
            ea = eb = e0;
          } else if (k == nV - 1) {
            ea = eb = e1 - 1;
          } else {
            ea = e0 + k - 1;
            eb = e0 + k;
          }
          if (hasA) pts[k].alpha = 0.5f * (w.alp[ea] + w.alp[eb]);
          if (hasVzArr) pts[k].vz = 0.5f * (w.vz[ea] + w.vz[eb]);
        }
      }
      collapseCollinear(pts, runClosed);
      // SHARP feature corners survive the smoothing. Chaikin's corner
      // cutting is meant for staircase jaggies, but a true corner (a box
      // edge, a contour cusp) cut by 1/4 of its adjacent segments lets the
      // drawn line SHORTCUT the apex -- and with an offset band (only the
      // thin inner pad on the object side) the sliver between the line and
      // the object shows as a triangular gap. Measure the turn over a
      // +-4 px arc window (single staircase steps average out) and PIN
      // vertices turning more than ~60 degrees: the Chaikin pass then runs
      // piecewise between pins, which stay exact.
      std::vector<char> pin;
      if (pts.size() >= 3 && se.screenSmoothIters > 0) {
        pin.assign(pts.size(), 0);
        const float wArc = 4.0f;
        bool any = false;
        for (std::size_t v = 1; v + 1 < pts.size(); ++v) {
          std::size_t a = v;
          float arcA = 0.0f;
          while (a > 0 && arcA < wArc) {
            const float dx = pts[a].x - pts[a - 1].x;
            const float dy = pts[a].y - pts[a - 1].y;
            arcA += std::sqrt(dx * dx + dy * dy);
            --a;
          }
          std::size_t b = v;
          float arcB = 0.0f;
          while (b + 1 < pts.size() && arcB < wArc) {
            const float dx = pts[b + 1].x - pts[b].x;
            const float dy = pts[b + 1].y - pts[b].y;
            arcB += std::sqrt(dx * dx + dy * dy);
            ++b;
          }
          const float ix = pts[v].x - pts[a].x, iy = pts[v].y - pts[a].y;
          const float ox2 = pts[b].x - pts[v].x, oy2 = pts[b].y - pts[v].y;
          const float li = std::sqrt(ix * ix + iy * iy);
          const float lo = std::sqrt(ox2 * ox2 + oy2 * oy2);
          if (li <= 1.0e-5f || lo <= 1.0e-5f) continue;
          if ((ix * ox2 + iy * oy2) / (li * lo) < 0.5f) {  // > ~60 deg
            pin[v] = 1;
            any = true;
          }
        }
        if (!any) pin.clear();
      }
      // Bound Chaikin's corner cut elsewhere. The weaving and the
      // draw-span coalescing removed the run ends that used to PIN the
      // backbone near junctions, leaving long straight segments -- and
      // Chaikin cuts 1/4 of EACH adjacent segment at a corner, so a 30 px
      // edge would round a mild corner by ~8 px and sag the drawn line
      // near its junctions. Subdividing long segments first caps the cut;
      // the inserted straight midpoints are exactly collinear and the RDP
      // below removes them again, so the node count is unchanged.
      if (pts.size() >= 2) {
        const float maxSeg =
            std::max(4.0f, 2.0f * halfFor(runClass, in.group));
        std::vector<ScreenChainVert> sub;
        std::vector<char> subPin;
        sub.reserve(pts.size() * 2);
        subPin.reserve(pts.size() * 2);
        for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
          const ScreenChainVert& a = pts[k];
          const ScreenChainVert& b = pts[k + 1];
          sub.push_back(a);
          subPin.push_back(pin.empty() ? 0 : pin[k]);
          const float dx = b.x - a.x, dy = b.y - a.y;
          const float len = std::sqrt(dx * dx + dy * dy);
          const int nSeg = static_cast<int>(std::ceil(len / maxSeg));
          for (int s2 = 1; s2 < nSeg; ++s2) {
            const float t =
                static_cast<float>(s2) / static_cast<float>(nSeg);
            ScreenChainVert m = a;
            m.x = a.x + dx * t;
            m.y = a.y + dy * t;
            m.vz = a.vz + (b.vz - a.vz) * t;
            m.alpha = a.alpha + (b.alpha - a.alpha) * t;
            sub.push_back(m);
            subPin.push_back(0);
          }
        }
        sub.push_back(pts.back());
        subPin.push_back(pin.empty() ? 0 : pin.back());
        pts.swap(sub);
        if (!pin.empty()) pin.swap(subPin);
      }
      if (pin.empty()) {
        chaikinSmooth(pts, runClosed, se.screenSmoothIters);
      } else {
        // Piecewise Chaikin between pinned corners (chaikinSmooth pins the
        // endpoints of each open piece). A closed loop rotates so its seam
        // sits ON a pinned corner first; the loop then smooths as open
        // pieces and the RDP below still sees front == back.
        if (runClosed && pts.size() >= 3) {
          std::size_t p0 = 0;
          for (std::size_t v = 1; v + 1 < pts.size(); ++v)
            if (pin[v]) {
              p0 = v;
              break;
            }
          if (p0 > 0) {
            pts.pop_back();
            pin.pop_back();
            std::rotate(pts.begin(), pts.begin() + p0, pts.end());
            std::rotate(pin.begin(), pin.begin() + p0, pin.end());
            pts.push_back(pts.front());
            pin.push_back(pin.front());
          }
        }
        std::vector<std::size_t> cuts;
        cuts.push_back(0);
        for (std::size_t v = 1; v + 1 < pts.size(); ++v)
          if (pin[v]) cuts.push_back(v);
        cuts.push_back(pts.size() - 1);
        std::vector<ScreenChainVert> out;
        out.reserve(pts.size() * 2);
        for (std::size_t c = 0; c + 1 < cuts.size(); ++c) {
          std::vector<ScreenChainVert> piece(pts.begin() + cuts[c],
                                             pts.begin() + cuts[c + 1] + 1);
          chaikinSmooth(piece, false, se.screenSmoothIters);
          out.insert(out.end(), piece.begin() + (c == 0 ? 0 : 1),
                     piece.end());
        }
        pts.swap(out);
      }
      simplifyRdp(pts, runClosed, rdpEps);
      if (pts.size() < 2) continue;
      // Junction extension: append a vertex at the chain end's resolved
      // target (a probed free end's on-line intersection, or the small
      // overlap of a barless taper end).
      if (si == 0 && extendStart > 0.0f) {
        const std::array<float, 2> din = endDirIn(w, 0);
        ScreenChainVert v = pts.front();
        v.x -= din[0] * extendStart;
        v.y -= din[1] * extendStart;
        pts.insert(pts.begin(), v);
      }
      if (si + 1 == spans.size() && extendEnd > 0.0f) {
        const std::array<float, 2> din = endDirIn(w, 1);
        ScreenChainVert v = pts.back();
        v.x -= din[0] * extendEnd;
        v.y -= din[1] * extendEnd;
        pts.push_back(v);
      }
      if (dbgRuns)
        std::fprintf(stderr,
                     "[screen-edges]   run ch=%zu [%zu,%zu) cls=%d grp=%u "
                     "side=%d taper=%d/%d clip=%d/%d ext=%.1f/%.1f deg=%d/%d "
                     "closed=%d vz=%.1f..%.1f p0=(%.1f,%.1f) p1=(%.1f,%.1f)\n",
                     w.chIdx, e0, e1, static_cast<int>(runClass), in.group,
                     static_cast<int>(in.outsideSide), in.taperStart ? 1 : 0,
                     in.taperEnd ? 1 : 0, in.clipStart.enabled ? 1 : 0,
                     in.clipEnd.enabled ? 1 : 0, extendStart, extendEnd,
                     ch.deg0, ch.deg1, runClosed ? 1 : 0,
                     hasVzArr ? w.vz[e0] : 0.0f,
                     hasVzArr ? w.vz[e1 - 1] : 0.0f, pts.front().x,
                     pts.front().y, pts.back().x, pts.back().y);
      if (dbgPts) {
        std::fprintf(stderr, "[screen-edges]     pts:");
        for (const ScreenChainVert& v : pts)
          std::fprintf(stderr, " (%.1f,%.1f)", v.x, v.y);
        std::fprintf(stderr, "\n");
      }
      in.pts.reserve(pts.size());
      for (const ScreenChainVert& v : pts)
        in.pts.push_back({v.x, v.y, v.vz, v.alpha, true});
      drawChains.push_back(std::move(in));
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
      umbreon::logMessage(umbreon::LogLevel::Info,
                   "[screen-edges]   chain %zu bbox=(%.0f,%.0f)-(%.0f,%.0f) "
                   "edgels=%zu sil=%d obj=%d gap=%d crease=%d strong=%zu "
                   "closed=%d deg=%d/%d",
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
    umbreon::logMessage(umbreon::LogLevel::Info,
                 "[screen-edges] traced=%zu kept=%zu edgels=%zu (sil=%d "
                 "obj=%d gap=%d crease=%d) gapStrong=%zu drawn=%zu pts=%zu",
                 tracedRaw, traced.size(), nEdgels, clsCount[1], clsCount[2],
                 clsCount[3], clsCount[4], nStrong, drawChains.size(), nPts);
  }

  renderStrokeChains(frame, scene, opt, drawChains);
}

}  // namespace umbreon
