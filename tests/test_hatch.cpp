// Tone hatching (--hatch) regression tests: the TAM nesting invariants of
// applyHatch (Phase 1: Line layers), the byte-identical default path, the
// ss-invariant line width and thread-count determinism.
// Design record: docs/plans/npr-tone-hatching.md section 8.
#include <cmath>
#include <cstddef>
#include <vector>

#include <tbb/global_control.h>

#include "render_test_util.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"
#include "npr/hatch_ink.hpp"
#include "npr/hatch_shade.hpp"

namespace {

bool framesEqual(const umbreon::FrameResult& a, const umbreon::FrameResult& b) {
  if (a.color.size() != b.color.size()) return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (a.color[i] != b.color[i]) return false;
  return true;
}

// Run applyHatch over a w*h canvas with a UNIFORM tone; rgba starts all-white
// opaque, mask all-surface. Returns the resulting rgba buffer.
std::vector<float> hatchUniform(int w, int h, float tone,
                                const umbreon::HatchOptions& opt) {
  std::vector<float> rgba(static_cast<std::size_t>(w) * h * 4, 1.0f);
  std::vector<float> toneBuf(static_cast<std::size_t>(w) * h, tone);
  std::vector<float> mask(static_cast<std::size_t>(w) * h, 1.0f);
  umbreon::applyHatch(w, h, rgba.data(), toneBuf.data(), mask.data(), nullptr,
                      opt);
  return rgba;
}

// Fraction of pixels darker than 0.5 in the red channel (ink coverage proxy;
// paper is white, default ink black).
float darkFraction(const std::vector<float>& rgba, int w, int h) {
  std::size_t n = 0;
  const std::size_t npix = static_cast<std::size_t>(w) * h;
  for (std::size_t p = 0; p < npix; ++p)
    if (rgba[p * 4 + 0] < 0.5f) ++n;
  return static_cast<float>(n) / static_cast<float>(npix);
}

umbreon::Scene makeQuadScene() {
  umbreon::Scene sc;
  sc.mesh = makeQuad({0.8f, 0.8f, 0.8f, 1.0f});
  sc.camera = makeOrthoCam();
  sc.lights.push_back(makeKeyLight());
  sc.background = {1.0f, 1.0f, 1.0f};
  return sc;
}

}  // namespace

int main() {
  umbreon::test::Suite s("hatch");

  // --- 1. firstLevel: the nesting-level bit logic. j on the coarsest lattice
  // (multiples of 2^K) is level 0; odd j is level K; halving the stride steps
  // one level down. Negative indices mirror.
  {
    using umbreon::detail::hatchFirstLevel;
    s.check_eq("firstLevel: j=0 -> 0", hatchFirstLevel(0, 2), 0);
    s.check_eq("firstLevel: j=4,K=2 -> 0", hatchFirstLevel(4, 2), 0);
    s.check_eq("firstLevel: j=2,K=2 -> 1", hatchFirstLevel(2, 2), 1);
    s.check_eq("firstLevel: j=1,K=2 -> 2", hatchFirstLevel(1, 2), 2);
    s.check_eq("firstLevel: j=3,K=2 -> 2", hatchFirstLevel(3, 2), 2);
    s.check_eq("firstLevel: j=-2,K=2 -> 1", hatchFirstLevel(-2, 2), 1);
    s.check_eq("firstLevel: j=-3,K=2 -> 2", hatchFirstLevel(-3, 2), 2);
    s.check_eq("firstLevel: K=0 -> 0 for all j", hatchFirstLevel(7, 0), 0);
    s.check_eq("firstLevel: j=8,K=3 -> 0", hatchFirstLevel(8, 3), 0);
    s.check_eq("firstLevel: j=12,K=3 -> 1", hatchFirstLevel(12, 3), 1);
  }

  // --- 2. Nesting monotonicity: darkening the tone can only ADD ink at any
  // pixel (marks appear and grow; they never move or vanish). Direct
  // applyHatch on a uniform tone swept 1.0 -> 0.0 with the pen-cross preset.
  {
    const int W = 64, H = 64;
    umbreon::HatchOptions opt;
    opt.enable = true;
    umbreon::applyHatchPreset(opt, "pen-cross");
    bool monotone = true;
    std::vector<float> prev;
    for (int i = 0; i <= 20 && monotone; ++i) {
      const float tone = 1.0f - static_cast<float>(i) / 20.0f;
      std::vector<float> cur = hatchUniform(W, H, tone, opt);
      if (!prev.empty()) {
        for (std::size_t p = 0; p < cur.size() / 4; ++p) {
          // white base * multiply composite: the red channel can only fall.
          if (cur[p * 4 + 0] > prev[p * 4 + 0] + 1.0e-4f) {
            monotone = false;
            break;
          }
        }
      }
      prev = std::move(cur);
    }
    s.check("nesting: per-pixel ink monotone as tone darkens", monotone);
    // Non-trivial: the darkest sweep step actually inked something.
    s.check("nesting: deep tone produced ink",
            darkFraction(prev, W, H) > 0.05f);
    // Paper stays clean: tone 1.0 is exactly ink-free.
    const std::vector<float> paper = hatchUniform(W, H, 1.0f, opt);
    bool clean = true;
    for (std::size_t i = 0; i < paper.size(); ++i)
      if (paper[i] != 1.0f) clean = false;
    s.check("nesting: fully lit tone leaves the paper untouched", clean);
  }

  // --- 3. Min-feature guard: subdiv beyond the 2px finest pitch is clamped,
  // so spacing 10 / K 5 normalizes to the same lattice as K 2 (byte-equal).
  {
    const int W = 48, H = 48;
    umbreon::HatchOptions a;
    a.enable = true;
    umbreon::HatchLayer l;
    l.spacingPx = 10.0f;
    l.subdiv = 5;
    l.toneHi = 0.9f;
    l.toneLo = 0.1f;
    a.layers.push_back(l);
    umbreon::HatchOptions b = a;
    b.layers[0].subdiv = 2;
    const std::vector<float> ra = hatchUniform(W, H, 0.3f, a);
    const std::vector<float> rb = hatchUniform(W, H, 0.3f, b);
    s.check("min-feature: spacing10 K5 == K2 byte-identical", ra == rb);
  }

  // --- 4. Contrast guarantee: white ink on white paper is still visible
  // (the ink is darkened to paper - inkMinContrast, ink side only).
  {
    umbreon::HatchOptions opt;
    opt.enable = true;
    umbreon::applyHatchPreset(opt, "pen-cross");
    opt.inkColor[0] = opt.inkColor[1] = opt.inkColor[2] = 1.0f;
    const std::vector<float> out = hatchUniform(48, 48, 0.3f, opt);
    float minR = 1.0f;
    for (std::size_t p = 0; p < out.size() / 4; ++p)
      minR = std::min(minR, out[p * 4 + 0]);
    s.check("contrast: white-on-white ink still marks the paper",
            minR < 1.0f - 0.15f);
  }

  // --- 5. Byte-identical default: hatch fields may be configured, but with
  // enable == false the render equals the plain default render bitwise.
  {
    const umbreon::Scene sc = makeQuadScene();
    umbreon::RenderOptions o0;
    o0.width = 32;
    o0.height = 32;
    umbreon::RenderOptions o1 = o0;
    umbreon::applyHatchPreset(o1.hatch, "pen-cross");
    o1.hatch.mode = umbreon::HatchMode::Over;
    o1.hatch.inkColor[0] = 0.5f;
    o1.hatch.enable = false;  // the master gate
    const umbreon::FrameResult f0 = umbreon::render(sc, o0);
    const umbreon::FrameResult f1 = umbreon::render(sc, o1);
    s.check("default: hatch.enable=false is byte-identical", framesEqual(f0, f1));
    s.check("default: no hatch AOVs allocated", f1.hatchTone.empty() &&
                                                    f1.hatchMask.empty());
    // Non-trivial: enabling the pass changes the image.
    umbreon::RenderOptions o2 = o0;
    o2.hatch.enable = true;
    const umbreon::FrameResult f2 = umbreon::render(sc, o2);
    s.check("default: hatch.enable=true differs", !framesEqual(f0, f2));
    s.check("default: hatch AOVs sized when enabled",
            f2.hatchTone.size() == 32u * 32u &&
                f2.hatchMask.size() == 32u * 32u);
  }

  // --- 6. ss-invariant line width: the ink is laid at FINAL resolution, so
  // the inked fraction barely moves between ss=1 and ss=3 (the tone gets
  // smoother, the lines do not fatten).
  {
    const umbreon::Scene sc = makeQuadScene();
    umbreon::RenderOptions o;
    o.width = 48;
    o.height = 48;
    o.hatch.enable = true;
    o.supersample = 1;
    const umbreon::FrameResult f1 = umbreon::render(sc, o);
    o.supersample = 3;
    const umbreon::FrameResult f3 = umbreon::render(sc, o);
    const float d1 = darkFraction(f1.color, 48, 48);
    const float d3 = darkFraction(f3.color, 48, 48);
    s.check("ss width: ink visible at ss=1", d1 > 0.02f);
    s.check("ss width: inked fraction ss1 ~= ss3",
            std::fabs(d1 - d3) < 0.05f);
  }

  // --- 7. Determinism: thread-count invariant (pure coordinate functions in
  // the ink pass; the tone AOV rides the deterministic AO/shadow seeding).
  {
    const umbreon::Scene sc = makeQuadScene();
    umbreon::RenderOptions o;
    o.width = 24;
    o.height = 24;
    o.supersample = 2;
    o.aoSamples = 8;
    o.aoDistance = 10.0f;
    o.hatch.enable = true;
    umbreon::FrameResult a, b;
    {
      tbb::global_control one(tbb::global_control::max_allowed_parallelism, 1);
      a = umbreon::render(sc, o);
    }
    b = umbreon::render(sc, o);
    s.check("determinism: 1 thread == N threads (bitwise)", framesEqual(a, b));
    s.check("determinism: hatch AOVs bitwise", a.hatchTone == b.hatchTone &&
                                                   a.hatchMask == b.hatchMask);
  }

  // --- 8. Albedo base: the flat pigment shows through between the marks
  // (base = display-encoded albedo, not the shaded color).
  {
    umbreon::Scene sc = makeQuadScene();
    for (auto& c : sc.mesh.colors) c = {0.2f, 0.3f, 0.9f, 1.0f};  // blue quad
    umbreon::RenderOptions o;
    o.width = 32;
    o.height = 32;
    o.hatch.enable = true;
    o.hatch.base = umbreon::HatchBase::Albedo;
    const umbreon::FrameResult f = umbreon::render(sc, o);
    double sr = 0.0, sb = 0.0;
    for (std::size_t p = 0; p < f.color.size() / 4; ++p) {
      sr += f.color[p * 4 + 0];
      sb += f.color[p * 4 + 2];
    }
    s.check("albedo base: blue pigment shows through the hatch", sb > sr);
    s.check("albedo base: albedo AOV allocated for the hatch",
            f.albedo.size() == 32u * 32u * 3u);
  }

  // --- 9. Stroke edges survive the Ink composite: the flat base is painted
  // BEFORE the edge pass, so an interior (surface-over-surface) silhouette
  // line must still be inked after the hatch. Two same-group spheres with a
  // depth gap; the tone recipe is pinned to paper (ambient 1) so the ONLY
  // dark pixels are stroke-edge ink -- this is the regression that caught
  // the base-replacement erasing every interior line (outline-only look).
  {
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {1.0f, 1.0f, 1.0f};
    umbreon::Sphere front;
    front.center = {-0.5f, 0.0f, 0.5f};
    front.radius = 1.0f;
    front.color = {1.0f, 1.0f, 1.0f, 1.0f};
    umbreon::Sphere back = front;
    back.center = {0.5f, 0.0f, -1.5f};
    sc.spheres.push_back(front);
    sc.spheres.push_back(back);
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.supersample = 2;
    o.strokeEdges.enable = true;
    o.hatch.enable = true;
    o.hatch.tone.ambient = 1.0f;  // paper everywhere: no hatch ink at all
    const umbreon::FrameResult f = umbreon::render(sc, o);
    auto minR = [&f](int cx, int cy, int r) {
      float m = 1.0f;
      for (int y = cy - r; y <= cy + r; ++y)
        for (int x = cx - r; x <= cx + r; ++x)
          m = std::min(m,
                       f.color[(static_cast<std::size_t>(y) * 64 + x) * 4]);
      return m;
    };
    // Front sphere's right silhouette over the back sphere: world (0.5, 0)
    // maps to pixel (40, 32) under the height-4 ortho frame.
    s.check("edges survive ink: interior depth-gap line inked",
            minR(40, 32, 3) < 0.5f);
    // Outer rim against the background: world (-1.5, 0) -> pixel (8, 32).
    s.check("edges survive ink: outer rim inked", minR(8, 32, 3) < 0.5f);
    // And the paper between the lines stayed clean (tone pinned to 1).
    s.check("edges survive ink: paper clean away from lines",
            minR(24, 20, 2) > 0.95f);
  }

  return s.report();
}
