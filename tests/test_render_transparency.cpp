// Group-alpha (blendpng-equivalent multipass) and fragment-alpha
// transparency integration tests.
// Split out of the monolithic test_render.cpp (same assertions, relocated).
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "render_test_util.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"
#include "postprocess/image_ops.hpp"  // srgbEncodeF: group-blend display-domain checks

int main() {
  umbreon::test::Suite s("render_transparency");
  const umbreon::Vec4 pigment{0.5f, 0.6f, 0.7f, 1.0f};

  // ===== Group-alpha transparency (blendpng-equivalent multi-pass blend) =====
  // Flat material (ambient 1, diffuse 0, ambientColor 1) => shaded color == raw
  // pigment, so the blended center pixel equals an exact analytic value. Each
  // quad spans [-2,2]^2 facing the ortho camera (which frames [-2,2]); the
  // center ray pierces them all. The transparency group lives in triGroupId,
  // the blend weight in Scene::groupBlend (the geometry itself stays OPAQUE).
  // The blend combines the passes' DISPLAY-encoded values (blendpng operates
  // on the finished PNGs), so the color checks compare in the display domain
  // via dsp() = the writer's sRGB encode. This locks the blendpng closed form:
  //   dsp(out) = sum_g beta_g * dsp(render(with g)) + (1 - sum) * dsp(render(bg))
  // where each pass is a full opaque render -- ORDER-INDEPENDENT across groups.
  // dsp(0) = 0 and dsp(1) = 1, so tests blending PURE channels keep the same
  // expected numbers as a linear-domain blend would give.
  {
    using umbreon::Vec3;
    using umbreon::Vec4;
    auto dsp = [](float v) { return umbreon::srgbEncodeF(v); };
    // Append a flat quad at depth z (color.w = opacity) tagged with a group.
    auto addQuad = [](umbreon::Mesh& m, Vec4 color, float z, std::uint16_t g) {
      const Vec3 c[6] = {{-2, -2, z}, {2, -2, z}, {2, 2, z},
                         {-2, -2, z}, {2, 2, z},  {-2, 2, z}};
      const Vec3 n{0, 0, 1};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(c[i]);
        m.normals.push_back(n);
        m.colors.push_back(color);
      }
      m.triGroupId.push_back(g);
      m.triGroupId.push_back(g);
    };
    // The group-alpha tests T1-T6 pass their blend spec via sceneOfBlend();
    // the fragment (over) tests F1-F4 pass {} so their transparency uses
    // front-to-back "over" in a single pass.
    auto sceneOfBlend = [&](umbreon::Mesh mesh, Vec3 bg,
                            std::vector<umbreon::GroupBlend> blend) {
      umbreon::Scene sc;
      sc.mesh = std::move(mesh);
      sc.mesh.material = umbreon::Material::flatOutline();  // raw color shading
      sc.camera = makeOrthoCam();
      sc.background = bg;
      sc.ambientColor = {1, 1, 1};
      sc.groupBlend = std::move(blend);
      return sc;
    };

    // T1: OPAQUE blue quad (group 1, beta 0.6) over opaque red(group 0)
    // => 0.6*render(red+blue) + 0.4*render(red) = 0.6*blue + 0.4*red.
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 1);  // blend group 1 (front, opaque)
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 0.6f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T1 blend-opaque R=0.4", approx(dsp(f.color[kCenterRgba + 0]), 0.4f, 1e-4f));
      s.check("T1 blend-opaque G=0", approx(dsp(f.color[kCenterRgba + 1]), 0.0f, 1e-4f));
      s.check("T1 blend-opaque B=0.6", approx(dsp(f.color[kCenterRgba + 2]), 0.6f, 1e-4f));
      s.check("T1 alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // T2: blue (beta 0.6) over an opaque background (0.2) => 0.6*blue + 0.4*bg.
    {
      umbreon::Mesh m;
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 1);
      umbreon::Scene sc =
          sceneOfBlend(std::move(m), {0.2f, 0.2f, 0.2f}, {{1, 0.6f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const float dspBg = dsp(0.2f);
      s.check("T2 blend-bg R", approx(dsp(f.color[kCenterRgba + 0]), 0.4f * dspBg, 1e-4f));
      s.check("T2 blend-bg B", approx(dsp(f.color[kCenterRgba + 2]), 0.6f + 0.4f * dspBg, 1e-4f));
      s.check("T2 alpha=1 (opaque bg)", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // T3: same scene with a transparent background => the bg pass is fully
    // transparent (0,0,0,0), the layer premultiplied opaque blue; the blend
    // stays premultiplied with alpha = the blended coverage 0.6.
    {
      umbreon::Mesh m;
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 1);
      umbreon::Scene sc =
          sceneOfBlend(std::move(m), {0.2f, 0.2f, 0.2f}, {{1, 0.6f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparentBackground = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T3 transp-bg R=0 (premult)", approx(dsp(f.color[kCenterRgba + 0]), 0.0f, 1e-4f));
      s.check("T3 transp-bg B=0.6 (premult)", approx(dsp(f.color[kCenterRgba + 2]), 0.6f, 1e-4f));
      s.check("T3 transp-bg alpha=0.6", approx(f.color[kCenterRgba + 3], 0.6f, 1e-4f));
    }

    // T4: double-wall avoidance. Two SAME-group(1) blue quads (z=1, z=0.5) over
    // opaque red. The layer pass renders the group OPAQUE, so only its front
    // wall is visible and the back wall cannot double up: 0.6*blue + 0.4*red.
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red
      addQuad(m, {0, 0, 1, 1.0f}, 0.5f, 1);  // back wall  (group 1)
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 1);  // front wall (group 1)
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 0.6f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T4 double-wall R=0.4 (single layer)", approx(dsp(f.color[kCenterRgba + 0]), 0.4f, 1e-4f));
      s.check("T4 double-wall B=0.6 (single layer)", approx(dsp(f.color[kCenterRgba + 2]), 0.6f, 1e-4f));
    }

    // T5: multi-group blend. green(g1, 0.5) + blue(g2, 0.3) + opaque red(g0)
    // => 0.5*green + 0.3*blue + 0.2*red = (0.2, 0.5, 0.3): each layer pass
    // hides the OTHER blend group, so the weights combine additively.
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 1.0f}, 0.5f, 2);  // blue (mid)
      addQuad(m, {0, 1, 0, 1.0f}, 1.0f, 1);  // green (front)
      umbreon::Scene sc =
          sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 0.5f}, {2, 0.3f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T5 multigroup R=0.2", approx(dsp(f.color[kCenterRgba + 0]), 0.2f, 1e-4f));
      s.check("T5 multigroup G=0.5", approx(dsp(f.color[kCenterRgba + 1]), 0.5f, 1e-4f));
      s.check("T5 multigroup B=0.3", approx(dsp(f.color[kCenterRgba + 2]), 0.3f, 1e-4f));
    }

    // T5b: order-independence. Swap the depths of the two blend layers; the
    // blended result is identical (no z-order between groups, as blendpng).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 1, 0, 1.0f}, 0.5f, 1);  // green now mid
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 2);  // blue now front
      umbreon::Scene sc =
          sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 0.5f}, {2, 0.3f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T5b order-indep R=0.2", approx(dsp(f.color[kCenterRgba + 0]), 0.2f, 1e-4f));
      s.check("T5b order-indep G=0.5", approx(dsp(f.color[kCenterRgba + 1]), 0.5f, 1e-4f));
      s.check("T5b order-indep B=0.3", approx(dsp(f.color[kCenterRgba + 2]), 0.3f, 1e-4f));
    }

    // T6: blend weight 1.0 => the bg pass gets weight 0 and the layer replaces
    // it entirely: pure blue, red fully hidden, alpha 1.
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 1);  // blue front (group 1)
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 1.0f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T6 beta1 hides bg pass B=1", approx(dsp(f.color[kCenterRgba + 2]), 1.0f, 1e-4f));
      s.check("T6 beta1 hides bg pass R=0", approx(dsp(f.color[kCenterRgba + 0]), 0.0f, 1e-4f));
      s.check("T6 alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // T7: DISPLAY-SPACE blending (the blendpng property that distinguishes the
    // multi-pass blend from any in-render compositing). With assumed_gamma 2.2
    // each pass stores pow(v, 2.2) and the writer sRGB-encodes that; the blend
    // combines those DISPLAY values: dsp(out) = 0.5*dsp(1) + 0.5*dsp(0.2^2.2),
    // NOT the encode of a linear-domain blend.
    {
      umbreon::Mesh m;
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 1);
      umbreon::Scene sc =
          sceneOfBlend(std::move(m), {0.2f, 0.2f, 0.2f}, {{1, 0.5f}});
      sc.assumedGamma = 2.2f;
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const float dspBg22 = dsp(std::pow(0.2f, 2.2f));
      const float expB = 0.5f * 1.0f + 0.5f * dspBg22;
      const float expR = 0.5f * 0.0f + 0.5f * dspBg22;
      s.check("T7 display-space blend B", approx(dsp(f.color[kCenterRgba + 2]), expB, 1e-4f));
      s.check("T7 display-space blend R", approx(dsp(f.color[kCenterRgba + 0]), expR, 1e-4f));
    }

    // T8: fragment alpha INSIDE a blend group survives the layer pass (the
    // group renders "opaque" only in the sense that no group weight is applied
    // in-pass): blue(w=0.5) group-1 quad over opaque red, beta 0.5. The layer
    // pass composites (0.5, 0, 0.5) linearly IN-pass; the blend then combines
    // display values: dsp(out) = 0.5*dsp(layer) + 0.5*dsp(red).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 0.5f}, 1.0f, 1);  // fragment-alpha blue in group 1
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 0.5f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const float dspHalf = dsp(0.5f);
      s.check("T8 fragment-in-blend R", approx(dsp(f.color[kCenterRgba + 0]), 0.5f * dspHalf + 0.5f, 1e-4f));
      s.check("T8 fragment-in-blend B", approx(dsp(f.color[kCenterRgba + 2]), 0.5f * dspHalf, 1e-4f));
    }

    // T9: weights summing to MORE than 1 must not scale the rest of the frame.
    // Geometry outside every blend group appears identically in every pass, so
    // it survives only while the weights sum to exactly 1 -- which needs the
    // background weight 1 - sum to go NEGATIVE (-0.85 here), exactly as
    // blendpng's solvebeta + lerp chain produces it. Clamping that weight to 0
    // would leave the total at 1.85 and scale the grey quad by it, clipping to
    // white. The grey quad sits in FRONT of both blend groups, so every pass
    // shows it at the center and the expected value is the quad itself.
    // The alphas must DIFFER to reach a sum above 1 at all: equal alphas are
    // one veil counted once (T10).
    {
      umbreon::Mesh m;
      addQuad(m, {0, 0, 1, 1.0f}, 0.0f, 1);           // blend group 1 (behind)
      addQuad(m, {0, 1, 0, 1.0f}, 0.5f, 2);           // blend group 2 (behind)
      addQuad(m, {0.5f, 0.5f, 0.5f, 1.0f}, 1.0f, 0);  // opaque grey, no group
      umbreon::Scene sc =
          sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 0.95f}, {2, 0.9f}});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const float dspGrey = dsp(0.5f);
      s.check("T9 sum>1 keeps opaque R", approx(dsp(f.color[kCenterRgba + 0]), dspGrey, 1e-4f));
      s.check("T9 sum>1 keeps opaque G", approx(dsp(f.color[kCenterRgba + 1]), dspGrey, 1e-4f));
      s.check("T9 sum>1 keeps opaque B", approx(dsp(f.color[kCenterRgba + 2]), dspGrey, 1e-4f));
      s.check("T9 sum>1 alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // T10: blend entries sharing an alpha are ONE veil, so a pixel they both
    // cover keeps a non-negative background weight. Two sections at 0.6 asked
    // for 1.2 and left the background at -0.2, and a negative background
    // coefficient INVERTS what the veils cover (dark ink comes out brighter
    // than its lit surroundings). Merged, they are one 0.6 veil whose pass
    // shows the frontmost of its groups, and the background keeps 0.4.
    // The grey quad sits BEHIND both veils -- the arrangement T9 avoids, which
    // is why the defect escaped it.
    {
      umbreon::Mesh base;
      addQuad(base, {0.5f, 0.5f, 0.5f, 1.0f}, 0.0f, 0);  // opaque grey, behind
      addQuad(base, {0, 1, 0, 1.0f}, 0.5f, 1);           // veil group 1 (green)
      addQuad(base, {0, 0, 1, 1.0f}, 1.0f, 2);           // veil group 2 (blue, front)
      const float dspGrey = dsp(0.5f);
      const float expRG = 0.4f * dspGrey;         // grey through one 0.6 veil
      const float expB = 0.4f * dspGrey + 0.6f;   // + the blue veil itself
      // Exactly equal, then just inside the 1e-4 bucketing tolerance (a scene
      // round trip must not split one veil in two).
      const float second[2] = {0.6f, 0.6f + 5.0e-5f};
      const char* tag[2] = {"T10 equal alpha", "T10 near-equal alpha"};
      for (int i = 0; i < 2; ++i) {
        umbreon::Scene sc = sceneOfBlend(umbreon::Mesh(base), {0, 0, 0},
                                         {{1, 0.6f}, {2, second[i]}});
        umbreon::RenderOptions o; o.width = 5; o.height = 5;
        umbreon::FrameResult f = umbreon::render(sc, o);
        s.check(std::string(tag[i]) + " is one veil R",
                approx(dsp(f.color[kCenterRgba + 0]), expRG, 1e-4f));
        s.check(std::string(tag[i]) + " is one veil G",
                approx(dsp(f.color[kCenterRgba + 1]), expRG, 1e-4f));
        s.check(std::string(tag[i]) + " is one veil B",
                approx(dsp(f.color[kCenterRgba + 2]), expB, 1e-4f));
      }
    }

    // ===== PerPixel group blend (RenderOptions::groupBlendMode = 1) =====
    // The weights are built per SAMPLE from the veils that cover it:
    //   T = prod(1 - a_i) is the background's weight, and 1 - T is shared out
    //   in a_i proportion. Never negative, whatever the alphas.
    // The domain is the same display-encoded one LayerWeights blends in, so a
    // sample covered by ONE veil is IDENTICAL to the layer blend (its weights
    // are already 1 - a and a there); only samples under two or more veils
    // differ, and there the background keeps its transmittance instead of
    // going negative.

    // P1: distinct alphas whose sum exceeds 1, overlapping over opaque
    // geometry -- the case LayerWeights composites with a negative background
    // weight (1 - 1.1 = -0.1), which inverts what the veils cover. Per pixel
    // the background keeps 0.4 * 0.5 = 0.2 and nothing inverts.
    {
      umbreon::Mesh m;
      addQuad(m, {0.5f, 0.5f, 0.5f, 1.0f}, 0.0f, 0);  // opaque grey, behind
      addQuad(m, {0, 1, 0, 1.0f}, 0.5f, 1);           // veil 1 (green) a = 0.6
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 2);           // veil 2 (blue)  a = 0.5
      umbreon::Scene sc =
          sceneOfBlend(std::move(m), {0, 0, 0}, {{1, 0.6f}, {2, 0.5f}});
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.groupBlendMode = static_cast<int>(umbreon::GroupBlendMode::PerPixel);
      umbreon::FrameResult f = umbreon::render(sc, o);
      const float T = 0.4f * 0.5f;                 // background transmittance
      const float k = (1.0f - T) / (0.6f + 0.5f);  // share of 1 - T per alpha
      const float bg = T * dsp(0.5f);
      s.check("P1 overlap keeps the background R",
              approx(dsp(f.color[kCenterRgba + 0]), bg, 1e-4f));
      s.check("P1 overlap keeps the background G",
              approx(dsp(f.color[kCenterRgba + 1]), bg + k * 0.6f, 1e-4f));
      s.check("P1 overlap keeps the background B",
              approx(dsp(f.color[kCenterRgba + 2]), bg + k * 0.5f, 1e-4f));
      s.check("P1 overlap alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // P2: where the veils do NOT overlap, each keeps the alpha it was given and
    // the frame is IDENTICAL to the layer blend -- no rescaling, no
    // approximation, no domain change: out = (1 - a) * B + a * S per sample,
    // which is what LayerWeights computes for a lone veil too.
    {
      // A quad spanning x0..x1 (the shared addQuad spans the whole frame).
      auto addHalfQuad = [](umbreon::Mesh& m, Vec4 color, float z,
                            std::uint16_t g, float x0, float x1) {
        const Vec3 c[6] = {{x0, -2, z}, {x1, -2, z}, {x1, 2, z},
                           {x0, -2, z}, {x1, 2, z},  {x0, 2, z}};
        const Vec3 n{0, 0, 1};
        for (int i = 0; i < 6; ++i) {
          m.positions.push_back(c[i]);
          m.normals.push_back(n);
          m.colors.push_back(color);
        }
        m.triGroupId.push_back(g);
        m.triGroupId.push_back(g);
      };
      umbreon::Mesh base;
      addQuad(base, {0.5f, 0.5f, 0.5f, 1.0f}, 0.0f, 0);          // opaque grey
      // A gap around x = 0: sharing the edge would put both veils on the
      // middle column's sample, i.e. an overlap, which is the one thing this
      // check must not contain.
      addHalfQuad(base, {0, 1, 0, 1.0f}, 0.5f, 1, -2.0f, -0.5f);  // left,  0.6
      addHalfQuad(base, {0, 0, 1, 1.0f}, 0.5f, 2, 0.5f, 2.0f);    // right, 0.5
      auto renderMode = [&](umbreon::GroupBlendMode mode) {
        umbreon::Scene sc = sceneOfBlend(umbreon::Mesh(base), {0, 0, 0},
                                         {{1, 0.6f}, {2, 0.5f}});
        umbreon::RenderOptions o;
        o.width = 5; o.height = 5;
        o.groupBlendMode = static_cast<int>(mode);
        return umbreon::render(sc, o);
      };
      const umbreon::FrameResult f =
          renderMode(umbreon::GroupBlendMode::PerPixel);
      const std::size_t left = (2 * 5 + 1) * 4;
      const std::size_t right = (2 * 5 + 3) * 4;
      const float dspGrey = dsp(0.5f);
      s.check("P2 single veil keeps alpha 0.6 (bg)",
              approx(dsp(f.color[left + 0]), 0.4f * dspGrey, 1e-4f));
      s.check("P2 single veil keeps alpha 0.6 (veil)",
              approx(dsp(f.color[left + 1]), 0.4f * dspGrey + 0.6f, 1e-4f));
      s.check("P2 single veil keeps alpha 0.5 (bg)",
              approx(dsp(f.color[right + 0]), 0.5f * dspGrey, 1e-4f));
      s.check("P2 single veil keeps alpha 0.5 (veil)",
              approx(dsp(f.color[right + 2]), 0.5f * dspGrey + 0.5f, 1e-4f));
      // ... and no pixel of a no-overlap frame differs from the layer blend.
      const umbreon::FrameResult g =
          renderMode(umbreon::GroupBlendMode::LayerWeights);
      bool same = f.color.size() == g.color.size();
      for (std::size_t q = 0; same && q < f.color.size(); ++q)
        same = approx(f.color[q], g.color[q], 1e-4f);
      s.check("P2 no overlap matches the layer blend everywhere", same);
    }

    // P3: veils and EDGE GROUPS are independent partitions of the same group
    // ids. Two sections at the SAME alpha are ONE veil -- one pass, both
    // visible -- and yet their edge grouping still decides whether the contact
    // contour between them inks: the identity map (two edge groups) draws it,
    // one shared edge group does not. If the veil bucketing ever merged group
    // IDS instead of passes, the two cases would render alike, which is why
    // the merge lives in the pass plan and not in section identity.
    {
      auto inkCount = [&](std::vector<std::uint16_t> edgeMap) {
        umbreon::Mesh m;
        // Two touching coplanar quads, groups 1 and 2 (both veiled at 0.6).
        auto half = [&m](Vec4 color, float x0, float x1, std::uint16_t g) {
          const Vec3 c[6] = {{x0, -2, 0.0f}, {x1, -2, 0.0f}, {x1, 2, 0.0f},
                             {x0, -2, 0.0f}, {x1, 2, 0.0f},  {x0, 2, 0.0f}};
          const Vec3 n{0, 0, 1};
          for (int i = 0; i < 6; ++i) {
            m.positions.push_back(c[i]);
            m.normals.push_back(n);
            m.colors.push_back(color);
          }
          m.triGroupId.push_back(g);
          m.triGroupId.push_back(g);
        };
        half({0.7f, 0.7f, 0.7f, 1.0f}, -2.0f, 0.0f, 1);
        half({0.7f, 0.7f, 0.7f, 1.0f}, 0.0f, 2.0f, 2);
        umbreon::Scene sc =
            sceneOfBlend(std::move(m), {1, 1, 1}, {{1, 0.6f}, {2, 0.6f}});
        sc.edgeGroupOfGroup = std::move(edgeMap);
        umbreon::RenderOptions o;
        o.width = 48; o.height = 32; o.supersample = 1;
        o.strokeEdges.enable = true;   // ink on a blank background only, so
        o.strokeEdges.edgesOnly = true;  // the count IS the line set
        o.strokeEdges.contact = true;
        umbreon::FrameResult f = umbreon::render(sc, o);
        std::size_t dark = 0;
        for (std::size_t p = 0; p + 3 < f.color.size(); p += 4)
          if (f.color[p] < 0.7f) ++dark;
        return dark;
      };
      const std::size_t twoGroups = inkCount({0, 1, 2});  // identity
      const std::size_t oneGroup = inkCount({0, 1, 1});   // merged edge group
      s.check("P3 edge groups are independent of the veil",
              twoGroups > oneGroup);
    }

    // ===== Fragment alpha (intrinsic per-color opacity): front-to-back "over",
    // EVERY surface composited (no dedup), order-DEPENDENT -- POV native
    // transmit. Selected whenever the group has no blend entry (groupBlend
    // empty => plain single-pass render). =====

    // F1: single fragment over opaque == a*C + (1-a)*A (matches T1 numerically).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 0.6f}, 1.0f, 1);  // fragment blue 0.6
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F1 over R=0.4", approx(f.color[kCenterRgba + 0], 0.4f, 1e-4f));
      s.check("F1 over B=0.6", approx(f.color[kCenterRgba + 2], 0.6f, 1e-4f));
      s.check("F1 alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // F2: ORDER DEPENDENCE (unlike the group blend, cf. T5b). green(0.5) front +
    // blue(0.5) mid over opaque red => 0.5*green + 0.25*blue + 0.25*red.
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 0.5f}, 0.5f, 2);  // blue (mid)
      addQuad(m, {0, 1, 0, 0.5f}, 1.0f, 1);  // green (front)
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F2 green-front G=0.5", approx(f.color[kCenterRgba + 1], 0.5f, 1e-4f));
      s.check("F2 green-front B=0.25", approx(f.color[kCenterRgba + 2], 0.25f, 1e-4f));
    }
    // ...swap depths: blue front => 0.5*blue + 0.25*green + 0.25*red (DIFFERENT).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 1, 0, 0.5f}, 0.5f, 1);  // green (mid)
      addQuad(m, {0, 0, 1, 0.5f}, 1.0f, 2);  // blue (front)
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F2 blue-front B=0.5 (order-dependent)", approx(f.color[kCenterRgba + 2], 0.5f, 1e-4f));
      s.check("F2 blue-front G=0.25 (order-dependent)", approx(f.color[kCenterRgba + 1], 0.25f, 1e-4f));
    }

    // F3: NO dedup -- both walls composite. Two same-group(1) blue(0.5) quads
    // over opaque red => 0.75*blue + 0.25*red (the group blend gives 0.5/0.5).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red
      addQuad(m, {0, 0, 1, 0.5f}, 0.5f, 1);  // back wall
      addQuad(m, {0, 0, 1, 0.5f}, 1.0f, 1);  // front wall (same group)
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0, 0, 0}, {});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F3 no-dedup B=0.75 (both walls)", approx(f.color[kCenterRgba + 2], 0.75f, 1e-4f));
      s.check("F3 no-dedup R=0.25", approx(f.color[kCenterRgba + 0], 0.25f, 1e-4f));
    }

    // F4: transparent background => premultiplied "over" output (no bg tint),
    // alpha = accumulated coverage.
    {
      umbreon::Mesh m;
      addQuad(m, {0, 0, 1, 0.6f}, 1.0f, 1);  // fragment blue 0.6, nothing behind
      umbreon::Scene sc = sceneOfBlend(std::move(m), {0.2f, 0.2f, 0.2f}, {});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparentBackground = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F4 transp-bg B=0.6 (premult)", approx(f.color[kCenterRgba + 2], 0.6f, 1e-4f));
      s.check("F4 transp-bg R=0 (no bg tint)", approx(f.color[kCenterRgba + 0], 0.0f, 1e-4f));
      s.check("F4 transp-bg alpha=0.6", approx(f.color[kCenterRgba + 3], 0.6f, 1e-4f));
    }
  }


  return s.report();
}
