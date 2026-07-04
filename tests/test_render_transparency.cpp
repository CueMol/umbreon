// Group-alpha (blendpng-equivalent multipass) and fragment-alpha
// transparency integration tests.
// Split out of the monolithic test_render.cpp (same assertions, relocated).
#include <cmath>
#include <cstddef>
#include <cstdint>
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
