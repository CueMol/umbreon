// Async render + progress channel integration tests. Verifies that:
//  - the async path (renderAsync) is byte-identical to the blocking render()
//    (the progress hooks must not perturb the deterministic pixel values);
//  - a completed render drives the progress channel to fraction 1 / Done;
//  - cooperative cancellation returns a partial frame with cancelled == true;
//  - RenderProgress::fraction() is monotonic and spans [0, 1].
#include <chrono>

#include "render_test_util.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

umbreon::Scene makeScene() {
  umbreon::Material mat;
  mat.ambient = 0.2f;
  mat.diffuse = 0.8f;
  return makeMaterialSphereScene(umbreon::Vec4{0.5f, 0.5f, 0.5f, 1.0f}, mat,
                                 umbreon::Vec3{0.1f, 0.1f, 0.1f});
}

umbreon::RenderOptions makeOpts() {
  umbreon::RenderOptions o;
  o.width = 16;
  o.height = 16;
  return o;
}

bool colorsEqual(const umbreon::FrameResult& a, const umbreon::FrameResult& b) {
  if (a.color.size() != b.color.size()) return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (a.color[i] != b.color[i]) return false;  // deterministic -> exact match
  return true;
}

}  // namespace

int main() {
  umbreon::test::Suite s("render_async");

  // 1) renderAsync produces a bit-identical frame to the blocking render(). The
  //    scene/options are passed by value (copied) into the task, so the originals
  //    stay usable and both calls see identical input.
  {
    const umbreon::Scene scene = makeScene();
    const umbreon::RenderOptions opt = makeOpts();
    const umbreon::FrameResult ref = umbreon::render(scene, opt);
    umbreon::RenderTask task = umbreon::renderAsync(scene, opt);
    const umbreon::FrameResult got = task.get();
    s.check("async frame equals blocking frame", colorsEqual(ref, got));
    s.check("async frame not cancelled", !got.cancelled);
    s.check("task done after get", task.done());
    s.check("task progress == 1 after get", approx(task.progress(), 1.0f, 1e-4f));
  }

  // 2) A normal 3-arg render drives the progress channel to completion.
  {
    const umbreon::Scene scene = makeScene();
    const umbreon::RenderOptions opt = makeOpts();
    umbreon::RenderProgress prog;
    const umbreon::FrameResult f = umbreon::render(scene, opt, prog);
    s.check("completed render is not cancelled", !f.cancelled);
    s.check("progress fraction reaches 1", approx(prog.fraction(), 1.0f, 1e-4f));
    s.check("progress phase is Done",
            prog.phase() == umbreon::RenderPhase::Done);
  }

  // 3) Cooperative cancel: requesting cancel BEFORE the render makes the primary
  //    loop bail at its first row, so render() returns a partial, flagged frame.
  {
    const umbreon::Scene scene = makeScene();
    const umbreon::RenderOptions opt = makeOpts();
    umbreon::RenderProgress prog;
    prog.requestCancel();
    const umbreon::FrameResult f = umbreon::render(scene, opt, prog);
    s.check("pre-cancelled render sets cancelled flag", f.cancelled);
    s.check("cancel did not mark Done",
            prog.phase() != umbreon::RenderPhase::Done);
  }

  // 4) RenderProgress fraction math: monotonic non-decreasing, spanning [0, 1].
  {
    umbreon::RenderProgress p;
    float last = p.fraction();
    bool monotonic = last >= 0.0f;
    auto step = [&](float f) { monotonic = monotonic && (f + 1e-5f >= last); last = f; };

    p.beginPhase(umbreon::RenderPhase::Setup);
    step(p.fraction());
    p.beginPhase(umbreon::RenderPhase::Primary, 100);
    step(p.fraction());
    p.advance(50);
    const float mid = p.fraction();
    step(mid);
    p.advance(50);
    step(p.fraction());
    p.beginPhase(umbreon::RenderPhase::Postprocess);
    step(p.fraction());
    p.markDone();
    step(p.fraction());

    s.check("fraction monotonic non-decreasing", monotonic);
    s.check("Primary half-done ~0.425", approx(mid, 0.425f, 1e-3f));
    s.check("markDone -> fraction 1", approx(p.fraction(), 1.0f, 1e-4f));
    s.check("markDone -> phase Done",
            p.phase() == umbreon::RenderPhase::Done);
  }

  return s.report();
}
