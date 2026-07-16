// Async render + progress channel integration tests. Verifies that:
//  - the async path (renderAsync) is byte-identical to the blocking render()
//    (the progress hooks must not perturb the deterministic pixel values);
//  - a completed render drives the progress channel to fraction 1 / Done;
//  - cooperative cancellation returns a partial frame with cancelled == true;
//  - RenderProgress::fraction() is monotonic and spans [0, 1];
//  - a RenderPhasePlan reweights the bar to the declared per-phase costs, and a
//    zero-share (skipped) phase leaves no gap.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

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

// A pt1-GI render big enough to exercise the GlobalIllum sub-stage
// instrumentation (gather / denoise / upsample / composite). The 16x16 sphere
// of makeOpts() never enters GI at all.
umbreon::RenderOptions makeGiOpts(int size, int spp) {
  umbreon::RenderOptions o;
  o.width = size;
  o.height = size;
  o.gi = true;
  o.giIntegrator = 1;  // pt1
  o.pt1Spp = spp;
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

  // 5) setPhasePlan reweights the bar to the declared costs. Shares are relative
  //    (the renderer passes estimated seconds), so they are normalized by their
  //    sum. The numbers below are the MEASURED draft profile of 1ab0_scene1:
  //    setup .003 / primary .25 / GI 1.54 / postprocess .37 seconds -- the case
  //    the fixed table got wrong by handing GI 0.15 of the bar for 71% of the
  //    time.
  {
    umbreon::RenderProgress p;
    umbreon::RenderPhasePlan plan;
    plan.setup = 0.003f;
    plan.coarseAo = 0.0f;
    plan.primary = 0.25f;
    plan.globalIllum = 1.54f;
    plan.edges = 0.0f;
    plan.postprocess = 0.37f;
    p.setPhasePlan(plan);

    const umbreon::RenderPhasePlan got = p.phasePlan();
    const float sum = got.setup + got.coarseAo + got.primary + got.globalIllum +
                      got.edges + got.postprocess;
    s.check("phasePlan normalizes to 1", approx(sum, 1.0f, 1e-3f));
    s.check("GI share tracks its measured cost", got.globalIllum > 0.6f);

    // base(GlobalIllum) = (0.003 + 0.25) / 2.163 = 0.117
    p.beginPhase(umbreon::RenderPhase::GlobalIllum, 100);
    s.check("GI starts at its plan base", approx(p.fraction(), 0.117f, 2e-3f));
    p.advance(50);
    s.check("GI half-done spans half its share",
            approx(p.fraction(), 0.117f + 0.5f * got.globalIllum, 2e-3f));
  }

  // 6) A zero share means the pass does not run that phase: entering it must
  //    leave the bar exactly where the previous phase ended. Regression for the
  //    CoarseAo gap -- embree_renderer enters that phase unconditionally even
  //    though the pre-pass is a no-op by default, which jumped the fixed-table
  //    bar by a hard 0.05.
  {
    umbreon::RenderProgress p;
    umbreon::RenderPhasePlan plan;
    plan.coarseAo = 0.0f;
    p.setPhasePlan(plan);

    p.beginPhase(umbreon::RenderPhase::Setup);
    p.advance(1);
    const float afterSetup = p.fraction();
    p.beginPhase(umbreon::RenderPhase::CoarseAo);
    const float inSkipped = p.fraction();
    p.beginPhase(umbreon::RenderPhase::Primary, 100);
    const float atPrimary = p.fraction();
    s.check("zero-share phase adds no span", approx(inSkipped, afterSetup, 1e-6f));
    s.check("zero-share phase leaves no gap", approx(atPrimary, afterSetup, 1e-6f));
  }

  // 7) advanceTo is a monotonic seek: OIDN's filter monitor reports an ABSOLUTE
  //    position and may do so from several worker threads out of order, so a
  //    stale report must never rewind the bar.
  {
    umbreon::RenderProgress p;
    p.beginPhase(umbreon::RenderPhase::Primary, 100);
    p.advanceTo(50);
    s.check("advanceTo advances", p.unitsDone() == 50);
    p.advanceTo(20);
    s.check("advanceTo never rewinds", p.unitsDone() == 50);

    p.advanceTo(500);  // overshoot must clamp at the phase end, not run past it
    const umbreon::RenderPhasePlan pl = p.phasePlan();
    s.check("fraction clamps at the phase end",
            approx(p.fraction(), pl.setup + pl.coarseAo + pl.primary, 1e-4f));
  }

  // 8) Degenerate plans must not divide by zero or NaN the bar. An all-zero plan
  //    carries no information, so the defaults stand; negative / non-finite
  //    shares count as zero rather than running the bar backwards.
  {
    umbreon::RenderProgress p;
    umbreon::RenderPhasePlan zero;
    zero.setup = zero.coarseAo = zero.primary = 0.0f;
    zero.globalIllum = zero.edges = zero.postprocess = 0.0f;
    p.setPhasePlan(zero);
    s.check("all-zero plan falls back to defaults",
            approx(p.phasePlan().primary, 0.65f, 1e-3f));

    umbreon::RenderProgress q;
    umbreon::RenderPhasePlan bad;
    bad.setup = 0.0f;
    bad.coarseAo = 0.0f;
    bad.primary = -1.0f;                                    // negative -> 0
    bad.globalIllum = std::numeric_limits<float>::quiet_NaN();  // NaN -> 0
    bad.edges = 0.0f;
    bad.postprocess = 2.0f;                                 // the only real share
    q.setPhasePlan(bad);
    const umbreon::RenderPhasePlan qp = q.phasePlan();
    s.check("negative / NaN shares count as zero",
            approx(qp.postprocess, 1.0f, 1e-3f) && approx(qp.primary, 0.0f, 1e-3f));
    q.beginPhase(umbreon::RenderPhase::Primary, 100);
    q.advance(50);
    const float f = q.fraction();
    s.check("degenerate plan keeps fraction in [0, 1]", f >= 0.0f && f <= 1.0f);
  }

  // 9) Bit-exactness THROUGH the GI progress hooks. Test 1 renders a 16x16
  //    sphere that never enters GI, so it cannot catch a hook perturbing the
  //    gather tiles or the OIDN filter -- the whole risk of instrumenting this
  //    path. Same scene, with and without a RenderProgress: the pixels must be
  //    identical, not merely close.
  {
    const umbreon::Scene scene = makeScene();
    const umbreon::RenderOptions opt = makeGiOpts(64, 8);
    const umbreon::FrameResult ref = umbreon::render(scene, opt);
    umbreon::RenderProgress prog;
    const umbreon::FrameResult got = umbreon::render(scene, opt, prog);
    s.check("GI frame identical with and without progress",
            colorsEqual(ref, got));
    s.check("GI render reaches fraction 1",
            approx(prog.fraction(), 1.0f, 1e-4f));
  }

  // 10) The regression this change exists for: with GI on, the bar must ANIMATE
  //     inside the GlobalIllum phase. Before the fix GI consumed a single unit
  //     at its very end, so fraction() sat frozen for 71-90% of the render and
  //     then jumped.
  {
    const umbreon::Scene scene = makeScene();
    const umbreon::RenderOptions opt = makeGiOpts(192, 128);

    umbreon::RenderProgress prog;
    std::atomic<bool> running{true};
    std::vector<float> giSamples;
    std::thread poller([&] {
      while (running.load(std::memory_order_relaxed)) {
        if (prog.phase() == umbreon::RenderPhase::GlobalIllum)
          giSamples.push_back(prog.fraction());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    umbreon::render(scene, opt, prog);
    running.store(false, std::memory_order_relaxed);
    poller.join();

    std::sort(giSamples.begin(), giSamples.end());
    const std::size_t distinct =
        static_cast<std::size_t>(std::unique(giSamples.begin(),
                                             giSamples.end()) -
                                 giSamples.begin());
    // Only assert when the phase was actually sampled: on a fast machine GI can
    // finish between two polls, and a timing-dependent failure would be worse
    // than a skipped check. A FROZEN bar still yields many samples that are all
    // identical, so this catches the regression wherever it can observe it.
    if (giSamples.size() >= 3)
      s.check("GI phase animates (fraction advances inside GI)", distinct >= 3);
  }

  // 11) Cancel from inside GI. This is the path that exercises the OIDN
  //     monitor's false return -> Error::Cancelled -> "do not run the a-trous
  //     fallback" chain (the fallback would spend exactly the time the cancel
  //     asked to save), as well as the gather tiles' cancel poll.
  {
    const umbreon::Scene scene = makeScene();
    const umbreon::RenderOptions opt = makeGiOpts(192, 128);

    umbreon::RenderProgress prog;
    std::atomic<bool> running{true};
    std::atomic<bool> sawGi{false};
    std::thread poller([&] {
      while (running.load(std::memory_order_relaxed)) {
        if (prog.phase() == umbreon::RenderPhase::GlobalIllum) {
          sawGi.store(true, std::memory_order_relaxed);
          prog.requestCancel();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    const umbreon::FrameResult f = umbreon::render(scene, opt, prog);
    running.store(false, std::memory_order_relaxed);
    poller.join();

    // Same reasoning as above: only assert when the phase was actually observed.
    if (sawGi.load(std::memory_order_relaxed)) {
      s.check("cancel during GI returns a flagged frame", f.cancelled);
      s.check("cancelled GI render is not marked Done",
              prog.phase() != umbreon::RenderPhase::Done);
    }
  }

  return s.report();
}
