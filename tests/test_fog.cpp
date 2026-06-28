// Tests for OpenGL linear fog reproduced as a depth-based post-process.
//
// Covers: the linear factor over plane eye-z (near=1, far=0, midpoint linear,
// clamping, degenerate slab), the opaque-background RGB mix toward the fog
// color (alpha untouched), and the transparent-background alpha fade. The last
// block locks the load-bearing property: compositing the straight-alpha
// transparent result OVER the fog color reproduces the opaque render, so a
// transparent fogged render can be re-composited over any backdrop later.
#include <cmath>

#include "postprocess/fog.hpp"
#include "test_util.hpp"

namespace {
bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

umbreon::Fog makeFog(float start, float end, umbreon::Vec3 color) {
  umbreon::Fog f;
  f.enabled = true;
  f.start = start;
  f.end = end;
  f.color = color;
  return f;
}
}  // namespace

int main() {
  umbreon::test::Suite s("fog");

  // --- linear factor f = clamp((end - z)/(end - start), 0, 1) over plane eye-z.
  {
    umbreon::Fog f = makeFog(100.0f, 300.0f, {1, 1, 1});
    s.check("factor at start is 1 (unfogged)",
            approx(umbreon::fogFactor(f, 100.0f), 1.0f, 1e-6f));
    s.check("factor before start clamps to 1",
            approx(umbreon::fogFactor(f, 50.0f), 1.0f, 1e-6f));
    s.check("factor at end is 0 (full fog)",
            approx(umbreon::fogFactor(f, 300.0f), 0.0f, 1e-6f));
    s.check("factor after end clamps to 0",
            approx(umbreon::fogFactor(f, 500.0f), 0.0f, 1e-6f));
    s.check("factor at midpoint is linear 0.5",
            approx(umbreon::fogFactor(f, 200.0f), 0.5f, 1e-6f));
    s.check("disabled fog -> factor 1",
            approx(umbreon::fogFactor(umbreon::Fog{}, 200.0f), 1.0f, 1e-6f));
    // Degenerate slab (end <= start): hard step at `end`, no division by zero.
    umbreon::Fog d = makeFog(300.0f, 300.0f, {1, 1, 1});
    s.check("degenerate slab: near side -> 1",
            approx(umbreon::fogFactor(d, 299.0f), 1.0f, 1e-6f));
    s.check("degenerate slab: far side -> 0",
            approx(umbreon::fogFactor(d, 301.0f), 0.0f, 1e-6f));
  }

  // --- opaque background: mix(fogColor, objRGB, f); alpha untouched.
  {
    umbreon::Fog f = makeFog(100.0f, 300.0f, {1, 1, 1});  // white fog
    float color[4] = {0.2f, 0.2f, 0.2f, 1.0f};            // gray surface
    float viewZ[1] = {200.0f};                            // f = 0.5
    umbreon::applyFog(f, 1, 1, 4, color, viewZ, /*transparentBackground=*/false);
    // mix(1.0, 0.2, 0.5) = 0.6
    s.check("opaque: RGB mixed toward fog color", approx(color[0], 0.6f, 1e-5f));
    s.check("opaque: alpha untouched", approx(color[3], 1.0f, 1e-6f));
  }
  {
    umbreon::Fog f = makeFog(100.0f, 300.0f, {1, 1, 1});
    float color[4] = {0.2f, 0.3f, 0.4f, 1.0f};
    float viewZ[1] = {0.0f};  // background sentinel
    umbreon::applyFog(f, 1, 1, 4, color, viewZ, false);
    s.check("background (viewZ<=0) is skipped",
            approx(color[0], 0.2f, 1e-6f) && approx(color[2], 0.4f, 1e-6f));
  }

  // --- transparent background: RGB kept, alpha *= f; "over" the fog color
  // reproduces the opaque render exactly (the re-compositing guarantee).
  {
    const umbreon::Vec3 fogColor{0.7f, 0.8f, 0.9f};
    umbreon::Fog f = makeFog(100.0f, 300.0f, fogColor);
    const float z = 175.0f;       // f = (300-175)/200 = 0.625
    const float fExpected = 0.625f;
    const umbreon::Vec3 obj{0.2f, 0.5f, 0.9f};

    float op[4] = {obj.x, obj.y, obj.z, 1.0f};  // opaque reference
    float vz[1] = {z};
    umbreon::applyFog(f, 1, 1, 4, op, vz, false);

    float tr[4] = {obj.x, obj.y, obj.z, 1.0f};  // transparent
    umbreon::applyFog(f, 1, 1, 4, tr, vz, true);
    s.check("transparent: RGB unchanged", approx(tr[0], obj.x, 1e-6f));
    s.check("transparent: alpha faded by f", approx(tr[3], fExpected, 1e-5f));

    // out = objRGB*alpha + B*(1 - alpha), with B = fogColor, must equal opaque.
    auto over = [](float c, float a, float b) { return c * a + b * (1.0f - a); };
    s.check("transparent over fogColor == opaque (R)",
            approx(over(tr[0], tr[3], fogColor.x), op[0], 1e-5f));
    s.check("transparent over fogColor == opaque (G)",
            approx(over(tr[1], tr[3], fogColor.y), op[1], 1e-5f));
    s.check("transparent over fogColor == opaque (B)",
            approx(over(tr[2], tr[3], fogColor.z), op[2], 1e-5f));
  }

  return s.report();
}
