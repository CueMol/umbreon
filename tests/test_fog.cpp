// Tests for the depth-based POV fog post-process: transmittance along the view
// ray for constant and ground fog, and the in-place color blend.
#include <cmath>
#include <limits>

#include "image/fog.hpp"
#include "test_util.hpp"

namespace {
bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }
}  // namespace

int main() {
  umbreon::test::Suite s("fog");

  const umbreon::Vec3 camPos{0, 0, 200};
  const umbreon::Vec3 camDir{0, 0, -1};

  // Disabled fog is a no-op (full transmittance).
  {
    umbreon::Fog f;
    f.enabled = false;
    s.check("disabled fog -> T=1",
            approx(umbreon::fogTransmittance(f, camPos, camDir, 250.0f), 1.0f, 1e-6f));
  }

  // Constant fog (type 1): T = exp(-depth / distance) over the whole ray.
  {
    umbreon::Fog f;
    f.enabled = true;
    f.type = 1;
    f.distance = 100.0f;
    s.check("constant fog T at depth=distance ~ 1/e",
            approx(umbreon::fogTransmittance(f, camPos, camDir, 100.0f),
                   std::exp(-1.0f), 1e-5f));
    s.check("constant fog T at depth=0 is 1",
            approx(umbreon::fogTransmittance(f, camPos, camDir, 0.0f), 1.0f, 1e-6f));
  }

  // Ground fog (type 2): up=+Z, offset 0, camera at z=200 looking -z.
  // The ray is fogged only below z=0, i.e. for depth > 200.
  {
    umbreon::Fog f;
    f.enabled = true;
    f.type = 2;
    f.distance = 16.666667f;  // 50/3, as in the CueMol scenes
    f.offset = 0.0f;
    f.up = umbreon::Vec3{0, 0, 1};

    // Hit in front of the offset plane (z_hit=+5) -> no fog.
    s.check("ground fog: above offset -> T=1",
            approx(umbreon::fogTransmittance(f, camPos, camDir, 195.0f), 1.0f, 1e-6f));
    // Hit at z_hit=-5 -> fogged length 5.
    s.check("ground fog: 5 below offset",
            approx(umbreon::fogTransmittance(f, camPos, camDir, 205.0f),
                   std::exp(-5.0f / 16.666667f), 1e-5f));
    // Hit at z_hit=-16.6667 -> fogged length = distance -> 1/e.
    s.check("ground fog: one distance below offset ~ 1/e",
            approx(umbreon::fogTransmittance(f, camPos, camDir, 216.666667f),
                   std::exp(-1.0f), 1e-4f));
  }

  // applyFog blends each pixel toward the fog color by (1 - T). With a white
  // fog and a hit deep below the offset, the surface lightens toward white.
  {
    umbreon::Fog f;
    f.enabled = true;
    f.type = 2;
    f.distance = 16.666667f;
    f.offset = 0.0f;
    f.up = umbreon::Vec3{0, 0, 1};
    f.color = umbreon::Vec3{1, 1, 1};

    umbreon::Camera cam;
    cam.position = camPos;
    cam.direction = camDir;

    // One pixel: mid-gray surface, hit at z_hit=-16.6667 (T ~ 1/e).
    float color[3] = {0.2f, 0.2f, 0.2f};
    float depth[1] = {216.666667f};
    umbreon::applyFog(f, cam, 1, 1, 3, color, depth);
    float T = std::exp(-1.0f);
    float expected = T * 0.2f + (1.0f - T) * 1.0f;
    s.check("applyFog blends toward white fog", approx(color[0], expected, 1e-4f));
    s.check("applyFog brightened the surface", color[0] > 0.2f);

    // Background pixel (infinite depth) is left untouched.
    float bgColor[3] = {0.04f, 0.05f, 0.06f};
    float bgDepth[1] = {std::numeric_limits<float>::infinity()};
    umbreon::applyFog(f, cam, 1, 1, 3, bgColor, bgDepth);
    s.check("applyFog skips background (inf depth)",
            approx(bgColor[0], 0.04f, 1e-6f));
  }

  return s.report();
}
