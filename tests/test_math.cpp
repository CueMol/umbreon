// Unit tests for the math primitives in scene.hpp that back the AO / soft-shadow
// secondary-ray work: safeNormalize (never returns NaN/Inf for degenerate input,
// unlike the plain normalize which returns a non-unit zero) and frameFromNormal
// (an orthonormal, right-handed basis around a normal, including the axis-aligned
// cases a naive cross(N, fixedAxis) silently degenerates on).
#include <cmath>
#include <string>

#include "scene.hpp"
#include "test_util.hpp"

namespace {
bool approx(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) <= eps; }
bool isUnit(umbreon::Vec3 v) { return approx(umbreon::length(v), 1.0f, 1e-5f); }
bool finite3(umbreon::Vec3 v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// A frameFromNormal result must be orthonormal and right-handed (cross(t,b)==n)
// for any unit normal n.
void checkFrame(umbreon::test::Suite& s, const std::string& name,
                umbreon::Vec3 n) {
  const umbreon::Frame f = umbreon::frameFromNormal(n);
  s.check(name + ": t unit", isUnit(f.t));
  s.check(name + ": b unit", isUnit(f.b));
  s.check(name + ": t perp n", approx(umbreon::dot(f.t, n), 0.0f));
  s.check(name + ": b perp n", approx(umbreon::dot(f.b, n), 0.0f));
  s.check(name + ": t perp b", approx(umbreon::dot(f.t, f.b), 0.0f));
  const umbreon::Vec3 c = umbreon::cross(f.t, f.b);
  s.check(name + ": cross(t,b)=n",
          approx(c.x, n.x) && approx(c.y, n.y) && approx(c.z, n.z));
}
}  // namespace

int main() {
  umbreon::test::Suite s("math");

  // --- safeNormalize: unit output for a normal input; finite (non-NaN) for the
  // degenerate zero vector, where the plain normalize() returns a non-unit zero.
  {
    const umbreon::Vec3 u = umbreon::safeNormalize(umbreon::Vec3{3.0f, 0.0f, 4.0f});
    s.check("safeNormalize: scales to unit", isUnit(u));
    s.check("safeNormalize: direction preserved",
            approx(u.x, 0.6f) && approx(u.z, 0.8f));

    const umbreon::Vec3 z = umbreon::safeNormalize(umbreon::Vec3{0.0f, 0.0f, 0.0f});
    s.check("safeNormalize: zero input is finite (no NaN/Inf)", finite3(z));

    // fallback overload: degenerate input returns the supplied direction;
    // a valid input still normalizes.
    const umbreon::Vec3 fb = umbreon::safeNormalize(
        umbreon::Vec3{0.0f, 0.0f, 0.0f}, umbreon::Vec3{0.0f, 1.0f, 0.0f});
    s.check("safeNormalize(fallback): degenerate returns fallback",
            approx(fb.x, 0.0f) && approx(fb.y, 1.0f) && approx(fb.z, 0.0f));
    const umbreon::Vec3 fb2 = umbreon::safeNormalize(
        umbreon::Vec3{0.0f, 0.0f, 2.0f}, umbreon::Vec3{1.0f, 0.0f, 0.0f});
    s.check("safeNormalize(fallback): valid input normalizes",
            isUnit(fb2) && approx(fb2.z, 1.0f));
  }

  // --- frameFromNormal: orthonormal right-handed basis for a general normal and
  // for every axis-aligned normal (the axis cases break a fixed-seed cross). ---
  checkFrame(s, "frame +x", umbreon::Vec3{1.0f, 0.0f, 0.0f});
  checkFrame(s, "frame +y", umbreon::Vec3{0.0f, 1.0f, 0.0f});
  checkFrame(s, "frame +z", umbreon::Vec3{0.0f, 0.0f, 1.0f});
  checkFrame(s, "frame -z", umbreon::Vec3{0.0f, 0.0f, -1.0f});
  checkFrame(s, "frame oblique",
             umbreon::safeNormalize(umbreon::Vec3{0.3f, -0.5f, 0.8f}));

  // A degenerate (zero) normal must not produce NaN/Inf.
  {
    const umbreon::Frame f = umbreon::frameFromNormal(umbreon::Vec3{0.0f, 0.0f, 0.0f});
    s.check("frame zero-normal: finite (no NaN)",
            finite3(f.t) && finite3(f.b) && finite3(f.n));
  }

  return s.report();
}
