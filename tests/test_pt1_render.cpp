// pt1 integration tests against the render() facade.
//
// Phase 2 (direct-lighting parity): pt1 replaces only the indirect post-pass;
// the direct shading comes from the exact same pixel loop as the cache path.
// Rendering the same GI scene with both integrators at giIntensity = 0 (the
// composite then adds exactly 0.0f) must therefore produce bitwise-identical
// color on mesh and background pixels -- this locks the "direct matches the
// existing renderer" acceptance criterion by construction and guards the
// dispatch seam against divergence.
//
// REAL CSG primitives (atom balls / bonds) are the one intended divergence:
// under pt1 they become GI receivers, so their constant ambient is dropped
// from the direct shade (replaced by the gathered indirect); under the cache
// they keep the flat ambient. At giIntensity = 0 a CSG pixel is therefore
// DARKER (never brighter) in the pt1 render, and at full strength it must
// carry a nonzero `indirect` AOV.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

// A GI test scene: a floor quad (mesh, gather-eligible) plus a sphere
// primitive (CSG occluder) under one angled distant light.
umbreon::Mesh makeFloor(umbreon::Vec4 color) {
  using umbreon::Vec3;
  umbreon::Mesh m;
  const Vec3 p00{-4, 0, -4}, p10{4, 0, -4}, p11{4, 0, 4}, p01{-4, 0, 4};
  const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
  const Vec3 n{0, 1, 0};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;
  return m;
}

umbreon::Scene makeGiScene() {
  umbreon::Scene sc;
  sc.mesh = makeFloor(umbreon::Vec4{0.7f, 0.7f, 0.7f, 1.0f});
  sc.camera.position = {0, 3, 8};
  sc.camera.direction = umbreon::normalize(umbreon::Vec3{0, -0.35f, -1});
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = false;
  sc.camera.fovy = 40.0f;
  umbreon::DistantLight l;
  l.direction = umbreon::normalize(umbreon::Vec3{-0.4f, -1.0f, -0.3f});
  l.color = {1, 1, 1};
  l.intensity = 0.8f;
  sc.lights.push_back(l);
  umbreon::Sphere sp;
  sp.center = {0, 1, 0};
  sp.radius = 1.0f;
  sp.color = {0.8f, 0.3f, 0.3f, 1.0f};
  sp.material.ambient = 0.2f;
  sp.material.diffuse = 0.8f;
  sc.spheres.push_back(sp);
  sc.background = {0.1f, 0.1f, 0.15f};
  sc.ambientColor = {0.5f, 0.5f, 0.5f};  // GI energy the gather collects
  return sc;
}

umbreon::RenderOptions makeGiOptions(int integrator, float giIntensity) {
  umbreon::RenderOptions o;
  o.width = 48;
  o.height = 36;
  o.gi = true;
  o.giIntegrator = integrator;
  o.giIntensity = giIntensity;
  o.giSamples = 16;   // keep the cache branch cheap
  o.pt1Spp = 4;       // keep the pt1 branch cheap
  o.pt1HalfRes = false;
  o.pt1Denoise = false;
  o.shadows = true;
  return o;
}

// Open-top Cornell box (plan Phase 3 adaptation: the repo has distant lights
// only, no area lights, so the box is lit from above through the missing
// ceiling by a straight-down distant light). White floor and back wall, red
// left wall, green right wall; bleeding shows on the floor next to each wall.
namespace cornell {

void addQuad(umbreon::Mesh& m, umbreon::Vec3 a, umbreon::Vec3 b,
             umbreon::Vec3 c, umbreon::Vec3 d, umbreon::Vec3 n,
             umbreon::Vec4 color) {
  const umbreon::Vec3 corners[6] = {a, b, c, a, c, d};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
}

umbreon::Scene makeScene() {
  using umbreon::Vec3;
  using umbreon::Vec4;
  const Vec4 white{0.75f, 0.75f, 0.75f, 1.0f};
  const Vec4 red{0.75f, 0.05f, 0.05f, 1.0f};
  const Vec4 green{0.05f, 0.75f, 0.05f, 1.0f};
  umbreon::Mesh m;
  // Box interior [-1,1]^2 x [0,2] in y, open top (no ceiling) and open front
  // (+z, toward the camera). Normals face inward.
  addQuad(m, {-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}, {0, 1, 0}, white);
  addQuad(m, {-1, 0, -1}, {-1, 2, -1}, {1, 2, -1}, {1, 0, -1}, {0, 0, 1}, white);
  addQuad(m, {-1, 0, -1}, {-1, 0, 1}, {-1, 2, 1}, {-1, 2, -1}, {1, 0, 0}, red);
  addQuad(m, {1, 0, -1}, {1, 2, -1}, {1, 2, 1}, {1, 0, 1}, {-1, 0, 0}, green);
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;

  umbreon::Scene sc;
  sc.mesh = std::move(m);
  sc.camera.position = {0, 1, 3.2f};
  sc.camera.direction = {0, 0, -1};
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = false;
  sc.camera.fovy = 45.0f;
  umbreon::DistantLight l;
  l.direction = {0, -1, 0};  // straight down through the open top
  l.color = {1, 1, 1};
  l.intensity = 1.0f;
  sc.lights.push_back(l);
  sc.background = {0, 0, 0};
  sc.ambientColor = {0.25f, 0.25f, 0.25f};  // small sky fill via the open top
  return sc;
}

}  // namespace cornell

}  // namespace

int main() {
  umbreon::test::Suite s("pt1_render");
  const umbreon::Scene sc = makeGiScene();

  // --- direct-lighting parity at giIntensity = 0 (both integrators add
  // exactly +0.0f indirect). Mesh/background pixels must match bitwise; the
  // pixels covered by the REAL CSG sphere are the intended exception -- pt1
  // drops their constant ambient (the gather replaces it), so they are darker,
  // never brighter. The per-pixel divergence mask feeds the next check.
  const std::size_t npix =
      static_cast<std::size_t>(48) * 36;  // makeGiOptions dimensions
  std::vector<uint8_t> diverged(npix, 0);
  {
    const umbreon::FrameResult a = umbreon::render(sc, makeGiOptions(0, 0.0f));
    const umbreon::FrameResult b = umbreon::render(sc, makeGiOptions(1, 0.0f));
    bool sizeSame = a.color.size() == b.color.size() &&
                    a.color.size() == npix * 4;
    bool csgOnlyDarker = sizeSame;
    std::size_t nDiverged = 0;
    for (std::size_t p = 0; sizeSame && p < npix; ++p) {
      bool same = true, darker = true;
      for (int c = 0; c < 3; ++c) {
        const float va = a.color[p * 4 + c];
        const float vb = b.color[p * 4 + c];
        if (va != vb) same = false;
        if (vb > va) darker = false;
      }
      if (!same) {
        diverged[p] = 1;
        ++nDiverged;
        if (!darker) csgOnlyDarker = false;
      }
    }
    s.check("direct parity: divergence only where pt1 dropped CSG ambient "
            "(darker, never brighter)",
            sizeSame && csgOnlyDarker);
    // The sphere is visible, so SOME pixels must diverge; and it covers a
    // minority of the frame, so most pixels (floor/background) must not.
    s.check("direct parity: CSG divergence present but bounded",
            nDiverged > 0 && nDiverged < npix / 2);
  }

  // --- direct component parity at full GI strength: on non-CSG pixels,
  // color - indirect must agree within float rounding (the indirect
  // add/subtract costs <= 1 ulp). CSG pixels (diverged mask) must now carry
  // nonzero indirect under pt1 -- they are receivers, not flat ambient.
  {
    const umbreon::FrameResult a = umbreon::render(sc, makeGiOptions(0, 1.0f));
    const umbreon::FrameResult b = umbreon::render(sc, makeGiOptions(1, 1.0f));
    bool directSame = b.indirect.size() == npix * 3;
    float maxDiff = 0.0f;
    bool csgHasIndirect = directSame;
    for (std::size_t p = 0; p < npix && directSame; ++p) {
      if (diverged[p]) {
        const float ind = b.indirect[p * 3 + 0] + b.indirect[p * 3 + 1] +
                          b.indirect[p * 3 + 2];
        if (!(ind > 0.0f)) csgHasIndirect = false;
        continue;
      }
      for (int c = 0; c < 3; ++c) {
        const float da = a.color[p * 4 + c] - a.indirect[p * 3 + c];
        const float db = b.color[p * 4 + c] - b.indirect[p * 3 + c];
        maxDiff = std::fmax(maxDiff, std::fabs(da - db));
      }
    }
    s.check("direct parity: color - indirect matches within 1e-5 off-CSG",
            directSame && maxDiff <= 1e-5f);
    s.check("csg receiver: every CSG pixel gets indirect > 0 under pt1",
            csgHasIndirect);
  }

  // --- Cornell color bleeding (plan Phase 3 acceptance 2): with the pt1
  // full-res gather, the white floor next to the red wall must pick up red
  // indirect light (R > 1.15 * G in the `indirect` AOV), mirrored for green,
  // and the floor center must receive nonzero indirect.
  {
    const umbreon::Scene box = cornell::makeScene();
    umbreon::RenderOptions o;
    o.width = 64;
    o.height = 64;
    o.gi = true;
    o.giIntegrator = 1;
    o.pt1Spp = 128;
    o.pt1HalfRes = false;
    o.pt1Denoise = false;
    o.shadows = true;
    const umbreon::FrameResult f = umbreon::render(box, o);

    // Sample the floor in the lower image half: rows ~3/4 down look at the
    // floor; columns near the left/right edges sit next to the red/green wall.
    auto indirectAt = [&](int px, int py) {
      const std::size_t pix = static_cast<std::size_t>(py) * f.width + px;
      return umbreon::Vec3{f.indirect[pix * 3 + 0], f.indirect[pix * 3 + 1],
                           f.indirect[pix * 3 + 2]};
    };
    const int floorRow = 52;
    const umbreon::Vec3 nearRed = indirectAt(6, floorRow);
    const umbreon::Vec3 nearGreen = indirectAt(57, floorRow);
    const umbreon::Vec3 center = indirectAt(32, floorRow);
    s.check("cornell: floor near red wall bleeds red (R > 1.15 G)",
            nearRed.x > 1.15f * nearRed.y && nearRed.x > 0.0f);
    s.check("cornell: floor near green wall bleeds green (G > 1.15 R)",
            nearGreen.y > 1.15f * nearGreen.x && nearGreen.y > 0.0f);
    s.check("cornell: floor center indirect > 0",
            center.x > 0.0f && center.y > 0.0f && center.z > 0.0f);

    // Multi-bounce (quality plan Phase 2): a second bounce routes additional
    // light into the box (wall -> wall -> floor), so the floor-center indirect
    // must strictly grow versus the 1-bounce render above.
    o.giBounces = 2;
    const umbreon::FrameResult f2 = umbreon::render(box, o);
    auto indirectAt2 = [&](int px, int py) {
      const std::size_t pix = static_cast<std::size_t>(py) * f2.width + px;
      return umbreon::Vec3{f2.indirect[pix * 3 + 0], f2.indirect[pix * 3 + 1],
                           f2.indirect[pix * 3 + 2]};
    };
    const umbreon::Vec3 center2 = indirectAt2(32, floorRow);
    s.check("cornell: 2-bounce floor center indirect > 1-bounce",
            center2.x > center.x && center2.y > center.y &&
                center2.z > center.z);
  }

  // --- gather self-intersection regression: an ISOLATED convex sphere with a
  // distant camera. A convex body cannot occlude its own outward hemisphere,
  // so the gather occlusion at every sphere pixel must be ~0. The first-hit
  // position carries an absolute error ~ t * 2^-23 that grows with the camera
  // distance (t ~ 200 here); if the gather-origin epsilon does not scale with
  // that PRIMARY-RAY distance, origins quantize below the surface and gather
  // rays re-hit the sphere from inside (occlusion ~0.5, falsely darkened GI).
  // This scene has NO mesh, so the old mesh-AABB-diagonal epsilon scale
  // degenerated to its 1.0 fallback and exhibited exactly that.
  {
    umbreon::Scene sc2;
    umbreon::Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.0f;
    sp.color = {0.8f, 0.3f, 0.3f, 1.0f};
    sp.material.ambient = 0.2f;
    sp.material.diffuse = 0.8f;
    sc2.spheres.push_back(sp);
    sc2.camera.position = {0, 0, 200};
    sc2.camera.direction = {0, 0, -1};
    sc2.camera.up = {0, 1, 0};
    sc2.camera.orthographic = true;
    sc2.camera.height = 4.0f;
    umbreon::DistantLight l;
    l.direction = umbreon::normalize(umbreon::Vec3{-0.4f, -0.3f, -1.0f});
    l.color = {1, 1, 1};
    l.intensity = 0.8f;
    sc2.lights.push_back(l);
    sc2.background = {1, 1, 1};
    sc2.ambientColor = {0.5f, 0.5f, 0.5f};

    umbreon::RenderOptions o;
    o.width = 33;
    o.height = 33;
    o.gi = true;
    o.giIntegrator = 1;  // pt1
    o.pt1Spp = 16;
    o.pt1HalfRes = false;
    o.pt1Denoise = false;
    const umbreon::FrameResult f = umbreon::render(sc2, o);
    // Center pixel: the sphere's front pole (head-on hit, |P| ~ 1, t ~ 199).
    const std::size_t cpix = static_cast<std::size_t>(16) * f.width + 16;
    s.check("isolated sphere: gather occlusion ~0 at distant camera",
            f.giOcclusion[cpix] < 0.05f);
    // The whole sphere disc must be self-occlusion free, not just the pole.
    float maxOcc = 0.0f;
    for (std::size_t i = 0; i < f.giOcclusion.size(); ++i)
      maxOcc = std::max(maxOcc, f.giOcclusion[i]);
    s.check("isolated sphere: no pixel self-occludes", maxOcc < 0.2f);
  }

  // --- gather-grid divisor smoke: pt1GatherDiv = 4 renders deterministically
  // and still produces indirect light on the GI scene's floor.
  {
    umbreon::RenderOptions o = makeGiOptions(1, 1.0f);
    o.pt1GatherDiv = 4;
    const umbreon::FrameResult f1 = umbreon::render(sc, o);
    const umbreon::FrameResult f2 = umbreon::render(sc, o);
    s.check("gatherDiv=4: deterministic across runs",
            f1.color == f2.color && f1.indirect == f2.indirect);
    float maxInd = 0.0f;
    for (float v : f1.indirect) maxInd = std::max(maxInd, v);
    s.check("gatherDiv=4: nonzero indirect", maxInd > 0.0f);
    bool finite = true;
    for (float v : f1.color)
      if (!std::isfinite(v)) finite = false;
    s.check("gatherDiv=4: finite color", finite);
  }

  return s.report();
}
