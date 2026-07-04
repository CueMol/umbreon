// Ambient-occlusion integration tests: the legacy binary AO and the
// quality estimator family (falloff, multi-scale, bent normal, multibounce,
// low-discrepancy sampling, AOV gate).
// Split out of the monolithic test_render.cpp (same assertions, relocated).
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "render_test_util.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"

int main() {
  umbreon::test::Suite s("render_ao");
  const umbreon::Vec4 pigment{0.5f, 0.6f, 0.7f, 1.0f};

  // ===== Ambient occlusion (mesh hits only; modulates the ambient term) =====
  // AO is gated off by default (aoSamples == 0, locked bit-exact by the tests
  // above). These exercise the secondary-ray path: it must (1) leave an open
  // surface ~unchanged, (2) darken an occluded surface via the ambient term
  // only, (3) never touch flat outline primitives, (4) be deterministic.

  // AO-open: a lone quad with nothing above it -> every hemisphere ray escapes
  // -> aoFactor ~ 1 -> center ~ the no-AO value (0.30/0.36). Proves AO does not
  // darken an unoccluded surface (and that the AO path actually runs).
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0, 0, 0};
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5;
    o.aoSamples = 64;
    o.aoDistance = 100.0f;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("AO open: center R ~ no-AO 0.30", approx(f.color[kCenterRgba + 0], 0.30f, 0.02f));
    s.check("AO open: center G ~ no-AO 0.36", approx(f.color[kCenterRgba + 1], 0.36f, 0.02f));
  }

  // AO-occlusion: a floor with an AMBIENT-ONLY material (out == aoFactor*C) plus
  // a slab just above it, offset in +x so it clears the center camera ray (x=0)
  // but blocks the +x half of the floor point's hemisphere. The occluded floor
  // must be strictly darker than the same floor with no slab; only the ambient
  // term is affected.
  {
    using umbreon::Vec3;
    auto floor = [](bool withSlab) {
      umbreon::Mesh m;
      const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                          {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(fl[i]);
        m.normals.push_back({0, 0, 1});
        m.colors.push_back({1, 1, 1, 1});
      }
      if (withSlab) {
        const float z = 0.4f;  // close above the floor
        const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                            {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
        for (int i = 0; i < 6; ++i) {
          m.positions.push_back(sl[i]);
          m.normals.push_back({0, 0, -1});
          m.colors.push_back({0, 0, 0, 1});
        }
      }
      m.material.ambient = 1.0f;  // ambient-only: isolate AO's effect
      m.material.diffuse = 0.0f;
      return m;
    };
    auto sceneOf = [](umbreon::Mesh m) {
      umbreon::Scene sc;
      sc.mesh = std::move(m);
      sc.camera = makeOrthoCam();
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      return sc;
    };
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5;
    o.aoSamples = 128;
    o.aoDistance = 10.0f;
    umbreon::FrameResult openF = umbreon::render(sceneOf(floor(false)), o);
    umbreon::FrameResult occF = umbreon::render(sceneOf(floor(true)), o);
    s.check("AO occ: open floor ~ full ambient (R~1)",
            approx(openF.color[kCenterRgba + 0], 1.0f, 0.03f));
    s.check("AO occ: slab darkens the floor (occluded < open)",
            occF.color[kCenterRgba + 0] + 0.05f < openF.color[kCenterRgba + 0]);
    s.check("AO occ: occluded floor not fully black",
            occF.color[kCenterRgba + 0] > 0.05f);
  }

  // AO on REAL primitives vs the OUTLINE gate (current design). A REAL CSG sphere
  // (fromEdgeMacro == false, e.g. a CPK atom ball) is AO-darkened exactly like
  // the mesh, so an atom shadowed by neighbouring geometry darkens like the SES
  // around it. A baked NPR OUTLINE primitive (fromEdgeMacro == true, silhouette
  // decoration) keeps the original "never AO-darkened" gate. The occluder is
  // always an outline sphere (it occludes via the BVH but never receives AO), so
  // any AO on/off delta comes solely from the probed sphere. Ambient-only
  // material (ambient 1 / diffuse 0) isolates AO's effect.
  {
    auto sphereScene = [](bool probedFromEdge) {
      umbreon::Scene sc;
      sc.camera = makeOrthoCam();
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      umbreon::Sphere sp;  // probed sphere, centered in view
      sp.center = {0, 0, 0};
      sp.radius = 1.0f;
      sp.color = {1, 1, 1, 1};
      sp.material.ambient = 1.0f;
      sp.material.diffuse = 0.0f;
      sp.fromEdgeMacro = probedFromEdge;
      sc.spheres.push_back(sp);
      umbreon::Sphere occ;  // big occluder beside it; ALWAYS outline (no AO recv)
      occ.center = {0, 2.2f, 0};
      occ.radius = 2.0f;
      occ.color = {0, 0, 0, 1};
      occ.material.ambient = 1.0f;
      occ.material.diffuse = 0.0f;
      occ.fromEdgeMacro = true;
      sc.spheres.push_back(occ);
      return sc;
    };
    umbreon::RenderOptions off;
    off.width = 5; off.height = 5; off.aoSamples = 0;
    umbreon::RenderOptions on = off;
    on.aoSamples = 128;
    on.aoDistance = 10.0f;
    // Real sphere (fromEdgeMacro=false): AO must darken it (occluder blocks part
    // of its hemisphere), so the image changes between AO on and off.
    umbreon::FrameResult rOff = umbreon::render(sphereScene(false), off);
    umbreon::FrameResult rOn = umbreon::render(sphereScene(false), on);
    float diff = 0.0f;
    for (std::size_t i = 0; i < rOff.color.size(); ++i)
      diff += std::fabs(rOn.color[i] - rOff.color[i]);
    s.check("AO real primitive: AO darkens a real CSG sphere", diff > 0.05f);
    // Outline sphere (fromEdgeMacro=true): the gate holds -- AO on == off,
    // bit-identical (nothing in the scene receives AO).
    umbreon::FrameResult oOff = umbreon::render(sphereScene(true), off);
    umbreon::FrameResult oOn = umbreon::render(sphereScene(true), on);
    bool identical = oOff.color.size() == oOn.color.size();
    for (std::size_t i = 0; identical && i < oOff.color.size(); ++i)
      if (oOn.color[i] != oOff.color[i]) identical = false;
    s.check("AO outline gate: outline sphere identical AO on/off", identical);
  }

  // AO determinism: the RNG is seeded only from (px, py, sample), so two renders
  // of the same scene are bit-identical -- independent of TBB thread count.
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0, 0, 0};
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5;
    o.aoSamples = 32;
    o.aoDistance = 100.0f;
    umbreon::FrameResult a = umbreon::render(sc, o);
    umbreon::FrameResult b = umbreon::render(sc, o);
    bool identical = a.color.size() == b.color.size();
    for (std::size_t i = 0; identical && i < a.color.size(); ++i)
      if (a.color[i] != b.color[i]) identical = false;
    s.check("AO determinism: two renders bit-identical", identical);
  }

  // ===== AO quality: distance falloff + multi-scale (computeAOQuality) =====
  // Shared rig: an ambient-only white floor with a black slab offset in +x (so
  // it clears the center camera ray at x=0 but blocks the +x half of the floor
  // point's hemisphere). Raising the slab moves the occluder farther away.
  {
    using umbreon::Vec3;
    auto floorSlab = [](float z) {
      umbreon::Mesh m;
      const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                          {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(fl[i]);
        m.normals.push_back({0, 0, 1});
        m.colors.push_back({1, 1, 1, 1});
      }
      const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                          {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(sl[i]);
        m.normals.push_back({0, 0, -1});
        m.colors.push_back({0, 0, 0, 1});
      }
      m.material.ambient = 1.0f;  // ambient-only: isolate AO's effect
      m.material.diffuse = 0.0f;
      return m;
    };
    auto centerR = [&](float z, float falloff, bool multiscale) {
      umbreon::Scene sc;
      sc.mesh = floorSlab(z);
      sc.camera = makeOrthoCam();
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.aoSamples = 256;
      o.aoDistance = 10.0f;
      o.aoFalloffPower = falloff;
      o.aoMultiScale = multiscale;
      return umbreon::render(sc, o).color[kCenterRgba + 0];
    };

    // Distance falloff: power-2 falloff never darkens MORE than binary (falloff
    // <= 1 => openness >= binary), and it releases a FAR occluder much more than
    // a NEAR one -- so contact stays dark while distant geometry stops counting.
    const float binNear = centerR(0.4f, 0.0f, false);
    const float binFar = centerR(3.0f, 0.0f, false);
    const float foNear = centerR(0.4f, 2.0f, false);
    const float foFar = centerR(3.0f, 2.0f, false);
    s.check("AO falloff: near never darker than binary", foNear >= binNear - 1e-4f);
    s.check("AO falloff: far never darker than binary", foFar >= binFar - 1e-4f);
    s.check("AO falloff: releases far occlusion more than near",
            (foFar - binFar) > (foNear - binNear) + 0.05f);

    // Multi-scale: vs single-scale binary at the SAME radius, the nested radii
    // down-weight a far occluder (only the 0.10-weight large scale reaches it)
    // yet keep a near contact dark (it falls in the heavy small scale).
    const float msFar = centerR(4.0f, 0.0f, true);
    const float msNear = centerR(0.4f, 0.0f, true);
    const float binFar4 = centerR(4.0f, 0.0f, false);
    s.check("AO multiscale: far occluder released vs binary",
            msFar > binFar4 + 0.05f);
    s.check("AO multiscale: near contact stays darker than far", msNear < msFar);
  }

  // ===== AO quality: bent normal directional ambient =====
  // A lone ambient-only white floor (normal +z, bent normal ~ +z, no occluder).
  // sky=white / ground=black so the ambient is the hemisphere gradient value
  // lerp(0, 1, 0.5*(dot(bent,up)+1)). up=+z -> w=1 -> full ambient (~1); up=-z ->
  // w=0 -> ground (~0); up=+x (perpendicular) -> w=0.5 -> mid. Proves the bent
  // normal steers the directional ambient and the gradient is monotonic in
  // dot(bent, up).
  {
    auto floorOnly = []() {
      umbreon::Mesh m;
      const umbreon::Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                                   {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(fl[i]);
        m.normals.push_back({0, 0, 1});
        m.colors.push_back({1, 1, 1, 1});
      }
      m.material.ambient = 1.0f;
      m.material.diffuse = 0.0f;
      return m;
    };
    auto centerWithUp = [&](float ux, float uy, float uz) {
      umbreon::Scene sc;
      sc.mesh = floorOnly();
      sc.camera = makeOrthoCam();
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.aoSamples = 64;
      o.aoDistance = 10.0f;
      o.aoBentNormal = true;
      o.aoSkyColor[0] = o.aoSkyColor[1] = o.aoSkyColor[2] = 1.0f;
      o.aoGroundColor[0] = o.aoGroundColor[1] = o.aoGroundColor[2] = 0.0f;
      o.aoUseCameraUp = false;
      o.aoUp[0] = ux; o.aoUp[1] = uy; o.aoUp[2] = uz;
      return umbreon::render(sc, o).color[kCenterRgba + 0];
    };
    const float upPlus = centerWithUp(0, 0, 1);
    const float upMinus = centerWithUp(0, 0, -1);
    const float upPerp = centerWithUp(1, 0, 0);
    s.check("AO bent: up=+z -> sky ambient (bright)", upPlus > 0.9f);
    s.check("AO bent: up=-z -> ground ambient (dark)", upMinus < 0.1f);
    s.check("AO bent: up=+x -> mid gradient", approx(upPerp, 0.5f, 0.1f));
    s.check("AO bent: gradient monotonic in dot(bent,up)",
            upMinus < upPerp && upPerp < upPlus);
  }

  // ===== AO quality: albedo-aware multibounce =====
  // Multibounce lifts the AO term per albedo channel, so a light cavity recovers
  // more than a dark one. Floor (variable gray) + black slab. The occluded/open
  // ratio (which cancels the floor color) must be higher for a white floor than a
  // gray one, and multibounce must actually brighten the occluded white floor vs
  // plain AO.
  {
    using umbreon::Vec3;
    auto floorSlab = [](float gray, bool withSlab) {
      umbreon::Mesh m;
      const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                          {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(fl[i]);
        m.normals.push_back({0, 0, 1});
        m.colors.push_back({gray, gray, gray, 1});
      }
      if (withSlab) {
        const float z = 0.4f;
        const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                            {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
        for (int i = 0; i < 6; ++i) {
          m.positions.push_back(sl[i]);
          m.normals.push_back({0, 0, -1});
          m.colors.push_back({0, 0, 0, 1});
        }
      }
      m.material.ambient = 1.0f;
      m.material.diffuse = 0.0f;
      return m;
    };
    auto centerR = [&](float gray, bool slab, bool mb) {
      umbreon::Scene sc;
      sc.mesh = floorSlab(gray, slab);
      sc.camera = makeOrthoCam();
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.aoSamples = 256;
      o.aoDistance = 10.0f;
      o.aoMultibounce = mb;
      return umbreon::render(sc, o).color[kCenterRgba + 0];
    };
    const float openWhite = centerR(1.0f, false, true);
    const float occWhite = centerR(1.0f, true, true);
    const float openGray = centerR(0.5f, false, true);
    const float occGray = centerR(0.5f, true, true);
    const float occWhiteNoMb = centerR(1.0f, true, false);
    const float ratioWhite = occWhite / openWhite;
    const float ratioGray = occGray / openGray;
    s.check("AO multibounce: white recovers more than gray (ratio)",
            ratioWhite > ratioGray + 0.02f);
    s.check("AO multibounce: lifts occluded white vs plain AO",
            occWhite > occWhiteNoMb + 0.01f);
  }

  // ===== AO quality: aoDiffuseFactor (also darken direct diffuse) =====
  // A diffuse-lit white floor (head-on key light) + black slab. aoDiffuseFactor>0
  // scales the DIRECT diffuse term down where the floor is occluded; 0 leaves it
  // (POV ambient-only contract). So the occluded floor is darker with it on.
  {
    using umbreon::Vec3;
    auto floorSlab = []() {
      umbreon::Mesh m;
      const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                          {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(fl[i]);
        m.normals.push_back({0, 0, 1});
        m.colors.push_back({1, 1, 1, 1});
      }
      const float z = 0.4f;
      const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                          {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(sl[i]);
        m.normals.push_back({0, 0, -1});
        m.colors.push_back({0, 0, 0, 1});
      }
      m.material.ambient = 0.2f;
      m.material.diffuse = 0.8f;  // a lit direct-diffuse term to scale
      return m;
    };
    auto centerR = [&](float df) {
      umbreon::Scene sc;
      sc.mesh = floorSlab();
      sc.camera = makeOrthoCam();
      sc.lights.push_back(makeKeyLight());
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.aoSamples = 256;
      o.aoDistance = 10.0f;
      o.aoDiffuseFactor = df;
      return umbreon::render(sc, o).color[kCenterRgba + 0];
    };
    s.check("AO diffuse factor: darkens occluded direct diffuse",
            centerR(0.7f) + 0.01f < centerR(0.0f));
  }

  // ===== AO quality: low-discrepancy sampling reduces variance =====
  // Vs white-noise tea2 at the same low sample count, Hammersley + per-pixel
  // Cranley-Patterson sampling sits closer to the converged AO. Aggregate the abs
  // error over the floor pixels against a high-sample reference; LD must be the
  // smaller total. (Multi-scale on so the LD path -- aoEnhanced -- runs for all.)
  {
    using umbreon::Vec3;
    auto scene = []() {
      umbreon::Scene sc;
      umbreon::Mesh m;
      const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                          {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(fl[i]);
        m.normals.push_back({0, 0, 1});
        m.colors.push_back({1, 1, 1, 1});
      }
      const float z = 0.6f;
      const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                          {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(sl[i]);
        m.normals.push_back({0, 0, -1});
        m.colors.push_back({0, 0, 0, 1});
      }
      m.material.ambient = 1.0f;
      m.material.diffuse = 0.0f;
      sc.mesh = std::move(m);
      sc.camera = makeOrthoCam();
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      return sc;
    };
    auto frame = [&](int samples, bool ld) {
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.aoSamples = samples;
      o.aoDistance = 10.0f;
      o.aoMultiScale = true;       // keep all three runs on the enhanced path
      o.aoLowDiscrepancy = ld;
      return umbreon::render(scene(), o);
    };
    const umbreon::FrameResult ref = frame(2048, false);
    const umbreon::FrameResult noi = frame(32, false);
    const umbreon::FrameResult ldr = frame(32, true);
    float errNoise = 0.0f, errLd = 0.0f;
    for (int p = 0; p < 5 * 5; ++p) {
      const float r = ref.color[p * 4 + 0];
      errNoise += std::fabs(noi.color[p * 4 + 0] - r);
      errLd += std::fabs(ldr.color[p * 4 + 0] - r);
    }
    s.check("AO low-discrepancy: lower aggregate error than white noise",
            errLd < errNoise);
  }

  // ===== AO quality: AOV gate is byte-identical to color; AOVs are valid =====
  // Enabling --ao-write-aov must not change the rendered color (AOVs live in
  // separate buffers), and the captured albedo/normal/contact/shape must hold
  // the expected values (albedo == pigment, normal == face normal).
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0, 0, 0};
    umbreon::RenderOptions base;
    base.width = 5; base.height = 5;
    base.aoSamples = 64;
    base.aoDistance = 100.0f;
    base.aoMultiScale = true;  // enhanced path so the contact/shape split runs
    umbreon::RenderOptions off = base; off.aoWriteAov = false;
    umbreon::RenderOptions on = base; on.aoWriteAov = true;
    umbreon::FrameResult fo = umbreon::render(sc, off);
    umbreon::FrameResult fn = umbreon::render(sc, on);
    bool colorSame = fo.color.size() == fn.color.size();
    for (std::size_t i = 0; colorSame && i < fo.color.size(); ++i)
      if (fo.color[i] != fn.color[i]) colorSame = false;
    s.check("AOV gate: color byte-identical with AOVs on", colorSame);
    s.check("AOV gate: off leaves AOV buffers empty", fo.albedo.empty());
    s.check("AOV gate: albedo populated when on", !fn.albedo.empty());
    s.check("AOV gate: contactAo populated when on", !fn.contactAo.empty());
    s.check("AOV gate: shapeAo populated when on", !fn.shapeAo.empty());
    s.check("AOV: center albedo R ~ pigment",
            approx(fn.albedo[kCenterPix * 3 + 0], 0.5f, 1e-4f));
    s.check("AOV: center albedo G ~ pigment",
            approx(fn.albedo[kCenterPix * 3 + 1], 0.6f, 1e-4f));
    s.check("AOV: center normal ~ +z",
            approx(fn.normal[kCenterPix * 3 + 2], 1.0f, 1e-3f));
  }

  // ===== AO quality: contact / shape are distinct per-scale values =====
  // contact (small radius) and shape (mid+large radii) are returned UNBLENDED.
  // A FAR occluder (beyond the contact radius, within the shape radius) drops
  // shape while contact stays open; moving the occluder NEAR also drops contact.
  {
    using umbreon::Vec3;
    auto cs = [&](float z, float& contact, float& shape) {
      umbreon::Mesh m;
      const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                          {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(fl[i]);
        m.normals.push_back({0, 0, 1});
        m.colors.push_back({1, 1, 1, 1});
      }
      const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                          {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
      for (int i = 0; i < 6; ++i) {
        m.positions.push_back(sl[i]);
        m.normals.push_back({0, 0, -1});
        m.colors.push_back({0, 0, 0, 1});
      }
      m.material.ambient = 1.0f;
      m.material.diffuse = 0.0f;
      umbreon::Scene sc;
      sc.mesh = std::move(m);
      sc.camera = makeOrthoCam();
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.aoSamples = 256;
      o.aoDistance = 10.0f;  // contact radius 0.8, shape radii 3 and 10
      o.aoMultiScale = true;
      o.aoWriteAov = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      contact = f.contactAo[kCenterPix];
      shape = f.shapeAo[kCenterPix];
    };
    float cFar, sFar, cNear, sNear;
    cs(2.0f, cFar, sFar);   // beyond contact radius (0.8): contact stays open
    cs(0.3f, cNear, sNear);  // within contact radius: contact drops
    s.check("AO contact/shape: far occluder drops shape, not contact",
            cFar > sFar + 0.1f);
    s.check("AO contact/shape: near occluder drops contact too",
            cNear < cFar - 0.1f);
  }


  return s.report();
}
