// Shadow (hard/soft/acne-guard) and diffuse-GI irradiance-cache
// integration tests, incl. the rejectDarkOutliers and denoiseAtrous units.
// Split out of the monolithic test_render.cpp (same assertions, relocated).
//
// The GI cases here exercise the IRRADIANCE CACHE integrator and its knobs
// (giSamples / giRecordSpacing / giAccuracy / giGradients / giSeedPerVertex),
// so they set giIntegrator = 0 explicitly: the default is pt1, which ignores
// those knobs entirely and would silently turn these into no-op assertions.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "render_test_util.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"
#include "experimental/irradiance_cache/irradiance_cache.hpp"  // detail::rejectDarkOutliers
#include "experimental/irradiance_cache/denoise.hpp"           // denoiseAtrous
#include "postprocess/image_ops.hpp"

int main() {
  umbreon::test::Suite s("render_shadows_gi");
  const umbreon::Vec4 pigment{0.5f, 0.6f, 0.7f, 1.0f};

  // ===== Hard shadows (per-light visibility; off by default) =====
  // A floor lit by an angled light, with a slab between the floor center and the
  // light (offset in +x so it clears the straight-down camera ray). Shadows on
  // remove the floor center's diffuse term (ambient survives); off = fully lit.
  {
    using umbreon::Vec3;
    umbreon::Mesh m;
    const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                        {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
    for (int i = 0; i < 6; ++i) {
      m.positions.push_back(fl[i]);
      m.normals.push_back({0, 0, 1});
      m.colors.push_back({1, 1, 1, 1});
    }
    const float z = 0.4f;  // slab just above the floor, offset to +x
    const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                        {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
    for (int i = 0; i < 6; ++i) {
      m.positions.push_back(sl[i]);
      m.normals.push_back({0, 0, -1});
      m.colors.push_back({0, 0, 0, 1});
    }
    m.material.ambient = 0.2f;
    m.material.diffuse = 0.8f;
    umbreon::Scene sc;
    sc.mesh = m;
    sc.camera = makeOrthoCam();
    sc.ambientColor = {1, 1, 1};
    sc.background = {0, 0, 0};
    umbreon::DistantLight l;
    l.direction = {-0.6f, 0.0f, -0.8f};  // travels -x,-z => L = (0.6, 0, 0.8)
    l.color = {1, 1, 1};
    l.intensity = 1.0f;
    sc.lights.push_back(l);
    umbreon::RenderOptions off;
    off.width = 5; off.height = 5; off.shadows = false;
    umbreon::RenderOptions on = off;
    on.shadows = true;
    umbreon::FrameResult fo = umbreon::render(sc, off);
    umbreon::FrameResult fn = umbreon::render(sc, on);
    // off: ambient 0.2 + diffuse 0.8*0.8 = 0.84; on: ambient only 0.2.
    s.check("shadow off: floor lit (R ~ 0.84)",
            approx(fo.color[kCenterRgba + 0], 0.84f, 0.03f));
    s.check("shadow on: floor center shadowed, diffuse removed (R ~ 0.2)",
            approx(fn.color[kCenterRgba + 0], 0.2f, 0.03f));
    s.check("shadow on darker than off",
            fn.color[kCenterRgba + 0] + 0.2f < fo.color[kCenterRgba + 0]);
  }

  // ===== Soft (area-light) shadows: a penumbra between lit and fully shadowed =
  // The floor-center shadow ray passes through the center of a sphere occluder.
  // A hard shadow (one ray) is fully blocked; a soft shadow (a light of angular
  // radius > 0, many cone samples) is only partially blocked, so the center
  // lands strictly between fully lit and fully shadowed.
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad({1, 1, 1, 1});  // floor z=0, +Z, material 0.2/0.8
    sc.camera = makeOrthoCam();
    sc.ambientColor = {1, 1, 1};
    sc.background = {0, 0, 0};
    umbreon::DistantLight l;
    l.direction = {-0.6f, 0.0f, -0.8f};  // L = (0.6, 0, 0.8)
    l.color = {1, 1, 1};
    l.intensity = 1.0f;
    sc.lights.push_back(l);
    umbreon::Sphere occ;  // centered on the floor-center shadow ray (t = 1.5)
    occ.center = {0.9f, 0.0f, 1.2f};
    occ.radius = 0.45f;
    occ.color = {0, 0, 0, 1.0f};
    sc.spheres.push_back(occ);
    umbreon::RenderOptions lit;
    lit.width = 5; lit.height = 5; lit.shadows = false;
    umbreon::RenderOptions hard = lit;
    hard.shadows = true;  // one ray through the sphere center -> fully shadowed
    umbreon::RenderOptions soft = hard;
    soft.lightRadius = 22.0f; soft.shadowSamples = 64;  // wider than the sphere
    umbreon::FrameResult fl = umbreon::render(sc, lit);
    umbreon::FrameResult fh = umbreon::render(sc, hard);
    umbreon::FrameResult fs = umbreon::render(sc, soft);
    const float litR = fl.color[kCenterRgba + 0];
    const float hardR = fh.color[kCenterRgba + 0];
    const float softR = fs.color[kCenterRgba + 0];
    s.check("soft shadow: hard fully shadows center (R ~ 0.2)", approx(hardR, 0.2f, 0.03f));
    s.check("soft shadow: lit center bright (R ~ 0.84)", approx(litR, 0.84f, 0.03f));
    s.check("soft shadow: penumbra strictly between hard and lit",
            hardR + 0.05f < softR && softR + 0.05f < litR);
    umbreon::FrameResult fs2 = umbreon::render(sc, soft);
    bool same = fs.color.size() == fs2.color.size();
    for (std::size_t i = 0; same && i < fs.color.size(); ++i)
      if (fs.color[i] != fs2.color[i]) same = false;
    s.check("soft shadow: deterministic (two renders identical)", same);
  }

  // ===== Shadow self-intersection (acne) guard =====
  // A large flat quad near the origin (small |P|), viewed by a FAR, TILTED ortho
  // camera (large tfar; non-axis-aligned rays => genuine hit-point rounding error
  // ~ tfar*2^-23, unlike an axis-aligned view where the arithmetic cancels
  // exactly). Head-on light, shadows on, NO occluder: every pixel must stay fully
  // lit (~0.6). If the shadow-ray offset epsilon is scaled by t=1 instead of the
  // primary ray length, the offset (~|P|*ulp) is far smaller than the hit-point
  // error, so points that round just below the surface shadow themselves and drop
  // to ambient (~0.2) -- black acne. (The near-camera shadow tests above, tfar~10
  // and axis-aligned, do not exercise this.)
  {
    using umbreon::Vec3;
    umbreon::Mesh m;
    const float q = 100.0f;  // quad >> framed region, so no ray misses to bg
    const Vec3 c[6] = {{-q, -q, 0}, {q, -q, 0}, {q, q, 0},
                       {-q, -q, 0}, {q, q, 0},  {-q, q, 0}};
    for (int i = 0; i < 6; ++i) {
      m.positions.push_back(c[i]);
      m.normals.push_back({0, 0, 1});
      m.colors.push_back({1, 1, 1, 1});
    }
    m.material.ambient = 0.2f;
    m.material.diffuse = 0.8f;
    umbreon::Scene sc;
    sc.mesh = m;
    sc.camera.position = {0.0f, 1200.0f, 1600.0f};  // ~2000 units from the origin
    sc.camera.direction = {0.0f, -0.6f, -0.8f};     // tilted, looking at origin
    sc.camera.up = {0.0f, 1.0f, 0.0f};
    sc.camera.orthographic = true;
    sc.camera.height = 6.0f;                         // hits stay near the origin
    sc.lights.push_back(makeKeyLight());             // L=(0,0,1), head-on => 0.6
    sc.background = {0, 0, 0};
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5; o.shadows = true;     // hard shadows, no occluder
    umbreon::FrameResult f = umbreon::render(sc, o);
    float minR = 1.0e9f;
    for (int p = 0; p < 25; ++p) minR = std::fmin(minR, f.color[p * 4 + 0]);
    s.check("acne guard: far/tilted lit surface, no self-shadow (min ~0.6)",
            minR > 0.5f);
  }

  // ===== Diffuse GI: surface irradiance cache (steps 1-4) =====
  // The cache is built (placement + gather/fill + interpolation) and the indirect
  // is composited into the color via the A-route: L = L_direct + giIntensity *
  // (mat.diffuse * pigment) * E_cached, with NO constant ambient and NO AO
  // multiply (occlusion lives inside E_cached -- counted once). These lock: GI
  // off is byte-identical, GI on is deterministic, occlusion darkens the color
  // without flattening, and a colored neighbor bleeds onto the receiver.

  // GI off byte-identical: a default (gi off) render must equal the locked
  // no-GI baseline (the cache pass is fully gated).
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0, 0, 0};
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5;  // gi stays off
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("GI off: center R == no-GI baseline 0.30",
            approx(f.color[kCenterRgba + 0], 0.30f, 1e-4f));
    s.check("GI off: no cache AOVs allocated", f.indirect.empty());
  }

  // GI determinism: placement (occupied-voxel set) + per-record gather seed
  // (record index only) + read-only interpolation are thread-count independent,
  // so two gi-on renders are bit-identical in the composited color.
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0, 0, 0};
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5;
    o.gi = true;
    o.giIntegrator = 0;
    o.giSamples = 48;
    umbreon::FrameResult a = umbreon::render(sc, o);
    umbreon::FrameResult b = umbreon::render(sc, o);
    bool identical = a.color.size() == b.color.size() && !a.color.empty();
    for (std::size_t i = 0; identical && i < a.color.size(); ++i)
      if (a.color[i] != b.color[i]) identical = false;
    s.check("GI determinism: two gi-on renders bit-identical color", identical);
    // FrameResult::denoiserUsed reporting on the render() facade. RenderOptions
    // defaults denoiser to None (the GI-conditional a-trous default lives in the
    // bench CLI, not the library), so an unmodified GI render reports 0; an
    // explicit a-trous request reports 1.
    s.check("denoiserUsed == 0 when denoiser None (default)",
            a.denoiserUsed == 0);
    umbreon::RenderOptions atr = o;
    atr.denoiser = 1;  // DenoiserBackend::AtrousBilateral
    s.check("denoiserUsed == 1 for an explicit a-trous render",
            umbreon::render(sc, atr).denoiserUsed == 1);
  }

  // Single counting (the A-route番人 test): a diffuse-lit white floor with a
  // slab above it. With GI on the occluded floor is strictly DARKER than the
  // open floor (the slab cuts indirect, and bounces no light), yet stays well
  // above black (direct diffuse is untouched). This is the "darker but not
  // washed flat" property -- the old constant-ambient + AO-multiply pipeline
  // could not produce it.
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
        const float z = 0.4f;  // close above the floor, offset +x
        const Vec3 sl[6] = {{0.1f, -2, z}, {4, -2, z}, {4, 2, z},
                            {0.1f, -2, z}, {4, 2, z},  {0.1f, 2, z}};
        for (int i = 0; i < 6; ++i) {
          m.positions.push_back(sl[i]);
          m.normals.push_back({0, 0, -1});
          m.colors.push_back({0, 0, 0, 1});
        }
      }
      m.material.ambient = 0.2f;
      m.material.diffuse = 0.8f;  // a real direct-diffuse + indirect term
      return m;
    };
    auto sceneOf = [](umbreon::Mesh m) {
      umbreon::Scene sc;
      sc.mesh = std::move(m);
      sc.camera = makeOrthoCam();
      sc.lights.push_back(makeKeyLight());  // head-on key, lights the floor
      sc.ambientColor = {1, 1, 1};
      sc.background = {0, 0, 0};
      return sc;
    };
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5;
    o.gi = true;
    o.giIntegrator = 0;
    o.giSamples = 128;
    o.giMaxDistance = 10.0f;
    o.giRecordSpacing = 0.5f;
    o.giAccuracy = 0.3f;
    umbreon::FrameResult openF = umbreon::render(sceneOf(floor(false)), o);
    umbreon::FrameResult occF = umbreon::render(sceneOf(floor(true)), o);
    const std::size_t c = kCenterRgba;  // center pixel, R channel
    // direct-diffuse floor (head-on light, mat.diffuse 0.8, intensity 0.5) = 0.4.
    s.check("GI single-count: occluded floor darker than open",
            occF.color[c + 0] + 0.05f < openF.color[c + 0]);
    s.check("GI single-count: occluded floor keeps its direct diffuse (~>=0.35)",
            occF.color[c + 0] > 0.35f);
    s.check("GI single-count: open floor brighter than direct-only (indirect adds)",
            openF.color[c + 0] > 0.45f);
  }

  // Color bleeding: a white floor next to a lit RED wall. Gather rays from the
  // floor that hit the wall pick up its red one-bounce radiance, so the floor's
  // composited indirect is redder than it is blue (the wall only feeds R). The
  // receiver is white, so any R>B at the floor comes from the neighbor's color.
  {
    using umbreon::Vec3;
    umbreon::Mesh m;
    // White floor on z=0 (normal +z).
    const Vec3 fl[6] = {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0},
                        {-2, -2, 0}, {2, 2, 0},  {-2, 2, 0}};
    for (int i = 0; i < 6; ++i) {
      m.positions.push_back(fl[i]);
      m.normals.push_back({0, 0, 1});
      m.colors.push_back({1, 1, 1, 1});  // white receiver
    }
    // Red wall at x=1 (normal -x, facing the floor center), z in [0,2].
    const Vec3 wl[6] = {{1, -2, 0}, {1, 2, 0}, {1, 2, 2},
                        {1, -2, 0}, {1, 2, 2}, {1, -2, 2}};
    for (int i = 0; i < 6; ++i) {
      m.positions.push_back(wl[i]);
      m.normals.push_back({-1, 0, 0});
      m.colors.push_back({1, 0, 0, 1});  // red source
    }
    m.material.ambient = 0.2f;
    m.material.diffuse = 0.8f;
    umbreon::Scene sc;
    sc.mesh = std::move(m);
    sc.camera = makeOrthoCam();
    umbreon::DistantLight l;  // travels +x and -z: lights both floor and wall
    l.direction = umbreon::Vec3{0.6f, 0.0f, -0.8f};
    l.color = {1, 1, 1};
    l.intensity = 0.6f;
    sc.lights.push_back(l);
    sc.ambientColor = {0.2f, 0.2f, 0.2f};  // modest env so the wall bounce shows
    sc.background = {0, 0, 0};
    umbreon::RenderOptions o;
    o.width = 5; o.height = 5;
    o.gi = true;
    o.giIntegrator = 0;
    o.giSamples = 256;
    o.giMaxDistance = 6.0f;
    o.giRecordSpacing = 0.4f;
    o.giAccuracy = 0.3f;
    umbreon::FrameResult f = umbreon::render(sc, o);
    const std::size_t c = kCenterRgba;
    s.check("GI bleed: floor indirect redder than blue near red wall",
            f.indirect[kCenterPix * 3 + 0] > f.indirect[kCenterPix * 3 + 2] + 0.01f);
    s.check("GI bleed: floor composited R > B (red bleed visible)",
            f.color[c + 0] > f.color[c + 2] + 0.01f);
  }

  // GI cavity depth (the deep-well番人): a white floor enclosed by tall walls
  // (deep well) lit top-down. With the auto-calibrated environment fill, an OPEN
  // floor keeps the gi-off baseline brightness (GI does not wash it brighter)
  // while the enclosed well floor is darkened by occlusion -- the depth cue. This
  // locks the env calibration (open ~= baseline) and the cavity darkening
  // magnitude together; a regression to a washing env or a phantom-lit bounce
  // would break one or the other.
  {
    using umbreon::Vec3;
    auto well = [](bool walls) {
      umbreon::Mesh m;
      auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n) {
        Vec3 v[6] = {a, b, c, a, c, d};
        for (int i = 0; i < 6; ++i) {
          m.positions.push_back(v[i]); m.normals.push_back(n);
          m.colors.push_back({1, 1, 1, 1});
        }
      };
      quad({-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{0,0,1});  // floor
      if (walls) {
        const float h = 3.0f;  // tall walls => deep well, floor center enclosed
        quad({1,-1,0},{1,1,0},{1,1,h},{1,-1,h},{-1,0,0});
        quad({-1,-1,0},{-1,1,h},{-1,1,0},{-1,-1,h},{1,0,0});
        quad({-1,1,0},{1,1,0},{1,1,h},{-1,1,h},{0,-1,0});
        quad({-1,-1,0},{-1,-1,h},{1,-1,h},{1,-1,0},{0,1,0});
      }
      m.material.ambient = 0.2f; m.material.diffuse = 0.8f;
      return m;
    };
    auto centerR = [&](bool walls, bool gi) {
      umbreon::Scene sc;
      sc.mesh = well(walls);
      sc.camera = makeOrthoCam();
      umbreon::DistantLight l;
      l.direction = {0, 0, -1}; l.color = {1, 1, 1}; l.intensity = 0.8f;
      sc.lights.push_back(l);
      sc.ambientColor = {1, 1, 1}; sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.gi = gi; o.giIntegrator = 0; o.giSamples = 256; o.giMaxDistance = 10.0f;
      o.giRecordSpacing = 0.3f; o.giAccuracy = 0.3f;
      return umbreon::render(sc, o).color[kCenterRgba + 0];
    };
    const float baseOpen = centerR(false, false);  // gi-off open floor
    const float giOpen = centerR(false, true);      // gi-on open floor
    const float giWell = centerR(true, true);       // gi-on enclosed well floor
    // The GI environment is a real occlusion-aware ambient light (env =
    // ambientColor * gi-env-intensity), so an open surface gathers the full
    // ambient (brighter than the flat gi-off material-ambient) while the enclosed
    // well floor, cut off from that ambient, darkens -- the depth cue.
    s.check("GI depth: env adds occlusion-aware ambient (open > gi-off flat)",
            giOpen > baseOpen);
    s.check("GI depth: enclosed well floor darkened vs open (>=15%)",
            giWell < giOpen * 0.85f);
    s.check("GI depth: well floor stays above black (fill present)",
            giWell > 0.3f);
  }

  // Multi-bounce (giBounces): an enclosed well lit top-down. The vertical walls
  // receive no direct light (n.L == 0), so with one bounce the well floor only
  // gets the (occluded) environment plus the walls' direct bounce (~0). A second
  // bounce lets the walls re-emit the light they gathered from the brightly lit
  // floor, so the well floor brightens. This locks the bounce energy as monotone
  // (>= one bounce) and the stage as deterministic (per-record seed, read-only
  // previous snapshot).
  //
  // Per-vertex seeding is used so every wall has cache records regardless of the
  // top-down camera's view coverage (edge-on walls leave almost no visible-
  // surface records). This isolates genuine inter-reflection: the multi-bounce
  // gather only adds a hit's indirect when a real record covers it, so without
  // wall records the second bounce would (correctly) add nothing.
  {
    using umbreon::Vec3;
    auto well = []() {
      umbreon::Mesh m;
      auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n) {
        Vec3 v[6] = {a, b, c, a, c, d};
        for (int i = 0; i < 6; ++i) {
          m.positions.push_back(v[i]); m.normals.push_back(n);
          m.colors.push_back({1, 1, 1, 1});
        }
      };
      quad({-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{0,0,1});  // floor
      const float h = 3.0f;  // tall walls => deep well
      quad({1,-1,0},{1,1,0},{1,1,h},{1,-1,h},{-1,0,0});
      quad({-1,-1,0},{-1,1,h},{-1,1,0},{-1,-1,h},{1,0,0});
      quad({-1,1,0},{1,1,0},{1,1,h},{-1,1,h},{0,-1,0});
      quad({-1,-1,0},{-1,-1,h},{1,-1,h},{1,-1,0},{0,1,0});
      m.material.ambient = 0.2f; m.material.diffuse = 0.8f;
      return m;
    };
    auto render = [&](int bounces) {
      umbreon::Scene sc;
      sc.mesh = well();
      sc.camera = makeOrthoCam();
      umbreon::DistantLight l;
      l.direction = {0, 0, -1}; l.color = {1, 1, 1}; l.intensity = 0.8f;
      sc.lights.push_back(l);
      sc.ambientColor = {1, 1, 1}; sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 5; o.height = 5;
      o.gi = true; o.giIntegrator = 0; o.giSamples = 256; o.giMaxDistance = 10.0f;
      o.giRecordSpacing = 0.3f; o.giAccuracy = 0.3f;
      o.giBounces = bounces;
      o.giSeedPerVertex = true;  // records on all walls (view-independent)
      return umbreon::render(sc, o);
    };
    const float well1 = render(1).color[kCenterRgba + 0];
    const float well2 = render(2).color[kCenterRgba + 0];
    s.check("GI multi-bounce: second bounce brightens the enclosed well",
            well2 > well1 + 1.0e-4f);
    // Determinism: two giBounces=2 renders must be bit-identical (each stage is
    // per-record seeded and reads only the read-only previous snapshot).
    umbreon::FrameResult a = render(2);
    umbreon::FrameResult b = render(2);
    bool identical = a.color.size() == b.color.size();
    for (std::size_t i = 0; identical && i < a.color.size(); ++i)
      if (a.color[i] != b.color[i]) identical = false;
    s.check("GI multi-bounce: two giBounces=2 renders bit-identical", identical);
  }

  // Irradiance gradients (giGradients): an enclosed well has strong spatial
  // irradiance variation (bright floor, dark walls). With gradients off the
  // floor reads piecewise-constant per record (Voronoi blocks); the Ward-
  // Heckbert rotational/translational gradients let irradiance vary smoothly
  // between records, so the interpolated result differs from plain blending.
  // This locks two properties: (1) gradients are wired in and non-zero (the
  // result changes vs gradients-off), and (2) the gradient stage stays
  // deterministic (per-record seed => two renders bit-identical).
  {
    using umbreon::Vec3;
    auto well = []() {
      umbreon::Mesh m;
      auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n) {
        Vec3 v[6] = {a, b, c, a, c, d};
        for (int i = 0; i < 6; ++i) {
          m.positions.push_back(v[i]); m.normals.push_back(n);
          m.colors.push_back({1, 1, 1, 1});
        }
      };
      quad({-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{0,0,1});  // floor
      const float h = 3.0f;
      quad({1,-1,0},{1,1,0},{1,1,h},{1,-1,h},{-1,0,0});
      quad({-1,-1,0},{-1,1,h},{-1,1,0},{-1,-1,h},{1,0,0});
      quad({-1,1,0},{1,1,0},{1,1,h},{-1,1,h},{0,-1,0});
      quad({-1,-1,0},{-1,-1,h},{1,-1,h},{1,-1,0},{0,1,0});
      m.material.ambient = 0.2f; m.material.diffuse = 0.8f;
      return m;
    };
    auto render = [&](bool gradients) {
      umbreon::Scene sc;
      sc.mesh = well();
      sc.camera = makeOrthoCam();
      umbreon::DistantLight l;
      l.direction = {0, 0, -1}; l.color = {1, 1, 1}; l.intensity = 0.8f;
      sc.lights.push_back(l);
      sc.ambientColor = {1, 1, 1}; sc.background = {0, 0, 0};
      umbreon::RenderOptions o;
      o.width = 7; o.height = 7;
      o.gi = true; o.giIntegrator = 0; o.giSamples = 256; o.giMaxDistance = 10.0f;
      o.giRecordSpacing = 0.3f; o.giAccuracy = 3.0f;  // overlap for gradients
      o.giSeedPerVertex = true;
      o.giGradients = gradients;
      return umbreon::render(sc, o);
    };
    umbreon::FrameResult off = render(false);
    umbreon::FrameResult on = render(true);
    bool differs = off.color.size() == on.color.size();
    bool anyDiff = false;
    for (std::size_t i = 0; differs && i < off.color.size(); ++i)
      if (off.color[i] != on.color[i]) anyDiff = true;
    s.check("GI gradients: result differs from plain interpolation",
            differs && anyDiff);
    // Determinism: two gradient renders must be bit-identical.
    umbreon::FrameResult a = render(true);
    umbreon::FrameResult b = render(true);
    bool identical = a.color.size() == b.color.size();
    for (std::size_t i = 0; identical && i < a.color.size(); ++i)
      if (a.color[i] != b.color[i]) identical = false;
    s.check("GI gradients: two gradient renders bit-identical", identical);
  }

  // Dark-outlier rejection (rejectDarkOutliers). A near-fully-occluded record
  // gathers ~0 in one-bounce; on bumpy geometry an isolated such record (a seed
  // in a sub-spacing micro-pocket) paints a black voxel square while its bright
  // rim neighbors do not. The pass must lift an ISOLATED dark record to its
  // bright neighbors, yet leave a CLUSTERED dark record (a genuine cavity, whose
  // neighbors are also dark) untouched. Tested directly on a hand-built cache so
  // the spec is locked independently of the (scene-dependent) gather.
  {
    using umbreon::Vec3;
    using umbreon::detail::IrradianceCache;
    using umbreon::detail::IrradianceCacheParams;
    using umbreon::detail::IrradianceRecord;
    auto lum = [](const Vec3& e) {
      return 0.2126f * e.x + 0.7152f * e.y + 0.0722f * e.z;
    };
    // 5x5 coplanar grid (z=0, normal +z), unit spacing. `centerDark` flags only
    // the middle record as dark+occluded; otherwise all are bright (isolated
    // case). `allDark` makes every record dark+occluded (clustered cavity case).
    auto build = [&](bool allDark) {
      IrradianceCache c;
      c.setCellSize(1.0f);
      for (int y = 0; y < 5; ++y)
        for (int x = 0; x < 5; ++x) {
          IrradianceRecord r;
          r.position = Vec3{static_cast<float>(x), static_cast<float>(y), 0.0f};
          r.normal = Vec3{0.0f, 0.0f, 1.0f};
          const bool center = (x == 2 && y == 2);
          const bool dark = allDark || center;
          r.irradiance = dark ? Vec3{0.0f, 0.0f, 0.0f} : Vec3{1.0f, 1.0f, 1.0f};
          r.occlusion = dark ? 1.0f : 0.3f;
          r.componentID = 0;
          c.records.push_back(r);
        }
      return c;
    };
    IrradianceCacheParams pp;
    pp.spacing = 1.0f;
    pp.normalReject = 0.85f;
    const std::size_t mid = 2 * 5 + 2;  // center index

    IrradianceCache iso = build(false);
    umbreon::detail::rejectDarkOutliers(iso, pp);
    s.check("GI outlier reject: isolated dark record lifted to bright neighbors",
            lum(iso.records[mid].irradiance) > 0.5f);

    IrradianceCache cav = build(true);
    umbreon::detail::rejectDarkOutliers(cav, pp);
    s.check("GI outlier reject: clustered dark cavity stays dark",
            lum(cav.records[mid].irradiance) < 0.05f);
  }

  // Edge-aware a-trous denoise (denoiseAtrous). A frame split into a dark-left /
  // bright-right surface, each flat half carrying deterministic per-pixel noise,
  // with a NORMAL discontinuity on the seam. The pass must (1) reduce the noise
  // variance inside a flat half, (2) keep the left/right luminance step (the
  // normal edge-stop rejects cross-seam taps), and (3) leave background pixels
  // (no normal) byte-identical. A top strip is background. Position guide is
  // omitted so only the normal/luminance edge-stops drive the test.
  {
    const int W = 24, H = 24;
    const std::size_t N = static_cast<std::size_t>(W) * H;
    umbreon::FrameResult f;
    f.width = W;
    f.height = H;
    f.color.assign(N * 4, 0.0f);
    f.normal.assign(N * 3, 0.0f);
    const int bgRows = 4;            // top strip: background (zero normal)
    const float baseL = 0.3f, baseR = 0.8f;
    const float amp = 0.08f;        // deterministic noise amplitude
    auto noiseAt = [](std::size_t i) {
      // Deterministic LCG-style hash in [-0.5, 0.5]; no <random>, reproducible.
      std::uint32_t h = static_cast<std::uint32_t>(i) * 1103515245u + 12345u;
      return (static_cast<float>((h >> 16) & 0x7fffu) / 32768.0f) - 0.5f;
    };
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        const std::size_t p = static_cast<std::size_t>(y) * W + x;
        if (y < bgRows) {
          // Background: zero normal, fixed sentinel color (must stay untouched).
          const float bg = 0.123f;
          f.color[p * 4 + 0] = f.color[p * 4 + 1] = f.color[p * 4 + 2] = bg;
          f.color[p * 4 + 3] = 1.0f;
          continue;
        }
        const bool left = x < W / 2;
        f.normal[p * 3 + 0] = left ? 0.0f : 1.0f;  // normal discontinuity at seam
        f.normal[p * 3 + 2] = left ? 1.0f : 0.0f;
        const float base = left ? baseL : baseR;
        const float v = base + amp * noiseAt(p);
        f.color[p * 4 + 0] = f.color[p * 4 + 1] = f.color[p * 4 + 2] = v;
        f.color[p * 4 + 3] = 1.0f;
      }
    const std::vector<float> before = f.color;

    auto lumStats = [&](const std::vector<float>& c, int x0, int x1) {
      // mean/variance of luminance over a flat interior window, rows [bgRows+2,H-2).
      double s1 = 0.0, s2 = 0.0;
      int n = 0;
      for (int y = bgRows + 2; y < H - 2; ++y)
        for (int x = x0; x < x1; ++x) {
          const std::size_t p = static_cast<std::size_t>(y) * W + x;
          const float l = 0.2126f * c[p * 4 + 0] + 0.7152f * c[p * 4 + 1] +
                          0.0722f * c[p * 4 + 2];
          s1 += l;
          s2 += l * l;
          ++n;
        }
      const double mean = s1 / n;
      return std::pair<double, double>{mean, s2 / n - mean * mean};
    };
    const auto lBefore = lumStats(before, 2, 9);   // left flat interior
    const auto rBefore = lumStats(before, 15, 22);  // right flat interior

    umbreon::RenderOptions o;
    o.denoiseIters = 5;
    o.denoiseDemodulateAlbedo = false;  // no albedo buffer; denoise raw color
    umbreon::denoiseAtrous(f, o);

    const auto lAfter = lumStats(f.color, 2, 9);
    const auto rAfter = lumStats(f.color, 15, 22);

    // (1) Noise variance falls substantially inside a flat half.
    s.check("denoise atrous: left-half variance reduced",
            lAfter.second < 0.4 * lBefore.second && rAfter.second < 0.4 * rBefore.second);
    // (2) The left/right luminance step survives (normal edge-stop, no smearing).
    const double gapBefore = rBefore.first - lBefore.first;
    const double gapAfter = rAfter.first - lAfter.first;
    s.check("denoise atrous: left/right luminance edge preserved",
            gapAfter > 0.9 * gapBefore);
    // (3) Background (zero normal) pixels are byte-identical.
    bool bgSame = true;
    for (int x = 0; x < W; ++x) {
      const std::size_t p = static_cast<std::size_t>(0) * W + x;
      for (int c = 0; c < 3; ++c)
        if (f.color[p * 4 + c] != before[p * 4 + c]) bgSame = false;
    }
    s.check("denoise atrous: background pixels untouched", bgSame);

    // (4) iters=0 is a strict no-op (byte-identical), guarding the early-out.
    umbreon::FrameResult f0 = f;
    const std::vector<float> pre0 = f0.color;
    umbreon::RenderOptions z = o;
    z.denoiseIters = 0;
    umbreon::denoiseAtrous(f0, z);
    bool noop = f0.color == pre0;
    s.check("denoise atrous: iters=0 is a byte-identical no-op", noop);
  }

  return s.report();
}
