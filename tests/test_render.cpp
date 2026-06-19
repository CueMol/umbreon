// Integration tests for the umbreon render pipeline: primary-ray direct shading
// (mesh local illumination + flat outline primitives), the background/miss path,
// and the render() facade steps (supersample downsample, assumed_gamma). These
// lock the current no-AO / no-shadow baseline so later AO / soft-shadow work can
// detect any unintended change to the existing shading output.
#include <cmath>
#include <cstddef>

#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

// A flat quad in the z=0 plane spanning [-2,2]^2, facing +Z (toward the camera),
// with a single pigment color. Two de-indexed triangles, material 0.2/0.8.
umbreon::Mesh makeQuad(umbreon::Vec4 color) {
  using umbreon::Vec3;
  umbreon::Mesh m;
  const Vec3 p00{-2, -2, 0}, p10{2, -2, 0}, p11{2, 2, 0}, p01{-2, 2, 0};
  const Vec3 corners[6] = {p00, p10, p11, p00, p11, p01};
  const Vec3 n{0, 0, 1};
  for (int i = 0; i < 6; ++i) {
    m.positions.push_back(corners[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;
  return m;
}

// Orthographic camera at +Z looking down -Z. height=4 frames y in [-2,2]; with a
// square image that frames x in [-2,2] as well, so the quad fills the view.
umbreon::Camera makeOrthoCam() {
  umbreon::Camera c;
  c.position = {0, 0, 10};
  c.direction = {0, 0, -1};
  c.up = {0, 1, 0};
  c.orthographic = true;
  c.height = 4.0f;
  return c;
}

// A single head-on distant light traveling -Z (lighting the +Z faces) at half
// intensity. With material 0.2/0.8 and a head-on normal (N.L = 1) this gives
// lit = 0.2 + 0.8 * 1.0 * 0.5 = 0.6, so out = pigment * 0.6.
umbreon::DistantLight makeKeyLight() {
  umbreon::DistantLight l;
  l.direction = {0, 0, -1};
  l.color = {1, 1, 1};
  l.intensity = 0.5f;
  return l;
}

// Build a scene with one shaded material sphere at the origin (radius 1), framed
// head-on by the ortho camera so the center pixel hits the apex with N=(0,0,1),
// V=(0,0,1) and the key light L=(0,0,1). Every cos term is then 1, so shadeLocal
// reduces to the closed-form values the material checks below assert against.
umbreon::Scene makeMaterialSphereScene(umbreon::Vec4 color,
                                       const umbreon::Material& mat,
                                       umbreon::Vec3 bg) {
  umbreon::Scene sc;
  sc.camera = makeOrthoCam();
  sc.lights.push_back(makeKeyLight());
  sc.background = bg;
  umbreon::Sphere sp;
  sp.center = {0, 0, 0};
  sp.radius = 1.0f;
  sp.color = color;
  sp.material = mat;
  sc.spheres.push_back(sp);
  return sc;
}

// Index of the center pixel of a 5x5 image, in float-RGBA element units.
constexpr std::size_t kCenterRgba = (2 * 5 + 2) * 4;
constexpr std::size_t kCenterPix = 2 * 5 + 2;

}  // namespace

int main() {
  umbreon::test::Suite s("render");
  const umbreon::Vec4 pigment{0.5f, 0.6f, 0.7f, 1.0f};

  // --- mesh direct shading: out = C * (Ka*ambLight + Kd * max(N.L,0) * Lc) ---
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0.4f, 0.1f, 0.2f};

    umbreon::RenderOptions o;
    o.width = 5;
    o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("mesh: center R = C.r*0.6", approx(f.color[kCenterRgba + 0], 0.30f, 1e-4f));
    s.check("mesh: center G = C.g*0.6", approx(f.color[kCenterRgba + 1], 0.36f, 1e-4f));
    s.check("mesh: center B = C.b*0.6", approx(f.color[kCenterRgba + 2], 0.42f, 1e-4f));
    s.check("mesh: alpha = 1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    s.check("mesh: center depth = ortho cam distance (10)",
            approx(f.depth[kCenterPix], 10.0f, 1e-3f));
  }

  // --- background / miss path: empty scene -> every ray misses -> bg, depth 0 ---
  {
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0.4f, 0.1f, 0.2f};

    umbreon::RenderOptions o;
    o.width = 5;
    o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("background: R", approx(f.color[kCenterRgba + 0], 0.4f, 1e-6f));
    s.check("background: G", approx(f.color[kCenterRgba + 1], 0.1f, 1e-6f));
    s.check("background: B", approx(f.color[kCenterRgba + 2], 0.2f, 1e-6f));
    s.check("background: depth 0 on miss", approx(f.depth[kCenterPix], 0.0f, 1e-6f));
  }

  // --- flat outline primitive: a sphere is rendered with its raw color and no
  // lighting (ambient 1, diffuse 0), so outlines stay exactly their color
  // (black outlines stay black) regardless of the light present in the scene. ---
  {
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());  // present, but must not affect the sphere
    sc.background = {0.4f, 0.1f, 0.2f};
    umbreon::Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.0f;
    sp.color = {0.3f, 0.4f, 0.5f, 1.0f};
    sc.spheres.push_back(sp);

    umbreon::RenderOptions o;
    o.width = 5;
    o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("outline: sphere keeps raw R (no shading)", approx(f.color[kCenterRgba + 0], 0.3f, 1e-4f));
    s.check("outline: sphere keeps raw G (no shading)", approx(f.color[kCenterRgba + 1], 0.4f, 1e-4f));
    s.check("outline: sphere keeps raw B (no shading)", approx(f.color[kCenterRgba + 2], 0.5f, 1e-4f));
  }

  // --- facade: assumed_gamma raises the linear output to the power g after
  // shading. lit pigment 0.30/0.36/0.42 at g=2 -> squared; alpha untouched. ---
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());

    sc.assumedGamma = 2.0f;
    umbreon::RenderOptions o;
    o.width = 5;
    o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("gamma: R squared", approx(f.color[kCenterRgba + 0], 0.30f * 0.30f, 1e-4f));
    s.check("gamma: G squared", approx(f.color[kCenterRgba + 1], 0.36f * 0.36f, 1e-4f));
    s.check("gamma: alpha unchanged by gamma", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
  }

  // --- facade: supersample renders at width*ss then box-averages back down. Over
  // a uniformly shaded region the value is preserved, and the final framebuffer
  // is the requested (non-supersampled) size. ---
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad(pigment);
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());

    umbreon::RenderOptions o;
    o.width = 5;
    o.height = 5;
    o.supersample = 2;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check_eq("supersample: final width", f.width, 5);
    s.check_eq("supersample: final height", f.height, 5);
    s.check("supersample: uniform region value preserved",
            approx(f.color[kCenterRgba + 0], 0.30f, 1e-4f));
  }

  // ===== Per-material sphere shading (shadeLocal model) =====
  // Head-on geometry: N=V=L=(0,0,1), so every cos term is 1. The key light is
  // white at intensity 0.5 (Lc=0.5); ambientColor defaults to (1,1,1).

  // Matte (ambient 0, diffuse 0.8, brilliance 0): out = diffuse*Lc*C = 0.4*C.
  {
    umbreon::Material m;
    m.ambient = 0.0f; m.diffuse = 0.8f; m.brilliance = 0.0f;
    m.specular = 0.0f; m.phong = 0.0f; m.metallic = false; m.reflection = 0.0f;
    umbreon::Scene sc =
        makeMaterialSphereScene({0.6f, 0.4f, 0.2f, 1.0f}, m, {0, 0, 0});
    umbreon::RenderOptions o; o.width = 5; o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("matte sphere: R = 0.4*C.r", approx(f.color[kCenterRgba + 0], 0.24f, 1e-4f));
    s.check("matte sphere: G = 0.4*C.g", approx(f.color[kCenterRgba + 1], 0.16f, 1e-4f));
    s.check("matte sphere: B = 0.4*C.b", approx(f.color[kCenterRgba + 2], 0.08f, 1e-4f));
  }

  // Metallic F_MetalA (ambient 0.35, diffuse 0.3 brilliance 2, specular 0.80
  // roughness 0.05, metallic). Head-on, bg=0 (no reflection): ambient 0.35*C +
  // diffuse 0.3*0.5*C + metallic Blinn 0.80*(C*0.5) = (0.35+0.15+0.40)*C = 0.90*C.
  {
    umbreon::Material m;
    m.ambient = 0.35f; m.diffuse = 0.3f; m.brilliance = 2.0f;
    m.specular = 0.80f; m.roughness = 1.0f / 20.0f; m.metallic = true;
    m.phong = 0.0f; m.reflection = 0.10f;
    umbreon::Scene sc =
        makeMaterialSphereScene({0.6f, 0.4f, 0.2f, 1.0f}, m, {0, 0, 0});
    umbreon::RenderOptions o; o.width = 5; o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("metallic sphere: R = 0.90*C.r", approx(f.color[kCenterRgba + 0], 0.54f, 1e-3f));
    s.check("metallic sphere: G = 0.90*C.g", approx(f.color[kCenterRgba + 1], 0.36f, 1e-3f));
    s.check("metallic sphere: B = 0.90*C.b", approx(f.color[kCenterRgba + 2], 0.18f, 1e-3f));
  }

  // Reflection term: the same metallic material on a WHITE background adds
  // reflection*bg = 0.10*(1,1,1) on top of the 0.90*C body.
  {
    umbreon::Material m;
    m.ambient = 0.35f; m.diffuse = 0.3f; m.brilliance = 2.0f;
    m.specular = 0.80f; m.roughness = 1.0f / 20.0f; m.metallic = true;
    m.phong = 0.0f; m.reflection = 0.10f;
    umbreon::Scene sc =
        makeMaterialSphereScene({0.6f, 0.4f, 0.2f, 1.0f}, m, {1, 1, 1});
    umbreon::RenderOptions o; o.width = 5; o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("reflection: R = 0.90*C.r + 0.10*bg", approx(f.color[kCenterRgba + 0], 0.64f, 1e-3f));
    s.check("reflection: G = 0.90*C.g + 0.10*bg", approx(f.color[kCenterRgba + 1], 0.46f, 1e-3f));
  }

  // Phong highlight (ambient 0.3, diffuse 0.5 brilliance 0, phong 10000
  // phong_size 50, non-metallic). For C=(0.2,0.4,0.6) the body without the
  // highlight is 0.55*C = (0.11,0.22,0.33); phong adds an ACHROMATIC lift (the
  // same scalar to each channel, since not metallic). Assert the lift is present
  // and equal across channels, without pinning the tuned phong clamp constant.
  {
    umbreon::Material m;
    m.ambient = 0.3f; m.diffuse = 0.5f; m.brilliance = 0.0f;
    m.specular = 0.0f; m.phong = 10000.0f; m.phongSize = 50.0f;
    m.metallic = false; m.reflection = 0.0f;
    umbreon::Scene sc =
        makeMaterialSphereScene({0.2f, 0.4f, 0.6f, 1.0f}, m, {0, 0, 0});
    umbreon::RenderOptions o; o.width = 5; o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    const float liftR = f.color[kCenterRgba + 0] - 0.11f;
    const float liftG = f.color[kCenterRgba + 1] - 0.22f;
    const float liftB = f.color[kCenterRgba + 2] - 0.33f;
    s.check("phong: highlight present", liftR > 0.05f);
    s.check("phong: achromatic lift (R==G)", approx(liftR, liftG, 1e-4f));
    s.check("phong: achromatic lift (G==B)", approx(liftG, liftB, 1e-4f));
  }

  // Fill (shadowless) light: POV computes diffuse only for a FILL_LIGHT_SOURCE,
  // no specular/phong highlight. Pure-highlight material (ambient 0, diffuse 0,
  // specular 0.5) head-on: a normal light yields the highlight
  // specular*Lc = 0.5*0.5 = 0.25 (achromatic); a fill light yields nothing.
  {
    umbreon::Material m;
    m.ambient = 0.0f; m.diffuse = 0.0f; m.brilliance = 1.0f;
    m.specular = 0.5f; m.roughness = 0.05f; m.phong = 0.0f;
    m.metallic = false; m.reflection = 0.0f;

    umbreon::Scene lit =
        makeMaterialSphereScene({0.6f, 0.4f, 0.2f, 1.0f}, m, {0, 0, 0});
    umbreon::RenderOptions o; o.width = 5; o.height = 5;
    umbreon::FrameResult fl = umbreon::render(lit, o);
    s.check("highlight light: specular present (R=0.25)",
            approx(fl.color[kCenterRgba + 0], 0.25f, 1e-3f));

    umbreon::Scene fill =
        makeMaterialSphereScene({0.6f, 0.4f, 0.2f, 1.0f}, m, {0, 0, 0});
    fill.lights[0].castsHighlight = false;
    umbreon::FrameResult ff = umbreon::render(fill, o);
    s.check("fill light: no specular highlight (R=0)",
            approx(ff.color[kCenterRgba + 0], 0.0f, 1e-4f));
    s.check("fill light: no specular highlight (G=0)",
            approx(ff.color[kCenterRgba + 1], 0.0f, 1e-4f));
  }

  return s.report();
}
