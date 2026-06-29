// Integration tests for the umbreon render pipeline: primary-ray direct shading
// (mesh local illumination + flat outline primitives), the background/miss path,
// and the render() facade steps (supersample downsample, assumed_gamma). These
// lock the current no-AO / no-shadow baseline so later AO / soft-shadow work can
// detect any unintended change to the existing shading output.
#include <cmath>
#include <cstddef>
#include <cstdint>

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
  // same scalar to each channel, since not metallic). POV-faithful (no clamp):
  // head-on R.V=1, so the lift is phong*pow(1,phong_size)*Lc = 10000*0.5 = 5000.
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
    s.check("phong: unclamped POV lift = phong*Lc (=5000)", approx(liftR, 5000.0f, 1.0f));
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

  // Metallic Fresnel de-tint at a grazing angle (POV ComputeSpecularColour):
  // the highlight lerps pigment->light by f(N.L), so a pure-red metallic shows a
  // non-zero green/blue highlight (de-tinted toward white), not pure red.
  // Flat quad N=V=(0,0,1); light L=(sin60,0,cos60) => N.L=0.5, N.H=cos30.
  // specular 1.0 roughness 0.5 (exp 2): specW = 1.0*pow(cos30,2) = 0.75.
  // f(0.5)=0.05927; hl=l*(f+(1-f)*C); out = 0.75*hl = (0.75, 0.04445, 0.04445).
  {
    umbreon::Scene sc;
    sc.mesh = makeQuad({1.0f, 0.0f, 0.0f, 1.0f});  // pure red pigment
    sc.mesh.material.ambient = 0.0f; sc.mesh.material.diffuse = 0.0f;
    sc.mesh.material.specular = 1.0f; sc.mesh.material.roughness = 0.5f;
    sc.mesh.material.brilliance = 1.0f; sc.mesh.material.metallic = true;
    sc.mesh.material.phong = 0.0f; sc.mesh.material.reflection = 0.0f;
    sc.camera = makeOrthoCam();
    umbreon::DistantLight l;
    l.direction = {-0.8660254f, 0.0f, -0.5f};  // travels opposite L=(sin60,0,cos60)
    l.color = {1, 1, 1}; l.intensity = 1.0f;
    sc.lights.push_back(l);
    sc.background = {0, 0, 0};
    umbreon::RenderOptions o; o.width = 5; o.height = 5;
    umbreon::FrameResult f = umbreon::render(sc, o);
    s.check("metallic fresnel: R highlight (0.75)",
            approx(f.color[kCenterRgba + 0], 0.75f, 2e-3f));
    s.check("metallic fresnel: G de-tinted toward white (0.0445)",
            approx(f.color[kCenterRgba + 1], 0.04445f, 2e-3f));
    s.check("metallic fresnel: achromatic de-tint (G==B)",
            approx(f.color[kCenterRgba + 1], f.color[kCenterRgba + 2], 1e-5f));
  }

  // ===== Single-layer transparency (additive, order-independent) =====
  // Flat material (ambient 1, diffuse 0, ambientColor 1) => shaded color == raw
  // pigment, so the composited center pixel equals an exact analytic value. Each
  // quad spans [-2,2]^2 facing the ortho camera (which frames [-2,2]); the
  // center ray pierces them all. Opacity is carried in the color's w channel,
  // the transparency group in triGroupId. assumed_gamma defaults to 1 so the
  // linear FrameResult.color is compared directly. This locks the model:
  //   out = sum_g beta_g*C_g + (1 - sum beta_g)*A   (A = nearest opaque floor)
  // composited front-to-back but ORDER-INDEPENDENT, one layer PER group.
  {
    using umbreon::Vec3;
    using umbreon::Vec4;
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
    // Two builders so the veil-group set is explicit. The group-alpha (additive
    // single-layer) tests T1-T6 take the default veils {1,2} via sceneOf(); the
    // fragment (over) tests F1-F4 pass {} via sceneOfVeil() so their
    // transparency uses front-to-back "over". The default lives in the sceneOf()
    // wrapper rather than as a lambda default argument, which tripped an MSVC
    // front-end internal compiler error.
    auto sceneOfVeil = [&](umbreon::Mesh mesh, Vec3 bg,
                           std::vector<std::uint16_t> veil) {
      umbreon::Scene sc;
      sc.mesh = std::move(mesh);
      sc.mesh.material = umbreon::Material::flatOutline();  // raw color shading
      sc.camera = makeOrthoCam();
      sc.background = bg;
      sc.ambientColor = {1, 1, 1};
      sc.veilGroups = std::move(veil);
      return sc;
    };
    auto sceneOf = [&](umbreon::Mesh mesh, Vec3 bg) {
      return sceneOfVeil(std::move(mesh), bg, std::vector<std::uint16_t>{1, 2});
    };

    // T1: blue(0.6, group 1, front) over opaque red(group 0) => 0.6*blue+0.4*red.
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 0.6f}, 1.0f, 1);  // transparent blue (front)
      umbreon::Scene sc = sceneOf(std::move(m), {0, 0, 0});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T1 over-opaque R=0.4", approx(f.color[kCenterRgba + 0], 0.4f, 1e-4f));
      s.check("T1 over-opaque G=0", approx(f.color[kCenterRgba + 1], 0.0f, 1e-4f));
      s.check("T1 over-opaque B=0.6", approx(f.color[kCenterRgba + 2], 0.6f, 1e-4f));
      s.check("T1 alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // T2: blue(0.6) over an opaque background (0.2) => 0.6*blue + 0.4*bg.
    {
      umbreon::Mesh m;
      addQuad(m, {0, 0, 1, 0.6f}, 1.0f, 1);
      umbreon::Scene sc = sceneOf(std::move(m), {0.2f, 0.2f, 0.2f});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T2 over-bg R=0.08", approx(f.color[kCenterRgba + 0], 0.08f, 1e-4f));
      s.check("T2 over-bg B=0.68", approx(f.color[kCenterRgba + 2], 0.68f, 1e-4f));
      s.check("T2 alpha=1 (opaque bg)", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // T3: same scene with a transparent background => premultiplied transparent
    // output (bg NOT mixed into the color), alpha = the coverage 0.6.
    {
      umbreon::Mesh m;
      addQuad(m, {0, 0, 1, 0.6f}, 1.0f, 1);
      umbreon::Scene sc = sceneOf(std::move(m), {0.2f, 0.2f, 0.2f});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparentBackground = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T3 transp-bg R=0 (premult)", approx(f.color[kCenterRgba + 0], 0.0f, 1e-4f));
      s.check("T3 transp-bg B=0.6 (premult)", approx(f.color[kCenterRgba + 2], 0.6f, 1e-4f));
      s.check("T3 transp-bg alpha=0.6", approx(f.color[kCenterRgba + 3], 0.6f, 1e-4f));
    }

    // T4: double-wall avoidance. Two SAME-group(1) blue(0.6) quads (z=1, z=0.5)
    // over opaque red. Only the frontmost of group 1 composites (single layer),
    // so the back wall does not double up: result == T1 (0.6*blue + 0.4*red).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red
      addQuad(m, {0, 0, 1, 0.6f}, 0.5f, 1);  // back wall  (group 1)
      addQuad(m, {0, 0, 1, 0.6f}, 1.0f, 1);  // front wall (group 1)
      umbreon::Scene sc = sceneOf(std::move(m), {0, 0, 0});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T4 double-wall R=0.4 (single layer)", approx(f.color[kCenterRgba + 0], 0.4f, 1e-4f));
      s.check("T4 double-wall B=0.6 (single layer)", approx(f.color[kCenterRgba + 2], 0.6f, 1e-4f));
    }

    // T5: multi-group additive. green(0.5,g1) + blue(0.3,g2) + opaque red(g0)
    // => 0.5*green + 0.3*blue + 0.2*red = (0.2, 0.5, 0.3).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 0.3f}, 0.5f, 2);  // blue (mid)
      addQuad(m, {0, 1, 0, 0.5f}, 1.0f, 1);  // green (front)
      umbreon::Scene sc = sceneOf(std::move(m), {0, 0, 0});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T5 multigroup R=0.2", approx(f.color[kCenterRgba + 0], 0.2f, 1e-4f));
      s.check("T5 multigroup G=0.5", approx(f.color[kCenterRgba + 1], 0.5f, 1e-4f));
      s.check("T5 multigroup B=0.3", approx(f.color[kCenterRgba + 2], 0.3f, 1e-4f));
    }

    // T5b: order-independence. Swap the depths of the two transparent layers;
    // the additive result is identical (no z-order between groups, as blendpng).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 1, 0, 0.5f}, 0.5f, 1);  // green now mid
      addQuad(m, {0, 0, 1, 0.3f}, 1.0f, 2);  // blue now front
      umbreon::Scene sc = sceneOf(std::move(m), {0, 0, 0});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T5b order-indep R=0.2", approx(f.color[kCenterRgba + 0], 0.2f, 1e-4f));
      s.check("T5b order-indep G=0.5", approx(f.color[kCenterRgba + 1], 0.5f, 1e-4f));
      s.check("T5b order-indep B=0.3", approx(f.color[kCenterRgba + 2], 0.3f, 1e-4f));
    }

    // T6: opacity 1.0 inside a transparent group is treated as opaque (a floor):
    // a blue(1.0) quad fully hides the red behind it, alpha 1 (no leakage).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 1.0f}, 1.0f, 1);  // opaque blue front (group 1)
      umbreon::Scene sc = sceneOf(std::move(m), {0, 0, 0});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("T6 opacity1 hides back B=1", approx(f.color[kCenterRgba + 2], 1.0f, 1e-4f));
      s.check("T6 opacity1 hides back R=0", approx(f.color[kCenterRgba + 0], 0.0f, 1e-4f));
      s.check("T6 alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // ===== Fragment alpha (intrinsic per-color opacity): front-to-back "over",
    // EVERY surface composited (no dedup), order-DEPENDENT -- POV native
    // transmit. Selected when the group is NOT a veil (veilGroups empty). =====

    // F1: single fragment over opaque == a*C + (1-a)*A (matches T1 numerically).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 0.6f}, 1.0f, 1);  // fragment blue 0.6
      umbreon::Scene sc = sceneOfVeil(std::move(m), {0, 0, 0}, {});  // no veils
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F1 over R=0.4", approx(f.color[kCenterRgba + 0], 0.4f, 1e-4f));
      s.check("F1 over B=0.6", approx(f.color[kCenterRgba + 2], 0.6f, 1e-4f));
      s.check("F1 alpha=1", approx(f.color[kCenterRgba + 3], 1.0f, 1e-6f));
    }

    // F2: ORDER DEPENDENCE (unlike additive veils, cf. T5b). green(0.5) front +
    // blue(0.5) mid over opaque red => 0.5*green + 0.25*blue + 0.25*red.
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red (back)
      addQuad(m, {0, 0, 1, 0.5f}, 0.5f, 2);  // blue (mid)
      addQuad(m, {0, 1, 0, 0.5f}, 1.0f, 1);  // green (front)
      umbreon::Scene sc = sceneOfVeil(std::move(m), {0, 0, 0}, {});
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
      umbreon::Scene sc = sceneOfVeil(std::move(m), {0, 0, 0}, {});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F2 blue-front B=0.5 (order-dependent)", approx(f.color[kCenterRgba + 2], 0.5f, 1e-4f));
      s.check("F2 blue-front G=0.25 (order-dependent)", approx(f.color[kCenterRgba + 1], 0.25f, 1e-4f));
    }

    // F3: NO dedup -- both walls composite. Two same-group(1) blue(0.5) quads
    // over opaque red => 0.75*blue + 0.25*red (a veil would give 0.5/0.5).
    {
      umbreon::Mesh m;
      addQuad(m, {1, 0, 0, 1.0f}, 0.0f, 0);  // opaque red
      addQuad(m, {0, 0, 1, 0.5f}, 0.5f, 1);  // back wall
      addQuad(m, {0, 0, 1, 0.5f}, 1.0f, 1);  // front wall (same group)
      umbreon::Scene sc = sceneOfVeil(std::move(m), {0, 0, 0}, {});
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
      umbreon::Scene sc = sceneOfVeil(std::move(m), {0.2f, 0.2f, 0.2f}, {});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparentBackground = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("F4 transp-bg B=0.6 (premult)", approx(f.color[kCenterRgba + 2], 0.6f, 1e-4f));
      s.check("F4 transp-bg R=0 (no bg tint)", approx(f.color[kCenterRgba + 0], 0.0f, 1e-4f));
      s.check("F4 transp-bg alpha=0.6", approx(f.color[kCenterRgba + 3], 0.6f, 1e-4f));
    }
  }

  // ===== Silhouette edge cylinders (POV edge_line / edge_line2) =====
  // POV draws silhouette outlines as a union of `open` cylinders; umbreon
  // renders them as ROUND_LINEAR_CURVE capsules. These tests lock the two
  // properties a prior capless-cylinder rewrite broke: (1) the line is SOLID
  // (the on-axis pixel is fully covered, not stippled/under-covered), and (2)
  // edge_line2's per-endpoint transmit produces a linear opacity fade p0->p1
  // (not a single mean opacity), continuous across joints.
  {
    using umbreon::Vec3;
    using umbreon::Vec4;

    auto cylScene = [&](const umbreon::Cylinder& cyl, Vec3 bg) {
      umbreon::Scene sc;
      sc.camera = makeOrthoCam();          // ortho, frames [-2,2]^2
      sc.background = bg;
      sc.ambientColor = {1, 1, 1};         // flatOutline => raw color shading
      sc.cylinders.push_back(cyl);
      return sc;
    };

    // C1: an opaque black cylinder along x at the view center over a WHITE
    // background. The on-axis center pixel must be (near) black -- a solid,
    // fully-covered line. A thin capless cylinder would under-cover here and
    // leave the pixel gray/white; the capsule covers it solidly.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {2, 0, 0};
      c.radius = 0.5f;                     // ~0.6 px-wide at 5x5; covers center
      c.color = {0, 0, 0, 1.0f};           // opaque black outline
      c.open = true;                       // silhouette edge: ROUND capsule path
      umbreon::Scene sc = cylScene(c, {1, 1, 1});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      umbreon::FrameResult f = umbreon::render(sc, o);
      s.check("C1 solid line: center R black (<0.05)", f.color[kCenterRgba + 0] < 0.05f);
      s.check("C1 solid line: center G black (<0.05)", f.color[kCenterRgba + 1] < 0.05f);
      s.check("C1 solid line: center B black (<0.05)", f.color[kCenterRgba + 2] < 0.05f);
      s.check("C1 solid line: center covered (alpha=1)",
              approx(f.color[kCenterRgba + 3], 1.0f, 1e-4f));
    }

    // C2: edge_line2 gradient. Same geometry, transparent black with opacity 1
    // at p0 (left) fading to 0 at p1 (right). The ray pierces BOTH cylinder
    // walls (front + back at the same x, hence the same axial u and opacity a),
    // composited "over" like POV's open-cylinder transmit, so the black-over-
    // white brightness is (1-a)^2 with a = 1 - u, i.e. u^2 -- a monotone ramp
    // left (dark) to right (bright). Asserts a real lerp, not a single mean.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {2, 0, 0};
      c.radius = 0.5f;
      c.color = {0, 0, 0, 1.0f};           // opacity at p0 = 1 (fully opaque)
      c.opacity1 = 0.0f;                    // opacity at p1 = 0 (fully transmit)
      c.open = true;                       // edge_line2: ROUND capsule path
      umbreon::Scene sc = cylScene(c, {1, 1, 1});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparency = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      // Center row pixels (py=2): the on-axis ray at each px hits the cylinder.
      const std::size_t lft = (2 * 5 + 1) * 4;  // px=1, u=0.3 -> bright u^2=0.09
      const std::size_t ctr = (2 * 5 + 2) * 4;  // px=2, u=0.5 -> bright u^2=0.25
      const std::size_t rgt = (2 * 5 + 3) * 4;  // px=3, u=0.7 -> bright u^2=0.49
      s.check("C2 gradient: left darker than center",
              f.color[lft + 0] + 0.05f < f.color[ctr + 0]);
      s.check("C2 gradient: center darker than right",
              f.color[ctr + 0] + 0.05f < f.color[rgt + 0]);
      // Exact two-wall transmit value at center (u=0.5): (1-a)^2 = u^2 = 0.25.
      s.check("C2 gradient: center brightness = u^2 (0.25)",
              approx(f.color[ctr + 0], 0.25f, 0.02f));
    }

    // C3: a uniform transparent cylinder (opacity1 < 0) uses color.w everywhere,
    // so the fade of C2 is NOT applied: every covered pixel has the same opacity
    // 0.5 on both walls => brightness (1-0.5)^2 = 0.25 over white, constant along
    // the line. Guards that the lerp only triggers for edge_line2 (opacity1 >= 0)
    // and never perturbs a plain edge_line's uniform opacity.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {2, 0, 0};
      c.radius = 0.5f;
      c.color = {0, 0, 0, 0.5f};           // uniform opacity 0.5
      c.opacity1 = -1.0f;                   // no gradient
      c.open = true;                       // edge_line: ROUND capsule path
      umbreon::Scene sc = cylScene(c, {1, 1, 1});
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparency = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const std::size_t lft = (2 * 5 + 1) * 4;
      const std::size_t ctr = (2 * 5 + 2) * 4;
      const std::size_t rgt = (2 * 5 + 3) * 4;
      s.check("C3 uniform: center brightness 0.25 (two walls)",
              approx(f.color[ctr + 0], 0.25f, 0.02f));
      s.check("C3 uniform: left == center (no fade)",
              approx(f.color[lft + 0], f.color[ctr + 0], 0.02f));
      s.check("C3 uniform: right == center (no fade)",
              approx(f.color[rgt + 0], f.color[ctr + 0], 0.02f));
    }

    // C4: SEAM GUARD. Two collinear transparent cylinders share the joint at
    // the origin. Rendered as independent capsules, each adds a hemispherical
    // end cap at the joint, so a ray through the joint pierces BOTH caps and the
    // extra transparent black layers darken it -- a dark bead at every segment
    // joint (the POV-vs-umbreon seam). The renderer instead stitches segments
    // that share an endpoint into one connected ROUND_LINEAR_CURVE with
    // RTC_CURVE_FLAG_NEIGHBOR_* so the internal caps are dropped and the joint
    // is a single shared swept-sphere: two walls, brightness (1-0.5)^2 = 0.25,
    // identical to a mid-segment pixel. Asserts the joint is NOT darker than the
    // mid-segments (would fail if the chaining/neighbor-flags were removed).
    {
      umbreon::Cylinder a;
      a.p0 = {-2, 0, 0};
      a.p1 = {0, 0, 0};
      a.radius = 0.5f;
      a.color = {0, 0, 0, 0.5f};             // uniform transparent black
      a.open = true;                         // silhouette edges: chain at joint
      umbreon::Cylinder b = a;               // inherits open=true
      b.p0 = {0, 0, 0};
      b.p1 = {2, 0, 0};                       // shares the joint at the origin
      umbreon::Scene sc = cylScene(a, {1, 1, 1});
      sc.cylinders.push_back(b);
      umbreon::RenderOptions o; o.width = 5; o.height = 5;
      o.transparency = true;
      umbreon::FrameResult f = umbreon::render(sc, o);
      const std::size_t lft = (2 * 5 + 1) * 4;  // mid-segment A (x=-0.8)
      const std::size_t ctr = (2 * 5 + 2) * 4;  // the joint (x=0)
      const std::size_t rgt = (2 * 5 + 3) * 4;  // mid-segment B (x=+0.8)
      s.check("C4 seam: joint brightness ~0.25 (single shared sphere)",
              approx(f.color[ctr + 0], 0.25f, 0.05f));
      s.check("C4 seam: joint not darker than mid-segment A",
              f.color[ctr + 0] + 0.03f >= f.color[lft + 0]);
      s.check("C4 seam: joint not darker than mid-segment B",
              f.color[ctr + 0] + 0.03f >= f.color[rgt + 0]);
    }

    // C5: CAP GUARD. POV stick bonds are CLOSED cylinders (open=false) with FLAT
    // disk caps at the exact endpoints; consecutive overlapping bonds must not
    // show a protruding cap. A ROUND_LINEAR_CURVE capsule (open=true) instead
    // bulges a hemisphere ~radius PAST the endpoint, which is what produced the
    // spurious colored arc through an overlapping transparent surface. This test
    // renders the SAME cylinder both ways over a white background and asserts
    // that just BEYOND the p1 endpoint the capped (CONE) cylinder leaves the
    // pixel uncovered (background) while the open (round) one bulges over it.
    // Locks the arc fix: a regression to round caps for bonds would re-cover it.
    {
      umbreon::Cylinder c;
      c.p0 = {-2, 0, 0};
      c.p1 = {0, 0, 0};                       // ends at the view center (x=0)
      c.radius = 0.45f;                       // round cap would reach x=+0.45
      c.color = {0, 0, 0, 1.0f};              // opaque black
      umbreon::RenderOptions o; o.width = 11; o.height = 11;
      // 11-wide ortho over [-2,2]: pixel centers x_i = -2 + (i+0.5)*4/11.
      // i=4 -> x=-0.36 (mid-body, covered both ways); i=6 -> x=+0.36 (beyond p1:
      // inside a round cap's x<=0.45 bulge, outside a flat cap that stops at x=0).
      const std::size_t row = 5;              // center row (py=5, y=0)
      const std::size_t body = (row * 11 + 4) * 4;   // mid-body probe
      const std::size_t past = (row * 11 + 6) * 4;   // just past the endpoint

      c.open = false;                         // capped bond: CONE flat caps
      umbreon::FrameResult fc = umbreon::render(cylScene(c, {1, 1, 1}), o);
      c.open = true;                          // silhouette edge: ROUND hemicap
      umbreon::FrameResult fo = umbreon::render(cylScene(c, {1, 1, 1}), o);

      // Sanity: the body is covered (near black) in BOTH cap modes.
      s.check("C5 cap: capped body covered (R<0.1)", fc.color[body + 0] < 0.1f);
      s.check("C5 cap: open body covered (R<0.1)", fo.color[body + 0] < 0.1f);
      // Discriminator: beyond p1 the flat cap leaves background (near white),
      // while the round cap bulges over it (notably darker).
      s.check("C5 cap: capped leaves background past endpoint (R>0.8)",
              fc.color[past + 0] > 0.8f);
      s.check("C5 cap: round bulges past endpoint (darker than capped)",
              fo.color[past + 0] + 0.2f < fc.color[past + 0]);
    }
  }

  // C6: FAR-SCENE SURFACE-SKIP GUARD. The front-to-back transparency walk steps
  // just past each hit to find the next surface. That step must clear only
  // floating-point jitter, never a whole distinct primitive. A relative step of
  // t*1e-5 was too coarse at a far camera (large t): an opaque surface sitting
  // just behind a transparent one (within ~t*1e-5) was stepped over, so a DEEPER
  // object showed through it (in the real scene an atom sphere appeared through a
  // bond cylinder). Three surfaces along the center ray at t~200: a transparent
  // white quad (z=0), then an opaque BLUE sphere whose near surface is 0.001
  // behind it (z=-0.001), then a deeper opaque RED sphere (z=-0.5). The blue
  // sphere is nearest and must occlude the red one; stepping over its near
  // surface would wrongly reveal red. (A two-surface test would not catch this:
  // skipping a lone sphere's near surface still leaves its far surface to
  // occlude, so a third, deeper, distinctly-colored object is required.)
  {
    using umbreon::Vec3;
    using umbreon::Vec4;
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();
    sc.camera.position = {0, 0, 200};        // far camera => large t (~200)
    sc.background = {0, 0, 0};
    sc.ambientColor = {1, 1, 1};
    sc.assumedGamma = 1.0f;                   // raw values (no gamma encode)
    sc.mesh = makeQuad(Vec4{1, 1, 1, 0.5f});  // transparent quad at z=0
    sc.mesh.material = umbreon::Material::flatOutline();  // raw white * 0.5
    umbreon::Sphere a;                         // nearest opaque: BLUE
    a.center = {0, 0, -5.001f};                // near surface at z=-0.001
    a.radius = 5.0f;
    a.color = Vec4{0, 0, 1, 1};
    a.material = umbreon::Material::flatOutline();
    sc.spheres.push_back(a);
    umbreon::Sphere b;                         // deeper opaque: RED
    b.center = {0, 0, -1.0f};                  // near surface at z=-0.5
    b.radius = 0.5f;
    b.color = Vec4{1, 0, 0, 1};
    b.material = umbreon::Material::flatOutline();
    sc.spheres.push_back(b);
    umbreon::RenderOptions o;
    o.width = 5;
    o.height = 5;
    o.transparency = true;
    umbreon::FrameResult f = umbreon::render(sc, o);
    // fix: quad(0.5 white) over BLUE a => (0.5, 0.5, 1.0). bug: a's near surface
    // is stepped over => quad over RED b => (1.0, 0.5, 0.5). blue-vs-red tells.
    s.check("C6 far skip: nearest opaque (just behind transparent) occludes deeper",
            f.color[kCenterRgba + 2] > f.color[kCenterRgba + 0] + 0.3f);
  }

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

  // AO gate: flat outline primitives are NEVER AO-darkened. The same outline
  // sphere with AO on vs off must be bit-identical (the primitive branch passes
  // aoFactor == 1.0f literally), even though a lone sphere would have AO ~ 1.
  {
    umbreon::Scene sc;
    sc.camera = makeOrthoCam();
    sc.lights.push_back(makeKeyLight());
    sc.background = {0.4f, 0.1f, 0.2f};
    umbreon::Sphere sp;
    sp.center = {0, 0, 0};
    sp.radius = 1.0f;
    sp.color = {0.3f, 0.4f, 0.5f, 1.0f};
    sc.spheres.push_back(sp);
    umbreon::RenderOptions off;
    off.width = 5; off.height = 5; off.aoSamples = 0;
    umbreon::RenderOptions on = off;
    on.aoSamples = 64;
    on.aoDistance = 100.0f;
    umbreon::FrameResult fo = umbreon::render(sc, off);
    umbreon::FrameResult fn = umbreon::render(sc, on);
    s.check("AO gate: outline R identical on/off",
            approx(fn.color[kCenterRgba + 0], fo.color[kCenterRgba + 0], 1e-6f));
    s.check("AO gate: outline G identical on/off",
            approx(fn.color[kCenterRgba + 1], fo.color[kCenterRgba + 1], 1e-6f));
    s.check("AO gate: outline B identical on/off",
            approx(fn.color[kCenterRgba + 2], fo.color[kCenterRgba + 2], 1e-6f));
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
    o.giSamples = 48;
    umbreon::FrameResult a = umbreon::render(sc, o);
    umbreon::FrameResult b = umbreon::render(sc, o);
    bool identical = a.color.size() == b.color.size() && !a.color.empty();
    for (std::size_t i = 0; identical && i < a.color.size(); ++i)
      if (a.color[i] != b.color[i]) identical = false;
    s.check("GI determinism: two gi-on renders bit-identical color", identical);
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

  return s.report();
}
