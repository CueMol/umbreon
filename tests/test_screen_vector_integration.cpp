// Integration test for the screen-space vector edge source
// (--edges): render a real tube scene end-to-end with
// strokeEdges.enable and verify
//   (a) the screen source INKS: the frame gains dark stroke pixels over the
//       no-edges render of the same scene;
//   (b) the extraction invariants hold on the frame's own AOVs: re-running
//       classifyCracks + traceCrackChains consumes every active crack exactly
//       once, and every open chain terminates at a lattice junction (degree
//       1, 3 or 4) -- the continuity-by-construction contract on real data.
#include <cstddef>
#include <string>
#include <vector>

#include "edges/mesh_feature_edges.hpp"
#include "edges/screen_vector_edges.hpp"
#include "geom/mesh2_reader.hpp"
#include "pov/pov_scene_reader.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

std::string resolveRelative(const std::string& base, const std::string& rel) {
  if (!rel.empty() && rel[0] == '/') return rel;
  std::size_t slash = base.find_last_of('/');
  std::string dir = (slash == std::string::npos) ? "." : base.substr(0, slash);
  return dir + "/" + rel;
}

umbreon::Scene loadScene(const std::string& povPath, int w, int h) {
  umbreon::PovParseOptions popt;
  popt.imageWidth = w;
  popt.imageHeight = h;
  popt.predefined["_no_fog"] = 1.0;
  umbreon::PovSceneResult ps = umbreon::readPovScene(povPath, popt);
  const std::string inc = resolveRelative(povPath, ps.includePath);
  umbreon::SceneGeometry geo = umbreon::readGeometryFromFile(inc, ps.declares);
  umbreon::Scene sc;
  sc.mesh = std::move(geo.mesh);
  sc.spheres = std::move(geo.spheres);
  sc.cylinders = std::move(geo.cylinders);
  sc.camera = ps.camera;
  sc.lights = ps.lights;
  sc.background = ps.background;
  sc.fog = ps.fog;
  sc.ambientColor = umbreon::Vec3{1, 1, 1};
  sc.ambientIntensity = 1.0f;
  sc.assumedGamma = ps.assumedGamma;
  return sc;
}

// Count pixels darker than `thresh` in linear luminance.
std::size_t darkPixels(const umbreon::FrameResult& f, float thresh) {
  std::size_t n = 0;
  for (std::size_t p = 0; p + 3 < f.color.size(); p += 4) {
    const float lum =
        0.2126f * f.color[p] + 0.7152f * f.color[p + 1] + 0.0722f * f.color[p + 2];
    if (lum < thresh) ++n;
  }
  return n;
}

}  // namespace

int main(int argc, char** argv) {
  umbreon::test::Suite s("screen_vector_integration");

  if (argc < 2) {
    std::printf("  (skipped: no scene path given)\n");
    return s.report();
  }
  const std::string path = argv[1];
  const int W = 200, H = 200;

  umbreon::Scene scene = loadScene(path, W, H);

  umbreon::RenderOptions base;
  base.width = W;
  base.height = H;
  umbreon::FrameResult fPlain = umbreon::render(scene, base);

  umbreon::RenderOptions ropt = base;
  ropt.strokeEdges.enable = true;
  umbreon::FrameResult fScreen = umbreon::render(scene, ropt);

  // (a) the screen source inks.
  s.check("screen source adds dark stroke pixels",
          darkPixels(fScreen, 0.1f) > darkPixels(fPlain, 0.1f) + 50);

  // AOVs captured at the frame resolution.
  const int fw = W, fh = H;  // supersample 1 in RenderOptions default
  s.check("edge AOVs captured",
          fScreen.viewZ.size() == static_cast<std::size_t>(fw) * fh &&
              fScreen.objectId.size() == static_cast<std::size_t>(fw) * fh);

  // (b) extraction invariants on the frame's own AOVs.
  const umbreon::ScreenProj sp = umbreon::makeScreenProj(scene.camera, fw, fh);
  umbreon::ScreenClassifyParams cp;
  umbreon::CrackField cf = umbreon::classifyCracks(
      fw, fh, fScreen.viewZ.data(), fScreen.objectId.data(), nullptr, sp, cp);
  int active = 0;
  for (std::uint8_t v : cf.right)
    if (v & umbreon::kCrackClassMask) ++active;
  for (std::uint8_t v : cf.down)
    if (v & umbreon::kCrackClassMask) ++active;
  const std::vector<umbreon::ScreenChain> chains = umbreon::traceCrackChains(
      cf, fScreen.viewZ.data(), fScreen.objectId.data());
  std::size_t edgels = 0;
  bool endpointsOk = true;
  for (const umbreon::ScreenChain& ch : chains) {
    edgels += ch.edgeClass.size();
    if (!ch.closed && (ch.deg0 == 2 || ch.deg1 == 2 || ch.deg0 == 0))
      endpointsOk = false;
  }
  s.check("tube scene produces chains", !chains.empty());
  s.check_eq("every active crack consumed exactly once", edgels,
             static_cast<std::size_t>(active));
  s.check("open chains terminate at junctions (degree != 2)", endpointsOk);

  return s.report();
}
