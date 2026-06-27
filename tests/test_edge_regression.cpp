// Real-data edge-pass equivalence regression.
//
// The mesh refactor welds vertices at load time and hands the feature-edge
// extractor a precomputed position-class map instead of letting it re-weld the
// triangle soup. The guarantee is that this is invisible: rendering a real
// CueMol scene with the stroke edges ON must produce the SAME image whether the
// mesh is indexed (load-time weld + posClass) or de-indexed (legacy soup, where
// the extractor falls back to its own positional weld).
//
// Each scene is loaded once, rendered with edges on, then re-rendered after
// Mesh::toDeindexed() (which drops index/posClass so the extractor self-welds).
// The two frames must be bit-identical. This is a same-process, same-machine
// self-comparison, so it needs no committed golden image and is platform
// independent -- it directly guards the load-time-weld / "weld once" path
// against any drift from the legacy weld.
//
// Scenes are passed as arguments (screen-space tube/ribbon edge-validation cases
// plus a ribbon scene); fog is forced off via the _no_fog predefine.
#include <cstddef>
#include <string>

#include "geom/mesh2_reader.hpp"
#include "pov/pov_scene_reader.hpp"
#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

std::string resolveRelative(const std::string& base, const std::string& rel) {
  if (!rel.empty() && rel[0] == '/') return rel;  // already absolute
  std::size_t slash = base.find_last_of('/');
  std::string dir = (slash == std::string::npos) ? "." : base.substr(0, slash);
  return dir + "/" + rel;
}

// Minimal CueMol .pov -> Scene load (camera/lights/background/mesh), mirroring
// the .pov branch of the CLI but without AO bounds / alpha / styling, none of
// which the edge-pass equivalence depends on. Fog is forced off.
umbreon::Scene loadScene(const std::string& povPath, int w, int h) {
  umbreon::PovParseOptions popt;
  popt.imageWidth = w;
  popt.imageHeight = h;
  popt.predefined["_no_fog"] = 1.0;  // umbreon ignores in-.pov _no_fog; force off
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

bool colorEqual(const umbreon::FrameResult& a, const umbreon::FrameResult& b) {
  if (a.width != b.width || a.height != b.height) return false;
  if (a.color.size() != b.color.size()) return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (a.color[i] != b.color[i]) return false;  // exact, bit-for-bit
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  umbreon::test::Suite s("edge_regression");

  if (argc < 2) {
    std::printf("  (skipped: no scene paths given)\n");
    return s.report();
  }

  const int W = 200, H = 200;
  umbreon::RenderOptions ropt;
  ropt.width = W;
  ropt.height = H;
  ropt.strokeEdges.enable = true;
  ropt.strokeEdges.silhouette = true;
  ropt.strokeEdges.crease = true;  // exercise crease + border + silhouette
  ropt.strokeEdges.border = true;

  for (int i = 1; i < argc; ++i) {
    const std::string path = argv[i];

    umbreon::Scene sIdx = loadScene(path, W, H);
    umbreon::Scene sDe = sIdx;                 // copy camera/lights/etc.
    sDe.mesh = sIdx.mesh.toDeindexed();        // legacy soup path

    s.check(path + ": indexing dedups vertices",
            sIdx.mesh.vertexCount() < sDe.mesh.vertexCount());
    s.check(path + ": same triangle count after de-index",
            sIdx.mesh.triangleCount() == sDe.mesh.triangleCount());

    umbreon::FrameResult fIdx = umbreon::render(sIdx, ropt);
    umbreon::FrameResult fDe = umbreon::render(sDe, ropt);
    s.check(path + ": indexed vs de-indexed render (edges on) is bit-identical",
            colorEqual(fIdx, fDe));
  }

  return s.report();
}
