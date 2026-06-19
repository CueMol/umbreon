// Tests for the scene builder: instance grid layout and camera framing.
#include <cmath>
#include <cstddef>

#include "geom/scene_builder.hpp"
#include "test_util.hpp"

namespace {
umbreon::Mesh makeTriangle() {
  umbreon::Mesh m;
  m.positions = {umbreon::Vec3{0, 0, 0}, umbreon::Vec3{2, 0, 0}, umbreon::Vec3{0, 2, 0}};
  m.normals = {umbreon::Vec3{0, 0, 1}, umbreon::Vec3{0, 0, 1}, umbreon::Vec3{0, 0, 1}};
  m.colors = {umbreon::Vec4{1, 1, 1, 1}, umbreon::Vec4{1, 1, 1, 1}, umbreon::Vec4{1, 1, 1, 1}};
  return m;
}
}  // namespace

int main() {
  umbreon::test::Suite s("scene_builder");
  umbreon::Mesh mesh = makeTriangle();

  // grid N=1: a single instance.
  {
    umbreon::BuildOptions bo;
    bo.gridN = 1;
    umbreon::Scene sc = umbreon::buildScene(mesh, bo);
    s.check_eq("N=1: instance count", sc.instanceCount(), std::size_t(1));
    s.check_eq("N=1: effective triangles", sc.effectiveTriangles(),
               std::size_t(1));
    s.check("N=1: has at least one light", !sc.lights.empty());
    s.check("N=1: camera direction is normalized",
            std::fabs(umbreon::length(sc.camera.direction) - 1.0f) < 1e-4f);
    // the camera must look roughly toward the geometry centroid
    umbreon::Vec3 toCenter = umbreon::normalize(umbreon::Vec3{1, 1, 0} - sc.camera.position);
    s.check("N=1: camera faces the geometry",
            umbreon::dot(toCenter, sc.camera.direction) > 0.9f);
  }

  // grid N=3: 27 instances, centered on the origin.
  {
    umbreon::BuildOptions bo;
    bo.gridN = 3;
    umbreon::Scene sc = umbreon::buildScene(mesh, bo);
    s.check_eq("N=3: instance count", sc.instanceCount(), std::size_t(27));
    s.check_eq("N=3: effective triangles", sc.effectiveTriangles(),
               std::size_t(27));
    umbreon::Vec3 sum;
    for (const umbreon::Vec3& o : sc.instanceOffsets) sum = sum + o;
    s.check("N=3: instance grid centered on origin",
            std::fabs(sum.x) < 1e-3f && std::fabs(sum.y) < 1e-3f &&
                std::fabs(sum.z) < 1e-3f);
  }

  // grid spacing must scale the grid extent.
  {
    umbreon::BuildOptions wide;
    wide.gridN = 2;
    wide.spacing = 3.0f;
    umbreon::Scene sc = umbreon::buildScene(mesh, wide);
    float span = 0.0f;
    for (const umbreon::Vec3& o : sc.instanceOffsets)
      span = std::fmax(span, std::fabs(o.x));
    s.check("spacing: larger spacing spreads instances apart", span > 2.0f);
  }

  return s.report();
}
