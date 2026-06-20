// Unit and integration tests for the focused mesh2 reader.
#include <cmath>
#include <cstddef>
#include <cstdio>

#include "geom/mesh2_reader.hpp"
#include "test_util.hpp"

int main(int argc, char** argv) {
  umbreon::test::Suite s("mesh2_reader");

  // --- unit test: small synthetic mesh2 ----------------------------------
  {
    const char* src = R"POV(
      #declare C0 = <1, 0, 0>;
      #declare C1 = <0, 0, 1>;
      // This macro must be skipped entirely: its #local texture finish must
      // NOT be adopted as the mesh material.
      #macro junk(a)
        #local tex0 = texture { finish { ambient 1.0 diffuse 0.111 }
                                pigment { color rgb a } }
      #end
      union {
        mesh2 {
          vertex_vectors { 4, <0,0,0>, <1,0,0>, <0,1,0>, <1,1,0> }
          normal_vectors { 4, <0,0,1>, <0,0,1>, <0,0,1>, <0,0,1> }
          texture_list {
            2,
            texture { pigment { color rgb C0 } },
            texture { pigment { color rgb C1 } }
          }
          face_indices { 2, <0,1,2>,0,0,0, <1,3,2>,1,1,1 }
        }
      }
    )POV";
    umbreon::Mesh m = umbreon::readMesh2FromString(src);
    s.check_eq("synthetic: triangle count", m.triangleCount(), std::size_t(2));
    s.check_eq("synthetic: de-indexed vertex count", m.vertexCount(),
               std::size_t(6));
    s.check("synthetic: face 0 resolves to C0 (red)",
            m.colors[0].x == 1.0f && m.colors[0].y == 0.0f &&
                m.colors[0].z == 0.0f);
    s.check("synthetic: face 1 resolves to C1 (blue)",
            m.colors[3].x == 0.0f && m.colors[3].y == 0.0f &&
                m.colors[3].z == 1.0f);
    s.check("synthetic: macro-local finish did not leak into material",
            std::fabs(m.material.diffuse - 0.8f) < 1e-4f);
  }

  // --- vertex expression evaluation --------------------------------------
  {
    // texture_list color uses a linear blend of two declared vectors,
    // exactly the pattern CueMol emits for color ramps.
    const char* src = R"POV(
      #declare A = <1, 1, 1>;
      #declare B = <0, 0, 0>;
      mesh2 {
        vertex_vectors { 3, <0,0,0>, <1,0,0>, <0,1,0> }
        normal_vectors { 3, <0,0,1>, <0,0,1>, <0,0,1> }
        texture_list { 1, texture { pigment { color rgbt A*0.25+B*0.75 } } }
        face_indices { 1, <0,1,2>, 0, 0, 0 }
      }
    )POV";
    umbreon::Mesh m = umbreon::readMesh2FromString(src);
    s.check("expr: A*0.25+B*0.75 evaluates to 0.25 gray",
            std::fabs(m.colors[0].x - 0.25f) < 1e-5f &&
                std::fabs(m.colors[0].z - 0.25f) < 1e-5f);
  }

  // --- cylinder cap semantics: the `open` flag ---------------------------
  // POV silhouette edges expand to `open` (capless) cylinders, while raw stick
  // bonds / density wireframes are CLOSED (flat disk caps) unless they carry the
  // explicit `open` keyword. The renderer routes open vs capped to different
  // Embree curve types, so the parser must classify each correctly. Locks: (1)
  // edge_line / edge_line2 => open, (2) raw cylinder{} => capped by default,
  // (3) cylinder{ ... open ... } => open.
  {
    // edge_line / edge_line2 take a declared texture identifier as the tex arg
    // (exactly as CueMol emits, e.g. `_52_55_tex_0`); the macro's color comes
    // from the trailing color arg, so the tex value itself is unused here.
    const char* src = R"POV(
      #declare _tex0 = texture { pigment { color rgb <0,0,0> } }
      edge_line(<0,0,0>, <0,0,1>, <1,0,0>, <0,0,1>, 0.0, 0.05, _tex0, <0,0,0>)
      edge_line2(<0,0,0>, <0,0,1>, 0.0, <1,0,0>, <0,0,1>, 0.0, 0.0, 0.05,
                 _tex0, <0,0,0>)
      cylinder { <0,0,0>, <1,0,0>, 0.05
                 texture { pigment { color rgb <1,0,0> } } }
      cylinder { <0,0,0>, <1,0,0>, 0.05 open
                 texture { pigment { color rgb <0,1,0> } } }
    )POV";
    umbreon::SceneGeometry g = umbreon::readGeometryFromString(src);
    s.check_eq("open-flag: cylinder count", g.cylinders.size(), std::size_t(4));
    if (g.cylinders.size() == 4) {
      s.check("open-flag: edge_line is open", g.cylinders[0].open);
      s.check("open-flag: edge_line2 is open", g.cylinders[1].open);
      s.check("open-flag: raw cylinder is capped (not open)",
              !g.cylinders[2].open);
      s.check("open-flag: cylinder with `open` keyword is open",
              g.cylinders[3].open);
    }
  }

  // --- integration test: the real CueMol file ----------------------------
  if (argc > 1) {
    umbreon::Mesh m = umbreon::readMesh2FromFile(argv[1]);
    s.check_eq("test1.inc: triangle count", m.triangleCount(),
               std::size_t(968));
    s.check_eq("test1.inc: de-indexed vertex count", m.vertexCount(),
               std::size_t(2904));
    s.check("test1.inc: normals present",
            m.normals.size() == m.positions.size());
    s.check("test1.inc: colors present",
            m.colors.size() == m.positions.size());
    umbreon::Aabb b = m.bounds();
    s.check("test1.inc: bounds valid", b.valid() && b.diagonal() > 0.0f);
    s.check("test1.inc: material diffuse ~= 0.8",
            std::fabs(m.material.diffuse - 0.8f) < 1e-4f);
    s.check("test1.inc: material specular ~= 0.4",
            std::fabs(m.material.specular - 0.4f) < 1e-4f);
    s.check("test1.inc: material ambient ~= 0.2",
            std::fabs(m.material.ambient - 0.2f) < 1e-4f);
    bool colorsOk = true;
    for (const umbreon::Vec4& c : m.colors) {
      if (c.x < 0.0f || c.x > 1.0f || c.y < 0.0f || c.y > 1.0f ||
          c.z < 0.0f || c.z > 1.0f) {
        colorsOk = false;
        break;
      }
    }
    s.check("test1.inc: vertex colors within [0,1]", colorsOk);
  } else {
    std::printf("  (skipped test1.inc integration test: no path argument)\n");
  }

  return s.report();
}
