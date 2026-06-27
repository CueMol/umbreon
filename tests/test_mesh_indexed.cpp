// Indexed-mesh equivalence regression.
//
// The mesh refactor lets a triangle mesh share vertices through an optional
// `index` buffer while keeping the de-indexed (triangle-soup) layout as the
// fallback when `index` is empty. The headline guarantee is that indexing is a
// pure storage optimization: an indexed mesh and its de-indexed expansion
// (Mesh::toDeindexed) must render to a BIT-IDENTICAL color buffer, because a
// vertex is shared only when position, normal and color all bit-match.
//
// This test builds a tiny indexed mesh by hand, checks the layout invariants
// (index size, vertex sharing) and asserts the rendered frame matches the
// de-indexed expansion exactly. The data-driven edge-pass equivalence lives in
// the separate edge-regression test; here we lock the render-side bit-identity
// at unit scale with no data file.
#include <cstddef>

#include "test_util.hpp"
#include "umbreon.hpp"

namespace {

// A flat quad in z=0 spanning [-2,2]^2 facing +Z as TWO triangles that share the
// p00..p11 diagonal. Indexed: 4 unique vertices, 6 corners -> the two shared
// vertices are welded. Same pigment/normal everywhere so the weld is legal.
umbreon::Mesh makeIndexedQuad(umbreon::Vec4 color) {
  using umbreon::Vec3;
  umbreon::Mesh m;
  const Vec3 p00{-2, -2, 0}, p10{2, -2, 0}, p11{2, 2, 0}, p01{-2, 2, 0};
  const Vec3 verts[4] = {p00, p10, p11, p01};
  const Vec3 n{0, 0, 1};
  for (int i = 0; i < 4; ++i) {
    m.positions.push_back(verts[i]);
    m.normals.push_back(n);
    m.colors.push_back(color);
  }
  m.index = {0, 1, 2, 0, 2, 3};  // two triangles, diagonal p00-p11 shared
  m.material.ambient = 0.2f;
  m.material.diffuse = 0.8f;
  return m;
}

umbreon::Scene makeScene(const umbreon::Mesh& mesh) {
  umbreon::Scene sc;
  sc.camera.position = {0, 0, 10};
  sc.camera.direction = {0, 0, -1};
  sc.camera.up = {0, 1, 0};
  sc.camera.orthographic = true;
  sc.camera.height = 4.0f;
  sc.background = {0.1f, 0.2f, 0.3f};
  sc.mesh = mesh;
  umbreon::DistantLight l;
  l.direction = {0, 0, -1};
  l.intensity = 0.5f;
  sc.lights.push_back(l);
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

int main() {
  umbreon::test::Suite s("mesh_indexed");

  const umbreon::Mesh indexed = makeIndexedQuad({0.5f, 0.6f, 0.7f, 1.0f});
  const umbreon::Mesh deidx = indexed.toDeindexed();

  // (1) Layout invariants.
  s.check("indexed: index size == 3 * triangleCount",
          indexed.index.size() == 3 * indexed.triangleCount());
  s.check("indexed and de-indexed agree on triangleCount",
          indexed.triangleCount() == deidx.triangleCount());
  s.check("indexing shares vertices (fewer vertices than corners)",
          indexed.vertexCount() < deidx.vertexCount());
  s.check("de-indexed has one vertex per corner",
          deidx.vertexCount() == 3 * deidx.triangleCount());
  s.check("de-indexed drops the index buffer", deidx.index.empty());

  // (2) Rendered frames must be bit-identical.
  umbreon::RenderOptions opt;
  opt.width = 24;
  opt.height = 24;
  opt.supersample = 2;
  umbreon::FrameResult fIdx = umbreon::render(makeScene(indexed), opt);
  umbreon::FrameResult fDe = umbreon::render(makeScene(deidx), opt);
  s.check("indexed vs de-indexed render is bit-identical",
          colorEqual(fIdx, fDe));

  return s.report();
}
