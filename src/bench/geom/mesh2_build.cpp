// Mesh2Reader mesh-build members: weld the current mesh2 block's faces into
// the accumulated indexed Mesh (exact-attribute render-vertex dedup + global
// position-class weld) and hand the finished SceneGeometry over. See
// mesh2_reader_impl.hpp for the class and the TU layout.
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geom/mesh2_reader_impl.hpp"

namespace umbreon {
namespace detail {
namespace {

// Exact-attribute weld key: the bit pattern of (position, normal, color). Two
// corners share a render vertex only when all ten floats bit-match, so any
// hard-edge normal split or per-corner color seam keeps its own vertex and the
// rendered image stays bit-identical to the de-indexed triangle soup. (Distinct
// from the position-only WeldKey in edges/mesh_weld.hpp used for topology.)
struct RKey {
  std::uint32_t b[10];
  bool operator==(const RKey& o) const {
    for (int i = 0; i < 10; ++i)
      if (b[i] != o.b[i]) return false;
    return true;
  }
};
struct RKeyHash {
  std::size_t operator()(const RKey& k) const {
    std::size_t h = 1469598103934665603ull;  // FNV-1a
    for (int i = 0; i < 10; ++i) {
      h ^= k.b[i];
      h *= 1099511628211ull;
    }
    return h;
  }
};
inline RKey rKeyOf(const Vec3& p, const Vec3& n, const Vec4& c) {
  RKey k;
  const float f[10] = {p.x, p.y, p.z, n.x, n.y, n.z, c.x, c.y, c.z, c.w};
  std::memcpy(k.b, f, sizeof(k.b));
  return k;
}

}  // namespace

// Build the current mesh2 block's faces into the accumulated INDEXED mesh,
// then clear the block buffers (a file may hold several mesh2 blocks, each
// with its own 0-based vertex indices).
//
// Two welds run together:
//  - Render vertices are deduplicated by EXACT (position, normal, color) bits,
//    PER BLOCK. mesh2 carries per-corner normals and per-corner texture/color
//    indices that can differ across a shared position (hard edges, color
//    seams); welding only on an exact attribute match keeps those corners on
//    separate vertices, so the rendered image is bit-identical to the legacy
//    triangle soup. The per-block reset keeps every vertex within a single
//    transparency group, which `--alpha` relies on.
//  - The position-class map (edges/mesh_weld.hpp) welds by POSITION ALONE
//    with a ~1e-4 tolerance, GLOBALLY across blocks, reconstructing the
//    water-tight ribbon topology that face_indices do not encode. Feature-edge
//    extraction reads this map instead of re-welding (see
//    render/mesh_feature_edges.cpp). Class ids are assigned in first-seen
//    corner order so they match the extractor's own weld numbering exactly.
void Mesh2Reader::buildIndexedBlock() {
  const std::size_t nFace = faces_.size();
  if (nFace == 0) return;
  const std::size_t nCorner = nFace * 3;

  // 1) Resolve the final per-corner (position, normal, color) for this block,
  //    INCLUDING the flat-normal fallback, before welding.
  std::vector<Vec3> cPos(nCorner), cNrm(nCorner);
  std::vector<Vec4> cCol(nCorner);
  for (std::size_t fi = 0; fi < nFace; ++fi) {
    const Face& f = faces_[fi];
    for (int k = 0; k < 3; ++k) {
      const std::size_t c = fi * 3 + static_cast<std::size_t>(k);
      const int vi = f.v[k];
      if (vi < 0 || static_cast<std::size_t>(vi) >= verts_.size()) {
        throw Mesh2ReadError("face vertex index out of range: " +
                             std::to_string(vi));
      }
      cPos[c] = verts_[static_cast<std::size_t>(vi)];
      cNrm[c] = static_cast<std::size_t>(vi) < norms_.size()
                    ? norms_[static_cast<std::size_t>(vi)]
                    : Vec3{0, 0, 0};
      Vec4 col{1.0f, 1.0f, 1.0f, 1.0f};
      if (f.hasTex && !texColors_.empty()) {
        int ti = f.tex[k];
        if (ti < 0) ti = 0;
        if (static_cast<std::size_t>(ti) >= texColors_.size())
          ti = static_cast<int>(texColors_.size()) - 1;
        col = texColors_[static_cast<std::size_t>(ti)];
      }
      cCol[c] = col;
    }
  }
  // No normals in this block: derive geometric (flat) normals per face.
  if (norms_.empty()) {
    for (std::size_t fi = 0; fi < nFace; ++fi) {
      const std::size_t c0 = fi * 3;
      const Vec3 nn =
          normalize(cross(cPos[c0 + 1] - cPos[c0], cPos[c0 + 2] - cPos[c0]));
      cNrm[c0] = nn;
      cNrm[c0 + 1] = nn;
      cNrm[c0 + 2] = nn;
    }
  }

  // 2) Weld: exact-attribute render vertices (per block) + global position
  //    classes, emitting one index per corner.
  std::unordered_map<RKey, std::uint32_t, RKeyHash> vmap;
  vmap.reserve(nCorner);
  for (std::size_t c = 0; c < nCorner; ++c) {
    const RKey rk = rKeyOf(cPos[c], cNrm[c], cCol[c]);
    auto it = vmap.find(rk);
    std::uint32_t vid;
    if (it == vmap.end()) {
      vid = static_cast<std::uint32_t>(mesh_.positions.size());
      vmap.emplace(rk, vid);
      mesh_.positions.push_back(cPos[c]);
      mesh_.normals.push_back(cNrm[c]);
      mesh_.colors.push_back(cCol[c]);
      // Position class for the new render vertex. A new position class always
      // coincides with a new render vertex (a new position => new attribute
      // tuple), so assigning here visits classes in first-seen corner order.
      const WeldKey wk = weldKey(cPos[c]);
      auto pit = posClassMap_.find(wk);
      if (pit == posClassMap_.end()) {
        mesh_.posClass.push_back(posClassCount_);
        posClassMap_.emplace(wk, posClassCount_);
        ++posClassCount_;
      } else {
        mesh_.posClass.push_back(pit->second);
      }
    } else {
      vid = it->second;
    }
    mesh_.index.push_back(vid);
  }
  mesh_.posClassCount = posClassCount_;

  // 3) Per-block material + transparency group, indexed by triangle. All of
  //    this block's indices are in place, so triangleCount() (= index/3) now
  //    spans through this block and the resize targets the right range.
  uint8_t matIdx = static_cast<uint8_t>(blockMaterials_.size());
  blockMaterials_.push_back(haveBlockMaterial_ ? blockMaterial_ : Material{});
  mesh_.triMaterialId.resize(mesh_.triangleCount(), matIdx);
  mesh_.triGroupId.resize(mesh_.triangleCount(),
                          static_cast<uint16_t>(curGroup_));

  verts_.clear();
  norms_.clear();
  texColors_.clear();
  faces_.clear();
}

SceneGeometry Mesh2Reader::finalize() {
  if (!haveMesh_ && spheres_.empty() && cylinders_.empty())
    throw Mesh2ReadError("no geometry (mesh2/sphere/edge_line) found in input");
  if (!blockMaterials_.empty()) {
    mesh_.materials = std::move(blockMaterials_);
    // mesh_.material keeps a sensible default (first block's material).
    mesh_.material = mesh_.materials[0];
  }
  SceneGeometry g;
  g.mesh = std::move(mesh_);
  g.spheres = std::move(spheres_);
  g.cylinders = std::move(cylinders_);
  g.groupNames = std::move(groupNames_);
  return g;
}

}  // namespace detail
}  // namespace umbreon
