// Shared feature-edge extraction + projection. See mesh_feature_edges.hpp.
//
// extractMeshFeatureEdges is the factored body of the former
// object_space_edges.cpp:emitMeshEdges: the same weld / emap / cluster-normal
// pipeline and the same silhouette / crease / border predicates, changed ONLY in
// the sink (topology-tagged FeatureSeg instead of disconnected RawSeg) and in
// tracking node ids for chaining.
#include "render/mesh_feature_edges.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "render/mesh_weld.hpp"

namespace umbreon {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// "Toward the viewer" unit vector at P (orthographic: constant -view direction;
// perspective: direction to the eye). Mirrors object_space_edges.cpp.
Vec3 viewerDirAt(const Vec3& P, const Camera& cam) {
  const Vec3 toCam = normalize(cam.direction * -1.0f);
  if (cam.orthographic) return toCam;
  const Vec3 d = cam.position - P;
  const float l = length(d);
  return l > 0.0f ? d * (1.0f / l) : toCam;
}

}  // namespace

FeatureMesh extractMeshFeatureEdges(const Mesh& mesh, const Camera& cam,
                                    const ExtractParams& opt) {
  FeatureMesh result;
  const std::size_t nCorner = mesh.cornerCount();
  const std::size_t nTri = nCorner / 3;
  if (nTri == 0) return result;
  const bool haveNrm = mesh.normals.size() == mesh.vertexCount();
  // Per-corner attribute reads routed through the optional index. With a
  // de-indexed mesh cornerVertex(c)==c, so this is the legacy access.
  auto cornerPos = [&](std::size_t c) -> const Vec3& {
    return mesh.positions[mesh.cornerVertex(c)];
  };
  auto cornerNrm = [&](std::size_t c) -> const Vec3& {
    return mesh.normals[mesh.cornerVertex(c)];
  };

  // 1) POSITIONAL WELD of the de-indexed corners into shared vertices
  //    (quantized-position hash), accumulating an averaged vertex normal.
  //
  // WHY THIS EXISTS (not a lossy shortcut -- it RECONSTRUCTS the true topology):
  // CueMol exports each ribbon/cartoon surface as a stack of triangle STRIPS, one
  // per sweep "winding", and the vertices (and normals) on the shared border
  // between adjacent windings are DUPLICATED -- each winding repeats the seam ring
  // with its own indices. So the POV mesh2 `face_indices` encode each strip's
  // internal adjacency but carry NO connectivity BETWEEN strips: used as-is the
  // surface is a soup of disconnected strips (measured on ribbon1: 1060 connected
  // components, 54520 boundary/1-face edges, 0 non-manifold). The real surface
  // connectivity is present ONLY as coincident vertex positions. Welding by
  // quantized position folds those redundant duplicates back together and recovers
  // the intended water-tight mesh (ribbon1 after weld: 1 component, ~1144 genuine
  // boundary edges = caps/strand-ends). The welded mesh IS the true topology; the
  // per-strip index is the redundant representation. (De-indexing upstream is also
  // unavoidable: mesh2 carries per-CORNER normals and per-corner texture/color
  // indices that differ across a shared position, e.g. at a hard edge.)
  //
  // WHAT IT DRIVES: the averaged vertex normal (vN), the edge adjacency map (emap,
  // for hard-edge silhouette + crease/border), the smooth-contour chain node ids,
  // and the QI self-face exclude set.
  //
  // LIMITATION: the merge is position-only, so it ALSO stitches genuinely-distinct
  // sheets that happen to pass within kQuant of each other (e.g. a tight self-fold,
  // or many strip ends converging at a cap), producing a few non-manifold edges
  // (ribbon1: 72, max 16 faces on one edge). This is harmless to silhouette
  // EXTRACTION; it only constrains a Freestyle-style TOPOLOGICAL QI self-skip
  // (sharing a vertex no longer implies "same continuous sheet"). A normal/
  // orientation-aware weld would separate those touches while still stitching the
  // smooth winding seams. NOTE: this weld is unrelated to the smooth-silhouette
  // CUSP visibility gap (the missing outline at grazing twists); that is a contour
  // fold-back handled by Freestyle's computeCusps, independent of topology.
  // The weld key (WeldKey/weldKey, render/mesh_weld.hpp) and ~1e-4 tolerance are
  // shared with the mesh2 reader, which bakes the very same partition into
  // mesh.posClass at load time. When that map is present we CONSUME it (the fast
  // path) instead of rehashing; otherwise (hand-built meshes) we weld here.
  std::vector<Vec3> vpos, vacc;
  std::vector<int> corner2v(nCorner);
  std::size_t nV;
  const bool usePosClass =
      mesh.posClass.size() == mesh.vertexCount() && mesh.posClassCount > 0;
  if (usePosClass) {
    // The reader assigned class ids in first-seen corner order with the same
    // weldKey, and we visit corners in that same order, so the representative
    // position (first corner of each class) and the per-class normal sum
    // (accumulated in corner order) are bit-for-bit identical to the weld below.
    nV = static_cast<std::size_t>(mesh.posClassCount);
    vpos.assign(nV, Vec3{0.0f, 0.0f, 0.0f});
    vacc.assign(nV, Vec3{0.0f, 0.0f, 0.0f});
    std::vector<char> seen(nV, 0);
    for (std::size_t c = 0; c < nCorner; ++c) {
      const std::size_t v =
          static_cast<std::size_t>(mesh.posClass[mesh.cornerVertex(c)]);
      corner2v[c] = static_cast<int>(v);
      if (!seen[v]) {
        vpos[v] = cornerPos(c);
        seen[v] = 1;
      }
      if (haveNrm) vacc[v] = vacc[v] + cornerNrm(c);
    }
  } else {
    std::unordered_map<WeldKey, int, WeldKeyHash> vmap;
    vmap.reserve(nCorner);
    for (std::size_t c = 0; c < nCorner; ++c) {
      const WeldKey k = weldKey(cornerPos(c));
      auto it = vmap.find(k);
      if (it == vmap.end()) {
        const int vi = static_cast<int>(vpos.size());
        vmap.emplace(k, vi);
        vpos.push_back(cornerPos(c));
        vacc.push_back(haveNrm ? cornerNrm(c) : Vec3{0.0f, 0.0f, 0.0f});
        corner2v[c] = vi;
      } else {
        corner2v[c] = it->second;
        if (haveNrm)
          vacc[static_cast<std::size_t>(it->second)] =
              vacc[static_cast<std::size_t>(it->second)] + cornerNrm(c);
      }
    }
    nV = vpos.size();
  }

  // 2) Face welded indices and geometric face normals.
  std::vector<int> fa(nTri), fb(nTri), fc(nTri);
  std::vector<Vec3> fNg(nTri);
  for (std::size_t f = 0; f < nTri; ++f) {
    fa[f] = corner2v[3 * f];
    fb[f] = corner2v[3 * f + 1];
    fc[f] = corner2v[3 * f + 2];
    fNg[f] = normalize(cross(vpos[static_cast<std::size_t>(fb[f])] - vpos[static_cast<std::size_t>(fa[f])],
                             vpos[static_cast<std::size_t>(fc[f])] - vpos[static_cast<std::size_t>(fa[f])]));
  }
  std::vector<Vec3> vN(nV, Vec3{0.0f, 0.0f, 0.0f});
  for (std::size_t v = 0; v < nV; ++v) {
    const float l = length(vacc[v]);
    if (haveNrm && l > 1.0e-6f) vN[v] = vacc[v] * (1.0f / l);
  }
  for (std::size_t f = 0; f < nTri; ++f)
    for (int e = 0; e < 3; ++e) {
      const std::size_t v = static_cast<std::size_t>(e == 0 ? fa[f] : e == 1 ? fb[f] : fc[f]);
      if (length(vN[v]) < 0.5f) vN[v] = fNg[f];
    }

  // Welded vertex -> incident triangle ids (for the EXPANDED QI self/adjacent-
  // face exclusion). For each triangle, append it to the table of each of its 3
  // welded vertices. Drives the 1-ring exclude set below (Freestyle's "skip an
  // occluder that shares any vertex with the edge's face", ViewMapBuilder.cpp
  // :2152-2196).
  std::vector<std::vector<int>> vFaces(nV);
  for (std::size_t f = 0; f < nTri; ++f) {
    vFaces[static_cast<std::size_t>(fa[f])].push_back(static_cast<int>(f));
    vFaces[static_cast<std::size_t>(fb[f])].push_back(static_cast<int>(f));
    vFaces[static_cast<std::size_t>(fc[f])].push_back(static_cast<int>(f));
  }

  result.vpos = vpos;

  // Mean triangle edge length, computed unconditionally (independent of which
  // natures are enabled) so the stroke pass can derive the obj-edges silhouette
  // camBias even when extracting with opt.silhouette off. This is the SAME
  // elsum/(3*nTri) the silhouette block computes locally (kept there to preserve
  // --obj-edges byte-identity); both produce the identical value.
  {
    double elsum = 0.0;
    for (std::size_t f = 0; f < nTri; ++f) {
      const std::size_t a = static_cast<std::size_t>(fa[f]);
      const std::size_t b = static_cast<std::size_t>(fb[f]);
      const std::size_t c = static_cast<std::size_t>(fc[f]);
      elsum += length(vpos[b] - vpos[a]) + length(vpos[c] - vpos[b]) +
               length(vpos[a] - vpos[c]);
    }
    result.meanEdge =
        nTri ? static_cast<float>(elsum / (3.0 * nTri)) : 0.0f;
  }

  // Edge adjacency (welded-position topology), SHARED by the hard-edge
  // silhouette and the crease/border pass.
  struct EAdj {
    int f1 = -1, f2 = -1;
  };
  std::unordered_map<std::uint64_t, EAdj> emap;
  emap.reserve(nTri * 3);
  auto ekey = [](int a, int b) {
    const std::uint32_t lo = static_cast<std::uint32_t>(a < b ? a : b);
    const std::uint32_t hi = static_cast<std::uint32_t>(a < b ? b : a);
    return (static_cast<std::uint64_t>(hi) << 32) | lo;
  };
  for (std::size_t f = 0; f < nTri; ++f) {
    const int v[3] = {fa[f], fb[f], fc[f]};
    for (int e = 0; e < 3; ++e) {
      EAdj& adj = emap[ekey(v[e], v[(e + 1) % 3])];
      if (adj.f1 < 0)
        adj.f1 = static_cast<int>(f);
      else if (adj.f2 < 0)
        adj.f2 = static_cast<int>(f);
    }
  }

  // Face -> up to 3 EDGE-adjacent neighbour faces (TRUE surface adjacency via the
  // welded edge map). A BFS over this follows the actual surface and does NOT jump
  // across a fold/gap the way the vertex 1-ring (vFaces) does: a fold's two sides
  // share at most a welded VERTEX, not a welded EDGE. So an N-ring exclude reaches
  // a geodesically-NEAR self-occluder (a ribbon's own surface a few faces across
  // its width) yet not a geodesically-FAR one (a tube's opposite wall / another
  // strand, many faces away) -- the discriminator the vertex 1-ring lacked.
  std::vector<std::array<int, 3>> faceNbr(nTri, std::array<int, 3>{-1, -1, -1});
  for (std::size_t f = 0; f < nTri; ++f) {
    const int v[3] = {fa[f], fb[f], fc[f]};
    for (int e = 0; e < 3; ++e) {
      auto it = emap.find(ekey(v[e], v[(e + 1) % 3]));
      if (it == emap.end()) continue;
      const EAdj& adj = it->second;
      faceNbr[f][static_cast<std::size_t>(e)] =
          (adj.f1 == static_cast<int>(f)) ? adj.f2 : adj.f1;
    }
  }
  // Self-exclude geodesic radius (edge-adjacency rings beyond the incident faces);
  // 0 => the byte-identical default ({f0,f1} only). See ExtractParams::selfExcludeRings.
  const int kSelfRings = opt.selfExcludeRings > 0 ? opt.selfExcludeRings : 0;

  // Synthetic node ids for smooth-contour crossings interior to a face. A
  // crossing on the welded edge (a,b) gets the SAME id from both incident faces,
  // so the contour chains across the shared edge. Ids start at nV.
  std::unordered_map<std::uint64_t, int> crossNode;
  int nextNode = static_cast<int>(nV);
  auto crossingNodeId = [&](int wa, int wb) -> int {
    const std::uint64_t key = ekey(wa, wb);
    auto it = crossNode.find(key);
    if (it != crossNode.end()) return it->second;
    const int id = nextNode++;
    crossNode.emplace(key, id);
    return id;
  };

  // Per-CORNER shading normal that PRESERVES hard edges (smoothing clusters).
  const float hardCos = std::cos(opt.meshHardEdgeDeg * kPi / 180.0f);
  std::vector<Vec3> cnrm(nCorner);
  {
    std::vector<std::vector<int>> posCorners(nV);
    for (std::size_t c = 0; c < nCorner; ++c)
      posCorners[static_cast<std::size_t>(corner2v[c])].push_back(static_cast<int>(c));
    for (std::size_t v = 0; v < nV; ++v) {
      const std::vector<int>& cs = posCorners[v];
      std::vector<Vec3> rep, sum;
      std::vector<int> cl(cs.size());
      for (std::size_t k = 0; k < cs.size(); ++k) {
        const std::size_t c = static_cast<std::size_t>(cs[k]);
        Vec3 n = haveNrm ? cornerNrm(c) : fNg[c / 3];
        const float ln = length(n);
        n = ln > 1.0e-12f ? n * (1.0f / ln) : fNg[c / 3];
        int found = -1;
        for (std::size_t t = 0; t < rep.size(); ++t)
          if (dot(rep[t], n) >= hardCos) { found = static_cast<int>(t); break; }
        if (found < 0) {
          rep.push_back(n);
          sum.push_back(n);
          found = static_cast<int>(rep.size()) - 1;
        } else {
          sum[static_cast<std::size_t>(found)] = sum[static_cast<std::size_t>(found)] + n;
        }
        cl[k] = found;
      }
      std::vector<Vec3> avg(rep.size());
      for (std::size_t t = 0; t < rep.size(); ++t) {
        const float l = length(sum[t]);
        avg[t] = l > 1.0e-12f ? sum[t] * (1.0f / l) : rep[t];
      }
      for (std::size_t k = 0; k < cs.size(); ++k)
        cnrm[static_cast<std::size_t>(cs[k])] = avg[static_cast<std::size_t>(cl[k])];
    }
  }

  // Build the EXPANDED 1-ring exclude set for an edge with incident faces f0,f1:
  // {f0, f1} UNION every triangle that shares a welded VERTEX with f0 or with f1
  // (f1 < 0 contributes nothing). Deduplicated. This is the QI self/adjacent-
  // face skip the stroke pass carries to the visibility ray-cast (Freestyle
  // ViewMapBuilder.cpp:2152-2196) so a silhouette grazing its own nearby surface
  // near a T-junction is not wrongly counted as self-occluded.
  // Build segment k's QI self-face exclude set: just the LITERAL incident faces
  // {f0,f1} (f1<0 border = no-op). The edge's own surface is then suppressed by
  // these ids PLUS the geometric COINCIDENT-PLANE skip in the QI ray cast
  // (EmbreeRenderer::occluded coplanarEps -- Freestyle GeomUtils::COINCIDENT): a
  // hit whose plane passes through the silhouette point is the point's OWN surface
  // and is not counted. That geometric test REPLACES the old topological vertex-
  // 1-ring, which could not distinguish a continuous self-surface (must skip, to
  // keep the outer outline solid) from a real front occluder that merely shares a
  // weld vertex on a tightly self-folded ribbon (must count, or the back fold
  // leaks over the front). The plane test handles both: the self-surface is
  // coplanar with the point (skip); the front occluder is a fold-gap away (count).
  // `smooth` is no longer consulted (kept for call-site stability).
  auto buildExclude = [&](int f0, int f1, bool smooth) {
    (void)smooth;
    std::vector<int> ex;
    ex.reserve(kSelfRings > 0 ? 16 : 4);
    auto addF = [&](int f) {
      if (f < 0) return;
      for (int g : ex)
        if (g == f) return;
      ex.push_back(f);
    };
    addF(f0);  // the edge's primary feature face
    addF(f1);  // the second fold/adjacent face (border f1<0 = no-op)
    // Grow kSelfRings edge-adjacency rings over the TRUE surface (faceNbr) so the
    // geodesically-near self-surface is skipped without jumping a fold (see above).
    std::size_t frontier = 0;
    for (int ring = 0; ring < kSelfRings; ++ring) {
      const std::size_t end = ex.size();
      for (std::size_t i = frontier; i < end; ++i)
        for (int nb : faceNbr[static_cast<std::size_t>(ex[i])]) addF(nb);
      frontier = end;
      if (ex.size() == end) break;  // ring did not grow: whole component covered
    }
    return ex;
  };

  // Emit a feature segment, carrying its incident mesh-triangle ids (f0,f1; -1
  // when absent) for Freestyle self-face exclusion, plus the narrowed self-face
  // exclude set used by the stroke pass's visibility ray-cast (smooth => f0 ring;
  // sharp => {f0,f1}).
  auto emit = [&](int n0, const Vec3& A, int n1, const Vec3& B, EdgeNature nat,
                  std::uint16_t grp, int f0, int f1, bool smooth = false,
                  const Vec3& nrm = Vec3{0.0f, 0.0f, 0.0f}) {
    FeatureSeg s{n0, n1, A, B, nat, grp, f0, f1, {}};
    s.excludeFaces = buildExclude(f0, f1, smooth);
    s.nrm = nrm;  // nonzero only for the smooth n.v==0 contour (cusp detection)
    result.segs.push_back(std::move(s));
  };

  // Border/crease emit helper: lift endpoints outward along the vertex normal by
  // opt.raise (matching emitMeshEdges::push) and tag with welded node ids plus
  // the two incident faces (f1 < 0 for a single-face border edge).
  auto pushEdge = [&](std::size_t a, std::size_t b, EdgeNature nat,
                      std::uint16_t grp, int f0, int f1) {
    emit(static_cast<int>(a), vpos[a] + vN[a] * opt.raise, static_cast<int>(b),
         vpos[b] + vN[b] * opt.raise, nat, grp, f0, f1);
  };

  // 3) SILHOUETTE = smooth n.v==0 contour (3a) + hard-edge straddle (3b).
  if (opt.silhouette) {
    const float w = opt.width > 0.0f ? opt.width : 0.0f;
    const float silOff = opt.raise + w;
    double elsum = 0.0;
    for (std::size_t f = 0; f < nTri; ++f) {
      const std::size_t a = static_cast<std::size_t>(fa[f]);
      const std::size_t b = static_cast<std::size_t>(fb[f]);
      const std::size_t c = static_cast<std::size_t>(fc[f]);
      elsum += length(vpos[b] - vpos[a]) + length(vpos[c] - vpos[b]) +
               length(vpos[a] - vpos[c]);
    }
    const float meanEdge = nTri ? static_cast<float>(elsum / (3.0 * nTri)) : 0.0f;
    // Eye-ward bias that lifts a silhouette off its grazing tangent shell. Kept for
    // --obj-edges (3D cylinder visibility); zeroed for the stroke/--edges pass
    // (opt.silhouetteCamBias==false) so its ray-cast QI starts on the TRUE surface
    // point (Freestyle center3d), not a point pulled toward the eye that floats in
    // front of its own occluder and wrongly votes visible.
    const float camBias =
        opt.silhouetteCamBias ? std::fmax(0.5f * w, 0.15f * meanEdge) : 0.0f;

    // 3a) SMOOTH silhouette: per face, connect the two n.v==0 zero-crossings.
    for (std::size_t f = 0; f < nTri; ++f) {
      const int cc[3] = {static_cast<int>(3 * f), static_cast<int>(3 * f + 1),
                         static_cast<int>(3 * f + 2)};
      const int wv[3] = {corner2v[static_cast<std::size_t>(cc[0])],
                         corner2v[static_cast<std::size_t>(cc[1])],
                         corner2v[static_cast<std::size_t>(cc[2])]};
      Vec3 P[3], NN[3];
      float d[3];
      for (int k = 0; k < 3; ++k) {
        P[k] = vpos[static_cast<std::size_t>(wv[k])];
        NN[k] = cnrm[static_cast<std::size_t>(cc[k])];
        d[k] = dot(NN[k], viewerDirAt(P[k], cam));
      }
      Vec3 cp[2], cn[2];
      int cid[2];
      // The other mesh face across the welded edge each crossing lands on (-1 if
      // the crossing is on a vertex or a border edge): the contour segment lives
      // inside face f, but on a smooth surface the adjacent face shares the same
      // grazing tangent, so it is an incident self-face too. Carried so the
      // visibility ray-cast excludes it (Freestyle adjacent-face exclusion).
      int cnb[2] = {-1, -1};
      int nc = 0;
      // A crossing on triangle edge (i,j) keys its synthetic node on the welded
      // edge (wv[i], wv[j]); a crossing landing on a vertex uses its welded id.
      auto addCross = [&](int i, int j) {
        const float t = d[i] / (d[i] - d[j]);
        cp[nc] = P[i] + (P[j] - P[i]) * t;
        cn[nc] = normalize(NN[i] + (NN[j] - NN[i]) * t);
        cid[nc] = crossingNodeId(wv[i], wv[j]);
        const auto it = emap.find(ekey(wv[i], wv[j]));
        if (it != emap.end()) {
          const EAdj& a = it->second;
          cnb[nc] = (a.f1 == static_cast<int>(f)) ? a.f2 : a.f1;
        }
        ++nc;
      };
      auto addVert = [&](int i) {
        cp[nc] = P[i];
        cn[nc] = NN[i];
        cid[nc] = wv[i];
        ++nc;
      };
      int nullIdx[3] = {-1, -1, -1};
      int nnull = 0;
      for (int k = 0; k < 3; ++k)
        if (d[k] == 0.0f) nullIdx[nnull++] = k;
      if (nnull == 0) {
        for (int e = 0; e < 3 && nc < 2; ++e)
          if (d[e] * d[(e + 1) % 3] < 0.0f) addCross(e, (e + 1) % 3);
      } else if (nnull == 1) {
        const int o0 = (nullIdx[0] + 1) % 3, o1 = (nullIdx[0] + 2) % 3;
        if (d[o0] * d[o1] < 0.0f) {
          addVert(nullIdx[0]);
          addCross(o0, o1);
        }
      } else if (nnull == 2) {
        addVert(nullIdx[0]);
        addVert(nullIdx[1]);
      }
      if (nc == 2) {
        const Vec3 v0 = viewerDirAt(cp[0], cam) * camBias;
        const Vec3 v1 = viewerDirAt(cp[1], cam) * camBias;
        // Incident faces: the contour segment lives inside face f (face0); the
        // two faces across the welded edges its endpoints crossed are adjacent
        // self-faces on the smooth surface (face1 = the first available one).
        const int adj = cnb[0] >= 0 ? cnb[0] : cnb[1];
        // Segment surface normal for cusp detection: the average of the two
        // crossing-point interpolated normals (Freestyle FEdgeSmooth::normal()).
        const Vec3 segN = normalize(cn[0] + cn[1]);
        emit(cid[0], cp[0] + cn[0] * silOff + v0, cid[1],
             cp[1] + cn[1] * silOff + v1, EdgeNature::Silhouette,
             mesh.groupForTri(f), static_cast<int>(f), adj, /*smooth=*/true, segN);
      }
    }

    // 3b) HARD-EDGE silhouette (face-normal straddle).
    for (const auto& kv : emap) {
      const EAdj& adj = kv.second;
      if (adj.f2 < 0) continue;  // border edge: handled in crease/border pass
      const std::size_t f1 = static_cast<std::size_t>(adj.f1);
      const std::size_t f2 = static_cast<std::size_t>(adj.f2);
      if (dot(fNg[f1], fNg[f2]) >= hardCos) continue;  // smooth edge: 3a covers it
      const std::size_t a = static_cast<std::size_t>(kv.first & 0xffffffffu);
      const std::size_t b = static_cast<std::size_t>(kv.first >> 32);
      const Vec3 da = viewerDirAt(vpos[a], cam);
      const Vec3 db = viewerDirAt(vpos[b], cam);
      const bool straddleA = dot(da, fNg[f1]) * dot(da, fNg[f2]) < 0.0f;
      const bool straddleB = dot(db, fNg[f1]) * dot(db, fNg[f2]) < 0.0f;
      if (!straddleA && !straddleB) continue;
      const Vec3 outw = normalize(fNg[f1] + fNg[f2]);
      emit(static_cast<int>(a), vpos[a] + outw * silOff + da * camBias,
           static_cast<int>(b), vpos[b] + outw * silOff + db * camBias,
           EdgeNature::Silhouette, mesh.groupForTri(adj.f1), adj.f1, adj.f2);
    }
  }

  // 4) CREASE (dihedral) + BORDER (single-face) on mesh edges.
  if (opt.crease || opt.border) {
    const float creaseCos = std::cos(opt.creaseAngleDeg * kPi / 180.0f);
    const bool smoothVeto = opt.meshCreaseSmoothVetoDeg > 0.0f;
    const float smoothCos = std::cos(opt.meshCreaseSmoothVetoDeg * kPi / 180.0f);
    const bool borderVeto = opt.meshBorderCoplanarVetoDeg > 0.0f;
    const float borderColinCos = std::cos(opt.meshBorderCoplanarVetoDeg * kPi / 180.0f);

    auto apexOf = [&](int f, std::size_t a, std::size_t b) -> Vec3 {
      const std::size_t x = static_cast<std::size_t>(fa[f]);
      const std::size_t y = static_cast<std::size_t>(fb[f]);
      const std::size_t z = static_cast<std::size_t>(fc[f]);
      if (x != a && x != b) return vpos[x];
      if (y != a && y != b) return vpos[y];
      return vpos[z];
    };

    // Per-corner SHADING normal of face f at welded vertex v -- the RAW vertex
    // normal the surface actually interpolates (mesh.normals), NOT the hard-edge
    // cluster normal. The smooth-shaded-across-edge crease veto compares these
    // across the edge: if face f1 and face f2 carry the same interpolated normal
    // at the two shared vertices, the surface is smooth-shaded across the edge
    // (a tessellation facet on a curved-but-smooth tube -- a cap ring or a tight
    // helix turn), not a real fold. Using the raw normals (not the cluster ones)
    // is what makes the veto robust at TIGHT curvature: there the facet dihedral
    // can exceed the hard-edge cluster split, yet the provided vertex normals
    // still agree, so the false tick is correctly suppressed. No provided normals
    // => fall back to the geometric face normal, so a genuine flat-shaded fold
    // (disagreeing face normals) still survives the veto.
    auto cornerNrmOf = [&](int f, std::size_t v) -> Vec3 {
      const int c0 = 3 * f;
      int c = c0;
      if (static_cast<std::size_t>(corner2v[static_cast<std::size_t>(c0 + 1)]) == v)
        c = c0 + 1;
      else if (static_cast<std::size_t>(corner2v[static_cast<std::size_t>(c0 + 2)]) == v)
        c = c0 + 2;
      if (!haveNrm) return fNg[static_cast<std::size_t>(f)];
      const Vec3 n = cornerNrm(static_cast<std::size_t>(c));
      const float l = length(n);
      return l > 1.0e-12f ? n * (1.0f / l) : fNg[static_cast<std::size_t>(f)];
    };

    std::unordered_map<int, std::vector<int>> borderAdj;
    if (opt.border && borderVeto) {
      borderAdj.reserve(nV);
      for (const auto& kv : emap) {
        if (kv.second.f2 >= 0) continue;
        const int a = static_cast<int>(kv.first & 0xffffffffu);
        const int b = static_cast<int>(kv.first >> 32);
        borderAdj[a].push_back(b);
        borderAdj[b].push_back(a);
      }
    }
    auto borderContinues = [&](std::size_t p, std::size_t other) -> bool {
      const auto it = borderAdj.find(static_cast<int>(p));
      if (it == borderAdj.end()) return false;
      const Vec3 in = normalize(vpos[p] - vpos[other]);
      for (const int q : it->second) {
        if (static_cast<std::size_t>(q) == other) continue;
        const Vec3 outDir = normalize(vpos[static_cast<std::size_t>(q)] - vpos[p]);
        if (dot(in, outDir) >= borderColinCos) return true;
      }
      return false;
    };

    struct CreaseCand {
      std::size_t a, b;
      std::uint16_t grp;
      int f0, f1;  // the two incident faces (crease is always interior)
    };
    std::vector<CreaseCand> creaseCand;
    std::vector<int> creaseDeg;
    const int maxDeg = opt.meshCreaseMaxDegree;
    if (maxDeg > 0) creaseDeg.assign(nV, 0);

    for (const auto& kv : emap) {
      const std::size_t a = static_cast<std::size_t>(kv.first & 0xffffffffu);
      const std::size_t b = static_cast<std::size_t>(kv.first >> 32);
      const EAdj& adj = kv.second;

      if (adj.f2 < 0) {
        if (!opt.border) continue;
        if (borderVeto && borderContinues(a, b) && borderContinues(b, a)) continue;
        pushEdge(a, b, EdgeNature::Border,
                 mesh.groupForTri(static_cast<std::size_t>(adj.f1)), adj.f1, -1);
        continue;
      }

      if (!opt.crease) continue;
      const std::size_t f1 = static_cast<std::size_t>(adj.f1);
      const std::size_t f2 = static_cast<std::size_t>(adj.f2);
      if (dot(fNg[f1], fNg[f2]) >= creaseCos) continue;

      // Smooth-facet crease veto (Freestyle fidelity). Suppress a "crease" where
      // the two faces are SMOOTH-SHADED across the edge: their interpolated
      // (cluster) vertex normals agree, so the edge is a tessellation facet on a
      // curved-but-smooth surface (a tube cap ring, a tight helix turn), not a
      // real fold. A genuine sharp fold (e.g. a beta-sheet box edge) splits the
      // hard-edge clusters, so its corner normals DISAGREE across the edge and it
      // survives. Comparing the two faces' own corner normals (not the global
      // welded average vs the geometric facet normals) is robust on tight
      // curvature, where the welded average can deviate from a facet by more than
      // the veto angle and let a false tick through.
      if (smoothVeto) {
        const Vec3 nA1 = cornerNrmOf(adj.f1, a), nA2 = cornerNrmOf(adj.f2, a);
        const Vec3 nB1 = cornerNrmOf(adj.f1, b), nB2 = cornerNrmOf(adj.f2, b);
        const float agree = std::fmin(dot(nA1, nA2), dot(nB1, nB2));
        if (agree >= smoothCos) continue;  // smooth-shaded across edge: no crease
      }

      if (opt.meshCreaseConvexOnly) {
        const Vec3 nAvg = normalize(fNg[f1] + fNg[f2]);
        const Vec3 edgeMid = (vpos[a] + vpos[b]) * 0.5f;
        const Vec3 apexMid =
            (apexOf(adj.f1, a, b) + apexOf(adj.f2, a, b)) * 0.5f;
        if (dot(nAvg, apexMid - edgeMid) > 0.0f) continue;
      }

      const std::uint16_t grp = mesh.groupForTri(static_cast<std::size_t>(adj.f1));
      if (maxDeg > 0) {
        creaseDeg[a]++;
        creaseDeg[b]++;
        creaseCand.push_back({a, b, grp, adj.f1, adj.f2});
      } else {
        pushEdge(a, b, EdgeNature::Crease, grp, adj.f1, adj.f2);
      }
    }
    for (const CreaseCand& c : creaseCand)
      if (creaseDeg[c.a] <= maxDeg && creaseDeg[c.b] <= maxDeg)
        pushEdge(c.a, c.b, EdgeNature::Crease, c.grp, c.f0, c.f1);
  }

  result.nodeCount = nextNode;
  return result;
}

ScreenProj makeScreenProj(const Camera& cam, int w, int h) {
  ScreenProj sp;
  sp.pos = cam.position;
  sp.dir = normalize(cam.direction);
  sp.right = normalize(cross(sp.dir, cam.up));
  sp.up = normalize(cross(sp.right, sp.dir));
  sp.ortho = cam.orthographic;
  const float aspect = static_cast<float>(w) / static_cast<float>(h);
  sp.halfH = cam.height * 0.5f;
  sp.halfW = sp.halfH * aspect;
  sp.persHalfH = std::tan(radians(cam.fovy) * 0.5f);
  sp.persHalfW = sp.persHalfH * aspect;
  sp.W = w;
  sp.H = h;
  return sp;
}

bool worldToScreen(const ScreenProj& sp, const Vec3& P, float& x, float& y,
                   float& vz) {
  const Vec3 d = P - sp.pos;
  const float zc = dot(d, sp.dir);
  float u, v;
  if (sp.ortho) {
    u = dot(d, sp.right) / sp.halfW;
    v = dot(d, sp.up) / sp.halfH;
    vz = zc;
  } else {
    if (zc <= 1.0e-6f) return false;  // at/behind the eye
    u = (dot(d, sp.right) / zc) / sp.persHalfW;
    v = (dot(d, sp.up) / zc) / sp.persHalfH;
    vz = zc;
  }
  x = (u + 1.0f) * 0.5f * static_cast<float>(sp.W) - 0.5f;  // top-left origin
  y = (1.0f - v) * 0.5f * static_cast<float>(sp.H) - 0.5f;
  return true;
}

}  // namespace umbreon
