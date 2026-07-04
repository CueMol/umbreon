// bench INTERNAL header (not installed). Implementation class behind the
// mesh2_reader.hpp public API, shared by the mesh2_*.cpp translation units:
//   mesh2_reader.cpp        driver loop, directives, expression evaluator
//   mesh2_texture_parse.cpp texture / pigment / finish parsing
//   mesh2_block_parser.cpp  mesh2 / sphere / cylinder / edge_line parsing
//   mesh2_build.cpp         per-block weld into the accumulated indexed Mesh
#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "edges/mesh_weld.hpp"
#include "geom/mesh2_reader.hpp"
#include "pov/pov_lexer.hpp"

namespace umbreon {
namespace detail {

// A texture identifier resolves to a representative color and (optionally) a
// finish block.
struct PovTexture {
  Vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
  Material finish;
  bool hasFinish = false;
};

// ==========================================================================
// Mesh2Reader
// ==========================================================================
// One instance parses one tokenized CueMol .inc: run() scans the token stream
// for directives and geometry, accumulating the indexed mesh and the analytic
// primitives, and finalize() hands the result over. Parsing state (cursor,
// declared values, textures, section stack) lives on the instance.
class Mesh2Reader {
 public:
  Mesh2Reader(std::vector<pov::Token> toks, const SymbolTable& seed)
      : t_(std::move(toks)) {
    for (const auto& kv : seed) {
      pov::PovValue v;
      v.n = std::min<int>(4, static_cast<int>(kv.second.size()));
      for (int i = 0; i < v.n; ++i) v.c[i] = kv.second[i];
      values_[kv.first] = v;
    }
  }

  // Driver: scan the whole token stream, dispatching to the directive and
  // geometry parsers, then finalize. Defined in mesh2_reader.cpp.
  SceneGeometry run();

 private:
  // ----- token cursor -----------------------------------------------------
  const pov::Token& cur() const { return t_[pos_]; }
  bool isEnd() const { return t_[pos_].kind == pov::Tk::End; }
  void advance() { if (!isEnd()) ++pos_; }
  bool curIsIdent() const { return cur().kind == pov::Tk::Ident; }
  bool curIsNum() const { return cur().kind == pov::Tk::Num; }
  bool isPunct(const char* p) const {
    return cur().kind == pov::Tk::Punct && cur().s == p;
  }
  bool peekPunct(std::size_t ahead, const char* p) const {
    std::size_t k = pos_ + ahead;
    return k < t_.size() && t_[k].kind == pov::Tk::Punct && t_[k].s == p;
  }

  // Index of the '}' matching the '{' at openIdx.
  std::size_t matchBrace(std::size_t openIdx) const;

  // ----- expression evaluator (advances pos_) -----------------------------
  // Defined in mesh2_reader.cpp.
  pov::PovValue evalPrimary();
  pov::PovValue evalTerm();
  pov::PovValue evalSum();

  // ----- directive handling (mesh2_reader.cpp) ----------------------------
  void skipMacro();
  void enterIf();
  void leaveIf();
  std::string detectSectionVar();
  int groupFor(const std::string& showVar);
  void parseDeclare();
  void skipToSemicolon();

  // ----- texture / pigment / finish (mesh2_texture_parse.cpp) -------------
  void adoptMaterial(const PovTexture& tex);
  PovTexture parseTextureBody(std::size_t begin, std::size_t end);
  bool parsePigmentColor(std::size_t begin, std::size_t end, Vec4& out);
  static Vec4 toColor(const pov::PovValue& v, const std::string& kind);
  Material parseFinish(std::size_t begin, std::size_t end);
  PovTexture textureIn(std::size_t begin, std::size_t end);

  // ----- geometry blocks (mesh2_block_parser.cpp) --------------------------
  void parseMesh2();
  void parseSphere();
  bool hasOpenKeyword(std::size_t begin, std::size_t end) const;
  void parseCylinder();
  void parseEdgeLine(bool two);
  std::vector<pov::PovValue> parseCallArgs();
  static Vec3 toVec3(const pov::PovValue& v);
  float readSignedNumber();
  Vec3 readVec3Literal();
  void readVec3Array(std::size_t begin, std::size_t end,
                     std::vector<Vec3>& dst);
  void readTextureList(std::size_t begin, std::size_t end);
  void readFaceIndices(std::size_t begin, std::size_t end);

  // ----- finalize (mesh2_build.cpp) ----------------------------------------
  void buildIndexedBlock();
  SceneGeometry finalize();

  struct Face {
    int v[3] = {0, 0, 0};
    int tex[3] = {0, 0, 0};
    bool hasTex = false;
  };

  std::vector<pov::Token> t_;
  std::size_t pos_ = 0;
  std::map<std::string, pov::PovValue> values_;
  std::map<std::string, PovTexture> textures_;
  std::vector<Vec3> verts_;
  std::vector<Vec3> norms_;
  std::vector<Vec4> texColors_;
  std::vector<Face> faces_;
  Mesh mesh_;
  // Global position-class weld (across all mesh2 blocks); see buildIndexedBlock.
  std::unordered_map<WeldKey, int, WeldKeyHash> posClassMap_;
  int posClassCount_ = 0;
  std::vector<Sphere> spheres_;
  std::vector<Cylinder> cylinders_;
  Material blockMaterial_;
  std::vector<Material> blockMaterials_;
  bool haveBlockMaterial_ = false;
  bool haveMesh_ = false;

  // section / transparency-group tracking
  std::vector<std::string> groupNames_{std::string()};  // [0] = default group
  std::map<std::string, int> groupIndex_;
  int curGroup_ = 0;
  std::vector<int> ifStack_;
};

}  // namespace detail
}  // namespace umbreon
