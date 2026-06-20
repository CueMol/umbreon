#include "geom/mesh2_reader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "pov/pov_lexer.hpp"

namespace umbreon {
namespace {

using pov::binOp;
using pov::comp;
using pov::PovValue;
using pov::Tk;
using pov::Token;
using pov::tokenize;

// A texture identifier resolves to a representative color and (optionally) a
// finish block.
struct PovTexture {
  Vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
  Material finish;
  bool hasFinish = false;
};

float srgbToLinear(float c) {
  return (c <= 0.04045f) ? c / 12.92f
                         : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

// Hardcoded metals.inc finishes (F_MetalA..E). These are POV stdinc presets
// not present in the scene file, so they are resolved by name here.
// Fields in order: ambient, brilliance, diffuse, specular, roughness,
// reflection. All are metallic with phong 0.
bool lookupNamedFinish(const std::string& name, Material& out) {
  struct Entry {
    const char* name;
    float ambient, brilliance, diffuse, specular, roughness, reflection;
  };
  static const Entry kTable[] = {
      {"F_MetalA", 0.35f, 2.0f, 0.3f, 0.80f, 1.0f / 20.0f, 0.10f},
      {"F_MetalB", 0.30f, 3.0f, 0.4f, 0.70f, 1.0f / 60.0f, 0.25f},
      {"F_MetalC", 0.25f, 4.0f, 0.5f, 0.80f, 1.0f / 80.0f, 0.50f},
      {"F_MetalD", 0.15f, 5.0f, 0.6f, 0.80f, 1.0f / 100.0f, 0.65f},
      {"F_MetalE", 0.10f, 6.0f, 0.7f, 0.80f, 1.0f / 120.0f, 0.80f},
  };
  for (const Entry& e : kTable) {
    if (name == e.name) {
      out = Material{};
      out.ambient = e.ambient;
      out.brilliance = e.brilliance;
      out.diffuse = e.diffuse;
      out.specular = e.specular;
      out.roughness = e.roughness;
      out.reflection = e.reflection;
      out.metallic = true;
      out.phong = 0.0f;
      return true;
    }
  }
  return false;
}

// ==========================================================================
// Reader
// ==========================================================================
class Reader {
 public:
  Reader(std::vector<Token> toks, const SymbolTable& seed)
      : t_(std::move(toks)) {
    for (const auto& kv : seed) {
      PovValue v;
      v.n = std::min<int>(4, static_cast<int>(kv.second.size()));
      for (int i = 0; i < v.n; ++i) v.c[i] = kv.second[i];
      values_[kv.first] = v;
    }
  }

  SceneGeometry run() {
    while (!isEnd()) {
      if (isPunct("#")) {
        advance();  // consume '#'
        const std::string dir = curIsIdent() ? cur().s : std::string();
        if (dir == "macro") {
          skipMacro();
        } else if (dir == "declare" || dir == "local") {
          advance();  // consume directive keyword
          parseDeclare();
        } else if (dir == "if" || dir == "ifdef" || dir == "ifndef") {
          advance();  // consume the directive keyword
          enterIf();  // detect a "_show_<id>" section and push the #if frame
        } else if (dir == "end") {
          advance();
          leaveIf();  // pop the #if frame, restore the enclosing group
        } else if (!dir.empty()) {
          // #version / #else / #default / ... : skip the keyword and scan the
          // body normally so geometry nested inside it is still found.
          advance();
        }
      } else if (curIsIdent() && cur().s == "mesh2") {
        advance();
        parseMesh2();
      } else if (curIsIdent() && cur().s == "sphere" && peekPunct(1, "{")) {
        advance();
        parseSphere();
      } else if (curIsIdent() && cur().s == "cylinder" && peekPunct(1, "{")) {
        advance();
        parseCylinder();
      } else if (curIsIdent() &&
                 (cur().s == "edge_line" || cur().s == "edge_line2") &&
                 peekPunct(1, "(")) {
        const bool two = cur().s == "edge_line2";
        advance();
        parseEdgeLine(two);
      } else {
        advance();
      }
    }
    return finalize();
  }

 private:
  // ----- token cursor -----------------------------------------------------
  const Token& cur() const { return t_[pos_]; }
  bool isEnd() const { return t_[pos_].kind == Tk::End; }
  void advance() { if (!isEnd()) ++pos_; }
  bool curIsIdent() const { return cur().kind == Tk::Ident; }
  bool curIsNum() const { return cur().kind == Tk::Num; }
  bool isPunct(const char* p) const {
    return cur().kind == Tk::Punct && cur().s == p;
  }
  bool peekPunct(std::size_t ahead, const char* p) const {
    std::size_t k = pos_ + ahead;
    return k < t_.size() && t_[k].kind == Tk::Punct && t_[k].s == p;
  }

  // Index of the '}' matching the '{' at openIdx.
  std::size_t matchBrace(std::size_t openIdx) const {
    int depth = 0;
    for (std::size_t k = openIdx; k < t_.size(); ++k) {
      if (t_[k].kind == Tk::Punct && t_[k].s == "{") {
        ++depth;
      } else if (t_[k].kind == Tk::Punct && t_[k].s == "}") {
        if (--depth == 0) return k;
      } else if (t_[k].kind == Tk::End) {
        break;
      }
    }
    throw Mesh2ReadError("unbalanced '{' near line " +
                         std::to_string(t_[openIdx].line));
  }

  // ----- expression evaluator (advances pos_) -----------------------------
  PovValue evalPrimary() {
    if (isPunct("-")) { advance(); PovValue v = evalPrimary();
      for (int i = 0; i < v.n; ++i) v.c[i] = -v.c[i]; return v; }
    if (isPunct("+")) { advance(); return evalPrimary(); }
    if (isPunct("(")) {
      advance();
      PovValue v = evalSum();
      if (isPunct(")")) advance();
      return v;
    }
    if (isPunct("<")) {
      advance();
      PovValue v;
      while (!isPunct(">") && !isEnd() && v.n < 4) {
        PovValue e = evalSum();
        v.c[v.n++] = (e.n > 0) ? e.c[0] : 0.0;
        if (isPunct(",")) advance();
        else break;
      }
      if (isPunct(">")) advance();
      return v;
    }
    if (curIsNum()) {
      PovValue v; v.n = 1; v.c[0] = cur().num; advance(); return v;
    }
    if (curIsIdent()) {
      const std::string name = cur().s;
      advance();
      if (isPunct("(")) {
        // function call: not needed for CueMol mesh data -> skip args, yield 0
        int depth = 0;
        do {
          if (isPunct("(")) ++depth;
          else if (isPunct(")")) --depth;
          advance();
        } while (depth > 0 && !isEnd());
        return PovValue{};
      }
      auto it = values_.find(name);
      if (it != values_.end()) return it->second;
      return PovValue{};  // unknown identifier -> tolerant zero
    }
    return PovValue{};
  }

  PovValue evalTerm() {
    PovValue a = evalPrimary();
    while (isPunct("*") || isPunct("/")) {
      char op = cur().s[0];
      advance();
      a = binOp(a, evalPrimary(), op);
    }
    return a;
  }

  PovValue evalSum() {
    PovValue a = evalTerm();
    while (isPunct("+") || isPunct("-")) {
      char op = cur().s[0];
      advance();
      a = binOp(a, evalTerm(), op);
    }
    return a;
  }

  // ----- directive handling ----------------------------------------------
  // Skip an entire "#macro ... #end" definition (handles nested directives).
  void skipMacro() {
    advance();  // consume 'macro'
    int depth = 1;
    while (!isEnd() && depth > 0) {
      if (isPunct("#")) {
        advance();
        if (curIsIdent()) {
          const std::string& d = cur().s;
          if (d == "macro" || d == "if" || d == "ifdef" || d == "ifndef" ||
              d == "while" || d == "for" || d == "switch") {
            ++depth;
          } else if (d == "end") {
            --depth;
          }
          advance();
        }
      } else {
        advance();
      }
    }
  }

  // ----- section (transparency-group) tracking ---------------------------
  // CueMol wraps each rendering object's geometry in "#if (_show_<id>) ... #end".
  // Every primitive parsed inside such a block is tagged with that section's
  // group index, so the renderer composites one transparent layer PER section.
  void enterIf() {
    ifStack_.push_back(curGroup_);
    const std::string showVar = detectSectionVar();
    if (!showVar.empty()) curGroup_ = groupFor(showVar);
  }
  void leaveIf() {
    if (!ifStack_.empty()) {
      curGroup_ = ifStack_.back();
      ifStack_.pop_back();
    }
  }
  // Peek the just-entered #if condition for a "_show_*" identifier (e.g.
  // "#if (_show_34_35)"). Does NOT advance the cursor.
  std::string detectSectionVar() {
    if (!isPunct("(")) return std::string();
    int depth = 0;
    for (std::size_t k = pos_; k < t_.size(); ++k) {
      const Token& tk = t_[k];
      if (tk.kind == Tk::End) break;
      if (tk.kind == Tk::Punct && tk.s == "(") {
        ++depth;
      } else if (tk.kind == Tk::Punct && tk.s == ")") {
        if (--depth == 0) break;
      } else if (tk.kind == Tk::Ident && tk.s.rfind("_show", 0) == 0) {
        return tk.s;
      }
    }
    return std::string();
  }
  // Map a "_show_<id>" variable to a group index (created on first use). The
  // stored group name is the section id with the "_show" prefix stripped.
  int groupFor(const std::string& showVar) {
    std::string id =
        (showVar.size() > 5) ? showVar.substr(5) : showVar;  // strip "_show"
    auto it = groupIndex_.find(id);
    if (it != groupIndex_.end()) return it->second;
    int idx = static_cast<int>(groupNames_.size());
    groupNames_.push_back(id);
    groupIndex_[id] = idx;
    return idx;
  }

  // Cursor is at the identifier being declared.
  void parseDeclare() {
    if (!curIsIdent()) return;
    const std::string name = cur().s;
    advance();
    if (!isPunct("=")) return;
    advance();  // consume '='

    if (curIsIdent() && cur().s == "texture") {
      advance();
      if (!isPunct("{")) return;
      std::size_t open = pos_;
      std::size_t close = matchBrace(open);
      PovTexture tex = parseTextureBody(open + 1, close);
      textures_[name] = tex;
      adoptMaterial(tex);
      pos_ = close + 1;
      return;
    }
    if (curIsIdent() && (cur().s == "pigment" || cur().s == "finish" ||
                         cur().s == "normal" || cur().s == "material")) {
      advance();
      if (isPunct("{")) pos_ = matchBrace(pos_) + 1;
      return;
    }
    // numeric / vector expression terminated by ';'
    PovValue v = evalSum();
    if (v.n > 0) values_[name] = v;
    skipToSemicolon();
  }

  void skipToSemicolon() {
    int depth = 0;
    while (!isEnd()) {
      if (cur().kind == Tk::Punct) {
        const std::string& p = cur().s;
        if (p == "{" || p == "(" || p == "[") ++depth;
        else if (p == "}" || p == ")" || p == "]") { if (depth > 0) --depth; }
        else if (depth == 0 && p == ";") { advance(); return; }
        else if (depth == 0 && p == "#") return;  // next directive
      }
      advance();
    }
  }

  // ----- texture / pigment / finish --------------------------------------
  void adoptMaterial(const PovTexture& tex) {
    if (tex.hasFinish && !haveBlockMaterial_) {
      blockMaterial_ = tex.finish;
      haveBlockMaterial_ = true;
    }
  }

  // Parse the interior of a "texture { ... }" block (token range [begin,end)).
  PovTexture parseTextureBody(std::size_t begin, std::size_t end) {
    PovTexture tex;
    bool haveColor = false;
    std::string baseName;
    std::size_t k = begin;
    while (k < end) {
      const Token& tk = t_[k];
      if (tk.kind == Tk::Ident) {
        if (tk.s == "pigment" && k + 1 < end && t_[k + 1].kind == Tk::Punct &&
            t_[k + 1].s == "{") {
          std::size_t c = matchBrace(k + 1);
          Vec4 col;
          if (parsePigmentColor(k + 2, c, col)) {
            tex.color = col;
            haveColor = true;
          }
          k = c + 1;
          continue;
        }
        if (tk.s == "finish" && k + 1 < end && t_[k + 1].kind == Tk::Punct &&
            t_[k + 1].s == "{") {
          std::size_t c = matchBrace(k + 1);
          tex.finish = parseFinish(k + 2, c);
          tex.hasFinish = true;
          k = c + 1;
          continue;
        }
        if (baseName.empty() && !haveColor && tk.s != "pigment" &&
            tk.s != "finish") {
          baseName = tk.s;  // reference to a previously declared texture
        }
        ++k;
      } else if (tk.kind == Tk::Punct && tk.s == "{") {
        k = matchBrace(k) + 1;  // skip an unrelated nested block
      } else {
        ++k;
      }
    }
    if (!baseName.empty()) {
      auto it = textures_.find(baseName);
      if (it != textures_.end()) {
        if (!haveColor) tex.color = it->second.color;
        if (!tex.hasFinish && it->second.hasFinish) {
          tex.finish = it->second.finish;
          tex.hasFinish = true;
        }
      }
    }
    return tex;
  }

  // Find "color <kind> <expr>" inside a pigment block and evaluate the color.
  bool parsePigmentColor(std::size_t begin, std::size_t end, Vec4& out) {
    auto isColorKw = [](const std::string& s) {
      return s == "rgb" || s == "rgbf" || s == "rgbt" || s == "rgbft" ||
             s == "srgb" || s == "srgbf" || s == "srgbt" || s == "srgbft";
    };
    for (std::size_t k = begin; k < end; ++k) {
      const Token& tk = t_[k];
      if (tk.kind != Tk::Ident) continue;
      std::string kind;
      std::size_t exprAt = 0;
      if (tk.s == "color" || tk.s == "colour") {
        kind = "rgb";
        std::size_t p = k + 1;
        if (p < end && t_[p].kind == Tk::Ident && isColorKw(t_[p].s)) {
          kind = t_[p].s;
          ++p;
        }
        exprAt = p;
      } else if (isColorKw(tk.s)) {
        kind = tk.s;
        exprAt = k + 1;
      } else {
        continue;
      }
      pos_ = exprAt;
      PovValue v = evalSum();
      out = toColor(v, kind);
      return true;
    }
    return false;
  }

  static Vec4 toColor(const PovValue& v, const std::string& kind) {
    float r, g, b;
    if (v.n >= 3) {
      r = static_cast<float>(v.c[0]);
      g = static_cast<float>(v.c[1]);
      b = static_cast<float>(v.c[2]);
    } else if (v.n >= 1) {
      r = g = b = static_cast<float>(v.c[0]);
    } else {
      r = g = b = 1.0f;
    }
    bool transparency = kind == "rgbt" || kind == "rgbf" || kind == "rgbft" ||
                        kind == "srgbt" || kind == "srgbf" || kind == "srgbft";
    float opacity = 1.0f;
    if (transparency && v.n >= 4) {
      opacity = 1.0f - static_cast<float>(v.c[3]);
    }
    if (kind.rfind("srgb", 0) == 0) {
      r = srgbToLinear(r);
      g = srgbToLinear(g);
      b = srgbToLinear(b);
    }
    return Vec4{r, g, b, opacity};
  }

  Material parseFinish(std::size_t begin, std::size_t end) {
    Material m;
    std::size_t k = begin;
    while (k < end) {
      const Token& tk = t_[k];
      if (tk.kind == Tk::Punct && tk.s == "{") {
        k = matchBrace(k) + 1;  // skip nested block (e.g. reflection { ... })
        continue;
      }
      if (tk.kind == Tk::Ident &&
          (tk.s == "ambient" || tk.s == "diffuse" || tk.s == "specular" ||
           tk.s == "roughness" || tk.s == "brilliance" || tk.s == "phong" ||
           tk.s == "phong_size" || tk.s == "reflection" ||
           tk.s == "emission")) {
        pos_ = k + 1;
        PovValue v = evalSum();
        float f = (v.n > 0) ? static_cast<float>(v.c[0]) : 0.0f;
        if (tk.s == "ambient") m.ambient = f;
        else if (tk.s == "diffuse") m.diffuse = f;
        else if (tk.s == "specular") m.specular = f;
        else if (tk.s == "roughness") m.roughness = f;
        else if (tk.s == "brilliance") m.brilliance = f;
        else if (tk.s == "phong") m.phong = f;
        else if (tk.s == "phong_size") m.phongSize = f;
        else if (tk.s == "reflection") m.reflection = f;
        else m.emission = f;
        k = pos_;
        continue;
      }
      if (tk.kind == Tk::Ident && tk.s == "metallic") {
        m.metallic = true;
        // Optional amount: a value at or below 0 disables it.
        std::size_t n = k + 1;
        if (n < end && (t_[n].kind == Tk::Num ||
                        (t_[n].kind == Tk::Punct &&
                         (t_[n].s == "-" || t_[n].s == "+")))) {
          pos_ = n;
          PovValue v = evalSum();
          float f = (v.n > 0) ? static_cast<float>(v.c[0]) : 1.0f;
          m.metallic = f > 0.0f;
          k = pos_;
        } else {
          k = n;
        }
        continue;
      }
      // A bare identifier may be a named finish (e.g. F_MetalA). On a hit,
      // overwrite m with the preset so later keywords can still override.
      if (tk.kind == Tk::Ident) {
        Material named;
        if (lookupNamedFinish(tk.s, named)) {
          m = named;
          ++k;
          continue;
        }
      }
      ++k;
    }
    return m;
  }

  // ----- mesh2 ------------------------------------------------------------
  void parseMesh2() {
    if (!isPunct("{")) return;
    std::size_t open = pos_;
    std::size_t close = matchBrace(open);
    std::size_t k = open + 1;
    while (k < close) {
      const Token& tk = t_[k];
      if (tk.kind == Tk::Ident && k + 1 < close &&
          t_[k + 1].kind == Tk::Punct && t_[k + 1].s == "{") {
        std::size_t c = matchBrace(k + 1);
        if (tk.s == "vertex_vectors") {
          readVec3Array(k + 2, c, verts_);
        } else if (tk.s == "normal_vectors") {
          readVec3Array(k + 2, c, norms_);
        } else if (tk.s == "texture_list") {
          readTextureList(k + 2, c);
        } else if (tk.s == "face_indices") {
          readFaceIndices(k + 2, c);
        }
        // uv_vectors / normal_indices / uv_indices: intentionally ignored
        k = c + 1;
        continue;
      }
      if (tk.kind == Tk::Punct && tk.s == "{") {
        k = matchBrace(k) + 1;
        continue;
      }
      ++k;
    }
    haveMesh_ = true;
    deindexBlock();
    haveBlockMaterial_ = false;  // reset for the next mesh2 block
    pos_ = close + 1;
  }

  // ----- sphere / edge_line ----------------------------------------------
  // sphere { <center>, <radius> texture { ... pigment { color rgb COL } } }
  void parseSphere() {
    if (!isPunct("{")) return;
    std::size_t open = pos_;
    std::size_t close = matchBrace(open);
    pos_ = open + 1;
    PovValue center = evalSum();
    if (isPunct(",")) advance();
    PovValue radius = evalSum();
    Sphere s;
    s.center = toVec3(center);
    s.radius = static_cast<float>(comp(radius, 0));
    PovTexture tex = textureIn(pos_, close);
    s.color = tex.color;
    if (tex.hasFinish) s.material = tex.finish;  // else keep flatOutline
    s.group = static_cast<uint16_t>(curGroup_);
    if (s.radius > 0.0f) spheres_.push_back(s);
    pos_ = close + 1;
  }

  // cylinder { <p1>, <p2>, <radius> [open] texture { ... } }
  // CueMol emits these directly for the density-mesh wireframe (_39_40).
  void parseCylinder() {
    if (!isPunct("{")) return;
    std::size_t open = pos_;
    std::size_t close = matchBrace(open);
    pos_ = open + 1;
    PovValue p1 = evalSum();
    if (isPunct(",")) advance();
    PovValue p2 = evalSum();
    if (isPunct(",")) advance();
    PovValue radius = evalSum();
    Cylinder c;
    c.p0 = toVec3(p1);
    c.p1 = toVec3(p2);
    c.radius = static_cast<float>(comp(radius, 0));
    PovTexture tex = textureIn(pos_, close);
    c.color = tex.color;
    if (tex.hasFinish) c.material = tex.finish;  // else keep flatOutline
    c.group = static_cast<uint16_t>(curGroup_);
    if (c.radius > 0.0f) cylinders_.push_back(c);
    pos_ = close + 1;
  }

  // edge_line(v1, n1, v2, n2, raise, w, tex, col) and
  // edge_line2(v1, n1, a1, v2, n2, a2, raise, w, tex, col) both expand to a
  // cylinder from (v1 + raise*n1) to (v2 + raise*n2), radius w, color col.
  void parseEdgeLine(bool two) {
    std::vector<PovValue> a = parseCallArgs();
    const std::size_t need = two ? 10u : 8u;
    if (a.size() < need) return;
    const int iV1 = 0, iN1 = 1;
    const int iV2 = two ? 3 : 2, iN2 = two ? 4 : 3;
    const int iRaise = two ? 6 : 4, iW = two ? 7 : 5, iCol = two ? 9 : 7;
    double raise = comp(a[iRaise], 0);
    Cylinder c;
    c.p0 = toVec3(a[iV1]) + toVec3(a[iN1]) * static_cast<float>(raise);
    c.p1 = toVec3(a[iV2]) + toVec3(a[iN2]) * static_cast<float>(raise);
    c.radius = static_cast<float>(comp(a[iW], 0));
    c.color = toColor(a[iCol], "rgb");
    // edge_line2 carries per-endpoint transmit a1 (arg 2) and a2 (arg 5): the
    // POV macro pigments the segment "rgbt col+<0,0,0,a>" with a "gradient z"
    // from a1 at p0 to a2 at p1, so opacity = 1 - transmit varies linearly
    // along the segment. Store the p0 opacity in color.w and the p1 opacity in
    // opacity1; the renderer lerps by the axial hit fraction. This fades the
    // silhouette toward grazing edges exactly as POV does, and because adjacent
    // segments share endpoints with matching transmit (a2 == a1) the opacity is
    // continuous across joints (no step / seam).
    if (two) {
      const double t1 = comp(a[2], 0), t2 = comp(a[5], 0);
      float op0 = std::min(1.0f, std::max(0.0f, 1.0f - static_cast<float>(t1)));
      float op1 = std::min(1.0f, std::max(0.0f, 1.0f - static_cast<float>(t2)));
      c.color.w = op0;
      c.opacity1 = op1;
    }
    c.group = static_cast<uint16_t>(curGroup_);
    if (c.radius > 0.0f) cylinders_.push_back(c);
  }

  std::vector<PovValue> parseCallArgs() {
    std::vector<PovValue> args;
    if (!isPunct("(")) return args;
    advance();
    if (!isPunct(")")) {
      args.push_back(evalSum());
      while (isPunct(",")) { advance(); args.push_back(evalSum()); }
    }
    if (isPunct(")")) advance();
    return args;
  }

  static Vec3 toVec3(const PovValue& v) {
    return Vec3{static_cast<float>(comp(v, 0)), static_cast<float>(comp(v, 1)),
                static_cast<float>(comp(v, 2))};
  }

  // Find the first "texture { ... }" in [begin,end) and return the resolved
  // texture (color plus optional finish). If none is found, return a default
  // PovTexture with a black color and no finish.
  PovTexture textureIn(std::size_t begin, std::size_t end) {
    for (std::size_t k = begin; k < end; ++k) {
      if (t_[k].kind == Tk::Ident && t_[k].s == "texture" && k + 1 < end &&
          t_[k + 1].kind == Tk::Punct && t_[k + 1].s == "{") {
        std::size_t c = matchBrace(k + 1);
        return parseTextureBody(k + 2, c);
      }
    }
    PovTexture tex;
    tex.color = Vec4{0.0f, 0.0f, 0.0f, 1.0f};
    return tex;
  }

  float readSignedNumber() {
    float sign = 1.0f;
    while (isPunct("-") || isPunct("+")) {
      if (isPunct("-")) sign = -sign;
      advance();
    }
    if (curIsNum()) {
      float n = static_cast<float>(cur().num);
      advance();
      return sign * n;
    }
    return 0.0f;
  }

  Vec3 readVec3Literal() {
    Vec3 v;
    if (!isPunct("<")) return v;
    advance();
    float c[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
      c[i] = readSignedNumber();
      if (isPunct(",")) advance();
      else break;
    }
    while (!isPunct(">") && !isEnd()) advance();  // tolerate a 4th component
    if (isPunct(">")) advance();
    return Vec3{c[0], c[1], c[2]};
  }

  void readVec3Array(std::size_t begin, std::size_t end, std::vector<Vec3>& dst) {
    pos_ = begin;
    // optional leading element count
    if (curIsNum()) {
      std::size_t save = pos_;
      advance();
      if (isPunct(",")) advance();
      else pos_ = save;  // it was not a count
    }
    while (pos_ < end && !isEnd()) {
      if (isPunct("<")) {
        dst.push_back(readVec3Literal());
        if (isPunct(",")) advance();
      } else if (isPunct(",")) {
        advance();
      } else {
        break;
      }
    }
  }

  void readTextureList(std::size_t begin, std::size_t end) {
    std::size_t k = begin;
    if (k < end && t_[k].kind == Tk::Num) {
      ++k;
      if (k < end && t_[k].kind == Tk::Punct && t_[k].s == ",") ++k;
    }
    while (k < end) {
      if (t_[k].kind == Tk::Ident && t_[k].s == "texture" && k + 1 < end &&
          t_[k + 1].kind == Tk::Punct && t_[k + 1].s == "{") {
        std::size_t c = matchBrace(k + 1);
        PovTexture tex = parseTextureBody(k + 2, c);
        texColors_.push_back(tex.color);
        adoptMaterial(tex);
        k = c + 1;
        if (k < end && t_[k].kind == Tk::Punct && t_[k].s == ",") ++k;
        continue;
      }
      ++k;
    }
  }

  void readFaceIndices(std::size_t begin, std::size_t end) {
    pos_ = begin;
    if (curIsNum()) {  // optional leading face count
      advance();
      if (isPunct(",")) advance();
    }
    while (pos_ < end && !isEnd()) {
      if (isPunct("<")) {
        advance();
        Face f;
        f.v[0] = static_cast<int>(readSignedNumber());
        if (isPunct(",")) advance();
        f.v[1] = static_cast<int>(readSignedNumber());
        if (isPunct(",")) advance();
        f.v[2] = static_cast<int>(readSignedNumber());
        while (!isPunct(">") && !isEnd() && pos_ < end) advance();
        if (isPunct(">")) advance();
        // optional per-face or per-corner texture indices
        std::vector<int> tex;
        while (isPunct(",") && pos_ < end) {
          advance();
          if (curIsNum()) {
            tex.push_back(static_cast<int>(cur().num));
            advance();
          } else {
            break;  // reached '<' (next face) or '}'
          }
        }
        if (tex.size() == 1) {
          f.tex[0] = f.tex[1] = f.tex[2] = tex[0];
          f.hasTex = true;
        } else if (tex.size() >= 3) {
          f.tex[0] = tex[0];
          f.tex[1] = tex[1];
          f.tex[2] = tex[2];
          f.hasTex = true;
        }
        faces_.push_back(f);
      } else if (isPunct(",")) {
        advance();
      } else {
        break;
      }
    }
  }

  // ----- finalize ---------------------------------------------------------
  // De-index the current mesh2 block's faces and append the triangles to the
  // accumulated mesh, then clear the block buffers (a file may hold several
  // mesh2 blocks, each with its own 0-based vertex indices).
  void deindexBlock() {
    const std::size_t firstTri = mesh_.triangleCount();
    mesh_.positions.reserve(mesh_.positions.size() + faces_.size() * 3);
    mesh_.normals.reserve(mesh_.normals.size() + faces_.size() * 3);
    mesh_.colors.reserve(mesh_.colors.size() + faces_.size() * 3);

    for (const Face& f : faces_) {
      for (int k = 0; k < 3; ++k) {
        int vi = f.v[k];
        if (vi < 0 || static_cast<std::size_t>(vi) >= verts_.size()) {
          throw Mesh2ReadError("face vertex index out of range: " +
                               std::to_string(vi));
        }
        mesh_.positions.push_back(verts_[vi]);
        mesh_.normals.push_back(static_cast<std::size_t>(vi) < norms_.size()
                                    ? norms_[vi]
                                    : Vec3{0, 0, 0});
        Vec4 col{1.0f, 1.0f, 1.0f, 1.0f};
        if (f.hasTex && !texColors_.empty()) {
          int ti = f.tex[k];
          if (ti < 0) ti = 0;
          if (static_cast<std::size_t>(ti) >= texColors_.size()) {
            ti = static_cast<int>(texColors_.size()) - 1;
          }
          col = texColors_[ti];
        }
        mesh_.colors.push_back(col);
      }
    }

    // If this block carried no normals, derive geometric (flat) normals for it.
    if (norms_.empty()) {
      for (std::size_t ti = firstTri; ti < mesh_.triangleCount(); ++ti) {
        Vec3 a = mesh_.positions[3 * ti];
        Vec3 b = mesh_.positions[3 * ti + 1];
        Vec3 c = mesh_.positions[3 * ti + 2];
        Vec3 nn = normalize(cross(b - a, c - a));
        mesh_.normals[3 * ti] = nn;
        mesh_.normals[3 * ti + 1] = nn;
        mesh_.normals[3 * ti + 2] = nn;
      }
    }

    // Record per-block material: assign a material index to every triangle in
    // this block so the renderer can shade each mesh2 block independently.
    if (haveBlockMaterial_) {
      uint8_t idx = static_cast<uint8_t>(blockMaterials_.size());
      blockMaterials_.push_back(blockMaterial_);
      const std::size_t endTri = mesh_.triangleCount();
      mesh_.triMaterialId.resize(endTri, idx);
    } else {
      // No finish found for this block; pad with a default material.
      uint8_t idx = static_cast<uint8_t>(blockMaterials_.size());
      blockMaterials_.push_back(Material{});
      const std::size_t endTri = mesh_.triangleCount();
      mesh_.triMaterialId.resize(endTri, idx);
    }

    // Tag this block's triangles with the enclosing section (transparency
    // group). Several mesh2 blocks in one "#if (_show_*)" share a group.
    mesh_.triGroupId.resize(mesh_.triangleCount(),
                            static_cast<uint16_t>(curGroup_));

    verts_.clear();
    norms_.clear();
    texColors_.clear();
    faces_.clear();
  }

  SceneGeometry finalize() {
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

  struct Face {
    int v[3] = {0, 0, 0};
    int tex[3] = {0, 0, 0};
    bool hasTex = false;
  };

  std::vector<Token> t_;
  std::size_t pos_ = 0;
  std::map<std::string, PovValue> values_;
  std::map<std::string, PovTexture> textures_;
  std::vector<Vec3> verts_;
  std::vector<Vec3> norms_;
  std::vector<Vec4> texColors_;
  std::vector<Face> faces_;
  Mesh mesh_;
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

}  // namespace

SceneGeometry readGeometryFromString(const std::string& text,
                                     const SymbolTable& seedSymbols) {
  Reader reader(tokenize(text), seedSymbols);
  return reader.run();
}

SceneGeometry readGeometryFromFile(const std::string& path,
                                   const SymbolTable& seedSymbols) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw Mesh2ReadError("cannot open file: " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return readGeometryFromString(ss.str(), seedSymbols);
}

Mesh readMesh2FromString(const std::string& text) {
  return readGeometryFromString(text).mesh;
}

Mesh readMesh2FromFile(const std::string& path) {
  return readGeometryFromFile(path).mesh;
}

}  // namespace umbreon
