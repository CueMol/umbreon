// Public API entry points and the Mesh2Reader driver: the token-stream scan
// loop, POV directive handling (#declare/#if section tracking/#macro skip) and
// the small expression evaluator. The texture, geometry-block and mesh-build
// members live in mesh2_texture_parse.cpp / mesh2_block_parser.cpp /
// mesh2_build.cpp (see mesh2_reader_impl.hpp).
#include "geom/mesh2_reader.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "geom/mesh2_reader_impl.hpp"

namespace umbreon {
namespace detail {

using pov::PovValue;
using pov::binOp;
using pov::Tk;
using pov::Token;

SceneGeometry Mesh2Reader::run() {
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

// Index of the '}' matching the '{' at openIdx.
std::size_t Mesh2Reader::matchBrace(std::size_t openIdx) const {
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

// ----- expression evaluator (advances pos_) --------------------------------
PovValue Mesh2Reader::evalPrimary() {
  if (isPunct("-")) {
    advance();
    PovValue v = evalPrimary();
    for (int i = 0; i < v.n; ++i) v.c[i] = -v.c[i];
    return v;
  }
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

PovValue Mesh2Reader::evalTerm() {
  PovValue a = evalPrimary();
  while (isPunct("*") || isPunct("/")) {
    char op = cur().s[0];
    advance();
    a = binOp(a, evalPrimary(), op);
  }
  return a;
}

PovValue Mesh2Reader::evalSum() {
  PovValue a = evalTerm();
  while (isPunct("+") || isPunct("-")) {
    char op = cur().s[0];
    advance();
    a = binOp(a, evalTerm(), op);
  }
  return a;
}

// ----- directive handling ---------------------------------------------------
// Skip an entire "#macro ... #end" definition (handles nested directives).
void Mesh2Reader::skipMacro() {
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

// ----- section (transparency-group) tracking --------------------------------
// CueMol wraps each rendering object's geometry in "#if (_show_<id>) ... #end".
// Every primitive parsed inside such a block is tagged with that section's
// group index, so the renderer composites one transparent layer PER section.
void Mesh2Reader::enterIf() {
  ifStack_.push_back(curGroup_);
  const std::string showVar = detectSectionVar();
  if (!showVar.empty()) curGroup_ = groupFor(showVar);
}

void Mesh2Reader::leaveIf() {
  if (!ifStack_.empty()) {
    curGroup_ = ifStack_.back();
    ifStack_.pop_back();
  }
}

// Peek the just-entered #if condition for a "_show_*" identifier (e.g.
// "#if (_show_34_35)"). Does NOT advance the cursor.
std::string Mesh2Reader::detectSectionVar() {
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
int Mesh2Reader::groupFor(const std::string& showVar) {
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
void Mesh2Reader::parseDeclare() {
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

void Mesh2Reader::skipToSemicolon() {
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

}  // namespace detail

// ----- public API -----------------------------------------------------------
SceneGeometry readGeometryFromString(const std::string& text,
                                     const SymbolTable& seedSymbols) {
  detail::Mesh2Reader reader(pov::tokenize(text), seedSymbols);
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
