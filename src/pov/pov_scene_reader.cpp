#include "pov/pov_scene_reader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
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

Vec3 toVec3(const PovValue& v) {
  return Vec3{static_cast<float>(comp(v, 0)), static_cast<float>(comp(v, 1)),
              static_cast<float>(comp(v, 2))};
}

PovValue scalarVal(double x) {
  PovValue v;
  v.n = 1;
  v.c[0] = x;
  return v;
}

// One #if / #ifdef / #ifndef frame.
struct Cond {
  bool parentActive = true;  // active() of the enclosing scope at push time
  bool condTrue = true;      // value of the #if/#ifdef/#ifndef condition
  bool inElse = false;       // currently inside the #else branch
};

// ==========================================================================
// Scene reader: a single-cursor walk over the token stream that evaluates the
// declares/conditionals and captures the camera, lights and background.
// ==========================================================================
class SceneReader {
 public:
  SceneReader(std::vector<Token> toks, const PovParseOptions& opt)
      : t_(std::move(toks)) {
    sym_["image_width"] = scalarVal(opt.imageWidth);
    sym_["image_height"] = scalarVal(opt.imageHeight);
    for (const auto& kv : opt.predefined) sym_[kv.first] = scalarVal(kv.second);
  }

  PovSceneResult run() {
    while (!isEnd()) {
      if (inCamera_ && pos_ >= camEnd_) {
        finalizeCamera();
        inCamera_ = false;
        advance();  // consume the camera's closing '}'
        continue;
      }
      if (isPunct("#")) {
        advance();
        handleDirective();
      } else if (active() && curIsIdent()) {
        handleIdent();
      } else {
        advance();
      }
    }

    PovSceneResult r;
    r.camera = camera_;
    r.lights = lights_;
    r.background = background_;
    r.fog = fog_;
    r.assumedGamma = assumedGamma_;
    for (const auto& kv : sym_) {
      const PovValue& v = kv.second;
      std::vector<double> comps(v.c, v.c + std::max(1, v.n));
      r.declares.emplace(kv.first, std::move(comps));
    }
    r.includePath = includePath_;
    r.radiosity = sym_.count("_radiosity") > 0;
    return r;
  }

 private:
  // ----- token cursor -----------------------------------------------------
  const Token& cur() const { return t_[pos_]; }
  bool isEnd() const { return t_[pos_].kind == Tk::End; }
  void advance() { if (!isEnd()) ++pos_; }
  bool curIsIdent() const { return cur().kind == Tk::Ident; }
  bool curIsNum() const { return cur().kind == Tk::Num; }
  bool curIsStr() const { return cur().kind == Tk::Str; }
  bool isPunct(const char* p) const {
    return cur().kind == Tk::Punct && cur().s == p;
  }

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
    throw PovSceneError("unbalanced '{' near line " +
                        std::to_string(t_[openIdx].line));
  }

  // ----- conditional stack ------------------------------------------------
  bool active() const {
    if (stack_.empty()) return true;
    const Cond& t = stack_.back();
    bool branch = t.inElse ? !t.condTrue : t.condTrue;
    return t.parentActive && branch;
  }
  void pushCond(bool condTrue) {
    Cond c;
    c.parentActive = active();
    c.condTrue = condTrue;
    stack_.push_back(c);
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
      PovValue v = scalarVal(cur().num);
      advance();
      return v;
    }
    if (curIsIdent()) {
      const std::string name = cur().s;
      advance();
      if (isPunct("(")) {
        advance();
        std::vector<PovValue> args;
        if (!isPunct(")")) {
          args.push_back(evalSum());
          while (isPunct(",")) { advance(); args.push_back(evalSum()); }
        }
        if (isPunct(")")) advance();
        return applyFunc(name, args);
      }
      auto it = sym_.find(name);
      if (it != sym_.end()) return it->second;
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

  static PovValue applyFunc(const std::string& name,
                            const std::vector<PovValue>& a) {
    auto s = [&](int i) { return i < static_cast<int>(a.size()) ? comp(a[i], 0)
                                                                : 0.0; };
    if (name == "degrees") return scalarVal(s(0) * 57.295779513082323);
    if (name == "radians") return scalarVal(s(0) * 0.017453292519943295);
    if (name == "atan2") return scalarVal(std::atan2(s(0), s(1)));
    if (name == "atan") return scalarVal(std::atan(s(0)));
    if (name == "tan") return scalarVal(std::tan(s(0)));
    if (name == "sin") return scalarVal(std::sin(s(0)));
    if (name == "cos") return scalarVal(std::cos(s(0)));
    if (name == "sqrt") return scalarVal(std::sqrt(std::max(0.0, s(0))));
    if (name == "abs") return scalarVal(std::fabs(s(0)));
    if (name == "floor") return scalarVal(std::floor(s(0)));
    if (name == "int") return scalarVal(std::trunc(s(0)));
    if (name == "min") return scalarVal(std::min(s(0), s(1)));
    if (name == "max") return scalarVal(std::max(s(0), s(1)));
    if (name == "pow") return scalarVal(std::pow(s(0), s(1)));
    if (name == "vlength" && !a.empty()) {
      const PovValue& v = a[0];
      return scalarVal(std::sqrt(comp(v, 0) * comp(v, 0) +
                                 comp(v, 1) * comp(v, 1) +
                                 comp(v, 2) * comp(v, 2)));
    }
    if (name == "vnormalize" && !a.empty()) {
      const PovValue& v = a[0];
      double l = std::sqrt(comp(v, 0) * comp(v, 0) + comp(v, 1) * comp(v, 1) +
                           comp(v, 2) * comp(v, 2));
      PovValue r;
      r.n = 3;
      if (l > 0.0) for (int i = 0; i < 3; ++i) r.c[i] = comp(v, i) / l;
      return r;
    }
    if (name == "vdot" && a.size() >= 2) {
      return scalarVal(comp(a[0], 0) * comp(a[1], 0) +
                       comp(a[0], 1) * comp(a[1], 1) +
                       comp(a[0], 2) * comp(a[1], 2));
    }
    if (name == "vcross" && a.size() >= 2) {
      PovValue r;
      r.n = 3;
      r.c[0] = comp(a[0], 1) * comp(a[1], 2) - comp(a[0], 2) * comp(a[1], 1);
      r.c[1] = comp(a[0], 2) * comp(a[1], 0) - comp(a[0], 0) * comp(a[1], 2);
      r.c[2] = comp(a[0], 0) * comp(a[1], 1) - comp(a[0], 1) * comp(a[1], 0);
      return r;
    }
    return PovValue{};  // unknown function -> tolerant zero
  }

  double symScalar(const std::string& name, double dflt) const {
    auto it = sym_.find(name);
    return (it != sym_.end()) ? comp(it->second, 0) : dflt;
  }

  // ----- directives -------------------------------------------------------
  void handleDirective() {
    if (!curIsIdent()) return;
    const std::string dir = cur().s;
    if (dir == "macro") {
      skipMacro();
    } else if (dir == "declare" || dir == "local") {
      advance();
      parseDeclare();
    } else if (dir == "ifdef" || dir == "ifndef") {
      advance();
      std::string name = parseParenIdent();
      bool defined = sym_.count(name) > 0;
      pushCond(dir == "ifdef" ? defined : !defined);
    } else if (dir == "if" || dir == "while" || dir == "switch") {
      advance();
      PovValue v = (isPunct("(")) ? evalPrimary() : evalSum();
      pushCond(comp(v, 0) != 0.0);
    } else if (dir == "else" || dir == "elseif") {
      if (!stack_.empty()) stack_.back().inElse = true;
      advance();
      if (dir == "elseif" && isPunct("(")) evalPrimary();  // value unused
    } else if (dir == "end" || dir == "break") {
      if (dir == "end" && !stack_.empty()) stack_.pop_back();
      advance();
    } else if (dir == "version") {
      advance();
      skipToSemicolon();
    } else if (dir == "include") {
      advance();
      if (curIsStr()) advance();  // skip a standalone #include "header.inc"
    } else {
      advance();  // #default / #debug / #warning / ... : ignore the keyword
    }
  }

  // Parse "( NAME )" and return NAME (cursor left after ')').
  std::string parseParenIdent() {
    std::string name;
    if (isPunct("(")) advance();
    if (curIsIdent()) { name = cur().s; advance(); }
    if (isPunct(")")) advance();
    return name;
  }

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

  // Cursor is at the identifier being declared.
  void parseDeclare() {
    if (!curIsIdent()) return;
    const std::string name = cur().s;
    advance();
    if (!isPunct("=")) return;
    advance();  // consume '='

    // `#declare _scene = #include "geometry.inc"`
    if (isPunct("#")) {
      advance();
      if (curIsIdent() && cur().s == "include") {
        advance();
        if (curIsStr()) {
          if (active() && name == "_scene") includePath_ = cur().s;
          advance();
        }
      }
      return;
    }
    // texture / pigment / finish / material blocks: skip (geometry-side).
    if (curIsIdent() && (cur().s == "texture" || cur().s == "pigment" ||
                         cur().s == "finish" || cur().s == "normal" ||
                         cur().s == "material")) {
      advance();
      if (isPunct("{")) pos_ = matchBrace(pos_) + 1;
      return;
    }
    // numeric / vector expression terminated by ';'
    PovValue v = evalSum();
    if (active() && v.n > 0) sym_[name] = v;
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

  // ----- statements in an active scope ------------------------------------
  void handleIdent() {
    const std::string name = cur().s;

    if (inCamera_) { handleCameraKeyword(name); return; }

    if (name == "camera") {
      advance();
      if (isPunct("{")) {
        camEnd_ = matchBrace(pos_);
        advance();  // consume '{'
        beginCamera();
      }
      return;
    }
    if (name == "background") {
      advance();
      parseBackground();
      return;
    }
    if (name == "SpecLighting") {
      advance();
      parseSpecLighting(parseCallArgs());
      return;
    }
    if (name == "FlashLighting") {
      advance();
      parseFlashLighting(parseCallArgs());
      return;
    }
    if (name == "fog") {
      advance();
      parseFog();
      return;
    }
    if (name == "global_settings") {
      advance();
      parseGlobalSettings();
      return;
    }

    // Any other object/block (sphere, plane, object, light_source, ...) is not
    // needed for the viewing setup: skip it.
    advance();
    if (isPunct("{")) {
      pos_ = matchBrace(pos_) + 1;
    } else if (isPunct("(")) {
      skipParens();
    }
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

  void skipParens() {
    int depth = 0;
    do {
      if (isPunct("(")) ++depth;
      else if (isPunct(")")) --depth;
      advance();
    } while (depth > 0 && !isEnd());
  }

  // ----- camera -----------------------------------------------------------
  void beginCamera() {
    inCamera_ = true;
    camOrtho_ = camPersp_ = false;
    haveUp_ = haveRight_ = haveLoc_ = haveLookAt_ = haveAngle_ = false;
    camDir_ = PovValue{};
    camDir_.n = 3;
    camDir_.c[0] = 0; camDir_.c[1] = 0; camDir_.c[2] = -1;
  }

  void handleCameraKeyword(const std::string& name) {
    if (name == "orthographic") { camOrtho_ = true; advance(); return; }
    if (name == "perspective") { camPersp_ = true; advance(); return; }
    if (name == "direction") { advance(); camDir_ = evalSum(); return; }
    if (name == "up") { advance(); camUp_ = evalSum(); haveUp_ = true; return; }
    if (name == "right") { advance(); camRight_ = evalSum(); haveRight_ = true; return; }
    if (name == "location") { advance(); camLoc_ = evalSum(); haveLoc_ = true; return; }
    if (name == "look_at") { advance(); camLookAt_ = evalSum(); haveLookAt_ = true; return; }
    if (name == "sky") { advance(); evalSum(); return; }  // ignored
    if (name == "angle") { advance(); camAngle_ = comp(evalSum(), 0); haveAngle_ = true; return; }
    advance();  // any other camera keyword: ignore
  }

  void finalizeCamera() {
    Vec3 loc = haveLoc_ ? toVec3(camLoc_) : Vec3{0.0f, 0.0f, 1.0f};
    Vec3 dir = haveLookAt_ ? normalize(toVec3(camLookAt_) - loc)
                           : normalize(toVec3(camDir_));
    Vec3 up = haveUp_ ? normalize(toVec3(camUp_)) : Vec3{0.0f, 1.0f, 0.0f};

    camera_.position = loc;
    camera_.direction = dir;
    camera_.up = up;

    if (camOrtho_) {
      camera_.orthographic = true;
      // POV up vector spans the full vertical extent of the ortho view.
      camera_.height = haveUp_ ? length(toVec3(camUp_)) : 1.0f;
    } else {
      camera_.orthographic = false;
      // POV `angle` is the horizontal field of view for the `right` vector.
      // Derive the vertical fov from the up/right length ratio.
      double rl = haveRight_ ? length(toVec3(camRight_)) : 1.0;
      double ul = haveUp_ ? length(toVec3(camUp_)) : 1.0;
      double fovx = haveAngle_ ? camAngle_ : 40.0;
      double fovy = degrees(2.0f * static_cast<float>(std::atan(
          std::tan(radians(static_cast<float>(fovx)) * 0.5f) * (ul / rl))));
      camera_.fovy = static_cast<float>(fovy);
    }
  }

  // ----- background -------------------------------------------------------
  void parseBackground() {
    if (!isPunct("{")) return;
    std::size_t open = pos_;
    std::size_t close = matchBrace(open);
    auto isColorKw = [](const std::string& s) {
      return s == "rgb" || s == "rgbf" || s == "rgbt" || s == "rgbft" ||
             s == "srgb" || s == "srgbf" || s == "srgbt" || s == "srgbft";
    };
    for (std::size_t k = open + 1; k < close; ++k) {
      const Token& tk = t_[k];
      if (tk.kind != Tk::Ident) continue;
      std::size_t exprAt = 0;
      if (tk.s == "color" || tk.s == "colour") {
        std::size_t p = k + 1;
        if (p < close && t_[p].kind == Tk::Ident && isColorKw(t_[p].s)) ++p;
        exprAt = p;
      } else if (isColorKw(tk.s)) {
        exprAt = k + 1;
      } else {
        continue;
      }
      pos_ = exprAt;
      PovValue v = evalSum();
      background_ = toVec3(v);
      break;
    }
    pos_ = close + 1;
  }

  // ----- global_settings --------------------------------------------------
  // Only `assumed_gamma <number>` is needed; everything else is skipped.
  void parseGlobalSettings() {
    if (!isPunct("{")) return;
    std::size_t open = pos_;
    std::size_t close = matchBrace(open);
    for (std::size_t k = open + 1; k < close; ++k) {
      if (t_[k].kind == Tk::Ident && t_[k].s == "assumed_gamma") {
        pos_ = k + 1;
        assumedGamma_ = static_cast<float>(comp(evalSum(), 0));
        break;
      }
    }
    pos_ = close + 1;
  }

  // ----- fog --------------------------------------------------------------
  // fog { distance D  color rgb[f] <...>  fog_type N  fog_offset O
  //       fog_alt A  up <...> }
  void parseFog() {
    if (!isPunct("{")) return;
    std::size_t open = pos_;
    std::size_t close = matchBrace(open);
    auto isColorKw = [](const std::string& s) {
      return s == "rgb" || s == "rgbf" || s == "rgbt" || s == "rgbft" ||
             s == "srgb" || s == "srgbf" || s == "srgbt" || s == "srgbft";
    };
    fog_.enabled = true;
    pos_ = open + 1;
    while (pos_ < close && !isEnd()) {
      if (curIsIdent()) {
        const std::string key = cur().s;
        advance();
        if (key == "distance") {
          fog_.distance = static_cast<float>(comp(evalSum(), 0));
        } else if (key == "fog_type") {
          fog_.type = static_cast<int>(comp(evalSum(), 0));
        } else if (key == "fog_offset") {
          fog_.offset = static_cast<float>(comp(evalSum(), 0));
        } else if (key == "fog_alt") {
          fog_.alt = static_cast<float>(comp(evalSum(), 0));
        } else if (key == "up") {
          fog_.up = toVec3(evalSum());
        } else if (key == "color" || key == "colour") {
          if (curIsIdent() && isColorKw(cur().s)) advance();
          fog_.color = toVec3(evalSum());
        } else if (isColorKw(key)) {
          fog_.color = toVec3(evalSum());
        }
        // turbulence / fog_type-specific extras: ignored
      } else {
        advance();
      }
    }
    pos_ = close + 1;
  }

  // ----- lights -----------------------------------------------------------
  // SpecLighting(aLightSpread, aDist, aInten, aShadow): a key light placed at
  // vnormalize(<1,1,1>)*aDist*2 pointing at the origin. With aLightSpread<=1 it
  // is a parallel point light; larger values make it an area light (same
  // direction, softer shadows). Modeled as a distant light along -<1,1,1>.
  void parseSpecLighting(const std::vector<PovValue>& args) {
    if (args.size() < 3) return;
    double inten = comp(args[2], 0);
    DistantLight L;
    Vec3 from = normalize(Vec3{1.0f, 1.0f, 1.0f});
    L.direction = normalize(Vec3{0, 0, 0} - from);  // travel: source -> origin
    L.color = Vec3{1.0f, 1.0f, 1.0f};
    L.intensity = static_cast<float>(inten);
    lights_.push_back(L);
  }

  // FlashLighting(aInten): a headlight at the camera position pointing at the
  // origin. For an orthographic / parallel camera this is a parallel light
  // along the view direction; otherwise a point light at the eye, modeled here
  // as a distant light along (origin - eye).
  void parseFlashLighting(const std::vector<PovValue>& args) {
    if (args.empty()) return;
    double inten = comp(args[0], 0);
    double stereo = symScalar("_stereo", 0.0);
    double distance = symScalar("_distance", 200.0);
    double iod = symScalar("_iod", 0.03);
    Vec3 eye{static_cast<float>(stereo * distance * iod), 0.0f,
             static_cast<float>(distance)};
    DistantLight L;
    L.direction = normalize(Vec3{0, 0, 0} - eye);
    L.color = Vec3{1.0f, 1.0f, 1.0f};
    L.intensity = static_cast<float>(inten);
    lights_.push_back(L);
  }

  // ----- state ------------------------------------------------------------
  std::vector<Token> t_;
  std::size_t pos_ = 0;
  std::map<std::string, PovValue> sym_;
  std::vector<Cond> stack_;

  // camera accumulation
  bool inCamera_ = false;
  std::size_t camEnd_ = 0;
  bool camOrtho_ = false, camPersp_ = false;
  bool haveUp_ = false, haveRight_ = false, haveLoc_ = false;
  bool haveLookAt_ = false, haveAngle_ = false;
  PovValue camDir_, camUp_, camRight_, camLoc_, camLookAt_;
  double camAngle_ = 0.0;

  // results
  Camera camera_;
  std::vector<DistantLight> lights_;
  Vec3 background_{0.0f, 0.0f, 0.0f};
  Fog fog_;
  float assumedGamma_ = 1.0f;
  std::string includePath_;
};

}  // namespace

PovSceneResult readPovSceneFromString(const std::string& text,
                                      const PovParseOptions& opt) {
  SceneReader reader(tokenize(text), opt);
  return reader.run();
}

PovSceneResult readPovScene(const std::string& path,
                            const PovParseOptions& opt) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw PovSceneError("cannot open file: " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return readPovSceneFromString(ss.str(), opt);
}

}  // namespace umbreon
