// Mesh2Reader geometry-block members: mesh2 { ... } (vertex/normal/texture
// lists, face_indices), sphere/cylinder primitives and the CueMol edge_line /
// edge_line2 macro calls. See mesh2_reader_impl.hpp for the class and the TU
// layout.
#include <algorithm>
#include <string>
#include <vector>

#include "geom/mesh2_reader_impl.hpp"

namespace umbreon {
namespace detail {

using pov::PovValue;
using pov::comp;
using pov::Tk;
using pov::Token;

void Mesh2Reader::parseMesh2() {
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
  buildIndexedBlock();
  haveBlockMaterial_ = false;  // reset for the next mesh2 block
  pos_ = close + 1;
}

// ----- sphere / cylinder / edge_line ----------------------------------------
// sphere { <center>, <radius> texture { ... pigment { color rgb COL } } }
void Mesh2Reader::parseSphere() {
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
  // Tag baked silhouette JOINT dots: CueMol's writePoint emits these as
  // sphere{...texture{<sec>_sl_tex ...}} to round the edge-line polyline joints
  // (parseEdgeLine handles the segment cylinders). The "<sec>_sl_tex" named
  // texture is the unambiguous marker, so the edge passes can drop them with
  // the baked cylinders instead of leaving black speckles.
  for (std::size_t k = open + 1; k < close; ++k) {
    const Token& tk = t_[k];
    if (tk.kind == Tk::Ident && tk.s.size() >= 7 &&
        tk.s.compare(tk.s.size() - 7, 7, "_sl_tex") == 0) {
      s.fromEdgeMacro = true;
      break;
    }
  }
  if (s.radius > 0.0f) spheres_.push_back(s);
  pos_ = close + 1;
}

// Detect the optional POV `open` keyword (capless cylinder) in the token range
// [begin, end). It appears at brace depth 0 after the radius and before any
// texture/pigment/finish/material block, so we scan only the top level and
// stop once a nested block begins.
bool Mesh2Reader::hasOpenKeyword(std::size_t begin, std::size_t end) const {
  int depth = 0;
  for (std::size_t k = begin; k < end; ++k) {
    const Token& tk = t_[k];
    if (tk.kind == Tk::Punct && tk.s == "{") {
      ++depth;
    } else if (tk.kind == Tk::Punct && tk.s == "}") {
      if (depth > 0) --depth;
    } else if (depth == 0 && tk.kind == Tk::Ident) {
      if (tk.s == "open") return true;
      if (tk.s == "texture" || tk.s == "pigment" || tk.s == "finish" ||
          tk.s == "normal" || tk.s == "material")
        break;  // nested block reached; `open` (if any) precedes it
    }
  }
  return false;
}

// cylinder { <p1>, <p2>, <radius> [open] texture { ... } }
// CueMol emits these directly for the density-mesh wireframe (_39_40).
void Mesh2Reader::parseCylinder() {
  if (!isPunct("{")) return;
  std::size_t open = pos_;
  std::size_t close = matchBrace(open);
  pos_ = open + 1;
  PovValue p1 = evalSum();
  if (isPunct(",")) advance();
  PovValue p2 = evalSum();
  if (isPunct(",")) advance();
  PovValue radius = evalSum();
  // Position just after the radius, before textureIn() advances pos_ into the
  // texture block: the optional `open` keyword (if any) lives in [here, close).
  const std::size_t afterRadius = pos_;
  Cylinder c;
  c.p0 = toVec3(p1);
  c.p1 = toVec3(p2);
  c.radius = static_cast<float>(comp(radius, 0));
  PovTexture tex = textureIn(pos_, close);
  c.color = tex.color;
  if (tex.hasFinish) c.material = tex.finish;  // else keep flatOutline
  c.group = static_cast<uint16_t>(curGroup_);
  // Raw cylinder{} bonds/wireframes are CLOSED (flat disk caps at p0/p1) unless
  // the optional `open` keyword (capless) is present after the radius. Capped
  // bonds are rendered as CONE_LINEAR_CURVE (flat caps); open ones join the
  // ROUND_LINEAR_CURVE edge path. (CueMol's raw cylinders are all CLOSED.)
  c.open = hasOpenKeyword(afterRadius, close);
  if (c.radius > 0.0f) cylinders_.push_back(c);
  pos_ = close + 1;
}

// edge_line(v1, n1, v2, n2, raise, w, tex, col) and
// edge_line2(v1, n1, a1, v2, n2, a2, raise, w, tex, col) both expand to a
// cylinder from (v1 + raise*n1) to (v2 + raise*n2), radius w, color col.
void Mesh2Reader::parseEdgeLine(bool two) {
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
  // POV silhouette edges expand to `open` (capless) cylinders; tag them so the
  // renderer routes them through the ROUND_LINEAR_CURVE chain (seam) path.
  c.open = true;
  // Mark this as a baked POV edge primitive (parseEdgeLine is the sole
  // producer). The screen-space edge pass drops these when --edges is on so the
  // generated edges do not double-draw against the baked ones; parseCylinder
  // never sets this, so a user's open black bond is preserved.
  c.fromEdgeMacro = true;
  if (c.radius > 0.0f) cylinders_.push_back(c);
}

std::vector<PovValue> Mesh2Reader::parseCallArgs() {
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

Vec3 Mesh2Reader::toVec3(const PovValue& v) {
  return Vec3{static_cast<float>(comp(v, 0)), static_cast<float>(comp(v, 1)),
              static_cast<float>(comp(v, 2))};
}

// ----- array readers ---------------------------------------------------------
float Mesh2Reader::readSignedNumber() {
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

Vec3 Mesh2Reader::readVec3Literal() {
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

void Mesh2Reader::readVec3Array(std::size_t begin, std::size_t end,
                                std::vector<Vec3>& dst) {
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

void Mesh2Reader::readTextureList(std::size_t begin, std::size_t end) {
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

void Mesh2Reader::readFaceIndices(std::size_t begin, std::size_t end) {
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

}  // namespace detail
}  // namespace umbreon
