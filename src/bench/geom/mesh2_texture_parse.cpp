// Mesh2Reader texture members: texture { ... } body parsing (pigment color,
// finish attributes, named-finish presets, references to declared textures).
// See mesh2_reader_impl.hpp for the class and the TU layout.
#include <cmath>
#include <string>

#include "geom/mesh2_reader_impl.hpp"

namespace umbreon {
namespace detail {
namespace {

using pov::PovValue;
using pov::Tk;
using pov::Token;

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

}  // namespace

void Mesh2Reader::adoptMaterial(const PovTexture& tex) {
  if (tex.hasFinish && !haveBlockMaterial_) {
    blockMaterial_ = tex.finish;
    haveBlockMaterial_ = true;
  }
}

// Parse the interior of a "texture { ... }" block (token range [begin,end)).
PovTexture Mesh2Reader::parseTextureBody(std::size_t begin, std::size_t end) {
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
bool Mesh2Reader::parsePigmentColor(std::size_t begin, std::size_t end,
                                    Vec4& out) {
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

Vec4 Mesh2Reader::toColor(const PovValue& v, const std::string& kind) {
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

Material Mesh2Reader::parseFinish(std::size_t begin, std::size_t end) {
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

// Find the first "texture { ... }" in [begin,end) and return the resolved
// texture (color plus optional finish). If none is found, return a default
// PovTexture with a black color and no finish.
PovTexture Mesh2Reader::textureIn(std::size_t begin, std::size_t end) {
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

}  // namespace detail
}  // namespace umbreon
