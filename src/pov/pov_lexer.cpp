#include "pov/pov_lexer.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace umbreon {
namespace pov {

std::vector<Token> tokenize(const std::string& src) {
  std::vector<Token> out;
  const std::size_t n = src.size();
  std::size_t i = 0;
  int line = 1;
  auto identStart = [](unsigned char c) { return std::isalpha(c) || c == '_'; };
  auto identChar = [](unsigned char c) { return std::isalnum(c) || c == '_'; };

  while (i < n) {
    unsigned char c = static_cast<unsigned char>(src[i]);
    if (c == '\n') { ++line; ++i; continue; }
    if (std::isspace(c)) { ++i; continue; }

    // line comment
    if (c == '/' && i + 1 < n && src[i + 1] == '/') {
      while (i < n && src[i] != '\n') ++i;
      continue;
    }
    // block comment
    if (c == '/' && i + 1 < n && src[i + 1] == '*') {
      i += 2;
      while (i + 1 < n && !(src[i] == '*' && src[i + 1] == '/')) {
        if (src[i] == '\n') ++line;
        ++i;
      }
      i = (i + 1 < n) ? i + 2 : n;
      continue;
    }
    // string literal
    if (c == '"') {
      std::size_t j = i + 1;
      std::string str;
      while (j < n && src[j] != '"') {
        if (src[j] == '\n') ++line;
        str += src[j++];
      }
      out.push_back({Tk::Str, 0.0, str, line});
      i = (j < n) ? j + 1 : n;
      continue;
    }
    // number: digits, or '.' directly followed by a digit
    if (std::isdigit(c) ||
        (c == '.' && i + 1 < n && std::isdigit(static_cast<unsigned char>(src[i + 1])))) {
      std::size_t j = i;
      while (j < n && std::isdigit(static_cast<unsigned char>(src[j]))) ++j;
      if (j < n && src[j] == '.') {
        ++j;
        while (j < n && std::isdigit(static_cast<unsigned char>(src[j]))) ++j;
      }
      if (j < n && (src[j] == 'e' || src[j] == 'E')) {
        std::size_t k = j + 1;
        if (k < n && (src[k] == '+' || src[k] == '-')) ++k;
        if (k < n && std::isdigit(static_cast<unsigned char>(src[k]))) {
          j = k;
          while (j < n && std::isdigit(static_cast<unsigned char>(src[j]))) ++j;
        }
      }
      Token t;
      t.kind = Tk::Num;
      t.num = std::strtod(src.substr(i, j - i).c_str(), nullptr);
      t.line = line;
      out.push_back(t);
      i = j;
      continue;
    }
    // identifier / keyword
    if (identStart(c)) {
      std::size_t j = i;
      while (j < n && identChar(static_cast<unsigned char>(src[j]))) ++j;
      out.push_back({Tk::Ident, 0.0, src.substr(i, j - i), line});
      i = j;
      continue;
    }
    // single-character punctuation (covers '#', '<', '>', '{', '}', ',', ...)
    out.push_back({Tk::Punct, 0.0, std::string(1, src[i]), line});
    ++i;
  }
  out.push_back({Tk::End, 0.0, "", line});
  return out;
}

double comp(const PovValue& v, int i) {
  if (v.n == 0) return 0.0;
  if (v.n == 1) return v.c[0];
  return (i < v.n) ? v.c[i] : 0.0;
}

PovValue binOp(const PovValue& a, const PovValue& b, char op) {
  int n = (a.n <= 1 && b.n <= 1) ? 1 : std::max(std::max(a.n, b.n), 1);
  PovValue r;
  r.n = n;
  for (int i = 0; i < n; ++i) {
    double av = comp(a, i), bv = comp(b, i);
    switch (op) {
      case '+': r.c[i] = av + bv; break;
      case '-': r.c[i] = av - bv; break;
      case '*': r.c[i] = av * bv; break;
      case '/': r.c[i] = (bv != 0.0) ? av / bv : 0.0; break;
    }
  }
  return r;
}

}  // namespace pov
}  // namespace umbreon
