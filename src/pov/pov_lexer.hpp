// Shared lexer and value type for the focused POV-Ray SDL readers.
//
// This is NOT a general POV-Ray parser. It provides a tokenizer and a small
// numeric/vector value type used by both the mesh2 geometry reader and the
// scene (camera/light/background) reader. Only the subset of SDL emitted by
// CueMol is handled.
#pragma once

#include <string>
#include <vector>

namespace umbreon {
namespace pov {

// --------------------------------------------------------------------------
// Tokenizer
// --------------------------------------------------------------------------
enum class Tk { Num, Ident, Str, Punct, End };

struct Token {
  Tk kind = Tk::End;
  double num = 0.0;
  std::string s;  // identifier text, string body, or single punctuation char
  int line = 1;
};

// Tokenize POV-Ray SDL text. Comments (// and /* */) are stripped. Numbers,
// identifiers, double-quoted strings and single-character punctuation are
// produced; the stream is terminated by a Tk::End token.
std::vector<Token> tokenize(const std::string& src);

// --------------------------------------------------------------------------
// Evaluated POV value: a scalar (n==1) or a 2..4 component vector.
// --------------------------------------------------------------------------
struct PovValue {
  double c[4] = {0, 0, 0, 0};
  int n = 0;
};

// Component accessor with scalar broadcast: a scalar acts as (s,s,s,s).
double comp(const PovValue& v, int i);

// Component-wise binary op with scalar/vector broadcast.
PovValue binOp(const PovValue& a, const PovValue& b, char op);

}  // namespace pov
}  // namespace umbreon
