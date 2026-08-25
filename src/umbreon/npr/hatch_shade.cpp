#include "npr/hatch_shade.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "npr/hatch_ink.hpp"
#include "postprocess/image_ops.hpp"

namespace umbreon {

namespace {

// Rec.709 display-luminance of a display-encoded triple (the contrast
// guarantee compares and moves DISPLAY values; see hatch_types.hpp).
inline float displayLuma(const float c[3]) {
  return 0.2126f * c[0] + 0.7152f * c[1] + 0.0722f * c[2];
}

// Contrast guarantee (ink side only -- moving the base would break the
// molecular color coding): ensure the ink sits at least inkMinContrast of
// display luminance BELOW the base. On a base too dark to allow that, lift
// the ink to base + contrast instead (bright hatching on dark ground,
// woodcut-like; a fixed rule, not an option). Hue-preserving scale, each
// channel clamped to [0,1].
inline void ensureInkContrast(float I[3], const float B[3],
                              float minContrast) {
  if (minContrast <= 0.0f) return;
  const float lb = displayLuma(B);
  const float li = displayLuma(I);
  if (lb - li >= minContrast) return;
  if (lb >= minContrast) {
    const float target = std::max(0.0f, lb - minContrast);
    const float s = (li > 1.0e-4f) ? target / li : 0.0f;
    for (int k = 0; k < 3; ++k)
      I[k] = std::min(1.0f, std::max(0.0f, I[k] * s));
  } else {
    const float target = std::min(1.0f, lb + minContrast);
    if (li > 1.0e-4f) {
      const float s = target / li;
      for (int k = 0; k < 3; ++k)
        I[k] = std::min(1.0f, std::max(0.0f, I[k] * s));
    } else {
      I[0] = I[1] = I[2] = target;
    }
  }
}

// The richardson look's tone recipe. Also the starting point of every mark
// preset's recipe (hatchPresetTone) until they are tuned individually: the
// default ToneRecipe (no wrap, no rim) leaves a flash-lit molecular scene
// almost entirely near tone 1, so nothing but the limb gets hatched.
ToneRecipe richardsonTone() {
  ToneRecipe t;
  // Lighting-only tone: a low ambient floor keeps the shadows readable,
  // the compressed white point opens the lit side up to bare paper, and
  // the specular cut punches the highlight through as pure paper.
  t.diffuseWeight = 0.85f;
  t.ambient = 0.05f;
  // A drawing does not reproduce a hard terminator, and a hand-drawn
  // figure is lit flatly from the front: wrap softens the terminator and
  // keeps a frontal key from clipping the tone flat, while the rim term
  // supplies the shading that follows the FORM (darkening toward each
  // silhouette) rather than one light direction.
  t.wrap = 0.5f;
  // The contour darkening must stay a BAND at the silhouette: a low
  // rimPower reaches deep into the interior (at 60 deg off-axis it
  // already cuts the tone by a quarter), dropping most of a rounded
  // form under the darker-pencil thresholds and blackening the whole
  // object. The high exponent keeps faces within ~30 deg of grazing
  // dark and leaves everything else at the object's own tone.
  t.rimDarken = 1.0f;      // strong contour shading
  t.rimPower = 3.5f;       // confined to the silhouette band
  t.rimLightBias = 0.35f;  // biased to each form's shaded side
  t.contactAoPow = 1.0f;
  t.shapeAoPow = 0.6f;
  // Do NOT clip the lit end: whitePoint below 1 turns every gently lit
  // face into bare paper, so the highlights spread into large white
  // holes. Opening it past 1 keeps the light side as sparse strokes and
  // leaves the paper for the true highlights only; the higher gamma
  // restores the dark end that the wider range would otherwise lift.
  t.whitePoint = 1.2f;
  t.gamma = 2.4f;
  // The specular blow-out is off by default for the same reason: on a
  // broad-lobe finish it paints a large white patch rather than a
  // highlight. Raise it per scene if a crisp glint is wanted.
  t.specularCut = 0.0f;
  // Highlights: a narrow band at the top of the range goes to EXACT
  // paper white. The knee sets where that band starts, so the white is
  // clean without the highlight spreading (which is what lowering
  // whitePoint would do).
  t.highlightAt = 0.86f;
  t.highlightSoft = 0.05f;
  return t;
}

// ---- spec text helpers ----------------------------------------------------

std::string trimWs(const std::string& v) {
  std::size_t a = 0, b = v.size();
  while (a < b && std::isspace(static_cast<unsigned char>(v[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(v[b - 1]))) --b;
  return v.substr(a, b - a);
}

// Locale-independent number parsing: the whole (trimmed) string must be
// consumed, so "1.5x" or "" are rejected.
bool parseNum(const std::string& v, float& out) {
  std::istringstream in(trimWs(v));
  in.imbue(std::locale::classic());
  float f = 0.0f;
  if (!(in >> f)) return false;
  if (!(in >> std::ws).eof()) return false;
  out = f;
  return true;
}

bool parseIntNum(const std::string& v, int& out) {
  std::istringstream in(trimWs(v));
  in.imbue(std::locale::classic());
  int i = 0;
  if (!(in >> i)) return false;
  if (!(in >> std::ws).eof()) return false;
  out = i;
  return true;
}

bool parseOnOff(const std::string& v, bool& out) {
  const std::string t = trimWs(v);
  if (t == "on" || t == "1" || t == "true") {
    out = true;
    return true;
  }
  if (t == "off" || t == "0" || t == "false") {
    out = false;
    return true;
  }
  return false;
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parseColor(const std::string& v, float out[3]) {
  const std::string t = trimWs(v);
  if (t.size() != 7 || t[0] != '#') return false;
  for (int k = 0; k < 3; ++k) {
    const int hi = hexNibble(t[1 + 2 * k]);
    const int lo = hexNibble(t[2 + 2 * k]);
    if (hi < 0 || lo < 0) return false;
    out[k] = static_cast<float>(hi * 16 + lo) / 255.0f;
  }
  return true;
}

std::string fmtNum(float f) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(6) << f;
  return out.str();
}

std::string fmtInt(int i) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << i;
  return out.str();
}

std::string fmtOnOff(bool b) { return b ? "on" : "off"; }

std::string fmtColor(const float c[3]) {
  static const char* kHex = "0123456789abcdef";
  std::string out = "#";
  for (int k = 0; k < 3; ++k) {
    const int v = static_cast<int>(std::lround(
        std::min(1.0f, std::max(0.0f, c[k])) * 255.0f));
    out += kHex[(v >> 4) & 15];
    out += kHex[v & 15];
  }
  return out;
}

// One "k=v" entry appended to a spec line.
void putKv(std::string& line, const char* key, const std::string& value) {
  if (!line.empty()) line += ',';
  line += key;
  line += '=';
  line += value;
}

// Split a spec text into lines: '\n' and ';' both separate, '\r' is
// dropped.
std::vector<std::string> splitSpecLines(const std::string& text) {
  std::vector<std::string> lines;
  std::string cur;
  for (char c : text) {
    if (c == '\n' || c == ';') {
      lines.push_back(cur);
      cur.clear();
    } else if (c != '\r') {
      cur += c;
    }
  }
  lines.push_back(cur);
  return lines;
}

}  // namespace

bool applyHatchPreset(HatchOptions& opt, const std::string& name) {
  if (name == "pen-cross") {
    // 3 hard crosshatch layers with staggered thresholds: the 45-deg layer
    // carries the light tones, -45 deg joins in the midtones (crosshatch),
    // the horizontal layer only in the deep shadows. No perturbation, so
    // the look is seed-independent. Values: plan section 6.5.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Line;
    l.spacingPx = 10.0f;
    l.subdiv = 2;
    l.widthPx = 1.1f;
    l.fadeInv = 0.0f;              // auto: continuous width growth
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.angleDeg = 45.0f;
    l.toneHi = 0.95f;
    l.toneLo = 0.55f;
    opt.layers.push_back(l);
    l.angleDeg = -45.0f;
    l.toneHi = 0.62f;
    l.toneLo = 0.30f;
    opt.layers.push_back(l);
    l.angleDeg = 0.0f;
    l.toneHi = 0.32f;
    l.toneLo = 0.10f;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "pencil") {
    // Three PENCILS (layers) of the same hue, each darker than the last,
    // built from INDIVIDUAL strokes: per-line position scatter off the
    // lattice, per-stroke length / pressure / angle scatter, wobble,
    // tapered ends and paper tooth. The light stick lays the midtone wash,
    // the darker sticks cross in as the tone deepens -- hand shading
    // reaches its darks by SWITCHING pencils, not only by pressing harder.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Line;
    l.spacingPx = 9.0f;
    l.subdiv = 2;
    l.widthPx = 3.6f;              // peak width; the stroke envelope thins it
    l.fadeInv = 0.0f;              // auto: continuous width growth
    l.opacity = 0.85f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.8f;
    l.mark.toothAmp = 0.15f;
    l.mark.toothScalePx = 3.0f;
    l.mark.jitter = 0.22f;         // per-line position scatter
    l.mark.wobbleAmpPx = 1.2f;
    l.mark.wobbleWavePx = 60.0f;
    l.mark.widthJitter = 0.45f;    // pressure scatter + belly swell
    l.mark.strokeLenPx = 50.0f;
    l.mark.strokeGapPx = 5.0f;
    l.mark.strokeTaper = 0.35f;    // asymmetric entry/tail scale
    l.mark.angleJitterDeg = 5.0f;  // coherent drift + per-stroke scatter
    l.mark.strokeLenJitter = 0.5f;
    l.angleDeg = 55.0f;            // light stick: midtone wash
    l.toneHi = 0.92f;
    l.toneLo = 0.55f;
    l.inkScale = 1.0f;
    opt.layers.push_back(l);
    l.angleDeg = -35.0f;           // darker stick crosses in
    l.toneHi = 0.62f;
    l.toneLo = 0.30f;
    l.inkScale = 0.62f;
    l.mark.seed = 1;               // decorrelate the layers' strokes
    opt.layers.push_back(l);
    l.angleDeg = 80.0f;            // darkest stick: shadow cores
    l.toneHi = 0.34f;
    l.toneLo = 0.12f;
    l.inkScale = 0.38f;
    l.mark.seed = 2;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "engraving") {
    // One direction, deep subdivision: tone is carried by line insertion
    // and width modulation alone (copperplate look).
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Line;
    l.angleDeg = 0.0f;
    l.spacingPx = 16.0f;
    l.subdiv = 3;
    l.widthPx = 2.4f;
    l.toneHi = 0.97f;
    l.toneLo = 0.12f;
    l.fadeInv = 0.0f;              // auto: continuous width growth
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.mark.widthJitter = 0.5f;
    l.mark.wobbleAmpPx = 0.5f;
    l.mark.wobbleWavePx = 64.0f;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "stipple") {
    // Stochastic stippling: fixed-size dots on a jittered lattice, the tone
    // carried by how many cells are inked (per-cell hashed thresholds over
    // the whole tone range), so the mid tones track the shading instead of
    // waiting for nesting levels to fill in. Initial values; tuned by eye.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Stipple;
    l.angleDeg = 0.0f;
    l.spacingPx = 3.0f;
    l.subdiv = 0;
    l.dotScale = 0.9f;
    l.toneHi = 1.0f;
    l.toneLo = 0.0f;
    l.fadeInv = 32.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.6f;
    l.mark.shapeExponent = 2.0f;
    l.mark.jitter = 0.45f;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "screentone-60") {
    // Classic AM halftone screen at 45 deg (K = 0: every dot present, the
    // radius alone carries the tone; ~60 lpi at a 300 dpi print figure).
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Dot;
    l.angleDeg = 45.0f;
    l.spacingPx = 5.0f;
    l.subdiv = 0;
    l.toneHi = 1.0f;
    l.toneLo = 1.0f;
    l.fadeInv = 32.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.mark.shapeExponent = 2.0f;
    l.mark.jitter = 0.0f;
    l.mark.invertAbove50 = true;
    opt.layers.push_back(l);
    return true;
  }
  if (name == "manga-square") {
    // Square-element screen (L-inf marks) at 45 deg, coarse enough to read
    // at full zoom.
    opt.layers.clear();
    HatchLayer l;
    l.kind = LayerKind::Dot;
    l.angleDeg = 45.0f;
    l.spacingPx = 6.0f;
    l.subdiv = 0;
    l.toneHi = 1.0f;
    l.toneLo = 1.0f;
    l.fadeInv = 32.0f;
    l.opacity = 1.0f;
    l.mark = MarkStyle{};
    l.mark.edgeSoftness = 0.5f;
    l.mark.shapeExponent = 16.0f;
    l.mark.jitter = 0.0f;
    l.mark.invertAbove50 = true;
    opt.layers.push_back(l);
    return true;
  }
  return false;
}

bool applyHatchLook(HatchOptions& opt, const std::string& name) {
  if (name == "richardson") {
    // Jane-Richardson-style colored-pencil ribbon drawing: NO flat fill --
    // the paper carries the highlights, the midtones are stroke density
    // and the darks are denser strokes of a darker pencil of the same hue
    // (the object's own color). Tuned against the published TIM drawings.
    applyHatchPreset(opt, "pencil");
    opt.mode = HatchMode::Ink;
    opt.base = HatchBase::Paper;     // no flat fill: bare paper between marks
    opt.ink = HatchInk::FromAlbedo;  // each section draws in its own color
    opt.paperColor[0] = 0.941f;      // warm drawing paper
    opt.paperColor[1] = 0.925f;
    opt.paperColor[2] = 0.867f;
    opt.inkMinContrast = 0.15f;
    opt.inkShadeDark = 0.40f;  // pencil pressure: shadows darken the stick
    opt.toneFog = true;        // far side fades into the paper
    opt.tone = richardsonTone();
    // Fine strokes: with the default supersampled ink these are OUTPUT
    // pixels, so ss decides how far below one pixel they actually land.
    for (HatchLayer& l : opt.layers) {
      l.spacingPx = 0.5f;   // dense drawing grain (OUTPUT px; see hatch-res)
      l.widthPx = 0.45f;
      l.mark.edgeSoftness = 0.55f;
      l.opacity = 1.0f;
      // The look was tuned with the pencil preset's original fade; it keeps
      // that (not the preset's auto fade) so the drawing stays as tuned.
      l.fadeInv = 10.0f;
    }
    // Colored pencils read darker than graphite at the same inkScale (the
    // composite multiplies in linear light), so the midtone stick is
    // lighter here than in the base pencil preset; the shadow stick keeps
    // its depth for the silhouette band.
    if (opt.layers.size() >= 2) opt.layers[1].inkScale = 0.74f;
    return true;
  }
  if (name == "ink-cross") {
    // The plain pen-and-ink look: white paper, black crosshatch.
    applyHatchPreset(opt, "pen-cross");
    opt.mode = HatchMode::Ink;
    opt.base = HatchBase::Paper;
    opt.ink = HatchInk::Fixed;
    for (int k = 0; k < 3; ++k) {
      opt.inkColor[k] = 0.0f;
      opt.paperColor[k] = 1.0f;
    }
    opt.inkMinContrast = 0.25f;
    opt.inkShadeDark = 1.0f;
    hatchPresetTone("pen-cross", opt.tone);
    return true;
  }
  if (name == "manga") {
    // Flat section fill under a halftone screen, hard black ink.
    applyHatchPreset(opt, "screentone-60");
    opt.mode = HatchMode::Ink;
    opt.base = HatchBase::Albedo;  // flat fill, unshaded
    opt.ink = HatchInk::Fixed;
    for (int k = 0; k < 3; ++k) {
      opt.inkColor[k] = 0.0f;
      opt.paperColor[k] = 1.0f;
    }
    opt.albedoQuantize = 4;
    opt.inkMinContrast = 0.35f;
    opt.inkShadeDark = 1.0f;
    hatchPresetTone("screentone-60", opt.tone);
    return true;
  }
  return false;
}

bool hatchPresetTone(const std::string& presetName, ToneRecipe& out) {
  // One recipe per mark kind is the intent; every entry starts from the
  // richardson recipe (form-following contour shading, compressed white
  // point) and is tuned by eye per preset.
  // TODO(phase4): tune per mark kind against rendered figures.
  if (presetName == "pen-cross" || presetName == "pencil" ||
      presetName == "engraving" || presetName == "stipple" ||
      presetName == "screentone-60" || presetName == "manga-square") {
    out = richardsonTone();
    return true;
  }
  return false;
}

bool applyHatchStyle(HatchOptions& opt, const std::string& name) {
  if (applyHatchLook(opt, name)) return true;
  if (!applyHatchPreset(opt, name)) return false;
  hatchPresetTone(name, opt.tone);
  return true;
}

bool applyHatchLayerKv(HatchLayer& l, const std::string& key,
                       const std::string& value) {
  float f = 0.0f;
  int i = 0;
  bool b = false;
  if (key == "kind") {
    const std::string v = trimWs(value);
    if (v == "line")
      l.kind = LayerKind::Line;
    else if (v == "dot")
      l.kind = LayerKind::Dot;
    else if (v == "stipple")
      l.kind = LayerKind::Stipple;
    else
      return false;
    return true;
  }
  if (key == "subdiv") {
    if (!parseIntNum(value, i)) return false;
    l.subdiv = i;
    return true;
  }
  if (key == "seed") {
    if (!parseIntNum(value, i) || i < 0) return false;
    l.mark.seed = static_cast<unsigned>(i);
    return true;
  }
  if (key == "invert") {
    if (!parseOnOff(value, b)) return false;
    l.mark.invertAbove50 = b;
    return true;
  }
  if (!parseNum(value, f)) return false;
  if (key == "angle") l.angleDeg = f;
  else if (key == "spacing") l.spacingPx = f;
  else if (key == "width") l.widthPx = f;
  else if (key == "dotscale") l.dotScale = f;
  else if (key == "tonehi") l.toneHi = f;
  else if (key == "tonelo") l.toneLo = f;
  else if (key == "fade") l.fadeInv = f;
  else if (key == "opacity") l.opacity = f;
  else if (key == "inkscale") l.inkScale = f;
  else if (key == "soft") l.mark.edgeSoftness = f;
  else if (key == "shape") l.mark.shapeExponent = f;
  else if (key == "aspect") l.mark.dotAspect = f;
  else if (key == "dotangle") l.mark.dotAngleDeg = f;
  else if (key == "jitter") l.mark.jitter = f;
  else if (key == "wobble") l.mark.wobbleAmpPx = f;
  else if (key == "wobwave") l.mark.wobbleWavePx = f;
  else if (key == "wjitter") l.mark.widthJitter = f;
  else if (key == "slen") l.mark.strokeLenPx = f;
  else if (key == "sgap") l.mark.strokeGapPx = f;
  else if (key == "taper") l.mark.strokeTaper = f;
  else if (key == "anglejitter") l.mark.angleJitterDeg = f;
  else if (key == "lenjitter") l.mark.strokeLenJitter = f;
  else if (key == "tooth") l.mark.toothAmp = f;
  else if (key == "toothscale") l.mark.toothScalePx = f;
  else return false;
  return true;
}

bool applyHatchToneKv(ToneRecipe& t, const std::string& key,
                      const std::string& value) {
  float f = 0.0f;
  if (!parseNum(value, f)) return false;
  if (key == "diffuse") t.diffuseWeight = f;
  else if (key == "ambient") t.ambient = f;
  else if (key == "wrap") t.wrap = f;
  else if (key == "rim") t.rimDarken = f;
  else if (key == "rimpow") t.rimPower = f;
  else if (key == "rimbias") t.rimLightBias = f;
  else if (key == "contact") t.contactAoPow = f;
  else if (key == "shape") t.shapeAoPow = f;
  else if (key == "black") t.blackPoint = f;
  else if (key == "white") t.whitePoint = f;
  else if (key == "hl") t.highlightAt = f;
  else if (key == "hlsoft") t.highlightSoft = f;
  else if (key == "gamma") t.gamma = f;
  else if (key == "speccut") t.specularCut = f;
  else if (key == "strength") t.strength = f;
  else if (key == "curve") t.curve = f;
  else return false;
  return true;
}

bool applyHatchInkKv(HatchOptions& o, const std::string& key,
                     const std::string& value) {
  float f = 0.0f;
  int i = 0;
  bool b = false;
  const std::string v = trimWs(value);
  if (key == "mode") {
    if (v == "ink") o.mode = HatchMode::Ink;
    else if (v == "over") o.mode = HatchMode::Over;
    else return false;
    return true;
  }
  if (key == "base") {
    if (v == "paper") o.base = HatchBase::Paper;
    else if (v == "albedo") o.base = HatchBase::Albedo;
    else return false;
    return true;
  }
  if (key == "ink") {
    if (v == "fixed") o.ink = HatchInk::Fixed;
    else if (v == "albedo") o.ink = HatchInk::FromAlbedo;
    else return false;
    return true;
  }
  if (key == "inkcolor") return parseColor(v, o.inkColor);
  if (key == "papercolor") return parseColor(v, o.paperColor);
  if (key == "tonefog") {
    if (!parseOnOff(v, b)) return false;
    o.toneFog = b;
    return true;
  }
  if (key == "albedoquant" || key == "levels") {
    if (!parseIntNum(v, i)) return false;
    (key == "levels" ? o.toneLevels : o.albedoQuantize) = i;
    return true;
  }
  if (!parseNum(v, f)) return false;
  if (key == "mincontrast") o.inkMinContrast = f;
  else if (key == "inkshade") o.inkShadeDark = f;
  else return false;
  return true;
}

bool applyHatchSpec(HatchOptions& opt, const std::string& text,
                    unsigned sections, std::string* error) {
  HatchOptions tmp = opt;
  std::vector<HatchLayer> layers;
  bool anyLayer = false;
  const std::vector<std::string> lines = splitSpecLines(text);
  for (std::size_t n = 0; n < lines.size(); ++n) {
    const std::string line = trimWs(lines[n]);
    if (line.empty() || line[0] == '#') continue;
    const std::string lineNo = "line " + fmtInt(static_cast<int>(n + 1));
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos) {
      if (error) *error = lineNo + ": expected 'layer:', 'tone:' or 'ink:'";
      return false;
    }
    const std::string section = trimWs(line.substr(0, colon));
    unsigned bit = 0;
    if (section == "layer") bit = kHatchSpecLayers;
    else if (section == "tone") bit = kHatchSpecTone;
    else if (section == "ink") bit = kHatchSpecInk;
    else {
      if (error) *error = lineNo + ": unknown section '" + section + "'";
      return false;
    }
    if ((sections & bit) == 0) continue;
    HatchLayer layer;
    std::string rest = line.substr(colon + 1);
    std::size_t pos = 0;
    while (pos <= rest.size()) {
      std::size_t comma = rest.find(',', pos);
      if (comma == std::string::npos) comma = rest.size();
      const std::string entry = trimWs(rest.substr(pos, comma - pos));
      pos = comma + 1;
      if (entry.empty()) continue;
      const std::size_t eq = entry.find('=');
      if (eq == std::string::npos) {
        if (error) *error = lineNo + ": '" + entry + "' is not key=value";
        return false;
      }
      const std::string key = trimWs(entry.substr(0, eq));
      const std::string value = trimWs(entry.substr(eq + 1));
      bool ok = false;
      if (bit == kHatchSpecLayers)
        ok = applyHatchLayerKv(layer, key, value);
      else if (bit == kHatchSpecTone)
        ok = (key == "levels") ? applyHatchInkKv(tmp, key, value)
                               : applyHatchToneKv(tmp.tone, key, value);
      else
        ok = applyHatchInkKv(tmp, key, value);
      if (!ok) {
        if (error) *error = lineNo + ": bad entry '" + entry + "'";
        return false;
      }
    }
    if (bit == kHatchSpecLayers) {
      layers.push_back(layer);
      anyLayer = true;
    }
  }
  if (anyLayer) tmp.layers = layers;
  opt = tmp;
  return true;
}

std::string hatchStyleToSpec(const HatchOptions& o, unsigned sections) {
  std::string out;
  if (sections & kHatchSpecLayers) {
    for (const HatchLayer& l : o.layers) {
      std::string s;
      const bool line = l.kind == LayerKind::Line;
      const bool dot = l.kind == LayerKind::Dot;
      putKv(s, "kind", line ? "line" : (dot ? "dot" : "stipple"));
      putKv(s, "angle", fmtNum(l.angleDeg));
      putKv(s, "spacing", fmtNum(l.spacingPx));
      putKv(s, "subdiv", fmtInt(l.subdiv));
      if (line) putKv(s, "width", fmtNum(l.widthPx));
      else putKv(s, "dotscale", fmtNum(l.dotScale));
      putKv(s, "tonehi", fmtNum(l.toneHi));
      putKv(s, "tonelo", fmtNum(l.toneLo));
      putKv(s, "fade", fmtNum(l.fadeInv));
      putKv(s, "opacity", fmtNum(l.opacity));
      putKv(s, "inkscale", fmtNum(l.inkScale));
      putKv(s, "soft", fmtNum(l.mark.edgeSoftness));
      putKv(s, "seed", fmtInt(static_cast<int>(l.mark.seed)));
      putKv(s, "jitter", fmtNum(l.mark.jitter));
      if (line) {
        putKv(s, "wobble", fmtNum(l.mark.wobbleAmpPx));
        putKv(s, "wobwave", fmtNum(l.mark.wobbleWavePx));
        putKv(s, "wjitter", fmtNum(l.mark.widthJitter));
        putKv(s, "slen", fmtNum(l.mark.strokeLenPx));
        putKv(s, "sgap", fmtNum(l.mark.strokeGapPx));
        putKv(s, "taper", fmtNum(l.mark.strokeTaper));
        putKv(s, "anglejitter", fmtNum(l.mark.angleJitterDeg));
        putKv(s, "lenjitter", fmtNum(l.mark.strokeLenJitter));
      } else {
        putKv(s, "shape", fmtNum(l.mark.shapeExponent));
        putKv(s, "aspect", fmtNum(l.mark.dotAspect));
        putKv(s, "dotangle", fmtNum(l.mark.dotAngleDeg));
        if (dot) putKv(s, "invert", fmtOnOff(l.mark.invertAbove50));
      }
      putKv(s, "tooth", fmtNum(l.mark.toothAmp));
      putKv(s, "toothscale", fmtNum(l.mark.toothScalePx));
      out += "layer: " + s + "\n";
    }
  }
  if (sections & kHatchSpecTone) {
    const ToneRecipe& t = o.tone;
    std::string s;
    putKv(s, "diffuse", fmtNum(t.diffuseWeight));
    putKv(s, "ambient", fmtNum(t.ambient));
    putKv(s, "wrap", fmtNum(t.wrap));
    putKv(s, "rim", fmtNum(t.rimDarken));
    putKv(s, "rimpow", fmtNum(t.rimPower));
    putKv(s, "rimbias", fmtNum(t.rimLightBias));
    putKv(s, "contact", fmtNum(t.contactAoPow));
    putKv(s, "shape", fmtNum(t.shapeAoPow));
    putKv(s, "black", fmtNum(t.blackPoint));
    putKv(s, "white", fmtNum(t.whitePoint));
    putKv(s, "hl", fmtNum(t.highlightAt));
    putKv(s, "hlsoft", fmtNum(t.highlightSoft));
    putKv(s, "gamma", fmtNum(t.gamma));
    putKv(s, "speccut", fmtNum(t.specularCut));
    putKv(s, "strength", fmtNum(t.strength));
    putKv(s, "curve", fmtNum(t.curve));
    putKv(s, "levels", fmtInt(o.toneLevels));
    out += "tone: " + s + "\n";
  }
  if (sections & kHatchSpecInk) {
    std::string s;
    putKv(s, "mode", o.mode == HatchMode::Over ? "over" : "ink");
    putKv(s, "base", o.base == HatchBase::Albedo ? "albedo" : "paper");
    putKv(s, "ink", o.ink == HatchInk::FromAlbedo ? "albedo" : "fixed");
    putKv(s, "inkcolor", fmtColor(o.inkColor));
    putKv(s, "papercolor", fmtColor(o.paperColor));
    putKv(s, "mincontrast", fmtNum(o.inkMinContrast));
    putKv(s, "inkshade", fmtNum(o.inkShadeDark));
    putKv(s, "tonefog", fmtOnOff(o.toneFog));
    putKv(s, "albedoquant", fmtInt(o.albedoQuantize));
    out += "ink: " + s + "\n";
  }
  return out;
}

void applyHatch(int w, int h, float* rgba, const float* tone,
                const float* mask, const float* albedo,
                const HatchOptions& opt, const std::uint16_t* groups,
                int groupSs, const GroupHatchStyle* styles,
                std::size_t styleCount, const float* uv) {
  if (!opt.enable || w <= 0 || h <= 0 || rgba == nullptr ||
      tone == nullptr || mask == nullptr)
    return;
  const bool perSection =
      groups != nullptr && styles != nullptr && styleCount > 0;
  const int gss = groupSs < 1 ? 1 : groupSs;
  const std::size_t gW = static_cast<std::size_t>(w) * gss;

  // Normalize the layers once (min-feature and perturbation clamps, Lp
  // area constants, search windows). The layer index keys the hash streams,
  // so two otherwise-identical layers still decorrelate.
  std::vector<detail::HatchLayerRt> layers;
  layers.reserve(opt.layers.size());
  for (std::size_t li = 0; li < opt.layers.size(); ++li) {
    const HatchLayer& L = opt.layers[li];
    if (L.opacity <= 0.0f) continue;
    layers.push_back(
        detail::hatchNormalizeLayer(L, static_cast<std::uint32_t>(li)));
  }
  // Sections that rescale the mark density / width need their own
  // normalized layer set (the pitch feeds the min-feature clamp and every
  // search window, so it cannot be applied at evaluation time). Sections at
  // the neutral scales share the global set.
  std::vector<std::vector<detail::HatchLayerRt>> sectionLayers;
  if (perSection) {
    sectionLayers.resize(styleCount);
    for (std::size_t si = 0; si < styleCount; ++si) {
      const GroupHatchStyle& g = styles[si];
      const float dens = std::max(0.1f, std::min(8.0f, g.density));
      const float wsc = std::max(0.1f, std::min(8.0f, g.widthScale));
      if (dens == 1.0f && wsc == 1.0f) continue;  // reuse the global set
      sectionLayers[si].reserve(opt.layers.size());
      for (std::size_t li = 0; li < opt.layers.size(); ++li) {
        HatchLayer L = opt.layers[li];
        if (L.opacity <= 0.0f) continue;
        L.spacingPx /= dens;
        L.widthPx *= wsc;
        L.dotScale *= wsc;
        // Perturbation amplitudes are pitch-relative in spirit; scale the
        // absolute-pixel ones with the pitch so a denser section keeps the
        // same look instead of turning into noise.
        L.mark.wobbleAmpPx /= dens;
        L.mark.strokeLenPx /= dens;
        L.mark.strokeGapPx /= dens;
        sectionLayers[si].push_back(
            detail::hatchNormalizeLayer(L, static_cast<std::uint32_t>(li)));
      }
    }
  }

  const ToneRecipe& tr = opt.tone;
  const float wpRange = std::max(1.0e-4f, tr.whitePoint - tr.blackPoint);
  // Ink-amount gain and curve (ToneRecipe::strength / curve), coverage
  // space; identity at (1, 1).
  const float inkStrength = std::max(0.0f, tr.strength);
  const float inkCurve = std::min(10.0f, std::max(0.1f, tr.curve));
  const bool inkRemap = inkStrength != 1.0f || inkCurve != 1.0f;

  // Row-parallel: every pixel is a pure function of (x, y) and the input
  // buffers, so the tiling is bit-exact at any thread count.
  tbb::parallel_for(
      tbb::blocked_range<int>(0, h),
      [&](const tbb::blocked_range<int>& rows) {
        for (int y = rows.begin(); y < rows.end(); ++y) {
          for (int x = 0; x < w; ++x) {
            const std::size_t p =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + x;
            const float m = mask[p];
            if (m <= 0.0f) continue;  // background: never painted

            // Per-section style: sample the hi-res id buffer at the cell
            // center. A disabled section keeps its frame color untouched.
            const GroupHatchStyle* st = nullptr;
            const std::vector<detail::HatchLayerRt>* lay = &layers;
            if (perSection) {
              const std::size_t gp =
                  (static_cast<std::size_t>(y) * gss + gss / 2) * gW +
                  static_cast<std::size_t>(x) * gss + gss / 2;
              const std::uint16_t g = groups[gp];
              if (g != 0xFFFFu && g < styleCount) {
                st = &styles[g];
                if (!st->enable) continue;
                if (!sectionLayers[g].empty()) lay = &sectionLayers[g];
              }
            }

            // Tone shaping: per-section scale -> linear black/white remap ->
            // artistic gamma (linear domain) -> display encode -> optional
            // quantization. Every stage maps 1 -> 1 (toneScale >= 1 lifts
            // toward paper and is clamped), so fully lit stays ink-free.
            float tLin = tone[p];
            if (st != nullptr) tLin *= st->toneScale;
            float t = detail::hatchClamp01((tLin - tr.blackPoint) / wpRange);
            if (tr.gamma != 1.0f) t = std::pow(t, tr.gamma);
            t = srgbEncodeF(t);
            // Highlight knee: lift only the top of the range to exact
            // paper white, leaving the mid tones (and so the highlight's
            // AREA) where the curve above put them.
            if (tr.highlightAt < 1.0f) {
              const float lo =
                  std::max(0.0f, tr.highlightAt - std::max(1.0e-4f,
                                                           tr.highlightSoft));
              const float k = detail::hatchSmoothstep(lo, tr.highlightAt, t);
              t = t + (1.0f - t) * k;
            }
            // Ink amount: c = strength * (1 - t)^curve in coverage space,
            // end points pinned (paper stays paper); the layers then read
            // the remapped tone, so lines gain nesting levels and dots gain
            // radius alike. Before the level quantization so the bands are
            // the final ones.
            if (inkRemap) {
              float c = 1.0f - t;
              if (inkCurve != 1.0f) c = std::pow(c, inkCurve);
              t = 1.0f - detail::hatchClamp01(inkStrength * c);
            }
            if (opt.toneLevels > 1) {
              const float n = static_cast<float>(opt.toneLevels - 1);
              t = std::round(t * n) / n;
            }

            float* px4 = rgba + p * 4;

            // Contrast-guarantee reference base: what this pixel shows
            // between the marks. In Ink mode the pipeline already painted
            // the flat base (paper / albedo) into rgba BEFORE the stroke
            // edge pass, so the composite below only multiplies ink in and
            // the contour ink survives; B here is recomputed from the
            // options purely as the contrast reference (the actual pixel
            // may carry edge ink or fog).
            const HatchBase baseSel = st != nullptr ? st->base : opt.base;
            const HatchInk inkSel = st != nullptr ? st->ink : opt.ink;
            const float* inkFixed =
                st != nullptr ? st->inkColor : opt.inkColor;
            // Contrast reference = what this pixel actually shows between
            // the marks. In Over mode that is the shaded frame; in Ink mode
            // the pipeline already painted the flat base into the frame, so
            // the frame is the base there too -- reading it (instead of
            // re-deriving the base from the options) keeps the reference
            // exact under fog, edge ink and any display transfer. Under a
            // transparent background rgba is premultiplied; the multiply
            // composite commutes with it (see the write-back below).
            float B[3] = {px4[0], px4[1], px4[2]};
            if (opt.mode == HatchMode::Ink && baseSel == HatchBase::Paper &&
                albedo == nullptr) {
              // Standalone callers may hand us an unpainted canvas; fall
              // back to the configured paper.
              for (int k = 0; k < 3; ++k)
                B[k] = detail::hatchClamp01(opt.paperColor[k]);
            }
            float I[3];
            if (inkSel == HatchInk::FromAlbedo && albedo != nullptr) {
              // Bring the LINEAR albedo into the frame's display space with
              // the pipeline's own transfer, so ink and base agree.
              for (int k = 0; k < 3; ++k) {
                const float a = std::max(0.0f, albedo[p * 3 + k]);
                I[k] = detail::hatchClamp01(
                    opt.displayGamma != 1.0f ? std::pow(a, opt.displayGamma)
                                             : a);
              }
            } else {
              for (int k = 0; k < 3; ++k)
                I[k] = detail::hatchClamp01(inkFixed[k]);
            }
            // Colored-pencil pressure: darken the ink with the tone (the
            // mark geometry stays tone-free; see HatchOptions::inkShadeDark).
            if (opt.inkShadeDark < 1.0f) {
              const float sh =
                  opt.inkShadeDark + (1.0f - opt.inkShadeDark) * t;
              for (int k = 0; k < 3; ++k) I[k] *= sh;
            }
            ensureInkContrast(I, B, opt.inkMinContrast);

            // Multiply-composite the layers: f *= 1 - c*(1 - I). Black ink
            // reduces to "over"; colored ink darkens where layers cross
            // (I^2), like real colored pencil / color tone stacking.
            float f[3] = {1.0f, 1.0f, 1.0f};
            // Mark coordinate: the surface parameterization when this
            // pixel has one, else the pixel raster. Everything downstream
            // (lattice, angle rotation, perturbations, paper tooth) is a
            // pure function of this pair, so the two spaces need no other
            // special-casing.
            float xc = static_cast<float>(x) + 0.5f;
            float yc = static_cast<float>(y) + 0.5f;
            if (uv != nullptr) {
              const float su = uv[p * 2 + 0], sv = uv[p * 2 + 1];
              if (su != 0.0f || sv != 0.0f) {
                xc = su * opt.uvScale;
                yc = sv * opt.uvScale;
              }
            }
            for (const detail::HatchLayerRt& L : *lay) {
              if (st != nullptr && L.layerId < 31u &&
                  ((st->layerMask >> L.layerId) & 1) == 0)
                continue;  // layer disabled for this section
              const float c = detail::hatchLayerInk(L, xc, yc, t) * L.opacity;
              if (c <= 0.0f) continue;
              // Each layer is a pencil: later (shadow) layers may draw with
              // a darker stick of the same hue (HatchLayer::inkScale).
              for (int k = 0; k < 3; ++k) {
                const float ik = I[k] * L.inkScale;
                f[k] *= 1.0f - c * (1.0f - ik);
              }
            }

            // Write-back with silhouette-coverage AA (mask): multiply-only
            // in BOTH modes, so the ink can only darken -- stroke-edge ink
            // already in the frame is preserved, and premultiplied pixels
            // stay premultiplied (the multiply commutes with the alpha).
            for (int k = 0; k < 3; ++k)
              px4[k] *= (1.0f - m) + m * f[k];
          }
        }
      });
}

}  // namespace umbreon
