#include "image/image_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace umbreon {
namespace {

// ----- color space --------------------------------------------------------
float linearToSrgb(float c) {
  c = std::min(1.0f, std::max(0.0f, c));
  return (c <= 0.0031308f) ? 12.92f * c
                           : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}

float srgbToLinear(float c) {
  return (c <= 0.04045f) ? c / 12.92f
                         : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

std::uint8_t quantize(float unit) {
  int v = static_cast<int>(unit * 255.0f + 0.5f);
  return static_cast<std::uint8_t>(std::min(255, std::max(0, v)));
}

// ----- PNG encoding (uncompressed deflate; self-contained, valid PNG) -----
std::uint32_t crc32(const std::uint8_t* data, std::size_t len,
                    std::uint32_t crc) {
  static std::uint32_t table[256];
  static bool ready = false;
  if (!ready) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k)
        c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      table[i] = c;
    }
    ready = true;
  }
  crc = ~crc;
  for (std::size_t i = 0; i < len; ++i)
    crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

std::uint32_t adler32(const std::uint8_t* data, std::size_t len) {
  std::uint32_t a = 1, b = 0;
  for (std::size_t i = 0; i < len; ++i) {
    a = (a + data[i]) % 65521;
    b = (b + a) % 65521;
  }
  return (b << 16) | a;
}

void putU32(std::vector<std::uint8_t>& v, std::uint32_t x) {
  v.push_back(static_cast<std::uint8_t>(x >> 24));
  v.push_back(static_cast<std::uint8_t>(x >> 16));
  v.push_back(static_cast<std::uint8_t>(x >> 8));
  v.push_back(static_cast<std::uint8_t>(x));
}

void writeChunk(std::vector<std::uint8_t>& out, const char* type,
                const std::vector<std::uint8_t>& data) {
  putU32(out, static_cast<std::uint32_t>(data.size()));
  std::size_t crcStart = out.size();
  out.insert(out.end(), type, type + 4);
  out.insert(out.end(), data.begin(), data.end());
  putU32(out, crc32(out.data() + crcStart, out.size() - crcStart, 0));
}

void writePng(const std::string& path, int w, int h,
              const std::vector<std::uint8_t>& pix, int channels) {
  // Filtered scanlines: a 0x00 filter byte then the raw row bytes.
  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(h) * (1 + w * channels));
  for (int y = 0; y < h; ++y) {
    raw.push_back(0);
    const std::uint8_t* row = pix.data() +
                              static_cast<std::size_t>(y) * w * channels;
    raw.insert(raw.end(), row, row + static_cast<std::size_t>(w) * channels);
  }

  // zlib stream wrapping deflate "stored" (uncompressed) blocks.
  std::vector<std::uint8_t> zlib;
  zlib.push_back(0x78);
  zlib.push_back(0x01);
  std::size_t pos = 0;
  while (pos < raw.size() || zlib.size() == 2) {
    std::size_t n = std::min<std::size_t>(65535, raw.size() - pos);
    bool last = (pos + n == raw.size());
    zlib.push_back(last ? 1 : 0);
    zlib.push_back(static_cast<std::uint8_t>(n & 0xFF));
    zlib.push_back(static_cast<std::uint8_t>((n >> 8) & 0xFF));
    std::uint16_t nlen = static_cast<std::uint16_t>(~n);
    zlib.push_back(static_cast<std::uint8_t>(nlen & 0xFF));
    zlib.push_back(static_cast<std::uint8_t>((nlen >> 8) & 0xFF));
    zlib.insert(zlib.end(), raw.begin() + pos, raw.begin() + pos + n);
    pos += n;
    if (last) break;
  }
  putU32(zlib, adler32(raw.data(), raw.size()));

  std::vector<std::uint8_t> ihdr;
  putU32(ihdr, static_cast<std::uint32_t>(w));
  putU32(ihdr, static_cast<std::uint32_t>(h));
  ihdr.push_back(8);                              // bit depth
  ihdr.push_back(channels == 4 ? 6 : 2);          // color type: RGBA / RGB
  ihdr.push_back(0);                              // compression
  ihdr.push_back(0);                              // filter
  ihdr.push_back(0);                              // interlace

  std::vector<std::uint8_t> out;
  const std::uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  out.insert(out.end(), sig, sig + 8);
  writeChunk(out, "IHDR", ihdr);
  writeChunk(out, "IDAT", zlib);
  writeChunk(out, "IEND", {});

  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open output image: " + path);
  f.write(reinterpret_cast<const char*>(out.data()),
          static_cast<std::streamsize>(out.size()));
}

void writePpm(const std::string& path, int w, int h,
              const std::vector<std::uint8_t>& rgb) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open output image: " + path);
  f << "P6\n" << w << ' ' << h << "\n255\n";
  f.write(reinterpret_cast<const char*>(rgb.data()),
          static_cast<std::streamsize>(rgb.size()));
}

bool endsWith(const std::string& s, const char* suffix) {
  std::string suf(suffix);
  return s.size() >= suf.size() &&
         s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

}  // namespace

void writeImage(const std::string& path, int width, int height,
                const float* pixels, int channels) {
  if (channels != 3 && channels != 4)
    throw std::runtime_error("writeImage: channels must be 3 or 4");

  const std::size_t count = static_cast<std::size_t>(width) * height;

  if (endsWith(path, ".ppm")) {
    std::vector<std::uint8_t> rgb(count * 3);
    for (std::size_t i = 0; i < count; ++i) {
      for (int c = 0; c < 3; ++c)
        rgb[i * 3 + c] = quantize(linearToSrgb(pixels[i * channels + c]));
    }
    writePpm(path, width, height, rgb);
    return;
  }

  std::vector<std::uint8_t> pix(count * channels);
  for (std::size_t i = 0; i < count; ++i) {
    for (int c = 0; c < channels; ++c) {
      float v = pixels[i * channels + c];
      // RGB is sRGB-encoded; an alpha channel stays linear.
      pix[i * channels + c] =
          (c < 3) ? quantize(linearToSrgb(v)) : quantize(v);
    }
  }
  writePng(path, width, height, pix, channels);
}

ImageRGB readPpm(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open image: " + path);

  std::string magic;
  f >> magic;
  if (magic != "P6") throw std::runtime_error("not a binary PPM: " + path);

  auto readInt = [&]() -> int {
    int value = -1;
    while (f) {
      int ch = f.peek();
      if (ch == '#') {  // skip comment line
        std::string skip;
        std::getline(f, skip);
        continue;
      }
      if (std::isspace(ch)) {
        f.get();
        continue;
      }
      f >> value;
      break;
    }
    return value;
  };

  ImageRGB img;
  img.width = readInt();
  img.height = readInt();
  int maxval = readInt();
  f.get();  // single whitespace after the header
  if (img.width <= 0 || img.height <= 0 || maxval != 255)
    throw std::runtime_error("unsupported PPM header: " + path);

  const std::size_t count = static_cast<std::size_t>(img.width) * img.height;
  std::vector<std::uint8_t> bytes(count * 3);
  f.read(reinterpret_cast<char*>(bytes.data()),
         static_cast<std::streamsize>(bytes.size()));

  img.rgb.resize(count * 3);
  for (std::size_t i = 0; i < count * 3; ++i)
    img.rgb[i] = srgbToLinear(bytes[i] / 255.0f);
  return img;
}

CompareResult compareImages(const ImageRGB& a, const ImageRGB& b) {
  CompareResult r;
  if (a.width != b.width || a.height != b.height || a.width <= 0)
    return r;

  const int w = a.width, h = a.height;
  const std::size_t count = static_cast<std::size_t>(w) * h;

  // PSNR over sRGB-encoded RGB values.
  double mse = 0.0;
  for (std::size_t i = 0; i < count * 3; ++i) {
    double d = linearToSrgb(a.rgb[i]) - linearToSrgb(b.rgb[i]);
    mse += d * d;
  }
  mse /= static_cast<double>(count * 3);
  r.psnr = (mse <= 1e-12) ? 99.0 : 10.0 * std::log10(1.0 / mse);

  // SSIM on sRGB luma, 8x8 sliding window, step 4.
  std::vector<double> la(count), lb(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto luma = [](const float* p) {
      return 0.299 * linearToSrgb(p[0]) + 0.587 * linearToSrgb(p[1]) +
             0.114 * linearToSrgb(p[2]);
    };
    la[i] = luma(&a.rgb[i * 3]);
    lb[i] = luma(&b.rgb[i * 3]);
  }
  const double c1 = 0.0001, c2 = 0.0009;  // (0.01)^2, (0.03)^2 on [0,1]
  const int win = 8, step = 4;
  double ssimSum = 0.0;
  long windows = 0;
  for (int y = 0; y + win <= h; y += step) {
    for (int x = 0; x + win <= w; x += step) {
      double ma = 0, mb = 0;
      for (int j = 0; j < win; ++j)
        for (int i = 0; i < win; ++i) {
          std::size_t idx = static_cast<std::size_t>(y + j) * w + (x + i);
          ma += la[idx];
          mb += lb[idx];
        }
      const double inv = 1.0 / (win * win);
      ma *= inv;
      mb *= inv;
      double va = 0, vb = 0, cov = 0;
      for (int j = 0; j < win; ++j)
        for (int i = 0; i < win; ++i) {
          std::size_t idx = static_cast<std::size_t>(y + j) * w + (x + i);
          double da = la[idx] - ma, db = lb[idx] - mb;
          va += da * da;
          vb += db * db;
          cov += da * db;
        }
      va *= inv;
      vb *= inv;
      cov *= inv;
      double s = ((2 * ma * mb + c1) * (2 * cov + c2)) /
                 ((ma * ma + mb * mb + c1) * (va + vb + c2));
      ssimSum += s;
      ++windows;
    }
  }
  r.ssim = (windows > 0) ? ssimSum / windows : 1.0;
  return r;
}

}  // namespace umbreon
