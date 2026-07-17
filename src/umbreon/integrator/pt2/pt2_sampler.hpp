// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// pt2 sampler: shuffled, hash-based Owen-scrambled Sobol (Burley, "Practical
// Hash-based Owen Scrambling", JCGT 9(4) 2020), with an optional blue-noise
// pixel arrangement. Replaces pt1's Hammersley + Cranley-Patterson toroidal
// shift for the FIRST gather bounce: true Owen scrambling distributes the
// residual error as high-frequency noise, which OIDN (and the eye) removes
// far better than the CP shift's structured residue.
//
// Two pixel arrangements (RenderOptions::pt2Pattern):
//   sobol      -- an independent scrambled sequence per pixel (seed = pixel
//                 hash). Error is white across pixels, like pt1.
//   bluenoise  -- ONE global sequence; each pixel owns a contiguous section
//                 of it, and pixels are ordered along a hierarchically
//                 shuffled Morton curve so neighboring pixels get maximally
//                 separated sections. Error distributes as blue noise in
//                 screen space (Cycles' SAMPLING_PATTERN_BLUE_NOISE_PURE).
//
// Everything is a pure function of (pixel, sample index, seed): stateless,
// thread-count invariant, bit-exact across runs -- the same determinism
// contract as pt1's tea2 stream.
#pragma once

#include <cstdint>

namespace umbreon {
namespace detail {

// Laine-Karras style hash acting on reversed bits (the core of hash-based
// Owen scrambling). Identical constants to pbrt-v4's FastOwenScrambler and
// Cycles' reversed_bit_owen.
inline std::uint32_t pt2ReversedBitOwen(std::uint32_t n,
                                        std::uint32_t seed) noexcept {
  n ^= n * 0x3d20adeau;
  n += seed;
  n *= (seed >> 16) | 1u;
  n ^= n * 0x05526c56u;
  n ^= n * 0x53a22864u;
  return n;
}

inline std::uint32_t pt2ReverseBits(std::uint32_t x) noexcept {
  x = ((x & 0x55555555u) << 1) | ((x >> 1) & 0x55555555u);
  x = ((x & 0x33333333u) << 2) | ((x >> 2) & 0x33333333u);
  x = ((x & 0x0f0f0f0fu) << 4) | ((x >> 4) & 0x0f0f0f0fu);
  x = ((x & 0x00ff00ffu) << 8) | ((x >> 8) & 0x00ff00ffu);
  return (x << 16) | (x >> 16);
}

// Base-4 variant of the Owen hash, used to shuffle the Morton pixel curve so
// each 2x2 quad stays together at every scale (Cycles' hash, verbatim).
inline std::uint32_t pt2ReversedBitOwenBase4(std::uint32_t n,
                                             std::uint32_t seed) noexcept {
  n ^= n * 0x3d20adeau;
  n ^= (n >> 1) & (n << 1) & 0x55555555u;
  n += seed;
  n *= (seed >> 16) | 1u;
  n ^= (n >> 1) & (n << 1) & 0x55555555u;
  n ^= n * 0x05526c56u;
  n ^= n * 0x53a22864u;
  return n;
}

inline std::uint32_t pt2NestedUniformScrambleBase4(std::uint32_t i,
                                                   std::uint32_t seed) noexcept {
  return pt2ReverseBits(pt2ReversedBitOwenBase4(pt2ReverseBits(i), seed));
}

// Finalizing hash for decorrelating dimensions/seeds (Cycles' hash_hp_uint).
inline std::uint32_t pt2HashHp(std::uint32_t i) noexcept {
  i ^= i >> 16;
  i *= 0x21f0aaadu;
  i ^= i >> 15;
  i *= 0xd35a2d97u;
  i ^= i >> 15;
  return i ^ 0xe6fe3bebu;
}

inline std::uint32_t pt2ExpandBits(std::uint32_t x) noexcept {
  x &= 0x0000ffffu;
  x = (x ^ (x << 8)) & 0x00ff00ffu;
  x = (x ^ (x << 4)) & 0x0f0f0f0fu;
  x = (x ^ (x << 2)) & 0x33333333u;
  x = (x ^ (x << 1)) & 0x55555555u;
  return x;
}

inline std::uint32_t pt2Morton2d(std::uint32_t x, std::uint32_t y) noexcept {
  return (pt2ExpandBits(x) << 1) | pt2ExpandBits(y);
}

// Maps [0, 2^32) -> [0, 1) (Cycles' uint_to_float_excl: the divisor is
// slightly above 2^32 so 0xffffffff stays strictly below 1).
inline float pt2UintToFloatExcl(std::uint32_t n) noexcept {
  return static_cast<float>(n) * (1.0f / 4294967808.0f);
}

// Direction numbers of the SECOND Sobol dimension in the reversed-bit
// convention (Burley 2020 / Cycles sobol_burley_table[1]; dimension 0 is the
// Van der Corput fast path and needs no table). Only the 2D sampler exists
// here -- pt2 stratifies the first gather bounce, and continuation bounces
// keep pt1's tea2 stream.
inline const std::uint32_t* pt2SobolDim1Table() noexcept {
  static const std::uint32_t k[32] = {
      0x00000001u, 0x00000003u, 0x00000005u, 0x0000000fu,
      0x00000011u, 0x00000033u, 0x00000055u, 0x000000ffu,
      0x00000101u, 0x00000303u, 0x00000505u, 0x00000f0fu,
      0x00001111u, 0x00003333u, 0x00005555u, 0x0000ffffu,
      0x00010001u, 0x00030003u, 0x00050005u, 0x000f000fu,
      0x00110011u, 0x00330033u, 0x00550055u, 0x00ff00ffu,
      0x01010101u, 0x03030303u, 0x05050505u, 0x0f0f0f0fu,
      0x11111111u, 0x33333333u, 0x55555555u, 0xffffffffu};
  return k;
}

// One dimension of an Owen-scrambled Sobol sample; rev_bit_index carries the
// (already shuffled) sample index with reversed bit order.
inline float pt2SobolBurley(std::uint32_t revBitIndex, int dimension,
                            std::uint32_t scrambleSeed) noexcept {
  std::uint32_t result = 0;
  if (dimension == 0) {
    result = pt2ReverseBits(revBitIndex);
  } else {
    const std::uint32_t* table = pt2SobolDim1Table();
    std::uint32_t i = 0;
    while (revBitIndex != 0) {
      // Count leading zeros without UB for 0 (loop guard excludes it).
      std::uint32_t j = 0;
      while (!(revBitIndex & (0x80000000u >> j))) ++j;
      result ^= table[i + j];
      i += j + 1;
      revBitIndex <<= j;
      revBitIndex <<= 1;
    }
  }
  result = pt2ReverseBits(pt2ReversedBitOwen(result, scrambleSeed));
  return pt2UintToFloatExcl(result);
}

// 2D shuffled, Owen-scrambled Sobol sample (Cycles sobol_burley_sample_2D
// with dimension_set = 0; the per-arity xor constants are Cycles').
inline void pt2SobolBurley2D(std::uint32_t index, std::uint32_t seed,
                             float* u1, float* u2) noexcept {
  index = pt2ReversedBitOwen(pt2ReverseBits(index), seed ^ 0xf8ade99au);
  *u1 = pt2SobolBurley(index, 0, seed ^ 0xe0aaaf76u);
  *u2 = pt2SobolBurley(index, 1, seed ^ 0x94964d4eu);
}

// Resolved per-point sampler state for one gather pixel: sample s draws the
// 2D point at global index (indexBase + s) under `seed`. Construct with
// pt2MakePixelSampler.
struct Pt2PixelSampler {
  std::uint32_t indexBase = 0;
  std::uint32_t seed = 0;
};

// pt2Pattern values (RenderOptions::pt2Pattern).
enum : int { kPt2PatternSobol = 0, kPt2PatternBlueNoise = 1 };

// Build the per-pixel sampler state.
//  sobol:      independent sequence per pixel (seed = pixel hash ^ frame).
//  bluenoise:  one global sequence; pixel sections of length sppPow2, ordered
//              along the scrambled Morton curve. seed is the FRAME seed only,
//              identical for every pixel -- that is what makes it one global
//              sequence.
inline Pt2PixelSampler pt2MakePixelSampler(int pattern, std::uint32_t px,
                                           std::uint32_t py,
                                           std::uint32_t frameSeed,
                                           std::uint32_t pixelHash,
                                           std::uint32_t sppPow2) noexcept {
  Pt2PixelSampler s;
  if (pattern == kPt2PatternBlueNoise) {
    s.indexBase =
        pt2NestedUniformScrambleBase4(pt2Morton2d(px, py), frameSeed) * sppPow2;
    s.seed = frameSeed;
  } else {
    s.indexBase = 0;
    s.seed = pixelHash;
  }
  return s;
}

// Smallest power of two >= n (n >= 1), for the blue-noise section length.
inline std::uint32_t pt2NextPow2(std::uint32_t n) noexcept {
  std::uint32_t p = 1;
  while (p < n) p <<= 1;
  return p;
}

}  // namespace detail
}  // namespace umbreon
