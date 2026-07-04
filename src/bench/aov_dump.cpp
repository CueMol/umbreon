#include "aov_dump.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "image/image_io.hpp"

namespace umbreon {

void dumpDebugImages(const Options& opt, const RenderOptions& ropt,
                     const FrameResult& frame) {
  const int finalW = ropt.width, finalH = ropt.height;
  const int ss = ropt.supersample;

  // Adaptive-AA refinement mask dump (--aa-debug): grayscale, one value per
  // OUTPUT pixel (1 = refined, 0 = replicated center). Written under the
  // --dump-aov prefix when given, else next to the output image; the tuning
  // aid for --aa-threshold.
  if (!frame.aaMask.empty()) {
    std::string prefix = opt.dumpAovPrefix;
    if (prefix.empty()) {
      prefix = opt.output;
      const std::size_t dot = prefix.find_last_of('.');
      if (dot != std::string::npos) prefix = prefix.substr(0, dot);
    }
    const std::size_t np = static_cast<std::size_t>(finalW) * finalH;
    if (frame.aaMask.size() == np) {
      std::vector<float> mimg(np * 3);
      for (std::size_t i = 0; i < np; ++i)
        mimg[i * 3 + 0] = mimg[i * 3 + 1] = mimg[i * 3 + 2] = frame.aaMask[i];
      umbreon::writeImage(prefix + "_aaMask.png", finalW, finalH, mimg.data(),
                          3);
      std::printf("  dumped adaptive-AA mask: %s_aaMask.png (%dx%d)\n",
                  prefix.c_str(), finalW, finalH);
    }
  }

  // Coarse-AO fallback mask dump (--ao-res-debug): grayscale at the hi-res
  // (supersampled) grid, 1 = first-hit lookup rejected -> gathered inline.
  // The tuning aid for judging how much of the frame patches inline.
  if (!frame.aoPatchMask.empty()) {
    std::string prefix = opt.dumpAovPrefix;
    if (prefix.empty()) {
      prefix = opt.output;
      const std::size_t dot = prefix.find_last_of('.');
      if (dot != std::string::npos) prefix = prefix.substr(0, dot);
    }
    const int hiW = finalW * ss, hiH = finalH * ss;
    const std::size_t np = static_cast<std::size_t>(hiW) * hiH;
    if (frame.aoPatchMask.size() == np) {
      std::vector<float> mimg(np * 3);
      for (std::size_t i = 0; i < np; ++i)
        mimg[i * 3 + 0] = mimg[i * 3 + 1] = mimg[i * 3 + 2] =
            frame.aoPatchMask[i];
      umbreon::writeImage(prefix + "_aoPatchMask.png", hiW, hiH, mimg.data(),
                          3);
      std::printf("  dumped coarse-AO patch mask: %s_aoPatchMask.png "
                  "(%dx%d)\n",
                  prefix.c_str(), hiW, hiH);
    }
  }

  // Debug AOV dump (verification only): false-color the captured AOVs. Two
  // independent sources feed it: the edge pass G-buffer (--edges on, kept at
  // the SUPERSAMPLE resolution) and the AO/cache AOVs (--ao-write-aov on,
  // downsampled to the final resolution). Either, both, or neither may be on.
  if (!opt.dumpAovPrefix.empty()) {
    bool dumpedAny = false;
    const uint32_t kBg = 0xFFFFFFFFu;
    // Edge G-buffer AOVs (hi-res): object/material id false color, normal, z.
    if (ropt.strokeEdges.enable && !frame.objectId.empty()) {
      const int hiW = finalW * ss;
      const int hiH = finalH * ss;
      const std::size_t np = static_cast<std::size_t>(hiW) * hiH;
      // Deterministic id -> RGB false color (background sentinel -> black).
      auto idColor = [&](uint32_t id, float* rgb) {
        if (id == kBg) { rgb[0] = rgb[1] = rgb[2] = 0.0f; return; }
        uint32_t h = id * 2654435761u;  // Knuth multiplicative hash
        rgb[0] = ((h >> 16) & 0xFF) / 255.0f;
        rgb[1] = ((h >> 8) & 0xFF) / 255.0f;
        rgb[2] = (h & 0xFF) / 255.0f;
      };
      std::vector<float> oimg(np * 3), mimg(np * 3), nimg(np * 3), zimg(np * 3);
      float zmin = 1e30f, zmax = -1e30f;
      for (std::size_t i = 0; i < np; ++i) {
        if (frame.objectId[i] == kBg) continue;
        const float z = frame.viewZ[i];
        zmin = std::min(zmin, z);
        zmax = std::max(zmax, z);
      }
      const float zspan = (zmax > zmin) ? (zmax - zmin) : 1.0f;
      for (std::size_t i = 0; i < np; ++i) {
        idColor(frame.objectId[i], &oimg[i * 3]);
        idColor(frame.materialId[i], &mimg[i * 3]);
        for (int c = 0; c < 3; ++c)
          nimg[i * 3 + c] = frame.normal[i * 3 + c] * 0.5f + 0.5f;
        float zn = 0.0f;
        if (frame.objectId[i] != kBg)
          zn = (frame.viewZ[i] - zmin) / zspan;
        zimg[i * 3 + 0] = zimg[i * 3 + 1] = zimg[i * 3 + 2] = zn;
      }
      umbreon::writeImage(opt.dumpAovPrefix + "_objectId.png", hiW, hiH,
                          oimg.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_materialId.png", hiW, hiH,
                          mimg.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_normal.png", hiW, hiH,
                          nimg.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_viewZ.png", hiW, hiH,
                          zimg.data(), 3);
      std::printf("  dumped edge AOVs: %s_{objectId,materialId,normal,viewZ}"
                  ".png (%dx%d)\n",
                  opt.dumpAovPrefix.c_str(), hiW, hiH);
      dumpedAny = true;
    }
    // AO / cache AOVs (final res): albedo, contact/shape AO, bent normal, mean
    // occluder distance (and normal, when not already dumped hi-res by edges).
    if (ropt.aoWriteAov && !frame.contactAo.empty()) {
      const int aw = frame.width, ah = frame.height;
      const std::size_t np = static_cast<std::size_t>(aw) * ah;
      std::vector<float> alb(np * 3), bent(np * 3), con(np * 3), shp(np * 3),
          avg(np * 3);
      const float invR = ropt.aoDistance > 0.0f ? 1.0f / ropt.aoDistance : 0.0f;
      for (std::size_t i = 0; i < np; ++i) {
        for (int c = 0; c < 3; ++c) alb[i * 3 + c] = frame.albedo[i * 3 + c];
        for (int c = 0; c < 3; ++c)
          bent[i * 3 + c] = frame.bentNormal[i * 3 + c] * 0.5f + 0.5f;
        const float cc = frame.contactAo[i];
        const float ss2 = frame.shapeAo[i];
        float a = frame.avgHitDist[i] * invR;  // normalize by AO radius
        if (a > 1.0f) a = 1.0f;
        con[i * 3 + 0] = con[i * 3 + 1] = con[i * 3 + 2] = cc;
        shp[i * 3 + 0] = shp[i * 3 + 1] = shp[i * 3 + 2] = ss2;
        avg[i * 3 + 0] = avg[i * 3 + 1] = avg[i * 3 + 2] = a;
      }
      umbreon::writeImage(opt.dumpAovPrefix + "_albedo.png", aw, ah,
                          alb.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_contactAo.png", aw, ah,
                          con.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_shapeAo.png", aw, ah,
                          shp.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_bentNormal.png", aw, ah,
                          bent.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_avgHitDist.png", aw, ah,
                          avg.data(), 3);
      // Dump the AO-path normal only when the edge pass did not already (its
      // hi-res normal takes precedence to avoid a dimension clash).
      if (!ropt.strokeEdges.enable && frame.normal.size() == np * 3) {
        std::vector<float> nimg(np * 3);
        for (std::size_t i = 0; i < np; ++i)
          for (int c = 0; c < 3; ++c)
            nimg[i * 3 + c] = frame.normal[i * 3 + c] * 0.5f + 0.5f;
        umbreon::writeImage(opt.dumpAovPrefix + "_normal.png", aw, ah,
                            nimg.data(), 3);
      }
      std::printf("  dumped AO AOVs: %s_{albedo,contactAo,shapeAo,bentNormal,"
                  "avgHitDist}.png (%dx%d)\n",
                  opt.dumpAovPrefix.c_str(), aw, ah);
      dumpedAny = true;
    }
    // GI cache AOVs (final res): E_cached (indirect) auto-normalized for
    // display, plus the record-density debug viz (bright = small radius =
    // dense records, e.g. in concavities). Lets the cache be eyeballed before
    // the final composite is wired.
    if (ropt.gi && !frame.indirect.empty()) {
      const int aw = frame.width, ah = frame.height;
      const std::size_t np = static_cast<std::size_t>(aw) * ah;
      // Luminance stats over MESH pixels only (indirect > 0). With a uniform
      // white environment the open surface saturates near the env value, so a
      // raw [0,max] map plus sRGB crushes the concavity gradient into a near-
      // binary image. Contrast-stretch [lo,hi] (robust 2nd/98th percentile)
      // instead, so the open-vs-concave variation fills the tonal range.
      std::vector<float> lum;
      lum.reserve(np);
      float emin = 1e30f, emax = 0.0f;
      for (std::size_t i = 0; i < np; ++i) {
        const float L = (frame.indirect[i * 3 + 0] + frame.indirect[i * 3 + 1] +
                         frame.indirect[i * 3 + 2]) / 3.0f;
        if (L <= 0.0f) continue;  // background / non-mesh first hit
        lum.push_back(L);
        emin = std::min(emin, L);
        emax = std::max(emax, L);
      }
      float lo = 0.0f, hi = (emax > 0.0f) ? emax : 1.0f;
      if (!lum.empty()) {
        std::sort(lum.begin(), lum.end());
        lo = lum[static_cast<std::size_t>(lum.size() * 0.02f)];
        hi = lum[static_cast<std::size_t>(lum.size() * 0.98f)];
        if (hi <= lo) hi = lo + 1e-6f;
      }
      const float inv = 1.0f / (hi - lo);
      std::vector<float> ind(np * 3), raw(np * 3);
      for (std::size_t i = 0; i < np * 3; ++i) {
        raw[i] = (emax > 0.0f) ? frame.indirect[i] / emax : 0.0f;  // [0,max]
        float v = (frame.indirect[i] - lo) * inv;                  // stretched
        ind[i] = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
      }
      umbreon::writeImage(opt.dumpAovPrefix + "_indirect.png", aw, ah,
                          ind.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_indirectRaw.png", aw, ah,
                          raw.data(), 3);
      umbreon::writeImage(opt.dumpAovPrefix + "_giRecords.png", aw, ah,
                          frame.giRecordViz.data(), 3);
      // Openness map (1 - gather occlusion fraction): env- and bounce-
      // independent, so it shows concavity darkening cleanly even when a bright
      // white environment fills the E_cached pits back in. White = open convex,
      // dark = occluded concavity. Background / non-mesh stays white (open).
      std::vector<float> opn(np * 3);
      for (std::size_t i = 0; i < np; ++i) {
        const float open = 1.0f - frame.giOcclusion[i];
        opn[i * 3 + 0] = opn[i * 3 + 1] = opn[i * 3 + 2] = open;
      }
      umbreon::writeImage(opt.dumpAovPrefix + "_giOpenness.png", aw, ah,
                          opn.data(), 3);
      std::printf(
          "  dumped GI AOVs: %s_{indirect,indirectRaw,giRecords,giOpenness}"
          ".png (%dx%d)\n"
          "    indirect luminance: min %.4g  max %.4g  stretch [%.4g, %.4g]\n",
          opt.dumpAovPrefix.c_str(), aw, ah, emin, emax, lo, hi);
      dumpedAny = true;
    }
    if (!dumpedAny)
      std::fprintf(
          stderr,
          "warning: --dump-aov ignored (needs --edges, --ao-write-aov or "
          "--gi on)\n");
  }
}

}  // namespace umbreon
