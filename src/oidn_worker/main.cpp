// umbreon_oidn_worker: out-of-process Intel Open Image Denoise service.
//
// The ONLY binary in the tree that links OpenImageDenoise. libumbreon's IPC
// client (src/umbreon/ipc/oidn_client.cpp) spawns it once per host process,
// keeps it alive across render() calls, and drives it with fixed-size request
// and response structs over stdin/stdout (see ipc/oidn_protocol.hpp); image
// buffers travel through a client-owned shared-memory region
// (ipc/shm_region.hpp). stdin EOF means the client is gone (clean shutdown or
// crash: either way the OS closed the pipe), so the worker just exits -- no
// orphan processes, no signals needed.
//
// Robustness contract: a malformed request must never crash the worker. Size
// and offset validation happens before any shm access; OIDN and shm errors
// are converted into an error response and the loop continues. Only a framing
// loss (bad magic/version/op on the pipe) is unrecoverable, because message
// boundaries are gone: report, then exit non-zero.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include <OpenImageDenoise/oidn.hpp>

#include "ipc/oidn_protocol.hpp"
#include "ipc/shm_region.hpp"

namespace {

using namespace umbreon::ipc;

bool readExact(void* dst, std::size_t n) {
  return std::fread(dst, 1, n, stdin) == n;
}

bool writeExact(const void* src, std::size_t n) {
  return std::fwrite(src, 1, n, stdout) == n && std::fflush(stdout) == 0;
}

void setErrMsg(DenoiseResponse& resp, const char* msg) {
  std::snprintf(resp.errMsg, sizeof(resp.errMsg), "%s",
                msg ? msg : "unknown error");
}

bool rangesOverlap(std::uint64_t a, std::uint64_t an, std::uint64_t b,
                   std::uint64_t bn) {
  return a < b + bn && b < a + an;
}

// Bounds-check one buffer slice against the declared region size.
bool sliceOk(std::uint64_t offset, std::uint64_t bytes, std::uint64_t shmSize) {
  return offset <= shmSize && bytes <= shmSize - offset;
}

// Validates the request, maps the region and runs the OIDN RT filter. Fills
// resp.status / resp.errMsg / resp.tFilter / resp.tExecute (tDevice is the
// caller's business). Never throws.
void handleDenoise(const DenoiseRequest& req, oidn::DeviceRef& device,
                   const char* deviceErr, DenoiseResponse& resp) {
  const std::uint64_t W = static_cast<std::uint64_t>(req.width);
  const std::uint64_t H = static_cast<std::uint64_t>(req.height);
  if (req.width <= 0 || req.height <= 0 || W > 65536 || H > 65536) {
    resp.status = kStatusBadRequest;
    setErrMsg(resp, "width/height out of range");
    return;
  }
  const std::uint64_t bufBytes = W * H * 3ull * sizeof(float);
  const bool hasAlbedo = (req.flags & kFlagHasAlbedo) != 0;
  const bool hasNormal = (req.flags & kFlagHasNormal) != 0;
  const bool inPlace = req.outputOffset == req.colorOffset;
  bool ok = sliceOk(req.colorOffset, bufBytes, req.shmSize) &&
            sliceOk(req.outputOffset, bufBytes, req.shmSize);
  if (hasAlbedo) ok = ok && sliceOk(req.albedoOffset, bufBytes, req.shmSize);
  if (hasNormal) ok = ok && sliceOk(req.normalOffset, bufBytes, req.shmSize);
  // The output may alias the color input (in-place, OIDN-supported) but must
  // not clobber a guide buffer while the filter reads it.
  if (!inPlace) {
    if (hasAlbedo)
      ok = ok && !rangesOverlap(req.outputOffset, bufBytes, req.albedoOffset,
                                bufBytes);
    if (hasNormal)
      ok = ok && !rangesOverlap(req.outputOffset, bufBytes, req.normalOffset,
                                bufBytes);
  }
  if (!ok) {
    resp.status = kStatusBadRequest;
    setErrMsg(resp, "buffer offsets exceed or overlap the shm region");
    return;
  }
  if (deviceErr) {
    resp.status = kStatusDeviceFailed;
    setErrMsg(resp, deviceErr);
    return;
  }

  ShmRegion shm;
  try {
    shm = ShmRegion::open(req.regionId);
  } catch (const std::exception& e) {
    resp.status = kStatusShmOpenFailed;
    setErrMsg(resp, e.what());
    return;
  }
  if (shm.size() < req.shmSize) {
    resp.status = kStatusShmOpenFailed;
    setErrMsg(resp, "shm region smaller than declared");
    return;
  }

  char* base = static_cast<char*>(shm.data());
  float* color = reinterpret_cast<float*>(base + req.colorOffset);
  float* output = reinterpret_cast<float*>(base + req.outputOffset);

  using clock = std::chrono::high_resolution_clock;
  const auto tFil0 = clock::now();
  // A fresh filter per request keeps the cost structure of the old in-process
  // implementation (device reuse is where the savings are) and sidesteps
  // stale-size caching bugs when the resolution changes between requests.
  oidn::FilterRef filter = device.newFilter("RT");
  filter.setImage("color", color, oidn::Format::Float3, req.width, req.height);
  if (hasAlbedo)
    filter.setImage("albedo", base + req.albedoOffset, oidn::Format::Float3,
                    req.width, req.height);
  if (hasNormal)
    filter.setImage("normal", base + req.normalOffset, oidn::Format::Float3,
                    req.width, req.height);
  filter.setImage("output", output, oidn::Format::Float3, req.width,
                  req.height);
  filter.set("hdr", (req.flags & kFlagHdr) != 0);
  if (hasAlbedo || hasNormal)
    filter.set("cleanAux", (req.flags & kFlagCleanAux) != 0);
  filter.commit();
  const auto tFil1 = clock::now();
  filter.execute();
  const auto tExe1 = clock::now();
  resp.tFilter = std::chrono::duration<double>(tFil1 - tFil0).count();
  resp.tExecute = std::chrono::duration<double>(tExe1 - tFil1).count();

  const char* msg = nullptr;
  if (device.getError(msg) != oidn::Error::None) {
    resp.status = kStatusExecuteFailed;
    setErrMsg(resp, msg);
    return;
  }
  resp.status = kStatusOk;
}

}  // namespace

int main() {
#ifdef _WIN32
  // The CRT defaults stdio to text mode, which mangles 0x0A bytes and would
  // corrupt the binary protocol.
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
#endif

  // Hello goes out before the (slow) OIDN device init so the client's
  // handshake timeout only covers process startup.
  HelloMsg hello = {};
  hello.magic = kOidnMagic;
  hello.version = kOidnProtoVersion;
  hello.op = kOpHello;
  hello.oidnVersion = OIDN_VERSION;
  if (!writeExact(&hello, sizeof(hello))) return 1;

  using clock = std::chrono::high_resolution_clock;
  const auto tDev0 = clock::now();
  oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
  device.commit();
  const auto tDev1 = clock::now();
  double deviceSeconds = std::chrono::duration<double>(tDev1 - tDev0).count();
  const char* deviceErrPtr = nullptr;
  char deviceErrBuf[216] = {0};
  {
    const char* msg = nullptr;
    if (device.getError(msg) != oidn::Error::None) {
      std::snprintf(deviceErrBuf, sizeof(deviceErrBuf), "%s",
                    msg ? msg : "OIDN device init failed");
      deviceErrPtr = deviceErrBuf;
    }
  }

  bool firstReply = true;
  for (;;) {
    DenoiseRequest req;
    if (!readExact(&req, sizeof(req))) return 0;  // EOF: client is gone

    DenoiseResponse resp = {};
    resp.magic = kOidnMagic;
    resp.version = kOidnProtoVersion;
    resp.op = kOpDenoiseReply;

    if (req.magic != kOidnMagic || req.version != kOidnProtoVersion ||
        req.op != kOpDenoise) {
      // Framing is lost; message boundaries can no longer be trusted.
      std::fprintf(stderr,
                   "umbreon_oidn_worker: protocol framing lost "
                   "(magic/version/op mismatch); exiting\n");
      resp.status = kStatusBadRequest;
      setErrMsg(resp, "protocol framing lost");
      writeExact(&resp, sizeof(resp));
      return 1;
    }
    req.regionId[sizeof(req.regionId) - 1] = '\0';

    try {
      handleDenoise(req, device, deviceErrPtr, resp);
    } catch (const std::exception& e) {
      resp.status = kStatusExecuteFailed;
      setErrMsg(resp, e.what());
    } catch (...) {
      resp.status = kStatusExecuteFailed;
      setErrMsg(resp, "unknown exception");
    }
    resp.tDevice = firstReply ? deviceSeconds : 0.0;
    firstReply = false;

    if (!writeExact(&resp, sizeof(resp))) return 1;  // client is gone
  }
}
