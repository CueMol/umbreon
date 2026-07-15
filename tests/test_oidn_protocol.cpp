// OIDN IPC wire-protocol unit tests: pinned struct layouts, trivially
// copyable round-trips (the structs are sent as raw bytes over a pipe), and
// the shared-memory name generator's macOS length budget. Boost-free and
// umbreon-free on purpose: the protocol header must stay self-contained so
// this test builds on a bare configure without Embree/TBB/Boost.
#include "ipc/oidn_protocol.hpp"

#include <cstddef>
#include <cstring>
#include <string>
#include <type_traits>

#include "test_util.hpp"

using namespace umbreon::ipc;

int main() {
  umbreon::test::Suite s("oidn_protocol");

  // Layout pins. The static_asserts in the header enforce the sizes at
  // compile time; re-check here so a report names the struct that moved.
  s.check_eq("HelloMsg size", sizeof(HelloMsg), std::size_t(24));
  s.check_eq("DenoiseRequest size", sizeof(DenoiseRequest), std::size_t(256));
  s.check_eq("DenoiseResponse size", sizeof(DenoiseResponse), std::size_t(256));
  s.check("HelloMsg trivially copyable",
          std::is_trivially_copyable<HelloMsg>::value);
  s.check("DenoiseRequest trivially copyable",
          std::is_trivially_copyable<DenoiseRequest>::value);
  s.check("DenoiseResponse trivially copyable",
          std::is_trivially_copyable<DenoiseResponse>::value);
  s.check_eq("regionId field offset", offsetof(DenoiseRequest, regionId),
             std::size_t(64));
  s.check_eq("errMsg field offset", offsetof(DenoiseResponse, errMsg),
             std::size_t(40));

  // Raw-byte round-trip: what goes over the pipe must reconstruct the struct.
  DenoiseRequest req = {};
  req.magic = kOidnMagic;
  req.version = kOidnProtoVersion;
  req.op = kOpDenoise;
  req.flags = kFlagHasAlbedo | kFlagHasNormal | kFlagHdr | kFlagCleanAux;
  req.width = 1920;
  req.height = 1080;
  req.shmSize = 1920ull * 1080ull * 3ull * 4ull * 3ull;
  req.colorOffset = 0;
  req.albedoOffset = 1920ull * 1080ull * 12ull;
  req.normalOffset = req.albedoOffset * 2;
  req.outputOffset = req.colorOffset;  // in-place
  std::snprintf(req.regionId, sizeof(req.regionId), "/tmp/um-12345-7");

  unsigned char wire[sizeof(DenoiseRequest)];
  std::memcpy(wire, &req, sizeof(req));
  DenoiseRequest back = {};
  std::memcpy(&back, wire, sizeof(back));
  s.check("request round-trips through raw bytes",
          std::memcmp(&req, &back, sizeof(req)) == 0);
  s.check_eq("round-trip width", back.width, 1920);
  s.check_eq("round-trip flags", back.flags,
             std::uint32_t(kFlagHasAlbedo | kFlagHasNormal | kFlagHdr |
                           kFlagCleanAux));
  s.check("round-trip in-place output", back.outputOffset == back.colorOffset);
  s.check_eq("round-trip region id", std::string(back.regionId),
             std::string("/tmp/um-12345-7"));

  // Flag bits are distinct.
  s.check("flag bits disjoint",
          (kFlagHasAlbedo & kFlagHasNormal) == 0u &&
              (kFlagHdr & kFlagCleanAux) == 0u &&
              ((kFlagHasAlbedo | kFlagHasNormal) & (kFlagHdr | kFlagCleanAux)) ==
                  0u);

  // makeRegionBasename: unique per (pid, counter). The client prepends the
  // temp dir on POSIX / uses it as the kernel object name on Windows; it must
  // fit the 192-byte regionId once a temp-dir prefix is added.
  char base[64];
  makeRegionBasename(base, sizeof(base), 0xffffffffull, 0xffffffffu);
  s.check_eq("worst-case basename", std::string(base),
             std::string("um-4294967295-4294967295"));
  s.check("worst-case basename leaves room in regionId",
          std::strlen(base) < sizeof(DenoiseRequest::regionId));
  // pid is truncated to 32 bits so a 64-bit pid cannot overflow the budget.
  char base2[64];
  makeRegionBasename(base2, sizeof(base2), 0x1'0000'0001ull, 1);
  s.check_eq("pid truncated to 32 bits", std::string(base2),
             std::string("um-1-1"));
  // Distinct counters give distinct basenames (resize-by-recreate relies on it).
  char base3[64];
  makeRegionBasename(base3, sizeof(base3), 0x1'0000'0001ull, 2);
  s.check("counter bump changes the basename",
          std::string(base2) != std::string(base3));

  return s.report();
}
