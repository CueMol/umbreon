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
  s.check_eq("DenoiseRequest size", sizeof(DenoiseRequest), std::size_t(128));
  s.check_eq("DenoiseResponse size", sizeof(DenoiseResponse), std::size_t(256));
  s.check("HelloMsg trivially copyable",
          std::is_trivially_copyable<HelloMsg>::value);
  s.check("DenoiseRequest trivially copyable",
          std::is_trivially_copyable<DenoiseRequest>::value);
  s.check("DenoiseResponse trivially copyable",
          std::is_trivially_copyable<DenoiseResponse>::value);
  s.check_eq("shmName field offset", offsetof(DenoiseRequest, shmName),
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
  makeShmName(req.shmName, 12345, 7);

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
  s.check_eq("round-trip shm name", std::string(back.shmName),
             std::string("um-12345-7"));

  // Flag bits are distinct.
  s.check("flag bits disjoint",
          (kFlagHasAlbedo & kFlagHasNormal) == 0u &&
              (kFlagHdr & kFlagCleanAux) == 0u &&
              ((kFlagHasAlbedo | kFlagHasNormal) & (kFlagHdr | kFlagCleanAux)) ==
                  0u);

  // Shared-memory names must fit the macOS 31-character shm limit (with the
  // implementation's leading '/'), i.e. <= 30 visible characters, even for
  // the worst-case 32-bit pid and counter.
  char name[64];
  makeShmName(name, 0xffffffffull, 0xffffffffu);
  s.check("worst-case shm name fits macOS limit", std::strlen(name) <= 30);
  s.check_eq("worst-case shm name", std::string(name),
             std::string("um-4294967295-4294967295"));
  // pid is truncated to 32 bits so a 64-bit pid cannot overflow the budget.
  char name2[64];
  makeShmName(name2, 0x1'0000'0001ull, 1);
  s.check_eq("pid truncated to 32 bits", std::string(name2),
             std::string("um-1-1"));
  // Distinct counters give distinct names (resize-by-recreate relies on it).
  char name3[64];
  makeShmName(name3, 0x1'0000'0001ull, 2);
  s.check("counter bump changes the name",
          std::string(name2) != std::string(name3));

  return s.report();
}
