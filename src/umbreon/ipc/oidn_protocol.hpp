// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Wire protocol between libumbreon's OIDN IPC client (ipc/oidn_client.cpp) and
// the out-of-process denoiser executable (src/oidn_worker/main.cpp). Control
// messages travel over the worker's stdin/stdout pipes; image buffers travel
// through a shared-memory region owned by the client (ipc/shm_region.hpp).
//
// Framing: every message is exactly one fixed-size struct (no length prefix).
// Both endpoints are built from the same source tree and run on the same
// machine, so native endianness and native struct layout are assumed; the
// layout is still pinned with fixed-width fields and static_asserts so an
// accidental padding change breaks the build, not the protocol. Any change to
// a struct or its semantics must bump kOidnProtoVersion; the client refuses a
// worker whose HelloMsg carries a different version (stale binary).
//
// This header is self-contained (no Boost, no OIDN) so the protocol can be
// unit-tested without either dependency.
#pragma once

#include <cstdint>
#include <cstdio>

namespace umbreon {
namespace ipc {

constexpr std::uint32_t kOidnMagic = 0x4E444F55u;  // "UODN" on little-endian
constexpr std::uint32_t kOidnProtoVersion = 1;

// Message opcodes.
enum : std::uint32_t {
  kOpHello = 1,         // worker -> client, once, immediately after startup
  kOpDenoise = 2,       // client -> worker
  kOpDenoiseReply = 3,  // worker -> client
};

// DenoiseRequest::flags bits.
enum : std::uint32_t {
  kFlagHasAlbedo = 1u << 0,  // albedoOffset holds a valid guide buffer
  kFlagHasNormal = 1u << 1,  // normalOffset holds a valid guide buffer
  kFlagHdr = 1u << 2,        // color is linear HDR (OIDN "hdr" parameter)
  kFlagCleanAux = 1u << 3,   // guides are noise-free (OIDN "cleanAux")
};

// DenoiseResponse::status values.
enum : std::int32_t {
  kStatusOk = 0,
  kStatusBadRequest = 1,     // malformed / out-of-range request fields
  kStatusShmOpenFailed = 2,  // shared-memory region missing or too small
  kStatusDeviceFailed = 3,   // OIDN device creation failed at worker startup
  kStatusExecuteFailed = 4,  // OIDN filter commit/execute reported an error
};

// Sent by the worker exactly once, before the OIDN device is created, so the
// client's handshake completes fast even though device init may take a while.
struct HelloMsg {
  std::uint32_t magic;        // kOidnMagic
  std::uint32_t version;      // kOidnProtoVersion
  std::uint32_t op;           // kOpHello
  std::uint32_t oidnVersion;  // OIDN_VERSION of the worker build (informational)
  std::uint32_t reserved[2];  // zero
};
static_assert(sizeof(HelloMsg) == 24, "HelloMsg layout is pinned");

// One denoise job. All buffers are float triplets (RGB / xyz), tightly packed
// N = width*height pixels, living at the given byte offsets inside the shared
// region identified by regionId. outputOffset == colorOffset requests in-place
// denoising (supported by OIDN and used by the client to halve the region).
struct DenoiseRequest {
  std::uint32_t magic;    // kOidnMagic
  std::uint32_t version;  // kOidnProtoVersion
  std::uint32_t op;       // kOpDenoise
  std::uint32_t flags;    // kFlag* bits
  std::int32_t width;
  std::int32_t height;
  std::uint64_t shmSize;       // total region size in bytes (for validation)
  std::uint64_t colorOffset;   // input color, N*3 floats
  std::uint64_t albedoOffset;  // guide, N*3 floats, valid iff kFlagHasAlbedo
  std::uint64_t normalOffset;  // guide, N*3 floats, valid iff kFlagHasNormal
  std::uint64_t outputOffset;  // denoised color, N*3 floats
  // Region identifier: on POSIX the absolute path of the memory-mapped file
  // (client-owned temp file, see ipc/shm_region.hpp); on Windows the name of
  // the windows_shared_memory kernel object. NUL-terminated. 192 bytes fit a
  // long temp-dir path plus the "um-<pid>-<counter>" basename.
  char regionId[192];
};
static_assert(sizeof(DenoiseRequest) == 256, "DenoiseRequest layout is pinned");

// Reply to one DenoiseRequest. The timing fields feed the client's pt1Stats
// diagnostic line; tDevice is the one-time OIDN device init cost, reported on
// the first reply of a worker's lifetime and 0.0 afterwards (the persistent
// worker reuses the device across requests).
struct DenoiseResponse {
  std::uint32_t magic;    // kOidnMagic
  std::uint32_t version;  // kOidnProtoVersion
  std::uint32_t op;       // kOpDenoiseReply
  std::int32_t status;    // kStatus*
  double tDevice;         // seconds; > 0 only on the first reply
  double tFilter;         // seconds; filter create + commit
  double tExecute;        // seconds; filter execute
  char errMsg[216];       // NUL-terminated detail when status != kStatusOk
};
static_assert(sizeof(DenoiseResponse) == 256,
              "DenoiseResponse layout is pinned");

// Unique basename for a region: "um-<pid>-<counter>". The client prepends the
// temp directory on POSIX (the file path) and uses it verbatim as the kernel
// object name on Windows. pid (truncated to 32 bits; real pids fit) plus a
// per-process counter avoids stale-name collisions across client restarts.
inline void makeRegionBasename(char* out, std::size_t cap, std::uint64_t pid,
                               std::uint32_t counter) {
  std::snprintf(out, cap, "um-%u-%u",
                static_cast<unsigned>(pid & 0xffffffffu), counter);
}

}  // namespace ipc
}  // namespace umbreon
