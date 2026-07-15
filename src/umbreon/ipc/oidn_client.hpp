// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// IPC client for the out-of-process OIDN denoiser (umbreon_oidn_worker). The
// worker is spawned lazily on the first denoise, kept alive for the rest of
// the host process (the OIDN device init cost is paid once) and driven with
// fixed-size messages over its stdin/stdout (ipc/oidn_protocol.hpp); image
// buffers live in a client-owned shared-memory region the accessors below
// point into, so a denoise costs the same number of buffer copies as the old
// in-process implementation.
//
// This header is deliberately Boost-free (pimpl); ipc/oidn_client.cpp is the
// only libumbreon translation unit that includes Boost.Process/Interprocess.
#pragma once

#include <memory>
#include <string>

namespace umbreon {
namespace ipc {

class OidnClient {
 public:
  // Process-wide singleton. The destructor (static teardown) closes the
  // worker's stdin, which makes it exit on its own; if the host dies without
  // running destructors the OS closes the pipe and the worker exits anyway.
  static OidnClient& instance();

  // One denoise transaction. Holds the client mutex for its lifetime, so
  // overlapping render()/renderAsync() denoise phases are serialized (OIDN
  // saturates the CPU internally; running two at once would not help).
  class Session {
   public:
    Session(Session&&) noexcept;
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session& operator=(Session&&) = delete;

    // False when the worker could not be started (or is disabled after
    // repeated failures) or the shared-memory region could not be created;
    // the caller should fall back to the built-in denoiser.
    bool valid() const;

    // Slices of the shared-memory region, each width*height*3 floats.
    // color() is both the input and (after a successful run()) the denoised
    // output. albedo()/normal() are null unless requested in begin().
    float* color();
    float* albedo();
    float* normal();

    // Sends the request and blocks until the worker replies. cleanAux is the
    // OIDN "cleanAux" toggle (guides are noise-free); stats prints the
    // "oidn: device/filter/execute" timing line to stderr. False on worker
    // death or an error reply: fall back for this frame (a died worker is
    // respawned on the next begin()).
    bool run(bool cleanAux, bool stats);

   private:
    friend class OidnClient;
    struct State;
    explicit Session(std::unique_ptr<State> st);
    std::unique_ptr<State> st_;
  };

  // Ensures the worker is running (lazy spawn + handshake) and the region
  // fits a width*height frame with the requested guide slices. explicitPath
  // is RenderOptions::oidnWorkerPath: when non-empty it is the only path
  // tried; when empty the search order is the UMBREON_OIDN_WORKER environment
  // variable, then umbreon_oidn_worker next to the host executable, then PATH.
  Session begin(int width, int height, bool hasAlbedo, bool hasNormal,
                const std::string& explicitPath);

  // Ensure a worker is running (resolve + spawn + handshake) WITHOUT running a
  // denoise; on success the worker stays warm. Same discovery + failure matrix
  // as begin() (a failed explicitPath latches Disabled for that path). Backs
  // the public oidnDenoiserAvailable().
  bool probe(const std::string& explicitPath);

  // Gracefully stop the worker (close stdin -> bounded wait -> terminate) and
  // release the shared region. Resets the state machine to Idle and clears the
  // death/disable latches so the next begin()/probe() spawns afresh (the shm
  // counter is NOT reset, keeping region identifiers unique). Backs the public
  // shutdownOidnDenoiser().
  void shutdown();

  OidnClient(const OidnClient&) = delete;
  OidnClient& operator=(const OidnClient&) = delete;

 private:
  OidnClient();
  ~OidnClient();

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ipc
}  // namespace umbreon
