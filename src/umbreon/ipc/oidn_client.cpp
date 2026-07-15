// OIDN IPC client implementation. The only libumbreon translation unit that
// includes Boost (Process v2 for spawning/pipes, Interprocess via
// ipc/shm_region.hpp); compiled only when the build enables the OIDN backend
// (CMake: UMBREON_WITH_OIDN + Boost found => UMBREON_HAVE_OIDN).
//
// Signal disclosure: on POSIX the constructor sets SIGPIPE to SIG_IGN if (and
// only if) the disposition is still SIG_DFL, so a write to a dead worker
// surfaces as EPIPE instead of killing the host process. A host application
// that installed its own handler is left untouched.
#include "ipc/oidn_client.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <optional>

#ifndef _WIN32
#include <signal.h>
#endif

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/read.hpp>
#include <boost/process/v2/environment.hpp>
#include <boost/process/v2/ext/exe.hpp>
#include <boost/process/v2/pid.hpp>
#include <boost/process/v2/popen.hpp>
#ifdef _WIN32
#include <boost/process/v2/windows/creation_flags.hpp>
#endif

#include "ipc/oidn_protocol.hpp"
#include "ipc/shm_region.hpp"

namespace umbreon {
namespace ipc {

namespace bp2 = boost::process::v2;
namespace asio = boost::asio;

namespace {

#ifdef _WIN32
constexpr const char* kWorkerName = "umbreon_oidn_worker.exe";
#else
constexpr const char* kWorkerName = "umbreon_oidn_worker";
#endif

bool writeExact(bp2::popen& p, const void* src, std::size_t n) {
  const char* c = static_cast<const char*>(src);
  while (n > 0) {
    boost::system::error_code ec;
    const std::size_t k = p.write_some(asio::buffer(c, n), ec);
    if (ec || k == 0) return false;
    c += k;
    n -= k;
  }
  return true;
}

bool readExact(bp2::popen& p, void* dst, std::size_t n) {
  char* c = static_cast<char*>(dst);
  while (n > 0) {
    boost::system::error_code ec;
    const std::size_t k = p.read_some(asio::buffer(c, n), ec);
    if (ec || k == 0) return false;
    c += k;
    n -= k;
  }
  return true;
}

std::uint64_t align64(std::uint64_t x) { return (x + 63u) & ~std::uint64_t(63); }

// Resolve the worker executable. An explicit path is authoritative: it is
// returned as-is (existing or not) so a misconfiguration fails loudly instead
// of silently running some other worker found on PATH.
std::filesystem::path resolveWorkerPath(const std::string& explicitPath) {
  namespace fs = std::filesystem;
  if (!explicitPath.empty()) return fs::path(explicitPath);
  if (const char* env = std::getenv("UMBREON_OIDN_WORKER")) {
    if (*env != '\0') return fs::path(env);
  }
  boost::system::error_code bec;
  const fs::path self = bp2::ext::exe(bp2::current_pid(), bec);
  if (!bec && !self.empty()) {
    const fs::path beside = self.parent_path() / kWorkerName;
    std::error_code fec;
    if (fs::exists(beside, fec)) return beside;
  }
  return bp2::environment::find_executable(kWorkerName);  // empty if absent
}

}  // namespace

struct OidnClient::Impl {
  enum class WorkerState { Idle, Running, Dead, Disabled };

  std::mutex mu;
  // ctx is declared before proc so pending asio handlers outlive the process
  // object during destruction.
  asio::io_context ctx;
  std::optional<bp2::popen> proc;
  WorkerState state = WorkerState::Idle;
  int deathsSinceSuccess = 0;
  std::string disabledPath;  // explicitPath in effect when disabled
  bool firstDenoise = true;

  ShmRegion shm;
  std::uint32_t shmCounter = 0;
  std::uint64_t colorOff = 0;
  std::uint64_t albedoOff = 0;
  std::uint64_t normalOff = 0;

  ~Impl() {
    if (state == WorkerState::Running && proc) {
      // Closing stdin asks the worker to exit on its own; escalate to a kill
      // only if it does not oblige. No logging here: this runs during static
      // teardown.
      boost::system::error_code ec;
      proc->get_stdin().close(ec);
      bool exited = false;
      proc->async_wait([&exited](boost::system::error_code, int) {
        exited = true;
      });
      ctx.restart();
      ctx.run_for(std::chrono::seconds(2));
      if (!exited) {
        proc->terminate(ec);
        ctx.restart();
        ctx.run_for(std::chrono::seconds(1));
      }
      proc.reset();
    }
  }

  void disable(const std::string& explicitPath) {
    if (proc) {
      boost::system::error_code ec;
      proc->terminate(ec);
      proc->wait(ec);
      proc.reset();
    }
    state = WorkerState::Disabled;
    disabledPath = explicitPath;
  }

  void onWorkerDeath() {
    std::fprintf(stderr,
                 "warning: OIDN worker died mid-request; falling back to the "
                 "built-in a-trous denoiser for this frame\n");
    if (proc) {
      boost::system::error_code ec;
      proc->terminate(ec);
      proc->wait(ec);
      proc.reset();
    }
    state = WorkerState::Dead;
    ++deathsSinceSuccess;
  }

  // Spawns the worker and completes the Hello handshake. On failure prints
  // one detailed warning and disables the client (a later begin() with a
  // different explicit path re-enables it).
  bool spawnWorker(const std::string& explicitPath) {
    const std::filesystem::path exe = resolveWorkerPath(explicitPath);
    if (exe.empty()) {
      std::fprintf(stderr,
                   "warning: OIDN worker executable (%s) not found -- set "
                   "RenderOptions::oidnWorkerPath, UMBREON_OIDN_WORKER, or "
                   "PATH; OIDN denoising disabled\n",
                   kWorkerName);
      disable(explicitPath);
      return false;
    }
    try {
#ifdef _WIN32
      // CREATE_NO_WINDOW: never flash a console when the host is a GUI app.
      proc.emplace(ctx.get_executor(), exe, std::initializer_list<bp2::string_view>{},
                   bp2::windows::process_creation_flags<CREATE_NO_WINDOW>{});
#else
      proc.emplace(ctx.get_executor(), exe,
                   std::initializer_list<bp2::string_view>{});
#endif
    } catch (const std::exception& e) {
      std::fprintf(stderr,
                   "warning: failed to start OIDN worker '%s' (%s); OIDN "
                   "denoising disabled\n",
                   exe.string().c_str(), e.what());
      disable(explicitPath);
      return false;
    }

    // The worker sends Hello before creating the OIDN device, so 5 seconds
    // only needs to cover process startup. An exec failure surfaces as EOF.
    HelloMsg hello = {};
    boost::system::error_code readEc = asio::error::would_block;
    std::size_t got = 0;
    asio::async_read(proc->get_stdout(), asio::buffer(&hello, sizeof(hello)),
                     [&](boost::system::error_code ec, std::size_t n) {
                       readEc = ec;
                       got = n;
                     });
    ctx.restart();
    ctx.run_for(std::chrono::seconds(5));
    if (readEc == asio::error::would_block) {  // handler never ran: timeout
      boost::system::error_code ec;
      proc->get_stdout().cancel(ec);
      ctx.restart();
      ctx.poll();
      std::fprintf(stderr,
                   "warning: OIDN worker '%s' handshake timed out; OIDN "
                   "denoising disabled\n",
                   exe.string().c_str());
      disable(explicitPath);
      return false;
    }
    if (readEc || got != sizeof(hello) || hello.magic != kOidnMagic ||
        hello.op != kOpHello) {
      std::fprintf(stderr,
                   "warning: OIDN worker '%s' failed the handshake; OIDN "
                   "denoising disabled\n",
                   exe.string().c_str());
      disable(explicitPath);
      return false;
    }
    if (hello.version != kOidnProtoVersion) {
      std::fprintf(stderr,
                   "warning: OIDN worker '%s' speaks protocol %u, this "
                   "library expects %u (stale binary?); OIDN denoising "
                   "disabled\n",
                   exe.string().c_str(), hello.version, kOidnProtoVersion);
      disable(explicitPath);
      return false;
    }
    state = WorkerState::Running;
    return true;
  }

  bool ensureWorker(const std::string& explicitPath) {
    if (state == WorkerState::Disabled) {
      if (explicitPath == disabledPath) return false;  // stay quiet
      state = WorkerState::Idle;  // new path: give it one fresh chance
      deathsSinceSuccess = 0;
    }
    if (state == WorkerState::Running) return true;
    if (state == WorkerState::Dead && deathsSinceSuccess >= 2) {
      std::fprintf(stderr,
                   "warning: OIDN worker keeps dying; OIDN denoising "
                   "disabled\n");
      disable(explicitPath);
      return false;
    }
    return spawnWorker(explicitPath);
  }

  // Lays out (and if needed (re)creates) the shared-memory region for a
  // width*height frame. Regions are never resized (macOS shm objects cannot
  // be re-truncated): when the frame outgrows the region, a fresh one is
  // created under a bumped-counter name and the old one is removed.
  bool ensureShm(int w, int h, bool hasAlbedo, bool hasNormal) {
    const std::uint64_t bufBytes =
        std::uint64_t(w) * std::uint64_t(h) * 3u * sizeof(float);
    colorOff = 0;
    std::uint64_t end = align64(bufBytes);
    albedoOff = 0;
    if (hasAlbedo) {
      albedoOff = end;
      end = align64(end + bufBytes);
    }
    normalOff = 0;
    if (hasNormal) {
      normalOff = end;
      end += bufBytes;
    }
    if (shm.valid() && shm.size() >= end) return true;
    shm.reset();
    char name[64];
    makeShmName(name, static_cast<std::uint64_t>(bp2::current_pid()),
                ++shmCounter);
    try {
      shm = ShmRegion::create(name, static_cast<std::size_t>(end));
    } catch (const std::exception& e) {
      std::fprintf(stderr,
                   "warning: failed to create the OIDN shared-memory region "
                   "(%llu bytes: %s)\n",
                   static_cast<unsigned long long>(end), e.what());
      return false;  // transient (e.g. memory pressure): do not disable
    }
    return true;
  }
};

struct OidnClient::Session::State {
  std::unique_lock<std::mutex> lock;
  OidnClient::Impl* impl = nullptr;  // null => invalid session
  int w = 0;
  int h = 0;
  bool hasAlbedo = false;
  bool hasNormal = false;
};

OidnClient::Session::Session(std::unique_ptr<State> st) : st_(std::move(st)) {}
OidnClient::Session::Session(Session&&) noexcept = default;
OidnClient::Session::~Session() = default;

bool OidnClient::Session::valid() const { return st_->impl != nullptr; }

float* OidnClient::Session::color() {
  Impl* im = st_->impl;
  return reinterpret_cast<float*>(static_cast<char*>(im->shm.data()) +
                                  im->colorOff);
}

float* OidnClient::Session::albedo() {
  Impl* im = st_->impl;
  if (!st_->hasAlbedo) return nullptr;
  return reinterpret_cast<float*>(static_cast<char*>(im->shm.data()) +
                                  im->albedoOff);
}

float* OidnClient::Session::normal() {
  Impl* im = st_->impl;
  if (!st_->hasNormal) return nullptr;
  return reinterpret_cast<float*>(static_cast<char*>(im->shm.data()) +
                                  im->normalOff);
}

bool OidnClient::Session::run(bool cleanAux, bool stats) {
  Impl* im = st_->impl;
  DenoiseRequest req = {};
  req.magic = kOidnMagic;
  req.version = kOidnProtoVersion;
  req.op = kOpDenoise;
  req.flags = kFlagHdr;  // frame.color is always linear HDR here
  if (st_->hasAlbedo) req.flags |= kFlagHasAlbedo;
  if (st_->hasNormal) req.flags |= kFlagHasNormal;
  if (cleanAux) req.flags |= kFlagCleanAux;
  req.width = st_->w;
  req.height = st_->h;
  req.shmSize = im->shm.size();
  req.colorOffset = im->colorOff;
  req.albedoOffset = im->albedoOff;
  req.normalOffset = im->normalOff;
  req.outputOffset = im->colorOff;  // in-place (OIDN-supported)
  std::snprintf(req.shmName, sizeof(req.shmName), "%s",
                im->shm.name().c_str());

  if (!writeExact(*im->proc, &req, sizeof(req))) {
    im->onWorkerDeath();
    return false;
  }
  DenoiseResponse resp = {};
  if (!readExact(*im->proc, &resp, sizeof(resp))) {
    im->onWorkerDeath();
    return false;
  }
  if (resp.magic != kOidnMagic || resp.version != kOidnProtoVersion ||
      resp.op != kOpDenoiseReply) {
    // Message boundaries can no longer be trusted; drop the worker.
    std::fprintf(stderr,
                 "warning: OIDN worker protocol desync; OIDN denoising "
                 "disabled\n");
    im->disable(std::string());
    return false;
  }
  if (resp.status != kStatusOk) {
    resp.errMsg[sizeof(resp.errMsg) - 1] = '\0';
    std::fprintf(stderr,
                 "warning: OIDN worker reported an error (status %d: %s); "
                 "falling back to the built-in a-trous denoiser for this "
                 "frame\n",
                 static_cast<int>(resp.status), resp.errMsg);
    return false;  // worker is healthy; keep it
  }
  if (stats)
    std::fprintf(stderr,
                 "oidn: device %.3fs  filter %.3fs  execute %.3fs (%dx%d)\n",
                 resp.tDevice, resp.tFilter, resp.tExecute, st_->w, st_->h);
  im->deathsSinceSuccess = 0;
  return true;
}

OidnClient& OidnClient::instance() {
  static OidnClient client;
  return client;
}

OidnClient::OidnClient() : impl_(new Impl) {
#ifndef _WIN32
  struct sigaction cur;
  if (sigaction(SIGPIPE, nullptr, &cur) == 0 && cur.sa_handler == SIG_DFL) {
    struct sigaction ign;
    std::memset(&ign, 0, sizeof(ign));
    ign.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &ign, nullptr);
  }
#endif
}

OidnClient::~OidnClient() = default;

OidnClient::Session OidnClient::begin(int width, int height, bool hasAlbedo,
                                      bool hasNormal,
                                      const std::string& explicitPath) {
  std::unique_ptr<Session::State> st(new Session::State);
  st->lock = std::unique_lock<std::mutex>(impl_->mu);
  if (impl_->ensureWorker(explicitPath) &&
      impl_->ensureShm(width, height, hasAlbedo, hasNormal)) {
    st->impl = impl_.get();
    st->w = width;
    st->h = height;
    st->hasAlbedo = hasAlbedo;
    st->hasNormal = hasNormal;
  } else {
    st->lock.unlock();  // invalid session: nothing left to protect
  }
  return Session(std::move(st));
}

}  // namespace ipc
}  // namespace umbreon
