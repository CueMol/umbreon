// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Thin RAII wrapper over a shared memory region, used by the OIDN IPC client
// (creator side, ipc/oidn_client.cpp) and the umbreon_oidn_worker executable
// (opener side). Header-only; the only Boost usage outside oidn_client.cpp's
// translation unit and the worker binary.
//
// Backend choice: on POSIX the region is a memory-mapped ordinary FILE
// (boost::interprocess::file_mapping), NOT POSIX shm. macOS caps POSIX/SysV
// shared memory at a small size, whereas a mapped file is bounded only by
// free disk space and the process address space (effectively unbounded on
// 64-bit) -- a 4K denoise needs a ~285 MB region. The identifier is therefore
// an absolute file path (the client owns a temp file). On Windows the region
// is a windows_shared_memory kernel object (pagefile-backed, no small cap,
// reference-counted so a crashed client leaves no stale region); the
// identifier is the kernel object name. Either way the client must keep its
// ShmRegion alive until the worker's reply arrives (create -> send request ->
// worker opens; never send before creating).
//
// Regions are never resized (a fresh region under a bumped-counter name is
// created when a frame outgrows the current one), so no ftruncate/resize
// dance is needed after creation.
#pragma once

#ifdef _WIN32
#include <boost/interprocess/windows_shared_memory.hpp>
#else
#include <boost/interprocess/file_mapping.hpp>
#include <cstdio>
#include <filesystem>
#endif
#include <boost/interprocess/mapped_region.hpp>

#include <cstddef>
#include <string>
#include <utility>

namespace umbreon {
namespace ipc {

class ShmRegion {
 public:
  ShmRegion() = default;
  ShmRegion(const ShmRegion&) = delete;
  ShmRegion& operator=(const ShmRegion&) = delete;
  ShmRegion(ShmRegion&& other) noexcept { swap(other); }
  ShmRegion& operator=(ShmRegion&& other) noexcept {
    if (this != &other) {
      reset();
      swap(other);
    }
    return *this;
  }
  ~ShmRegion() { reset(); }

  // Creator side (the IPC client). id is an absolute file path (POSIX) or a
  // kernel object name (Windows). Removes any stale same-id region first, then
  // creates and maps a read-write region of the given size. Throws
  // boost::interprocess::interprocess_exception (or std::filesystem_error on
  // POSIX file setup) on failure.
  static ShmRegion create(const std::string& id, std::size_t bytes) {
    namespace bip = boost::interprocess;
    ShmRegion r;
    r.id_ = id;
#ifdef _WIN32
    bip::windows_shared_memory shm(bip::create_only, id.c_str(),
                                   bip::read_write, bytes);
    r.region_ = bip::mapped_region(shm, bip::read_write);
#else
    // Create the backing file at exactly `bytes` (sparse: only touched pages
    // are committed, so albedo/normal-less frames do not pay for the slack).
    std::error_code ec;
    std::filesystem::remove(id, ec);  // stale file from a prior crashed run
    if (std::FILE* f = std::fopen(id.c_str(), "wb")) std::fclose(f);
    std::filesystem::resize_file(id, bytes);  // throws on failure
    bip::file_mapping fm(id.c_str(), bip::read_write);
    r.region_ = bip::mapped_region(fm, bip::read_write);
    r.removeOnDestroy_ = true;
#endif
    return r;
  }

  // Opener side (the worker). Maps an existing region read-write; never
  // removes it. Throws on failure.
  static ShmRegion open(const std::string& id) {
    namespace bip = boost::interprocess;
    ShmRegion r;
    r.id_ = id;
#ifdef _WIN32
    bip::windows_shared_memory shm(bip::open_only, id.c_str(),
                                   bip::read_write);
    r.region_ = bip::mapped_region(shm, bip::read_write);
#else
    bip::file_mapping fm(id.c_str(), bip::read_write);
    r.region_ = bip::mapped_region(fm, bip::read_write);
#endif
    return r;
  }

  void* data() const { return region_.get_address(); }
  std::size_t size() const { return region_.get_size(); }
  bool valid() const { return region_.get_size() != 0; }
  const std::string& id() const { return id_; }

  // Unmap and, on the creator side of a POSIX region, delete the backing file.
  void reset() {
    region_ = boost::interprocess::mapped_region();
#ifndef _WIN32
    if (removeOnDestroy_) {
      std::error_code ec;
      std::filesystem::remove(id_, ec);
    }
#endif
    removeOnDestroy_ = false;
    id_.clear();
  }

 private:
  void swap(ShmRegion& other) noexcept {
    region_.swap(other.region_);
    id_.swap(other.id_);
    std::swap(removeOnDestroy_, other.removeOnDestroy_);
  }

  boost::interprocess::mapped_region region_;
  std::string id_;
  bool removeOnDestroy_ = false;
};

}  // namespace ipc
}  // namespace umbreon
