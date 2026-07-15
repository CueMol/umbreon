// libumbreon INTERNAL header -- not installed, not part of the public API.
// Implementation detail; may change without notice. Do not include downstream.
//
// Thin RAII wrapper over Boost.Interprocess shared memory, used by the OIDN
// IPC client (creator side, ipc/oidn_client.cpp) and the umbreon_oidn_worker
// executable (opener side). Header-only; the only Boost usage outside
// oidn_client.cpp's translation unit and the worker binary.
//
// Platform choice: on POSIX this is a shared_memory_object (real shm; the
// creator must remove it, which the destructor does). On Windows it is a
// windows_shared_memory kernel object: reference-counted, vanishing with the
// last open handle, so a crashed client can never leave a stale region. That
// also means the region only stays openable while the creator holds it --
// the client must keep its ShmRegion alive until the worker's reply arrives
// (create -> send request -> worker opens; never send before creating).
//
// macOS gotcha: a POSIX shm object can be ftruncate'd only once, so a region
// is never resized. When a frame needs more bytes, destroy the region and
// create a fresh one under a new name (ipc::makeShmName with a bumped
// counter).
#pragma once

#ifdef _WIN32
#include <boost/interprocess/windows_shared_memory.hpp>
#else
#include <boost/interprocess/shared_memory_object.hpp>
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

  // Creator side (the IPC client). Removes any stale same-name POSIX region
  // first, then creates and maps a read-write region of the given size.
  // Throws boost::interprocess::interprocess_exception on failure.
  static ShmRegion create(const std::string& name, std::size_t bytes) {
    namespace bip = boost::interprocess;
    ShmRegion r;
    r.name_ = name;
#ifdef _WIN32
    bip::windows_shared_memory shm(bip::create_only, name.c_str(),
                                   bip::read_write, bytes);
    r.region_ = bip::mapped_region(shm, bip::read_write);
#else
    bip::shared_memory_object::remove(name.c_str());
    bip::shared_memory_object shm(bip::create_only, name.c_str(),
                                  bip::read_write);
    shm.truncate(static_cast<bip::offset_t>(bytes));  // once, never resized
    r.region_ = bip::mapped_region(shm, bip::read_write);
    r.removeOnDestroy_ = true;
#endif
    return r;
  }

  // Opener side (the worker). Maps an existing region read-write; never
  // removes it. Throws boost::interprocess::interprocess_exception on failure.
  static ShmRegion open(const std::string& name) {
    namespace bip = boost::interprocess;
    ShmRegion r;
    r.name_ = name;
#ifdef _WIN32
    bip::windows_shared_memory shm(bip::open_only, name.c_str(),
                                   bip::read_write);
#else
    bip::shared_memory_object shm(bip::open_only, name.c_str(),
                                  bip::read_write);
#endif
    r.region_ = bip::mapped_region(shm, bip::read_write);
    return r;
  }

  void* data() const { return region_.get_address(); }
  std::size_t size() const { return region_.get_size(); }
  bool valid() const { return region_.get_size() != 0; }
  const std::string& name() const { return name_; }

  // Unmap and, on the creator side of a POSIX region, remove the named object.
  void reset() {
    region_ = boost::interprocess::mapped_region();
#ifndef _WIN32
    if (removeOnDestroy_)
      boost::interprocess::shared_memory_object::remove(name_.c_str());
#endif
    removeOnDestroy_ = false;
    name_.clear();
  }

 private:
  void swap(ShmRegion& other) noexcept {
    region_.swap(other.region_);
    name_.swap(other.name_);
    std::swap(removeOnDestroy_, other.removeOnDestroy_);
  }

  boost::interprocess::mapped_region region_;
  std::string name_;
  bool removeOnDestroy_ = false;
};

}  // namespace ipc
}  // namespace umbreon
