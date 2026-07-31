#include "log.hpp"

#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace umbreon {
namespace {

// One mutex for both the sink pointer and the call: a sink installed by a GUI
// host is typically appending to a shared buffer, and the render worker thread
// is what calls it. Contention is irrelevant -- these are diagnostics, emitted
// a handful of times per render, never from a per-ray path.
std::mutex& logMutex() {
  static std::mutex m;
  return m;
}

LogSink& logSink() {
  static LogSink sink;
  return sink;
}

/// Default routing when no sink is installed: stderr, which is where every one
/// of these messages went before there was a sink. Both levels print, so
/// nothing a CLI user used to see is lost; only the "warning: " prefix is now
/// added here rather than baked into each format string.
void writeToStderr(LogLevel level, const char* text) {
  std::fprintf(stderr, "%s%s\n", level == LogLevel::Warning ? "warning: " : "",
               text);
}

}  // namespace

void setLogSink(LogSink sink) {
  std::lock_guard<std::mutex> lk(logMutex());
  logSink() = std::move(sink);
}

void logMessage(LogLevel level, const char* fmt, ...) {
  // Format first, outside the lock: vsnprintf on a caller-supplied format is
  // the expensive part and needs no serialisation.
  std::va_list ap;
  va_start(ap, fmt);
  std::va_list ap2;
  va_copy(ap2, ap);
  const int n = std::vsnprintf(nullptr, 0, fmt, ap);
  va_end(ap);
  std::string text;
  if (n > 0) {
    std::vector<char> buf(static_cast<std::size_t>(n) + 1);
    std::vsnprintf(buf.data(), buf.size(), fmt, ap2);
    text.assign(buf.data(), static_cast<std::size_t>(n));
  }
  va_end(ap2);

  std::lock_guard<std::mutex> lk(logMutex());
  if (logSink())
    logSink()(level, text.c_str());
  else
    writeToStderr(level, text.c_str());
}

}  // namespace umbreon
