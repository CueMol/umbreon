// libumbreon PUBLIC API header.
//
// Diagnostic output routing. The library used to write every warning straight
// to stderr, which is fine for the bench CLI and useless for an embedding
// application: a GUI host has no terminal, so warnings that explain a fallback
// ("--ao-res out is not supported with --gi yet", "OIDN device init failed",
// ...) were simply lost.
//
// A host installs a sink and receives the same messages as strings. The sink is
// process-wide because the messages originate deep in the render (Embree's
// error callback, the denoiser, the pipeline's option normalisation) where no
// per-render context is threaded through; a host that renders on a background
// thread should install it once at startup, not per render.
//
// Default behaviour is unchanged: with no sink installed every message goes to
// stderr exactly as before.
#pragma once

#include <functional>

namespace umbreon {

/// Severity of a diagnostic message.
enum class LogLevel {
  /// A fallback was taken or a request could not be honoured. The render still
  /// produces an image, but not the one that was literally asked for.
  Warning,
  /// Progress / cost information a host may want to surface: the resolved
  /// render configuration, group-alpha pass boundaries, per-stage timing and
  /// ray counts of a finished GI render.
  Info,
};

/// Receives one already-formatted message. `text` is NOT newline-terminated and
/// is only valid for the duration of the call.
using LogSink = std::function<void(LogLevel level, const char* text)>;

/// Route diagnostics to `sink`. Passing nullptr restores the stderr default.
///
/// Thread-safe: may be called while a render is in flight, and the sink itself
/// may be invoked from a render worker thread, so a sink that touches host
/// state must do its own locking.
void setLogSink(LogSink sink);

/// Emit one message. Formats with printf semantics.
void logMessage(LogLevel level, const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

}  // namespace umbreon
