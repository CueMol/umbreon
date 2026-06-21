# Umbreon

An offline molecular renderer backend built directly on Intel Embree 4 + TBB,
intended for static linking into CueMol (libcuemol2). It reproduces CueMol's
POV-Ray look (without radiosity) using primary rays plus direct local shading.
The renderer (the `umbreon` library) is fully decoupled from the `.pov`/`.inc`
SDL parser, which is used only by the benchmark harness.

## Layout

- **`umbreon`** (static library) — the rendering backend. Depends only on
  Embree 4 and TBB. Build an `umbreon::Scene` (geometry, camera, lights,
  material, fog), call `umbreon::render()`, and get a linear HDR framebuffer.
  This is what CueMol links and calls; no POV-Ray SDL involved. It installs as a
  CMake package (`find_package(umbreon)` -> `umbreon::umbreon`); the public
  headers install under `<umbreon/...>`. Full integration guide:
  [docs/api/libumbreon.md](docs/api/libumbreon.md).
  - `src/scene.hpp` — public scene / geometry / material API types.
  - `src/render/{render_types.hpp, embree_renderer.*}` — the Embree renderer.
  - `src/image/fog.*` — POV ground-fog depth post-process.
  - `src/umbreon.{hpp,cpp}` — the `render()` facade and helpers.
- **`bench_core`** (static library, pure C++17) — the `.pov`/`.inc` SDL parser,
  image IO (PNG/PPM + PSNR/SSIM) and CLI option parsing.
- **`umbreon_cli`** (executable) — parses a `.pov`/`.inc` scene, renders it
  through Umbreon and writes the image; also offers `--compare` / `--convert`.
  Quality-tuning guide: [docs/umbreon_cli.md](docs/umbreon_cli.md).
- **`examples/`** — a standalone `find_package(umbreon)` consumer
  (`minimal_render.cpp`) demonstrating the library API end to end.

## Build

Install the dependencies — Embree 4 and TBB (POV-Ray only to regenerate POV
references), or just run `task deps`:

```sh
# macOS
brew install embree tbb povray
# Ubuntu / Debian
sudo apt install libembree-dev libtbb-dev povray
```

Then configure and build:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

CMake finds Embree/TBB via their CMake package config. To link against a custom
build in a non-standard prefix (e.g. the eventual shipping setup), point CMake at
it with `-DCMAKE_PREFIX_PATH=/path/to/prefix` (or `-DEMBREE_ROOT=...`
`-DTBB_ROOT=...` when no package config is present).

### Static build (for CueMol)

For static linking into CueMol, build against prebuilt Embree/TBB instead of
system packages:

```sh
task build:static      # or: task test:static
```

On Linux x64 this downloads the CueMol2 **deplibs** bundle — prebuilt static
Embree 4 + TBB matching CueMol's own build — into `deps/deplibs` and points CMake
at it. On other platforms it falls back to building Embree/TBB from source into
`deps/prefix` (`task deps:build`).

## Usage

```sh
./build/umbreon_cli data/test1.pov -o out.png
```

## Use as a library (for CueMol)

libumbreon installs as a CMake package; link the `umbreon::umbreon` target. See
**[docs/api/libumbreon.md](docs/api/libumbreon.md)** for the full integration
guide (the CueMol2/3 starting point) and `examples/minimal_render.cpp` for a
complete, buildable consumer.

```sh
cmake -S . -B build && cmake --build build
cmake --install build --prefix /opt/umbreon
```

```cmake
# CueMol-side CMake (CMAKE_PREFIX_PATH must also reach Embree 4 / TBB)
find_package(umbreon REQUIRED)          # or: add_subdirectory(umbreon)
target_link_libraries(cuemol PRIVATE umbreon::umbreon)
```

```cpp
#include <umbreon/umbreon.hpp>

umbreon::Scene scene;        // geometry / camera / lights / material / fog
umbreon::RenderOptions opt;  // size, supersample, AO, shadows, transparency
umbreon::FrameResult f = umbreon::render(scene, opt);  // linear HDR framebuffer
auto rgba8 = umbreon::srgbEncode8(f, 4);               // 8-bit sRGB for display
```

## Name

A renderer's core job is shading — deciding how much shade falls on each surface.
*Umbreon* is coined from **umbra**, Latin for "shade / shadow" (also the term for
the fully-shadowed core of a shadow), with the **-on** suffix that makes it read
like a material or particle name (neon, argon, electron).
