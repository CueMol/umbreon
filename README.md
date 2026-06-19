# umbreon

An offline molecular renderer backend built directly on Intel Embree 4 + TBB,
intended for static linking into CueMol (libcuemol2). It reproduces CueMol's
POV-Ray look (without radiosity) using primary rays plus direct local shading.
The renderer (the `umbreon` library) is fully decoupled from the `.pov`/`.inc`
SDL parser, which is used only by the benchmark harness.

## Layout

- **`umbreon`** (static library) — the rendering backend. Depends only on
  Embree 4 and TBB. Build an `umbreon::Scene` (geometry, camera, lights,
  material, fog), call `umbreon::render()`, and get a linear HDR framebuffer.
  This is what CueMol links and calls; no POV-Ray SDL involved.
  - `src/scene.hpp` — public scene / geometry / material API types.
  - `src/render/{render_types.hpp, embree_renderer.*}` — the Embree renderer.
  - `src/image/fog.*` — POV ground-fog depth post-process.
  - `src/umbreon.{hpp,cpp}` — the `render()` facade and helpers.
- **`bench_core`** (static library, pure C++17) — the `.pov`/`.inc` SDL parser,
  image IO (PNG/PPM + PSNR/SSIM) and CLI option parsing.
- **`umbreon_cli`** (executable) — parses a `.pov`/`.inc` scene, renders it
  through umbreon and writes the image; also offers `--compare` / `--convert`.

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

## Usage

```sh
./build/umbreon_cli data/test1.pov -o out.png
```

## Public API (for CueMol)

```cpp
#include "umbreon.hpp"

umbreon::Scene scene;        // fill geometry / camera / lights / material / fog
umbreon::RenderOptions opt;  // width, height, supersample, assumedGamma
umbreon::FrameResult f = umbreon::render(scene, opt);  // linear HDR framebuffer
auto rgba8 = umbreon::srgbEncode8(f, 4);               // 8-bit sRGB for display
```

## Name

A renderer's core job is shading — deciding how much shade falls on each surface.
*umbreon* is coined from **umbra**, Latin for "shade / shadow" (also the term for
the fully-shadowed core of a shadow), with the **-on** suffix that makes it read
like a material or particle name (neon, argon, electron).
