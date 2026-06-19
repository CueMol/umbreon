cmake_minimum_required(VERSION 3.20)
project(umbreon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Let find_package see Homebrew packages on Apple Silicon.
if(APPLE AND EXISTS /opt/homebrew)
  list(APPEND CMAKE_PREFIX_PATH /opt/homebrew)
endif()

# --------------------------------------------------------------------------
# bench_core: pure C++17 .pov/.inc SDL parser, image IO and CLI option parsing
# for the benchmark harness. No rendering-library dependency, so it builds and
# tests without Embree installed.
# --------------------------------------------------------------------------
add_library(bench_core STATIC
  src/pov/pov_lexer.cpp
  src/pov/pov_scene_reader.cpp
  src/geom/mesh2_reader.cpp
  src/geom/scene_builder.cpp
  src/image/image_io.cpp
  src/povexport/pov_writer.cpp
  src/cli.cpp
)
target_include_directories(bench_core PUBLIC src)
target_compile_options(bench_core PRIVATE -Wall -Wextra)

# --------------------------------------------------------------------------
# Embree 4 + TBB: umbreon's only rendering dependencies.
# --------------------------------------------------------------------------
find_package(embree 4 CONFIG QUIET)
if(embree_FOUND)
  message(STATUS "Embree ${embree_VERSION} found via CONFIG")
  set(EMBREE_LINK embree)
else()
  set(EMBREE_ROOT /opt/homebrew/opt/embree)
  if(EXISTS ${EMBREE_ROOT}/include/embree4/rtcore.h)
    message(STATUS "Embree found via Homebrew fallback at ${EMBREE_ROOT}")
    add_library(embree_fallback INTERFACE)
    target_include_directories(embree_fallback INTERFACE ${EMBREE_ROOT}/include)
    target_link_libraries(embree_fallback INTERFACE ${EMBREE_ROOT}/lib/libembree4.dylib)
    set(EMBREE_LINK embree_fallback)
  else()
    message(WARNING "Embree 4 not found")
    set(EMBREE_LINK "")
  endif()
endif()

find_package(TBB CONFIG QUIET)
if(TBB_FOUND)
  message(STATUS "TBB ${TBB_VERSION} found via CONFIG")
  set(TBB_LINK TBB::tbb)
else()
  set(TBB_ROOT /opt/homebrew/opt/tbb)
  if(EXISTS ${TBB_ROOT}/include/tbb/parallel_for.h)
    message(STATUS "TBB found via Homebrew fallback at ${TBB_ROOT}")
    add_library(tbb_fallback INTERFACE)
    target_include_directories(tbb_fallback INTERFACE ${TBB_ROOT}/include)
    target_link_libraries(tbb_fallback INTERFACE ${TBB_ROOT}/lib/libtbb.dylib)
    set(TBB_LINK tbb_fallback)
  else()
    message(WARNING "TBB not found")
    set(TBB_LINK "")
  endif()
endif()

# --------------------------------------------------------------------------
# umbreon: the Embree-direct rendering backend, intended for static linking
# into libcuemol2. No POV-Ray SDL and no OSPRay; depends only on Embree 4 + TBB.
# CueMol builds an umbreon::Scene and calls umbreon::render().
# --------------------------------------------------------------------------
if(EMBREE_LINK AND TBB_LINK)
  message(STATUS "Building 'umbreon' (Embree backend)")
  add_library(umbreon STATIC
    src/render/embree_renderer.cpp
    src/image/fog.cpp
    src/umbreon.cpp
  )
  target_include_directories(umbreon PUBLIC src)
  target_link_libraries(umbreon PUBLIC ${EMBREE_LINK} ${TBB_LINK})
  target_compile_options(umbreon PRIVATE -Wall -Wextra)
else()
  message(WARNING "Embree/TBB not found - 'umbreon' library not built")
endif()

enable_testing()
add_subdirectory(tests)

# --------------------------------------------------------------------------
# umbreon_cli: the benchmark / demo CLI. Parses .pov/.inc via bench_core and
# renders through umbreon. Built only when umbreon is available.
# --------------------------------------------------------------------------
if(TARGET umbreon)
  message(STATUS "Building 'umbreon_cli'")
  add_executable(umbreon_cli src/main.cpp)
  target_link_libraries(umbreon_cli PRIVATE bench_core umbreon)
  target_compile_options(umbreon_cli PRIVATE -Wall -Wextra)
else()
  message(WARNING
    "umbreon not built - 'umbreon_cli' not built. brew install embree tbb")
endif()
