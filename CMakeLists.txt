cmake_minimum_required(VERSION 3.20)
project(umbreon VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

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

# Extra search hints for the fallbacks below (Homebrew on macOS, common
# system prefixes on Linux). find_library/find_path also honor CMAKE_PREFIX_PATH
# and the <pkg>_ROOT cache/environment variables.
set(_umbreon_dep_hints /opt/homebrew /usr/local /usr)

# Per-target warning flags. MSVC does not understand -Wall/-Wextra (it maps -Wall
# to /Wall, which floods warnings from system headers), so use a sane MSVC set.
if(MSVC)
  set(UMBREON_WARNINGS /W3)
else()
  set(UMBREON_WARNINGS -Wall -Wextra)
endif()

# --------------------------------------------------------------------------
# bench_core: pure C++17 .pov/.inc SDL parser, image IO and CLI option parsing
# for the benchmark harness (src/bench). No rendering-library dependency, so it
# builds and tests without Embree installed. It does consume libumbreon's public
# headers (scene.hpp, render_types.hpp), so src/umbreon is on its include path.
# --------------------------------------------------------------------------
add_library(bench_core STATIC
  src/bench/pov/pov_lexer.cpp
  src/bench/pov/pov_scene_reader.cpp
  src/bench/geom/mesh2_reader.cpp
  src/bench/geom/mesh2_texture_parse.cpp
  src/bench/geom/mesh2_block_parser.cpp
  src/bench/geom/mesh2_build.cpp
  src/bench/image/image_io.cpp
  src/bench/cli.cpp
)
target_include_directories(bench_core PUBLIC src/bench src/umbreon)
target_compile_options(bench_core PRIVATE ${UMBREON_WARNINGS})

# --------------------------------------------------------------------------
# Embree 4 + TBB: umbreon's only rendering dependencies.
# --------------------------------------------------------------------------
find_package(embree 4 CONFIG QUIET)
if(embree_FOUND)
  message(STATUS "Embree ${embree_VERSION} found via CONFIG")
  set(EMBREE_LINK embree)
else()
  # Fallback for installs without a CMake package config, e.g. a custom Embree
  # build in a non-standard prefix (the eventual shipping setup). Honors
  # -DEMBREE_ROOT=..., the EMBREE_ROOT environment variable, CMAKE_PREFIX_PATH
  # and the common hints; find_library picks the right suffix (.so/.dylib/.a)
  # for the platform.
  find_path(EMBREE_INCLUDE_DIR embree4/rtcore.h
    HINTS ${EMBREE_ROOT} ENV EMBREE_ROOT ${_umbreon_dep_hints}
    PATH_SUFFIXES include)
  find_library(EMBREE_LIBRARY NAMES embree4 embree
    HINTS ${EMBREE_ROOT} ENV EMBREE_ROOT ${_umbreon_dep_hints}
    PATH_SUFFIXES lib lib64)
  if(EMBREE_INCLUDE_DIR AND EMBREE_LIBRARY)
    message(STATUS "Embree found via fallback: ${EMBREE_LIBRARY}")
    add_library(embree_fallback INTERFACE)
    target_include_directories(embree_fallback INTERFACE ${EMBREE_INCLUDE_DIR})
    target_link_libraries(embree_fallback INTERFACE ${EMBREE_LIBRARY})
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
  # Same fallback strategy as Embree above (custom build / non-standard prefix).
  find_path(TBB_INCLUDE_DIR tbb/parallel_for.h
    HINTS ${TBB_ROOT} ENV TBB_ROOT ${_umbreon_dep_hints}
    PATH_SUFFIXES include)
  find_library(TBB_LIBRARY NAMES tbb
    HINTS ${TBB_ROOT} ENV TBB_ROOT ${_umbreon_dep_hints}
    PATH_SUFFIXES lib lib64)
  if(TBB_INCLUDE_DIR AND TBB_LIBRARY)
    message(STATUS "TBB found via fallback: ${TBB_LIBRARY}")
    add_library(tbb_fallback INTERFACE)
    target_include_directories(tbb_fallback INTERFACE ${TBB_INCLUDE_DIR})
    target_link_libraries(tbb_fallback INTERFACE ${TBB_LIBRARY})
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
    src/umbreon/render/embree_renderer.cpp
    src/umbreon/render/scene_build.cpp
    src/umbreon/render/curve_build.cpp
    src/umbreon/render/pipeline.cpp
    src/umbreon/experimental/irradiance_cache/denoise_atrous.cpp
    src/umbreon/experimental/pt1/pt1_denoise.cpp
    src/umbreon/postprocess/fog.cpp
    src/umbreon/postprocess/image_ops.cpp
    src/umbreon/edges/stroke_edges.cpp
    src/umbreon/edges/screen_edge_classify.cpp
    src/umbreon/edges/screen_edge_trace.cpp
    src/umbreon/edges/screen_edge_prune.cpp
    src/umbreon/edges/screen_edge_geometry.cpp
    src/umbreon/edges/screen_vector_edges.cpp
    src/umbreon/edges/mesh_feature_edges.cpp
    src/umbreon/edges/screen_projection.cpp
    src/umbreon/edges/object_space_edges.cpp
    src/umbreon/edges/edge_mesh_bvh.cpp
    src/umbreon/edges/analytic_silhouette.cpp
    src/umbreon/umbreon.cpp
  )
  # Namespaced alias so in-tree (add_subdirectory) consumers link the same
  # 'umbreon::umbreon' target name an installed find_package() would provide.
  add_library(umbreon::umbreon ALIAS umbreon)
  # Public headers live under src/umbreon in-tree and under <prefix>/include/umbreon
  # when installed; the public API headers #include each other with paths
  # relative to that root (e.g. "render/render_types.hpp"), so both layouts work.
  target_include_directories(umbreon PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/umbreon>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(umbreon PUBLIC ${EMBREE_LINK} ${TBB_LINK})
  target_compile_options(umbreon PRIVATE ${UMBREON_WARNINGS})
  set_target_properties(umbreon PROPERTIES VERSION ${PROJECT_VERSION})

  # Optional Intel Open Image Denoise backend (DenoiserBackend::OIDN). Provided by
  # the CueMol deplibs bundle (oidn-2.5.0, static). Default OFF so a bare configure
  # without the bundle still builds; the Taskfile's static path turns it ON and
  # adds the OIDN prefix to CMAKE_PREFIX_PATH. find_package is QUIET + guarded so an
  # ON build that cannot locate OIDN downgrades to a warning (the runtime then
  # falls back to the built-in a-trous denoiser) instead of failing.
  option(UMBREON_WITH_OIDN "Enable Intel OIDN denoiser backend" OFF)
  if(UMBREON_WITH_OIDN)
    find_package(OpenImageDenoise 2 CONFIG QUIET)
    if(OpenImageDenoise_FOUND)
      target_sources(umbreon PRIVATE
        src/umbreon/experimental/irradiance_cache/denoise_oidn.cpp)
      target_compile_definitions(umbreon PRIVATE UMBREON_HAVE_OIDN)
      target_link_libraries(umbreon PUBLIC OpenImageDenoise)
      message(STATUS
        "umbreon: OIDN backend enabled (OpenImageDenoise ${OpenImageDenoise_VERSION})")
    else()
      message(WARNING
        "umbreon: UMBREON_WITH_OIDN=ON but OpenImageDenoise not found; "
        "OIDN backend disabled (runtime falls back to a-trous)")
    endif()
  endif()
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
  add_executable(umbreon_cli src/bench/main.cpp)
  target_link_libraries(umbreon_cli PRIVATE bench_core umbreon::umbreon)
  target_compile_options(umbreon_cli PRIVATE ${UMBREON_WARNINGS})
else()
  message(WARNING
    "umbreon not built - 'umbreon_cli' not built. brew install embree tbb")
endif()

# --------------------------------------------------------------------------
# Install / export. Make libumbreon consumable by an external project (CueMol):
#   find_package(umbreon) -> umbreon::umbreon, or add_subdirectory(umbreon).
# Only the public API headers are installed; internal headers (the Embree
# renderer, fog, and the bench-only POV/geometry/image code) stay private.
#
# Public-surface SSOT: keep the install(FILES) list below in sync with the
# table in docs/api/libumbreon.md and the per-header visibility markers
# ("libumbreon PUBLIC API header" vs "... INTERNAL header"). All three must
# list the same 7 public headers (umbreon.hpp, scene.hpp,
# render/render_types.hpp and its three sub-headers edge_types.hpp /
# render_options.hpp / frame_result.hpp, postprocess/image_ops.hpp).
# --------------------------------------------------------------------------
if(TARGET umbreon)
  install(TARGETS umbreon EXPORT umbreonTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  # Public headers, preserving the layout umbreon.hpp #includes (scene.hpp and
  # render/render_types.hpp) so an installed <umbreon/umbreon.hpp> resolves them.
  install(FILES src/umbreon/umbreon.hpp src/umbreon/scene.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/umbreon)
  # render_types.hpp is an umbrella over the three sub-headers; all four ship
  # so both the historical single include and per-concern includes resolve.
  install(FILES src/umbreon/render/render_types.hpp
    src/umbreon/render/edge_types.hpp
    src/umbreon/render/render_options.hpp
    src/umbreon/render/frame_result.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/umbreon/render)
  # NPR edges are public only through their OPTION types (StrokeEdgeOptions /
  # ObjectSpaceEdgeOptions / EdgeStyle in render/render_types.hpp) driven via
  # RenderOptions + render(). The edges/*.hpp extraction/visibility/rasterizer
  # headers are INTERNAL (not installed) -- see their INTERNAL markers.
  install(FILES src/umbreon/postprocess/image_ops.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/umbreon/postprocess)

  # The exported target records its PUBLIC link to Embree/TBB. That only round-
  # trips when they were found as CONFIG packages (imported targets the generated
  # umbreonConfig can re-find); the fallback INTERFACE targets are not
  # exportable. add_subdirectory consumers work regardless via the alias.
  if(embree_FOUND AND TBB_FOUND)
    install(EXPORT umbreonTargets
      NAMESPACE umbreon::
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/umbreon
      FILE umbreonTargets.cmake)
    configure_package_config_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/umbreonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/umbreonConfig.cmake
      INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/umbreon)
    write_basic_package_version_file(
      ${CMAKE_CURRENT_BINARY_DIR}/umbreonConfigVersion.cmake
      VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
    install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}/umbreonConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/umbreonConfigVersion.cmake
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/umbreon)
    message(STATUS "umbreon: install exports find_package(umbreon)")
  else()
    message(STATUS
      "umbreon: Embree/TBB found via fallback - installing lib + headers but "
      "not the find_package export (consume via add_subdirectory instead)")
  endif()
endif()
