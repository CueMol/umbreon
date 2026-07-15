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
  # renderAsync()/RenderTask own a std::thread; pull in the platform thread lib.
  find_package(Threads REQUIRED)
  add_library(umbreon STATIC
    src/umbreon/render/embree_renderer.cpp
    src/umbreon/render/scene_build.cpp
    src/umbreon/render/curve_build.cpp
    src/umbreon/render/pipeline.cpp
    src/umbreon/render/render_progress.cpp
    src/umbreon/experimental/irradiance_cache/denoise_atrous.cpp
    src/umbreon/experimental/pt1/pt1_denoise.cpp
    src/umbreon/postprocess/fog.cpp
    src/umbreon/postprocess/image_ops.cpp
    src/umbreon/edges/stroke_edges.cpp
    src/umbreon/edges/stroke_render.cpp
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
  # when installed; the public API headers #include each other with paths relative
  # to that root (e.g. render_types.hpp does #include "render/edge_types.hpp"). The
  # in-tree root is src/umbreon (BUILD_INTERFACE) and the installed root is
  # <prefix>/include/umbreon, so that directory must be on the consumer include
  # path for those root-relative includes to resolve. Also keep <prefix>/include so
  # the umbreon-prefixed form (<umbreon/umbreon.hpp>) still works for consumers.
  target_include_directories(umbreon PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/umbreon>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/umbreon>)
  target_link_libraries(umbreon PUBLIC ${EMBREE_LINK} ${TBB_LINK} Threads::Threads)
  target_compile_options(umbreon PRIVATE ${UMBREON_WARNINGS})
  set_target_properties(umbreon PROPERTIES VERSION ${PROJECT_VERSION})

  # Out-of-process Intel Open Image Denoise backend (DenoiserBackend::OIDN).
  # libumbreon does NOT link OIDN: it embeds an IPC client (Boost.Process v2 +
  # Boost.Interprocess, header-only usage) that spawns the separate
  # umbreon_oidn_worker executable (the only binary linking OIDN, see below)
  # and exchanges image buffers through shared memory. Default OFF so a bare
  # configure without the CueMol deplibs bundle stays Boost-free; the
  # Taskfile's static path turns it ON and adds the Boost/OIDN prefixes to
  # CMAKE_PREFIX_PATH. ON but Boost missing: the client is not compiled and
  # the runtime falls back to the built-in a-trous denoiser. Boost present but
  # OIDN missing: the client IS compiled and only the worker build is skipped
  # (a worker from another build can still be supplied at runtime via
  # RenderOptions::oidnWorkerPath or UMBREON_OIDN_WORKER).
  option(UMBREON_WITH_OIDN
    "Enable the out-of-process Intel OIDN denoiser (IPC client + worker)" OFF)
  if(UMBREON_WITH_OIDN)
    find_package(Boost 1.84 CONFIG QUIET)
    find_package(OpenImageDenoise 2 CONFIG QUIET)
    if(Boost_FOUND)
      target_sources(umbreon PRIVATE
        src/umbreon/experimental/irradiance_cache/denoise_oidn.cpp
        src/umbreon/ipc/oidn_client.cpp)
      # BOOST_PROCESS_USE_STD_FS switches process v2 to std::filesystem so the
      # Boost usage stays header-only (no compiled Boost library to link).
      target_compile_definitions(umbreon PRIVATE
        UMBREON_HAVE_OIDN BOOST_PROCESS_USE_STD_FS)
      # Include dirs only -- deliberately NOT target_link_libraries(Boost::headers):
      # even a PRIVATE link on a STATIC library is recorded in the exported link
      # interface ($<LINK_ONLY:...>) and would force find_dependency(Boost) on
      # every find_package(umbreon) consumer for a build-time-only dependency.
      get_target_property(_umbreon_boost_inc
        Boost::headers INTERFACE_INCLUDE_DIRECTORIES)
      target_include_directories(umbreon SYSTEM PRIVATE ${_umbreon_boost_inc})
      if(WIN32)
        # Boost.Asio (under process v2) needs winsock; plain system libraries,
        # safe to appear in the export as $<LINK_ONLY:...>.
        target_link_libraries(umbreon PRIVATE ws2_32 mswsock)
      elseif(UNIX AND NOT APPLE)
        target_link_libraries(umbreon PRIVATE rt)  # shm_open/shm_unlink
      endif()
      set(UMBREON_OIDN_CLIENT ON)  # read by tests/CMakeLists.txt
      message(STATUS
        "umbreon: OIDN IPC client enabled (Boost ${Boost_VERSION})")
    else()
      message(WARNING
        "umbreon: UMBREON_WITH_OIDN=ON but Boost not found; "
        "OIDN IPC client disabled (runtime falls back to a-trous)")
    endif()
  endif()
else()
  message(WARNING "Embree/TBB not found - 'umbreon' library not built")
endif()

# --------------------------------------------------------------------------
# umbreon_oidn_worker: the out-of-process OIDN denoise service and the ONLY
# binary that links OpenImageDenoise (deplibs oidn-2.5.0, static). A
# persistent child process driven over stdin/stdout by libumbreon's IPC
# client. Installed (unlike umbreon_cli, which is a test harness): the worker
# ships next to the host application, which is also where the client's
# default discovery looks for it.
# --------------------------------------------------------------------------
if(UMBREON_WITH_OIDN AND Boost_FOUND AND OpenImageDenoise_FOUND)
  add_executable(umbreon_oidn_worker src/oidn_worker/main.cpp)
  # For ipc/oidn_protocol.hpp + ipc/shm_region.hpp only; does not link umbreon.
  target_include_directories(umbreon_oidn_worker PRIVATE src/umbreon)
  target_link_libraries(umbreon_oidn_worker PRIVATE
    OpenImageDenoise Boost::headers)
  target_compile_options(umbreon_oidn_worker PRIVATE ${UMBREON_WARNINGS})
  if(UNIX AND NOT APPLE)
    target_link_libraries(umbreon_oidn_worker PRIVATE rt)
  endif()
  # Part of the exported target set: find_package(umbreon) consumers see it as
  # the IMPORTED executable umbreon::umbreon_oidn_worker so they can stage it
  # into their own app bundle (e.g. install(PROGRAMS $<TARGET_FILE:...>)). It is
  # NOT a package component (present only in UMBREON_WITH_OIDN builds); consumers
  # gate on if(TARGET umbreon::umbreon_oidn_worker). Executables export no link
  # interface, so this adds no consumer dependency. (When embree/TBB were found
  # via fallback the install(EXPORT) below is skipped and this registration is
  # simply inert; the worker still installs to bin/.)
  install(TARGETS umbreon_oidn_worker EXPORT umbreonTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  message(STATUS
    "Building 'umbreon_oidn_worker' (OpenImageDenoise ${OpenImageDenoise_VERSION})")
elseif(UMBREON_WITH_OIDN AND Boost_FOUND)
  message(WARNING
    "umbreon_oidn_worker: OpenImageDenoise not found; worker not built "
    "(the IPC client will fall back to a-trous unless a worker binary is "
    "supplied at runtime)")
endif()

enable_testing()
add_subdirectory(tests)

# --------------------------------------------------------------------------
# umbreon_cli: the benchmark / demo CLI. Parses .pov/.inc via bench_core and
# renders through umbreon. Built only when umbreon is available.
# --------------------------------------------------------------------------
if(TARGET umbreon)
  message(STATUS "Building 'umbreon_cli'")
  add_executable(umbreon_cli
    src/bench/main.cpp
    src/bench/scene_setup.cpp
    src/bench/modes.cpp
    src/bench/timing_report.cpp
    src/bench/aov_dump.cpp
  )
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
    src/umbreon/render/render_progress.hpp
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
