# Standalone build of the libumbreon minimal example. This mirrors what a
# CueMol-side build does: find the installed umbreon package and link the
# umbreon::umbreon target. It is intentionally NOT part of the main umbreon
# build -- it is a self-contained template for an external consumer.
#
#   # 1. install umbreon somewhere
#   cmake -S . -B build && cmake --build build
#   cmake --install build --prefix /path/to/umbreon-prefix
#
#   # 2. build this example against that install
#   cmake -S examples -B build-ex \
#         -DCMAKE_PREFIX_PATH="/path/to/umbreon-prefix;/opt/homebrew"
#   cmake --build build-ex
#   ./build-ex/minimal_render
#
# CMAKE_PREFIX_PATH must also point at the Embree 4 / oneTBB installs (Homebrew
# on macOS), because umbreonConfig re-finds them via find_dependency().
cmake_minimum_required(VERSION 3.20)
project(umbreon_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(umbreon REQUIRED)

add_executable(minimal_render minimal_render.cpp)
target_link_libraries(minimal_render PRIVATE umbreon::umbreon)
