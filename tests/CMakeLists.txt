# Unit / integration tests.
# Parser / IO / scene tests link bench_core (pure C++17, no rendering libs).
# The fog test links the umbreon library and is built only when umbreon is.
#
# Tests that read a Git-LFS-backed data file (data/**/*.inc) are labelled
# `needs_lfs`. CI checks out WITHOUT LFS (to avoid spending the repo's LFS
# bandwidth budget) and excludes them with `ctest -LE needs_lfs`; a local
# `task test` with the LFS files materialised runs the full set. The .pov
# scene files are plain git content (see .gitattributes: only *.inc/*.png/
# *.qsc are LFS), so pov_scene_reader and the pt1/render tests need no label.

add_executable(test_mesh2_reader test_mesh2_reader.cpp)
target_link_libraries(test_mesh2_reader PRIVATE bench_core)
add_test(
  NAME mesh2_reader
  COMMAND test_mesh2_reader ${CMAKE_SOURCE_DIR}/data/test1.inc)
# Reads data/test1.inc (LFS) directly.
set_tests_properties(mesh2_reader PROPERTIES LABELS needs_lfs)

add_executable(test_math test_math.cpp)
target_link_libraries(test_math PRIVATE bench_core)
add_test(NAME math COMMAND test_math)

add_executable(test_pov_scene_reader test_pov_scene_reader.cpp)
target_link_libraries(test_pov_scene_reader PRIVATE bench_core)
add_test(
  NAME pov_scene_reader
  COMMAND test_pov_scene_reader
          ${CMAKE_SOURCE_DIR}/data/test1.pov
          ${CMAKE_SOURCE_DIR}/data/1ab0_scene1.pov)

# Tests that need the rendering library (fog, image helpers, the render pipeline)
# are built only when the umbreon target exists (Embree + TBB available).
if(TARGET umbreon)
  # pt1 Phase 0 smoke test: raw Embree 4 triangle hit/miss (no umbreon code,
  # but requires the Embree dependency the umbreon target implies).
  add_executable(test_pt1_embree test_pt1_embree.cpp)
  target_link_libraries(test_pt1_embree PRIVATE umbreon)
  add_test(NAME pt1_embree COMMAND test_pt1_embree)

  # pt1 integration tests: direct-lighting parity between the cache and pt1
  # integrator branches (and, from Phase 3 on, the Cornell color-bleeding check).
  add_executable(test_pt1_render test_pt1_render.cpp)
  target_link_libraries(test_pt1_render PRIVATE umbreon)
  add_test(NAME pt1_render COMMAND test_pt1_render)

  # pt1 numeric gather tests: estimator vs analytic irradiance values.
  add_executable(test_pt1_gather test_pt1_gather.cpp)
  target_link_libraries(test_pt1_gather PRIVATE umbreon)
  add_test(NAME pt1_gather COMMAND test_pt1_gather)

  # pt1 joint bilateral upsample unit tests.
  add_executable(test_pt1_upsample test_pt1_upsample.cpp)
  target_link_libraries(test_pt1_upsample PRIVATE umbreon)
  add_test(NAME pt1_upsample COMMAND test_pt1_upsample)

  add_executable(test_fog test_fog.cpp)
  target_link_libraries(test_fog PRIVATE umbreon)
  add_test(NAME fog COMMAND test_fog)

  add_executable(test_image_ops test_image_ops.cpp)
  target_link_libraries(test_image_ops PRIVATE umbreon)
  add_test(NAME image_ops COMMAND test_image_ops)

  add_executable(test_render test_render.cpp)
  target_link_libraries(test_render PRIVATE umbreon)
  add_test(NAME render COMMAND test_render)

  add_executable(test_mesh_indexed test_mesh_indexed.cpp)
  target_link_libraries(test_mesh_indexed PRIVATE umbreon)
  add_test(NAME mesh_indexed COMMAND test_mesh_indexed)

  add_executable(test_object_space_edges test_object_space_edges.cpp)
  target_link_libraries(test_object_space_edges PRIVATE umbreon)
  add_test(NAME object_space_edges COMMAND test_object_space_edges)

  add_executable(test_screen_vector_edges test_screen_vector_edges.cpp)
  target_link_libraries(test_screen_vector_edges PRIVATE umbreon)
  add_test(NAME screen_vector_edges COMMAND test_screen_vector_edges)

  # Adaptive-AA tests: sample-multiplier / seed plumbing, refinement mask
  # predicates and adaptive-vs-grid render equivalences (synthetic scenes only,
  # no LFS data).
  add_executable(test_adaptive_aa test_adaptive_aa.cpp)
  target_link_libraries(test_adaptive_aa PRIVATE umbreon)
  add_test(NAME adaptive_aa COMMAND test_adaptive_aa)

  add_executable(test_screen_vector_integration
                 test_screen_vector_integration.cpp)
  target_link_libraries(test_screen_vector_integration PRIVATE umbreon
                                                               bench_core)
  add_test(NAME screen_vector_integration
           COMMAND test_screen_vector_integration
                   ${CMAKE_SOURCE_DIR}/data/edges/edge_tube1.pov)
  # Parses a plain .pov but reads the geometry it includes
  # (data/edges/edge_tube1.inc), which is LFS.
  set_tests_properties(screen_vector_integration PROPERTIES LABELS needs_lfs)

endif()
