# Unit / integration tests.
# Parser / IO / scene tests link bench_core (pure C++17, no rendering libs).
# The fog test links the umbreon library and is built only when umbreon is.

add_executable(test_mesh2_reader test_mesh2_reader.cpp)
target_link_libraries(test_mesh2_reader PRIVATE bench_core)
add_test(
  NAME mesh2_reader
  COMMAND test_mesh2_reader ${CMAKE_SOURCE_DIR}/data/test1.inc)

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

  add_executable(test_edges_default test_edges_default.cpp)
  target_link_libraries(test_edges_default PRIVATE umbreon)
  add_test(NAME edges_default COMMAND test_edges_default)

  add_executable(test_object_space_edges test_object_space_edges.cpp)
  target_link_libraries(test_object_space_edges PRIVATE umbreon)
  add_test(NAME object_space_edges COMMAND test_object_space_edges)

  add_executable(test_edge_visibility test_edge_visibility.cpp)
  target_link_libraries(test_edge_visibility PRIVATE umbreon)
  add_test(NAME edge_visibility COMMAND test_edge_visibility)

  add_executable(test_stroke_edges test_stroke_edges.cpp)
  target_link_libraries(test_stroke_edges PRIVATE umbreon)
  add_test(NAME stroke_edges COMMAND test_stroke_edges)

  add_executable(test_stroke_analytic test_stroke_analytic.cpp)
  target_link_libraries(test_stroke_analytic PRIVATE umbreon)
  add_test(NAME stroke_analytic COMMAND test_stroke_analytic)

  # Real-data edge-pass equivalence: indexed vs de-indexed mesh must render the
  # same with stroke edges on. Loads .pov scenes via bench_core, renders via
  # umbreon. Screen-space tube/ribbon edge-validation scenes + a ribbon scene.
  add_executable(test_edge_regression test_edge_regression.cpp)
  target_link_libraries(test_edge_regression PRIVATE umbreon bench_core)
  add_test(
    NAME edge_regression
    COMMAND test_edge_regression
            ${CMAKE_SOURCE_DIR}/data/edges/edge_tube1.pov
            ${CMAKE_SOURCE_DIR}/data/edges/edge_ribbon1.pov
            ${CMAKE_SOURCE_DIR}/data/edges/edge_ribbon2.pov
            ${CMAKE_SOURCE_DIR}/data/1ab0_scene1.pov)
endif()
