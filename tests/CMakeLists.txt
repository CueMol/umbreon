# Unit / integration tests.
# Parser / IO / scene tests link bench_core (pure C++17, no rendering libs).
# The fog test links the umbreon library and is built only when umbreon is.

add_executable(test_mesh2_reader test_mesh2_reader.cpp)
target_link_libraries(test_mesh2_reader PRIVATE bench_core)
add_test(
  NAME mesh2_reader
  COMMAND test_mesh2_reader ${CMAKE_SOURCE_DIR}/data/test1.inc)

add_executable(test_scene_builder test_scene_builder.cpp)
target_link_libraries(test_scene_builder PRIVATE bench_core)
add_test(NAME scene_builder COMMAND test_scene_builder)

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

  add_executable(test_edge_detect test_edge_detect.cpp)
  target_link_libraries(test_edge_detect PRIVATE umbreon)
  add_test(NAME edge_detect COMMAND test_edge_detect)

  add_executable(test_edges_default test_edges_default.cpp)
  target_link_libraries(test_edges_default PRIVATE umbreon)
  add_test(NAME edges_default COMMAND test_edges_default)
endif()
