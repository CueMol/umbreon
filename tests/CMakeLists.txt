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

add_executable(test_pov_scene_reader test_pov_scene_reader.cpp)
target_link_libraries(test_pov_scene_reader PRIVATE bench_core)
add_test(
  NAME pov_scene_reader
  COMMAND test_pov_scene_reader
          ${CMAKE_SOURCE_DIR}/data/test1.pov
          ${CMAKE_SOURCE_DIR}/data/1ab0_scene1.pov)

# Fog moved into the umbreon library; build its test only when umbreon exists.
if(TARGET umbreon)
  add_executable(test_fog test_fog.cpp)
  target_link_libraries(test_fog PRIVATE umbreon)
  add_test(NAME fog COMMAND test_fog)
endif()
