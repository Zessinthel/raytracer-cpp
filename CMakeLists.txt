cmake_minimum_required(VERSION 3.20)
project(raytracer_v0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(engine INTERFACE)
target_include_directories(engine INTERFACE include)

add_library(physics INTERFACE)
target_link_libraries(physics INTERFACE engine)
target_include_directories(physics INTERFACE include)

add_library(scene INTERFACE)
target_link_libraries(scene INTERFACE engine)
target_include_directories(scene INTERFACE include)

add_library(shading INTERFACE)
target_link_libraries(shading INTERFACE engine scene)
target_include_directories(shading INTERFACE include)

add_library(io INTERFACE)
target_link_libraries(io INTERFACE engine)
target_include_directories(io INTERFACE include)

add_executable(raytracer src/main.cpp)
target_link_libraries(raytracer PRIVATE io shading scene physics engine)
target_include_directories(raytracer PRIVATE src)

# ---- Tests (run with: ctest --test-dir build --output-on-failure) ----------
# Each suite links only against the layers it is allowed to use, so a test of
# a lower layer cannot silently depend on a higher one.
enable_testing()

add_executable(test_engine tests/test_engine.cpp)
target_link_libraries(test_engine PRIVATE engine)
add_test(NAME engine COMMAND test_engine)

add_executable(test_geometry tests/test_geometry.cpp)
target_link_libraries(test_geometry PRIVATE engine)
add_test(NAME geometry COMMAND test_geometry)

add_executable(test_physics tests/test_physics.cpp)
target_link_libraries(test_physics PRIVATE physics engine)
add_test(NAME physics COMMAND test_physics)

add_executable(test_watertightness tests/test_watertightness.cpp)
target_link_libraries(test_watertightness PRIVATE physics engine)
add_test(NAME watertightness COMMAND test_watertightness)

add_executable(test_scene_shading tests/test_scene_shading.cpp)
target_link_libraries(test_scene_shading PRIVATE shading scene physics engine)
add_test(NAME scene_shading COMMAND test_scene_shading)

add_executable(test_scene_data tests/test_scene_data.cpp)
target_link_libraries(test_scene_data PRIVATE scene physics engine)
add_test(NAME scene_data COMMAND test_scene_data)

add_executable(test_lighting tests/test_lighting.cpp)
target_link_libraries(test_lighting PRIVATE shading scene physics engine)
add_test(NAME lighting COMMAND test_lighting)

add_executable(test_io tests/test_io.cpp)
target_link_libraries(test_io PRIVATE io engine)
add_test(NAME io COMMAND test_io)

add_executable(test_cli tests/test_cli.cpp)
target_link_libraries(test_cli PRIVATE shading scene physics engine)
target_include_directories(test_cli PRIVATE src)
add_test(NAME cli COMMAND test_cli)

add_test(NAME layering
         COMMAND ${CMAKE_COMMAND} -DROOT=${CMAKE_SOURCE_DIR}
                 -P ${CMAKE_SOURCE_DIR}/tests/check_layering.cmake)

# End to end: runs the real executable and inspects the files it writes.
add_test(NAME outputs
         COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:raytracer>
                 -DWORK_DIR=${CMAKE_BINARY_DIR}/outputs_test
                 -P ${CMAKE_SOURCE_DIR}/tests/check_outputs.cmake)
