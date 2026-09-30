cmake_minimum_required(VERSION 3.20)
project(raytracer_v0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The raytracer is brute-force and leans heavily on small inline functions
# (Vec3 arithmetic, dot/cross, reflect, intersect_triangle) in hot loops that
# run billions of times per render; without optimization those never get
# inlined, and a plain `cmake -S . -B build` with no build type otherwise
# compiles at -O0 on a Makefiles generator. Defaulting to Release here (only
# when the user hasn't asked for something else, e.g. -DCMAKE_BUILD_TYPE=Debug
# for the sanitizer builds) is the single highest-impact performance fix in
# this project, ahead of any algorithmic change.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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

# OpenMP parallelizes the per-pixel loop in shading::render (Step 1 of the
# multicore plan): each pixel only reads the (const) Scene and writes to its
# own Image cell, so there is no shared mutable state to protect and the
# result is bit-for-bit identical to the serial render, just faster. Linked
# on the shading target itself (an INTERFACE library) so every consumer --
# raytracer, flower_scene, and any test that calls render() -- picks up the
# -fopenmp compile flag and the OpenMP runtime automatically, with nothing to
# change at the call sites. Optional: without OpenMP on the system, the
# project still configures and builds, just as a serial renderer.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(shading INTERFACE OpenMP::OpenMP_CXX)
else()
    message(WARNING "OpenMP not found: rendering will run single-threaded.")
endif()

add_library(io INTERFACE)
target_link_libraries(io INTERFACE engine)
target_include_directories(io INTERFACE include)

add_executable(raytracer src/main.cpp)
target_link_libraries(raytracer PRIVATE io shading scene physics engine)
target_include_directories(raytracer PRIVATE src)

add_executable(flower_scene src/flower_main.cpp)
target_link_libraries(flower_scene PRIVATE io shading scene physics engine)
target_include_directories(flower_scene PRIVATE src)

# ---- Tests (run with: ctest --test-dir build --output-on-failure) ----------
# Each suite links only against the layers it is allowed to use, so a test of
# a lower layer cannot silently depend on a higher one.
enable_testing()

add_executable(test_engine tests/test_engine.cpp)
target_link_libraries(test_engine PRIVATE engine)
add_test(NAME engine COMMAND test_engine)

add_executable(test_geometry tests/test_geometry.cpp)
target_link_libraries(test_geometry PRIVATE physics engine)
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

add_test(NAME flower_outputs
         COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:flower_scene>
                 -DWORK_DIR=${CMAKE_BINARY_DIR}/flower_outputs_test
                 -P ${CMAKE_SOURCE_DIR}/tests/check_flower_outputs.cmake)
