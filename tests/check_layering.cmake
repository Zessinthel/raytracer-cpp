# tests/check_layering.cmake
#
# Verifies the one-way dependency between layers: a header may include headers
# of its own layer or of a lower one, never of a higher one. Every layer
# shares the same include/ directory, so the compiler alone cannot enforce
# this; this script does. Run with
#   cmake -DROOT=<project root> -P tests/check_layering.cmake
cmake_minimum_required(VERSION 3.20)

set(ALLOWED_engine  engine)
set(ALLOWED_physics engine physics)
set(ALLOWED_scene   engine physics scene)
set(ALLOWED_shading engine physics scene shading)
set(ALLOWED_io      engine io)

set(violations 0)
foreach(layer IN ITEMS engine physics scene shading io)
    file(GLOB_RECURSE headers "${ROOT}/include/raytracer/${layer}/*.hpp")
    foreach(header IN LISTS headers)
        file(STRINGS "${header}" include_lines REGEX "^[ \t]*#include[ \t]+[<\"]raytracer/")
        foreach(line IN LISTS include_lines)
            string(REGEX REPLACE ".*raytracer/([A-Za-z_]+)/.*" "\\1" included_layer "${line}")
            if(NOT included_layer IN_LIST ALLOWED_${layer})
                message(SEND_ERROR "layering: ${header} (layer ${layer}) includes the ${included_layer} layer: ${line}")
                math(EXPR violations "${violations} + 1")
            endif()
        endforeach()
    endforeach()
endforeach()

if(violations GREATER 0)
    message(FATAL_ERROR "${violations} layering violation(s)")
endif()
message(STATUS "layering: OK")
