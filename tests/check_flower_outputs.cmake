# tests/check_flower_outputs.cmake
#
# End-to-end check of the flower_scene executable: runs it and inspects the
# files it writes. Deliberately tiny (image size only -- the mesh resolution
# is fixed inside flower_main.cpp), since the brute-force intersector's cost
# depends on triangle count, not on how few pixels are sampled. Run with
#   cmake -DEXE=<path to flower_scene> -DWORK_DIR=<scratch dir> -P tests/check_flower_outputs.cmake
cmake_minimum_required(VERSION 3.20)

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")

function(run_ok)
    execute_process(COMMAND "${EXE}" ${ARGN}
                    WORKING_DIRECTORY "${WORK_DIR}"
                    RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT code EQUAL 0)
        message(FATAL_ERROR "flower_scene ${ARGN} exited with ${code}: ${err}")
    endif()
endfunction()

function(expect_exists path)
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "expected file was not written: ${path}")
    endif()
    file(SIZE "${path}" bytes)
    if(bytes EQUAL 0)
        message(FATAL_ERROR "file is empty: ${path}")
    endif()
endfunction()

# 1. No --output: both formats, under the "flower" default (not "gallery":
# this is the whole point of the default_output_stem parameter).
run_ok(--width 24 --height 18)
expect_exists("${WORK_DIR}/flower.ppm")
expect_exists("${WORK_DIR}/flower.png")
file(READ "${WORK_DIR}/flower.png" head HEX LIMIT 8)
if(NOT head STREQUAL "89504e470d0a1a0a")
    message(FATAL_ERROR "flower.png: not a PNG (signature ${head})")
endif()
file(STRINGS "${WORK_DIR}/flower.ppm" lines LIMIT_COUNT 2)
list(GET lines 1 size)
if(NOT size STREQUAL "24 18")
    message(FATAL_ERROR "flower.ppm: bad size header '${size}'")
endif()

# 2. --output still works normally, same parser as the main executable.
run_ok(--width 24 --height 18 --output "${WORK_DIR}/custom.png")
expect_exists("${WORK_DIR}/custom.png")

# 3. --help mentions the flower-specific program name and default.
execute_process(COMMAND "${EXE}" --help RESULT_VARIABLE code OUTPUT_VARIABLE text)
if(NOT code EQUAL 0 OR NOT text MATCHES "flower_scene" OR NOT text MATCHES "default flower")
    message(FATAL_ERROR "--help does not look flower-specific (${code}): ${text}")
endif()

message(STATUS "flower outputs: OK")
