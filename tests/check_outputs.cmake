# tests/check_outputs.cmake
#
# End-to-end check of the executable: run it as a user would and look at the
# files it leaves on disk. It covers what no unit test reaches, main() itself,
# option parsing feeding the renderer and the renderer feeding both writers.
# Run with
#   cmake -DEXE=<path to raytracer> -DWORK_DIR=<scratch dir> -P tests/check_outputs.cmake
cmake_minimum_required(VERSION 3.20)

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")

# A small image keeps the test fast; the picture itself is checked elsewhere.
set(SMALL --width 48 --height 20)

function(run_ok)
    execute_process(COMMAND "${EXE}" ${ARGN}
                    WORKING_DIRECTORY "${WORK_DIR}"
                    RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT code EQUAL 0)
        message(FATAL_ERROR "raytracer ${ARGN} exited with ${code}: ${err}")
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

function(expect_absent path)
    if(EXISTS "${path}")
        message(FATAL_ERROR "file should not have been written: ${path}")
    endif()
endfunction()

function(expect_ppm path width height)
    file(STRINGS "${path}" lines LIMIT_COUNT 2)
    list(GET lines 0 magic)
    list(GET lines 1 size)
    if(NOT magic STREQUAL "P3" OR NOT size STREQUAL "${width} ${height}")
        message(FATAL_ERROR "${path}: bad PPM header '${magic}' / '${size}'")
    endif()
endfunction()

function(expect_png path)
    file(READ "${path}" head HEX LIMIT 8)
    if(NOT head STREQUAL "89504e470d0a1a0a")
        message(FATAL_ERROR "${path}: not a PNG (signature ${head})")
    endif()
endfunction()

# 1. No --output: both formats, under the default name, in the working directory.
run_ok(${SMALL})
expect_exists("${WORK_DIR}/gallery.ppm")
expect_exists("${WORK_DIR}/gallery.png")
expect_ppm("${WORK_DIR}/gallery.ppm" 48 20)
expect_png("${WORK_DIR}/gallery.png")

# 2. A path with no extension: both formats, with that name.
run_ok(${SMALL} --mode object-id --output "${WORK_DIR}/ids")
expect_exists("${WORK_DIR}/ids.ppm")
expect_exists("${WORK_DIR}/ids.png")
expect_ppm("${WORK_DIR}/ids.ppm" 48 20)
expect_png("${WORK_DIR}/ids.png")

# 3. An extension: only that format.
run_ok(${SMALL} --output "${WORK_DIR}/only.png")
expect_exists("${WORK_DIR}/only.png")
expect_absent("${WORK_DIR}/only.ppm")
run_ok(${SMALL} --output "${WORK_DIR}/raw.ppm")
expect_exists("${WORK_DIR}/raw.ppm")
expect_absent("${WORK_DIR}/raw.png")

# 4. The same request twice gives the same bytes: the render is deterministic.
run_ok(${SMALL} --output "${WORK_DIR}/again.ppm")
file(SHA256 "${WORK_DIR}/again.ppm" first)
run_ok(${SMALL} --output "${WORK_DIR}/again.ppm")
file(SHA256 "${WORK_DIR}/again.ppm" second)
if(NOT first STREQUAL second)
    message(FATAL_ERROR "two runs with the same options wrote different images")
endif()

# 5. The mode changes the picture, and the size follows the options.
run_ok(${SMALL} --mode normals --output "${WORK_DIR}/m_normals.ppm")
run_ok(${SMALL} --mode distance --output "${WORK_DIR}/m_distance.ppm")
run_ok(${SMALL} --mode albedo --output "${WORK_DIR}/m_albedo.ppm")
file(SHA256 "${WORK_DIR}/m_normals.ppm" h_normals)
file(SHA256 "${WORK_DIR}/m_distance.ppm" h_distance)
file(SHA256 "${WORK_DIR}/m_albedo.ppm" h_albedo)
if(h_normals STREQUAL h_distance OR h_normals STREQUAL h_albedo OR h_distance STREQUAL h_albedo)
    message(FATAL_ERROR "different modes produced the same image")
endif()
run_ok(--width 30 --height 12 --output "${WORK_DIR}/size.ppm")
expect_ppm("${WORK_DIR}/size.ppm" 30 12)

# 6. Bad input: exit code 2, a message on stderr, and nothing written.
execute_process(COMMAND "${EXE}" --output "${WORK_DIR}/bad.jpg"
                RESULT_VARIABLE code OUTPUT_QUIET ERROR_VARIABLE err)
if(NOT code EQUAL 2)
    message(FATAL_ERROR "bad extension: expected exit code 2, got ${code}")
endif()
if(NOT err MATCHES "must end in")
    message(FATAL_ERROR "bad extension: message does not say what is wrong: ${err}")
endif()
expect_absent("${WORK_DIR}/bad.jpg")

execute_process(COMMAND "${EXE}" --mode fancy RESULT_VARIABLE code OUTPUT_QUIET ERROR_QUIET)
if(NOT code EQUAL 2)
    message(FATAL_ERROR "unknown mode: expected exit code 2, got ${code}")
endif()

# 7. A file that cannot be written: exit code 1.
execute_process(COMMAND "${EXE}" ${SMALL} --output "${WORK_DIR}/no_such_dir/x.png"
                RESULT_VARIABLE code OUTPUT_QUIET ERROR_VARIABLE err)
if(NOT code EQUAL 1)
    message(FATAL_ERROR "unwritable path: expected exit code 1, got ${code}")
endif()

# 8. --help succeeds and prints the usage.
execute_process(COMMAND "${EXE}" --help RESULT_VARIABLE code OUTPUT_VARIABLE text)
if(NOT code EQUAL 0 OR NOT text MATCHES "usage: raytracer")
    message(FATAL_ERROR "--help failed (${code}): ${text}")
endif()

message(STATUS "outputs: OK")
