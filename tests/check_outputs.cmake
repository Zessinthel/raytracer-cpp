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
run_ok(${SMALL} --mode lambert --output "${WORK_DIR}/m_lambert.ppm")
run_ok(${SMALL} --mode phong --output "${WORK_DIR}/m_phong.ppm")
file(SHA256 "${WORK_DIR}/m_normals.ppm" h_normals)
file(SHA256 "${WORK_DIR}/m_distance.ppm" h_distance)
file(SHA256 "${WORK_DIR}/m_albedo.ppm" h_albedo)

# normals, distance, albedo and object_id encode the answer in fundamentally
# different ways (sky+normal color, grayscale by depth, flat material color,
# a fixed id palette), so they must differ for any scene with at least one
# hit, regardless of where the lights or the camera happen to sit.
run_ok(${SMALL} --mode object-id --output "${WORK_DIR}/m_object_id.ppm")
file(SHA256 "${WORK_DIR}/m_object_id.ppm" h_object_id)
set(structural_hashes ${h_normals} ${h_distance} ${h_albedo} ${h_object_id})
list(LENGTH structural_hashes structural_count)
list(REMOVE_DUPLICATES structural_hashes)
list(LENGTH structural_hashes structural_distinct)
if(NOT structural_count EQUAL structural_distinct)
    message(FATAL_ERROR "two structurally different modes (normals/distance/albedo/object-id) produced the same image")
endif()

# lambert vs phong is different: phong = lambert + a specular term that is
# never negative, so the only guarantee that holds for ANY scene and ANY
# resolution is phong's total brightness >= lambert's, with equality when no
# sampled ray happens to catch a highlight (plausible at this small size).
file(STRINGS "${WORK_DIR}/m_lambert.ppm" lambert_lines)
file(STRINGS "${WORK_DIR}/m_phong.ppm" phong_lines)
set(lambert_sum 0)
set(phong_sum 0)
list(LENGTH lambert_lines n_lines)
math(EXPR last_line "${n_lines} - 1")
foreach(k RANGE 3 ${last_line})
    list(GET lambert_lines ${k} lrow)
    list(GET phong_lines ${k} prow)
    string(REGEX MATCHALL "[0-9]+" lvals "${lrow}")
    string(REGEX MATCHALL "[0-9]+" pvals "${prow}")
    foreach(v IN LISTS lvals)
        math(EXPR lambert_sum "${lambert_sum} + ${v}")
    endforeach()
    foreach(v IN LISTS pvals)
        math(EXPR phong_sum "${phong_sum} + ${v}")
    endforeach()
endforeach()
if(phong_sum LESS lambert_sum)
    message(FATAL_ERROR "phong (${phong_sum}) is darker than lambert (${lambert_sum}): the specular term must never subtract light")
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
