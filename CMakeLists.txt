cmake_minimum_required(VERSION 3.20)
project(raytracer_v0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(engine INTERFACE)
target_include_directories(engine INTERFACE include)

add_library(physics INTERFACE)
target_link_libraries(physics INTERFACE engine)
target_include_directories(physics INTERFACE include)

add_executable(raytracer src/main.cpp)
target_link_libraries(raytracer PRIVATE engine)