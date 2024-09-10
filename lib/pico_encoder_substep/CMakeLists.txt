cmake_minimum_required(VERSION 3.19)

include(pico_sdk_import.cmake)

project(Pico-Encoder)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(example)
add_subdirectory(lib)
