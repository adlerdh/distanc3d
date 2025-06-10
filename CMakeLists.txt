cmake_minimum_required(VERSION 3.16.4 FATAL_ERROR)
project(distanc3d VERSION 1.0.0 LANGUAGES CXX)

option(BUILD_EXAMPLES "Build examples" ON)

add_subdirectory(distanc3d)

if(BUILD_EXAMPLES)
  add_subdirectory(example)
endif()
