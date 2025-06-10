cmake_minimum_required(VERSION 3.16.4 FATAL_ERROR)

include(FetchContent)

message(STATUS "Using compiler ${CMAKE_CXX_COMPILER_ID}, version ${CMAKE_CXX_COMPILER_VERSION}")

# Flag to use either std::span or an external library for span due to lack of compiler support
set(SUPPORTS_STD_SPAN FALSE)

if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10) OR
   (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 7) OR
   (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0.0) OR
   (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.26))
  set(SUPPORTS_STD_SPAN TRUE)
endif()

# Header only C++ mathematics library for graphics software based on the
# OpenGL Shading Language (GLSL) specifications
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 0af55ccecd98d4e5a8d1fad7de25ba429d60e863 # 1.0.1
)

FetchContent_MakeAvailable(glm)
message(STATUS "GLM source: ${glm_SOURCE_DIR}")

add_library(distanc3d INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Dijkstra.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/IPriorityQueue.h
)

target_include_directories(distanc3d INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  {glm_SOURCE_DIR}
)

if(SUPPORTS_STD_SPAN)
  message(STATUS "Using native C++ std::span library")
  target_compile_definitions(distanc3d INTERFACE USE_STD_SPAN)
else()
  # Use tcb::span, a std::span implementation for C++11 and later, which is used
  # for compilers without C++20 support
  FetchContent_Declare(
    span
    GIT_REPOSITORY https://github.com/tcbrindle/span.git
    GIT_TAG 836dc6a0efd9849cb194e88e4aa2387436bb079b
  )

  FetchContent_GetProperties(span)

  if(NOT span_POPULATED)
    FetchContent_Populate(span)
    message(STATUS "Using external tcb::span library: ${span_SOURCE_DIR}")
    target_include_directories(distanc3d INTERFACE ${span_SOURCE_DIR}/include)
  endif()
endif()
