cmake_minimum_required(VERSION 3.16.4 FATAL_ERROR)

find_package(ITK 5.3.0 REQUIRED)

if(ITK_FOUND)
  include(${ITK_USE_FILE})
else()
  message(FATAL_ERROR "ITK was not found. Please set ITK_DIR.")
endif()

FetchContent_Declare(
  fmm
  GIT_REPOSITORY https://github.com/thinks/fast-marching-method.git
  GIT_TAG master
)

FetchContent_GetProperties(fmm)

if(NOT fmm_POPULATED)
  FetchContent_Populate(fmm)
  message(STATUS "fast-marching-method source: ${fmm_SOURCE_DIR}")
endif()

FetchContent_Declare(
  argparse
  GIT_REPOSITORY https://github.com/p-ranav/argparse.git
  GIT_TAG master
)

FetchContent_GetProperties(argparse)

if(NOT argparse_POPULATED)
  FetchContent_Populate(argparse)
  message(STATUS "argparse source: ${argparse_SOURCE_DIR}")
endif()


### Dijkstra example
add_executable(Dijkstra Dijkstra.cxx Params.cxx)

# Optionally include other priority queue implementations for testing.
# (These could be added as submodules.)
set(OTHER_PRIORITY_QUEUES
  # external/fiboheap # https://github.com/beniz/fiboheap
  # external/gheap # https://github.com/valyala/gheap
  # external/IndexedPriorityQueue # https://github.com/kartikkukreja/blog-codes/blob/master/src/Indexed%20Min%20Priority%20Queue.cpp
  # external/jkds/include # https://github.com/jkomyno/jkds
  # external/updatable_priority_queue # https://github.com/Ten0/updatable_priority_queue
)

target_include_directories(Dijkstra SYSTEM PRIVATE
  ${argparse_SOURCE_DIR}/include
  ${glm_SOURCE_DIR}
  ${OTHER_PRIORITY_QUEUES}
  ${ITK_INCLUDE_DIRS}
)

target_link_libraries(Dijkstra PRIVATE
  distanc3d
  ${ITK_LIBRARIES}
)

target_compile_options(Dijkstra PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
  -Werror -Wall -Wextra -Wpointer-arith -Winit-self -Wunreachable-code -Wshadow -Wpedantic
  -Wconversion -Wsign-conversion -O3 >
  $<$<CXX_COMPILER_ID:AppleClang>:
  -Werror -Wall -Wextra -Wpointer-arith -Winit-self -Wunreachable-code -Wpedantic -Wshadow -g -O3 >
  $<$<CXX_COMPILER_ID:MSVC>: /W4 /Ox >
)

set_target_properties(Dijkstra PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)


### Maurer distance map example
add_executable(Maurer Maurer.cxx)
target_include_directories(Maurer SYSTEM PRIVATE ${ITK_INCLUDE_DIRS})
target_link_libraries(Maurer PRIVATE ${ITK_LIBRARIES})

target_compile_options(Maurer PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
  -Werror -Wall -Wextra -Wpointer-arith -Winit-self -Wunreachable-code -Wshadow -Wpedantic
  -Wconversion -Wsign-conversion >
  $<$<CXX_COMPILER_ID:AppleClang>:
  -Werror -Wall -Wextra -Wpointer-arith -Winit-self -Wunreachable-code -Wpedantic -Wshadow>
  $<$<CXX_COMPILER_ID:MSVC>: /W4 /Ox >
)

set_target_properties(Maurer PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)


### Fast Marching Method example
add_executable(FastMarching FastMarching.cxx)

target_include_directories(FastMarching SYSTEM PRIVATE
  ${fmm_SOURCE_DIR}/include/thinks
  ${ITK_INCLUDE_DIRS}
)
target_link_libraries(FastMarching PRIVATE ${ITK_LIBRARIES})

target_compile_options(FastMarching PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
  -Werror -Wall -Wextra -Wpointer-arith -Winit-self -Wunreachable-code -Wshadow -Wpedantic
  -Wconversion -Wsign-conversion >
  $<$<CXX_COMPILER_ID:AppleClang>:
  -Werror -Wall -Wextra -Wpointer-arith -Winit-self -Wunreachable-code -Wpedantic -Wshadow >
  $<$<CXX_COMPILER_ID:MSVC>: /W4 /Ox >
)

set_target_properties(FastMarching PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
