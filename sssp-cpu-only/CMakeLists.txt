cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(SSSP_CPU_ONLY LANGUAGES CXX)

## C++ compiler options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")
set(CMAKE_CXX_EXTENSIONS OFF)

## Required imports
find_package(Threads REQUIRED)

## Sources and headers
include_directories(include)
set(SOURCES src/main.cpp src/bellman_ford.cpp)
set(MAIN_EXE_NAME sssp_cpu_only)

## Generate library
add_library(bellmanford STATIC src/bellman_ford.cpp src/util.cpp)
target_link_libraries(bellmanford ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(bellmanford graph)

## Generate executable
add_executable(${MAIN_EXE_NAME} ${SOURCES})
target_link_libraries(${MAIN_EXE_NAME} bellmanford)
target_link_libraries(${MAIN_EXE_NAME} libdocopt)
install(TARGETS ${MAIN_EXE_NAME} RUNTIME DESTINATION bin)

## Generate test executable
## enable_testing()
## include(GoogleTest)
## add_executable(bellmanford_test test/util_test.cpp test/bellman_ford_test.cpp)
## target_link_libraries(bellmanford_test bellmanford)
## target_link_libraries(bellmanford_test gtest gtest_main)
## add_test(BELLMANFORD_TEST bellmanford_test)
