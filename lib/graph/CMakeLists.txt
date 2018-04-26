cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(GRAPH LANGUAGES CXX)

## C++ compiler options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")
set(CMAKE_CXX_EXTENSIONS OFF)

## Sources and headers
set(LIB_NAME graph)
include_directories(include)

## Generate library
add_library(${LIB_NAME} SHARED src/graph.cpp)
target_include_directories(${LIB_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION lib)

## Generate graph utility executable
# add_executable(graph_gen src/main.cpp)
# target_link_libraries(graph_gen graph)
# target_link_libraries(graph_gen libdocopt)
# install(TARGETS graph_gen RUNTIME DESTINATION bin)

## Generate test executable
# enable_testing()
# add_executable(graph_test test/graph_test.cpp)
# target_link_libraries(graph_test graph)
# target_link_libraries(graph_test gtest gtest_main)
# add_test(GRAPH_TEST graph_test)
