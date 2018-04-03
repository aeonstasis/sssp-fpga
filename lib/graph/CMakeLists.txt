cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(GRAPH LANGUAGES CXX)

## C++ compiler options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")
set(CMAKE_CXX_EXTENSIONS OFF)

## Sources and headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
set(LIB_NAME graph)

## Generate executable
add_library(${LIB_NAME} SHARED ${SOURCES})
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION lib)
