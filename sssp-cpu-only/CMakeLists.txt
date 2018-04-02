cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(SSSP_CPU_ONLY LANGUAGES CXX)

## C++ compiler options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")
set(CMAKE_CXX_EXTENSIONS OFF)

## Required imports
find_package(Threads REQUIRED)

## Sources and headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
set(MAIN_EXE_NAME sssp_cpu_only)

## Generate executable
add_executable(${MAIN_EXE_NAME} ${SOURCES})
target_link_libraries(${MAIN_EXE_NAME} ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS ${MAIN_EXE_NAME} RUNTIME DESTINATION bin)
