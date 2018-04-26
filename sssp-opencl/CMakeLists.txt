cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(SSSP_OPENCL LANGUAGES CXX)

## C++ compiler options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")
set(CMAKE_CXX_EXTENSIONS OFF)

## Sources and headers
include_directories(include)
set(SOURCES src/main.cpp)
set(MAIN_EXE_NAME sssp_opencl)

## OpenCL
find_package(OpenCL REQUIRED)
include_directories(${OpenCL_INCLUDE_DIRS})
link_directories(${OpenCL_LIBRARY})

## Generate executable
add_executable(${MAIN_EXE_NAME} src/main.cpp)
target_link_libraries(${MAIN_EXE_NAME} OpenCL::OpenCL)
install(TARGETS ${MAIN_EXE_NAME} RUNTIME DESTINATION xcl)
