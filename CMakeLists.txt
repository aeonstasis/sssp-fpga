cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(SSSP)

# Load CMAKE configuration from environment variables
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})

# Set build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release"
       CACHE STRING "Build configuration 'Release' or 'Debug'."
       FORCE)
endif()

# Third-party dependencies
include(CTest)
file(GLOB cmakes ${PROJECT_SOURCE_DIR}/cmake/*.cmake)
foreach(cmake ${cmakes})
  include(${cmake})
endforeach(cmake)

# Subdirectories
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/graph graph)
add_subdirectory(${PROJECT_SOURCE_DIR}/sssp-cpu-only cpu)
add_subdirectory(${PROJECT_SOURCE_DIR}/sssp-cuda cuda)
# add_subdirectory(${PROJECT_SOURCE_DIR}/sssp-opencl xcl)
