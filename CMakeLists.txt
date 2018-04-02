cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
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

set(BINARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Subdirectories
add_subdirectory(${PROJECT_SOURCE_DIR}/sssp-cpu-only bin)
