cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(SSSP)
include(ExternalProject)

# Load CMAKE configuration from environment variables
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})

# Set build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release"
       CACHE STRING "Build configuration 'Release' or 'Debug'."
       FORCE)
endif()

# Third-party Dependencies
# Docopt
ExternalProject_Add(
  docopt
  PREFIX "${PROJECT_SOURCE_DIR}/third-party"
	GIT_REPOSITORY git@github.com:docopt/docopt.cpp.git
  BUILD_COMMAND make docopt_s
  INSTALL_COMMAND ""
	LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  LOG_INSTALL ON)
ExternalProject_Get_Property(docopt binary_dir)
ExternalProject_Get_Property(docopt source_dir)
include_directories(${source_dir})
add_library(libdocopt STATIC IMPORTED GLOBAL)
add_dependencies(libdocopt docopt)
set_target_properties(libdocopt PROPERTIES
  IMPORTED_LOCATION ${binary_dir}/libdocopt.a
)

# Boost
find_package(Boost 1.58.0 REQUIRED COMPONENTS serialization)

# Subdirectories
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/graph lib)
add_subdirectory(${PROJECT_SOURCE_DIR}/sssp-cpu-only bin)
