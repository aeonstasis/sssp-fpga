cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project (sssp_cuda LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")

# Included headers #
include_directories(include)

# Util lib
add_library(util ${PROJECT_SOURCE_DIR}/src/util.cpp)

# Cuda lib
add_library(bellman_ford_cuda STATIC ${PROJECT_SOURCE_DIR}/src/bellman_ford.cu)
target_link_libraries(bellman_ford_cuda graph)
set_target_properties(bellman_ford_cuda PROPERTIES CUDA_SEPARATE_COMPILATION ON CUDA_STANDARD 11)

# Add sources #
set(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp)

# Program binaries and linker
add_executable(sssp_cuda ${SOURCES})
set_property(TARGET sssp_cuda PROPERTY CUDA_SEPARATE_COMPILATION ON)
target_link_libraries(sssp_cuda PUBLIC util bellman_ford_cuda libdocopt graph ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS sssp_cuda RUNTIME DESTINATION bin)

# Run with the appropriate architecture >50 at the command line
# cmake -DCMAKE_CUDA_FLAGS="-arch=sm_61" -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
