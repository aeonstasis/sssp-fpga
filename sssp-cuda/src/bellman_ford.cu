#include "bellman_ford.h"

#include <algorithm>
#include <cfloat>
#include <ctime>
#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdio.h>
using std::vector;
using std::unique_ptr;
using graph::Graph;
using graph::Edge;

#define CHECK_CUDA_ERROR(ans)                                                  \
  { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPU error: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    exit(code);
  }
}

// Global variables
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr size_t kBlockSize = 512;
constexpr size_t kWordSize = 8;
__device__ double gMaxChange = 0.0;
struct deleter {
  void operator()(void *ptr) { cudaFree(ptr); }
};

class CudaEvent final {
public:
  CudaEvent() { cudaEventCreate(&event); }
  ~CudaEvent() { cudaEventDestroy(event); }
  void wait() { cudaEventSynchronize(event); }
  void record() { cudaEventRecord(event); }
  float since(const CudaEvent &earlier) {
    float elapsed = 0.0f;
    cudaEventElapsedTime(&elapsed, earlier.event, event);
    return elapsed;
  }

private:
  cudaEvent_t event;
};

/**
 * Thin RAII wrapper managing device view and memory of a 1D vector.
 */
template <typename T> struct CudaVector final {
  size_t num_elems;
  size_t num_bytes;
  unique_ptr<T[], deleter> device_ptr;

  CudaVector(size_t num_elems_)
      : num_elems(num_elems_), num_bytes(), device_ptr(nullptr) {
    // Align all allocations to 8-byte boundary
    num_bytes = num_elems_ * sizeof(T);
    size_t rem = num_bytes % kWordSize;
    num_bytes += kWordSize - rem;

    // Malloc memory and zero-initialize it
    CHECK_CUDA_ERROR(cudaMalloc((void **)&device_ptr, num_bytes));
    CHECK_CUDA_ERROR(cudaMemset(device_ptr.get(), 0, num_bytes));
  }

  CudaVector(const vector<T> &data_)
      : num_elems(data_.size()), num_bytes(0), device_ptr(nullptr) {
    num_bytes = data_.size() * sizeof(T);
    size_t rem = num_bytes % kWordSize;
    num_bytes += kWordSize - rem;

    const char* host_ptr = reinterpret_cast<const char*>(data_.data());

    CHECK_CUDA_ERROR(cudaMalloc((void **)&device_ptr, num_bytes));
    CHECK_CUDA_ERROR(cudaMemcpy(device_ptr.get(), host_ptr, num_elems * sizeof(T),
          cudaMemcpyHostToDevice));
  }

  void copyFromDevice(T *host_ptr) {
    CHECK_CUDA_ERROR(cudaMemcpy(host_ptr, device_ptr.get(), num_elems * sizeof(T),
          cudaMemcpyDeviceToHost));
  }

  const T *data() const { return device_ptr.get(); }
  T *data() { return device_ptr.get(); }
  void clear() { CHECK_CUDA_ERROR(cudaMemset(device_ptr.get(), 0, num_bytes)); }
};

/**
 * Thin RAII wrapper managing device view and memory of a 2D array.
 */
template <typename T> struct CudaArray final {
  size_t num_rows;
  size_t num_cols;
  size_t pitch;
  size_t num_bytes;
  unique_ptr<T[], deleter> device_ptr;

  CudaArray(size_t num_rows_, size_t num_cols_)
      : num_rows(num_rows_), num_cols(num_cols_), pitch(), num_bytes(),
        device_ptr(nullptr) {
    // Allocate 2D pitched memory on the device and get device ptr
    CHECK_CUDA_ERROR(cudaMallocPitch((void **)&device_ptr, &pitch,
                                     num_cols * sizeof(T), num_rows));
    CHECK_CUDA_ERROR(
        cudaMemset2D(device_ptr.get(), pitch, 0, num_cols, num_rows));
    num_bytes = num_rows * pitch;
  }

  CudaArray(const vector<T> &data_, size_t num_cols_)
      : CudaArray(data_.size() / num_cols_, num_cols_) {
    // Additionally memcpy host src vector to device
    auto host_ptr = reinterpret_cast<const char *>(data_.data());
    CHECK_CUDA_ERROR(cudaMemcpy2D(device_ptr.get(), pitch, host_ptr,
                                  num_cols * sizeof(T), num_cols * sizeof(T),
                                  num_rows, cudaMemcpyHostToDevice));
  }

  void copyFromDevice(T *host_ptr) {
    CHECK_CUDA_ERROR(cudaMemcpy2D(host_ptr, num_cols * sizeof(T),
                                  device_ptr.get(), pitch, num_cols * sizeof(T),
                                  num_rows, cudaMemcpyDeviceToHost));
  }

  const T *data() const { return device_ptr.get(); }
  T *data() { return device_ptr.get(); }
  void clear() {
    CHECK_CUDA_ERROR(
        cudaMemset2D(device_ptr.get(), pitch, 0, num_cols, num_rows));
  }
};

__global__ void init(unsigned int seed, curandState_t *states) {
  curand_init(seed, threadIdx.x, 0, &states[threadIdx.x]);
}

// Helper to allow atomicMax to be invoked on a double
// https://github.com/treecode/Bonsai/blob/master/runtime/profiling/derived_atomic_functions.h
__device__ __forceinline__ double atomicMax(double *address, double val) {
  unsigned long long ret = __double_as_longlong(*address);
  while (val > __longlong_as_double(ret)) {
    unsigned long long old = ret;
    if ((ret = atomicCAS((unsigned long long *)address, old,
                         __double_as_longlong(val))) == old)
      break;
  }
  return __longlong_as_double(ret);
}

// Helper to allow atomicMin to be invoked on a double
// https://github.com/treecode/Bonsai/blob/master/runtime/profiling/derived_atomic_functions.h
__device__ __forceinline__ double atomicMin(double *address, double val) {
  unsigned long long ret = __double_as_longlong(*address);
  while (val < __longlong_as_double(ret)) {
    unsigned long long old = ret;
    if ((ret = atomicCAS((unsigned long long *)address, old,
                         __double_as_longlong(val))) == old)
      break;
  }
  return __longlong_as_double(ret);
}

// Templated helper to index into a flat array
template <typename T>
__device__ __forceinline__ T *address(T *src, size_t row, size_t col,
                                      size_t pitch) {
  return (T *)((char *)src + row * pitch) + col;
}

// Helper function for L2 distance
__device__ __forceinline__ double
dist(const double *__restrict__ p0, const double *__restrict__ p1, size_t dim) {
  double distance = 0.0;
  for (int i = 0; i < dim; i++) {
    distance += (p0[i] - p1[i]) * (p0[i] - p1[i]);
  }
  return distance;
}

__device__ __forceinline__ void atomicVecAdd(double *__restrict__ dest,
                                             const double *__restrict__ src,
                                             size_t dim) {
  for (int i = 0; i < dim; i++) {
    atomicAdd(&dest[i], src[i]);
  }
}

__global__ void relax(const size_t num_edges, const double* distsRead, 
    double* distsWrite, const Edge* edges) {

  const size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if(index > num_edges) {
    return;
  }

  Edge edge = edges[index];

  double val = distsRead[edge.src] + edge.cost;
  atomicMin(&distsWrite[edge.dest], val);
}

__global__ void copyBack(const size_t num_points, double* distsRead,
    const double* distsWrite) {
  const size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if(index > num_points) {
    return;
  }

  distsRead[index] = distsWrite[index];
};

BellmanFordOutput bellmanFordCUDA(const Graph &graph, size_t source) {
  auto start = CudaEvent{};
  auto end = CudaEvent{};

  start.record();

  vector<double> localDistances = vector<double>(graph.num_vertices, kInfinity);
  localDistances[source] = 0.0;

  // data structure initialization
  CudaVector<double> distsRead = CudaVector<double>(localDistances);
  CudaVector<double> distsWrite = CudaVector<double>(localDistances);
  CudaVector<Edge> edges = CudaVector<Edge>(graph.getAllEdges());

  // grid and block size calculation
  auto threads = dim3{kBlockSize};
  auto blocks = dim3{};
  blocks.x = (graph.getNumEdges() + threads.x - 1) / threads.x;

  // iteration
  for(size_t iter = 0; iter < graph.num_vertices; iter++) {
    relax<<<blocks, threads>>>(graph.getNumEdges(), distsRead.data(), distsWrite.data(), edges.data());
    copyBack<<<blocks, threads>>>(graph.num_vertices, distsRead.data(), distsWrite.data());
  }

  end.record();
  distsRead.copyFromDevice(localDistances.data());
  end.wait();

  return {localDistances, end.since(start)};
}
