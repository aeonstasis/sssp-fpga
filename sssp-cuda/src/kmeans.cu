#include "kmeans.h"

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
constexpr size_t kBlockSize = 512;
constexpr size_t kWordSize = 8;
__device__ double gMaxChange = 0.0;
constexpr typeof(gMaxChange) kZero = 0.0;
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

// Device function that determines the nearest centroid for each data point
__global__ void assign_centroids(double *points, size_t num_points, size_t dim,
                                 size_t points_pitch, double *old_centroids,
                                 double *centroids, size_t num_centroids,
                                 size_t centroids_pitch, int *counts) {
  // Calculate which point this thread represents
  const size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= num_points)
    return;
  auto point = address<const double>(points, index, 0, points_pitch);

  // Iterate through each centroid to find closest to this point
  double best_distance = DBL_MAX;
  size_t best_index = 0;
  for (int i = 0; i < num_centroids; i++) {
    auto centroid = address<const double>(old_centroids, i, 0, centroids_pitch);
    auto distance = dist(point, centroid, dim);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  // Atomically update counts and new centroids
  auto centroid = address<double>(centroids, best_index, 0, centroids_pitch);
  atomicVecAdd(centroid, point, dim);
  atomicAdd(&counts[best_index], 1);
}

// Device function that determines the nearest centroid for each data point
__global__ void assign_centroids_private(const double *points, size_t num_points,
                                         size_t dim, size_t points_pitch,
                                         const double *old_centroids,
                                         double *centroids,
                                         size_t num_centroids,
                                         size_t centroids_pitch, int *counts) {
  // Dynamic block-specific shared memory
  extern __shared__ char s[];
  auto local_centroids = reinterpret_cast<double *>(s);
  auto local_counts =
      reinterpret_cast<int *>(s + (num_centroids * centroids_pitch));

  // Calculate which point this thread represents
  const size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= num_points)
    return;
  auto point = address<const double>(points, index, 0, points_pitch);

  // Clear shared memory block
  if (threadIdx.x < num_centroids) {
    local_counts[threadIdx.x] = 0;
    auto local_centroid =
        address<double>(local_centroids, threadIdx.x, 0, centroids_pitch);
    memset(local_centroid, 0, centroids_pitch);
  }
  __syncthreads();

  // Iterate through each centroid to find closest to this point
  double best_distance = DBL_MAX;
  size_t best_index = 0;
  for (int i = 0; i < num_centroids; i++) {
    auto centroid = address<const double>(old_centroids, i, 0, centroids_pitch);
    auto distance = dist(point, centroid, dim);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  // Atomically update shared memory structures
  auto local_centroid =
      address<double>(local_centroids, best_index, 0, centroids_pitch);
  atomicVecAdd(local_centroid, point, dim);
  atomicAdd(&local_counts[best_index], 1);
  __syncthreads();

  // Aggregate updates to global memory
  if (threadIdx.x < num_centroids) {
    auto dest = address<double>(centroids, threadIdx.x, 0, centroids_pitch);
    auto src =
        address<double>(local_centroids, threadIdx.x, 0, centroids_pitch);
    atomicVecAdd(dest, src, dim);
    atomicAdd(&counts[threadIdx.x], local_counts[threadIdx.x]);
  }
}

// Device function that determines the nearest centroid for each data point
__global__ void
assign_centroids_extra(const double *__restrict__ points, size_t num_points,
                       size_t dim, size_t points_pitch,
                       const double *__restrict__ old_centroids,
                       double *__restrict__ centroids, size_t num_centroids,
                       size_t centroids_pitch, int *__restrict__ counts) {
  // Dynamic block-specific shared memory
  extern __shared__ char s[];
  auto local_centroids = reinterpret_cast<double *>(s);
  auto local_counts =
      reinterpret_cast<int *>(s + (num_centroids * centroids_pitch));

  // Calculate which point this thread represents
  const size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= num_points)
    return;
  auto point = address<const double>(points, index, 0, points_pitch);

  // Clear shared memory block
  if (threadIdx.x < num_centroids) {
    local_counts[threadIdx.x] = 0;
    auto local_centroid =
        address<double>(local_centroids, threadIdx.x, 0, centroids_pitch);
    memset(local_centroid, 0, centroids_pitch);
  }
  __syncthreads();

  // Iterate through each centroid to find closest to this point
  double best_distance = DBL_MAX;
  size_t best_index = 0;
  for (int i = 0; i < num_centroids; i++) {
    auto centroid = address<const double>(old_centroids, i, 0, centroids_pitch);
    auto distance = dist(point, centroid, dim);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  // Atomically update shared memory structures
  auto local_centroid =
      address<double>(local_centroids, best_index, 0, centroids_pitch);
  atomicVecAdd(local_centroid, point, dim);
  atomicAdd(&local_counts[best_index], 1);
  __syncthreads();

  // Aggregate updates to global memory
  if (threadIdx.x < num_centroids) {
    auto dest = address<double>(centroids, threadIdx.x, 0, centroids_pitch);
    auto src =
        address<double>(local_centroids, threadIdx.x, 0, centroids_pitch);
    atomicVecAdd(dest, src, dim);
    atomicAdd(&counts[threadIdx.x], local_counts[threadIdx.x]);
  }
}

// Recalculate the centroids based on which points map to which centroids
__global__ void normalize_centroids(double *points, size_t num_points,
                                    size_t dim, size_t points_pitch,
                                    double *old_centroids, double *centroids,
                                    size_t num_centroids,
                                    size_t centroids_pitch, int *counts,
                                    curandState_t *states) {
  // Calculate which centroid this thread represents
  const size_t index = threadIdx.x;
  auto centroid = address<double>(centroids, index, 0, centroids_pitch);

  // Divide each sum to get the new mean
  if (counts[index] > 0) {
    for (size_t i = 0; i < dim; i++) {
      centroid[i] /= counts[index];
    }
  } else {
    // Set random point
    size_t point_index = curand(&states[threadIdx.x]) % num_points;
    auto point = address<const double>(points, point_index, 0, points_pitch);
    memcpy(centroid, point, dim * sizeof(double));
  }

  // Update max change for this iteration
  auto old_centroid = address<double>(old_centroids, index, 0, centroids_pitch);
  auto change = sqrt(dist(centroid, old_centroid, dim));
  atomicMax(&gMaxChange, change);

  // Clear the old_centroids (swapped with centroids at end of iter)
  memset(old_centroid, 0, centroids_pitch);
  counts[index] = 0;
}

// Recalculate the centroids based on which points map to which centroids
__global__ void normalize_centroids_extra(
    const double *__restrict__ points, size_t num_points, size_t dim,
    size_t points_pitch, double *__restrict__ old_centroids,
    double *__restrict__ centroids, size_t num_centroids,
    size_t centroids_pitch, int *__restrict__ counts, curandState_t *states) {
  // Calculate which centroid this thread represents
  const size_t index = threadIdx.x;
  auto centroid = address<double>(centroids, index, 0, centroids_pitch);

  // Divide each sum to get the new mean
  if (counts[index] > 0) {
    for (size_t i = 0; i < dim; i++) {
      centroid[i] /= counts[index];
    }
  } else {
    // Set random point
    size_t point_index = curand(&states[threadIdx.x]) % num_points;
    auto point = address<const double>(points, point_index, 0, points_pitch);
    memcpy(centroid, point, dim * sizeof(double));
  }

  // Update max change for this iteration
  auto old_centroid = address<double>(old_centroids, index, 0, centroids_pitch);
  auto change = sqrt(dist(centroid, old_centroid, dim));
  atomicMax(&gMaxChange, change);

  // Clear the old_centroids (swapped with centroids at end of iter)
  memset(old_centroid, 0, centroids_pitch);
  counts[index] = 0;
}

KmeansOutput cuda_kmeans(const vector<double> &data, size_t dim,
                         size_t clusters, size_t iterations, double threshold,
                         bool privatize, bool extra) {
  auto start = CudaEvent{};
  auto end = CudaEvent{};

  // Data structure initialization
  auto dataset = CudaArray<double>(data, dim);
  start.record();
  auto num_points = dataset.num_rows;
  auto counts = CudaVector<int>(clusters);

  // Random initial centroids
  auto init_clusters = vector<double>(clusters * dim);
  std::generate(init_clusters.begin(), init_clusters.end(),
                []() { return static_cast<double>(std::rand()) / RAND_MAX; });
  auto centroids = CudaArray<double>(init_clusters, dim);
  auto old_centroids = CudaArray<double>(clusters, dim);

  // Random initialization for GPU
  auto states = CudaVector<curandState_t>(clusters);
  init<<<1, clusters>>>(std::time(0), states.data());

  // Calculate grid and block sizes to guarantee at least one thread per point
  auto threads = dim3{kBlockSize};
  auto blocks = dim3{};
  blocks.x = (num_points + threads.x - 1) / threads.x;

  // Kernel-based k-means implementation
  auto current_max_change = typeof(gMaxChange){};
  auto shared_bytes = centroids.num_bytes + counts.num_bytes;
  size_t iter = 0;
  do {
    std::swap(old_centroids, centroids);

    // Stage where we find nearest centroids and calculate sum
    if (privatize) {
      assign_centroids_private<<<blocks, threads, shared_bytes>>>(
          dataset.data(), num_points, dim, dataset.pitch, old_centroids.data(),
          centroids.data(), clusters, centroids.pitch, counts.data());
    } else if (extra) {
      assign_centroids_extra<<<blocks, threads, shared_bytes>>>(
          dataset.data(), num_points, dim, dataset.pitch, old_centroids.data(),
          centroids.data(), clusters, centroids.pitch, counts.data());
    } else {
      assign_centroids<<<blocks, threads>>>(
          dataset.data(), num_points, dim, dataset.pitch, old_centroids.data(),
          centroids.data(), clusters, centroids.pitch, counts.data());
    }
    CHECK_CUDA_ERROR(cudaPeekAtLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());

    // Stage where we divide out the sums to get the means and check convergence
    if (privatize || extra) {
      normalize_centroids_extra<<<1, clusters>>>(
          dataset.data(), num_points, dim, dataset.pitch, old_centroids.data(),
          centroids.data(), clusters, centroids.pitch, counts.data(),
          states.data());
    } else {
      normalize_centroids<<<1, clusters>>>(
          dataset.data(), num_points, dim, dataset.pitch, old_centroids.data(),
          centroids.data(), clusters, centroids.pitch, counts.data(),
          states.data());
    }
    CHECK_CUDA_ERROR(cudaPeekAtLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());

    // Check convergence and swap old_centroids and centroids
    CHECK_CUDA_ERROR(cudaMemcpyFromSymbol(&current_max_change, gMaxChange,
                                          sizeof(current_max_change)));
    CHECK_CUDA_ERROR(cudaMemcpyToSymbol(gMaxChange, &kZero, sizeof(kZero)));
  } while ((++iter < iterations || iterations == 0) &&
           (current_max_change > threshold));

  // Copy back final centroids data
  end.record();
  auto host_centroids = vector<double>(clusters * dim * sizeof(double));
  centroids.copyFromDevice(host_centroids.data());
  end.wait();

  return {iter, host_centroids, end.since(start)};
}
