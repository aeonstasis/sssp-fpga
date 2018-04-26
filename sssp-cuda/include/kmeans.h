#pragma once

#include <cstddef>
#include <vector>

struct KmeansOutput {
  size_t iter;                   // number of iterations to converge
  std::vector<double> centroids; // final calculated cluster centers
  float elapsed;                 // elapsed time
};

/**
 * CUDA-accelerated kmeans implementation.
 *
 * @param data - nested point data
 * @param clusters - number of clusters to find
 * @param iterations - maximum number of iterations to run
 * @param threshold - convergence threshold
 * @return output - KmeansOutput instance
 */
KmeansOutput cuda_kmeans(const std::vector<double> &data, size_t dim,
                         size_t clusters, size_t iterations, double threshold,
                         bool privatize, bool extra);
