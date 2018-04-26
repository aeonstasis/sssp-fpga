#pragma once

#include "graph.hpp"
#include <cstddef>
#include <vector>

struct BellmanFordOutput {
  std::vector<double> distances;
  float elapsed;
};

BellmanFordOutput bellmanFordCUDA(const graph::Graph &graph, size_t source);
