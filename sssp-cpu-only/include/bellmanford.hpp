#pragma once

#include "graph.hpp"
#include <vector>

namespace sssp {

// Computes single-source shortest paths for a graph instance and source (vertex
// id) using Bellman-Ford. This algorithm tolerates negative-weight edges.
// Returns the cost of the shortest path from the source to every other vertex.
std::vector<double> bellmanFord(const graph::Graph &graph, size_t source);

} /* namespace sssp */
