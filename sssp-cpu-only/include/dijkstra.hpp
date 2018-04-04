#pragma once

#include "graph.hpp"
#include <vector>

namespace sssp {

// Computes single-source shortest paths for a graph instance and source (vertex
// id) using Dijkstra's algorith,. Note that correctness is not guaranteed for
// graphs with negative-weight edges. Returns the cost of the shortest path from
// the source to every other vertex.
//
// Is read-only on the graph and thus thread-safe if no interleaving writes are
// made.
std::vector<double> dijkstra(const graph::Graph &graph, size_t source);

} /* namespace sssp */
