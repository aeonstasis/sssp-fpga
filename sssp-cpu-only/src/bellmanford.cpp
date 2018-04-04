#include "bellmanford.hpp"

#include <limits>
#include <stdexcept>

using std::vector;
using graph::Edge;
using graph::Graph;

namespace sssp {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

vector<double> bellmanFord(const Graph& graph, size_t source) {
  vector<double> distances(graph.num_vertices, 0.0);

  for (size_t vertex = 0; vertex < graph.num_vertices; vertex++) {
    if (vertex != source) {
      distances[vertex] = kInfinity;
    }
  }

  for (size_t iter = 0; iter < graph.num_vertices; iter++) {
    for (const Edge& edge : graph.getAllEdges()) {
      // relaaaaaaaax.
      if (distances.at(edge.src) + edge.cost < distances.at(edge.dest)) {
        distances[edge.dest] = distances.at(edge.src) + edge.cost;
      }
    }
  }

  for (const Edge& edge : graph.getAllEdges()) {
    if (distances.at(edge.src) + edge.cost < distances.at(edge.dest)) {
      throw std::domain_error("Graph has negative weight cycle.");
    }
  }

  return distances;
}

} /* namespace sssp */
