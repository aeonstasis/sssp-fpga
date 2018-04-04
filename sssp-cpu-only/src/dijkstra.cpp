#include "dijkstra.hpp"

#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

using std::priority_queue;
using std::vector;
using std::pair;
using graph::Graph;
using graph::GraphPriorityQueue;

namespace sssp {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

vector<double> dijkstra(const Graph &graph, size_t source) {
  auto num_vertices = graph.num_vertices;
  if (source > num_vertices) {
    throw std::invalid_argument("Source must be less than number of vertices");
  }

  auto distances = vector<double>(num_vertices, 0.0);
  auto frontier = GraphPriorityQueue(num_vertices);

  for (size_t vertex = 0; vertex < num_vertices; vertex++) {
    if (vertex != source) {
      distances.at(vertex) = kInfinity;
    }
    frontier.add(vertex, distances.at(vertex));
  }

  while (!frontier.empty()) {
    auto vertex = frontier.remove_min();
    for (const auto &neighbor : graph.getNeighbors(vertex)) {
      auto alt_cost = distances.at(vertex) + graph.cost(vertex, neighbor);
      if (alt_cost < distances.at(neighbor)) {
        distances[neighbor] = alt_cost;
        frontier.decrease_priority(neighbor, alt_cost);
      }
    }
  }

  return distances;
}

} /* namespace sssp */
