#include "bellman_ford.hpp"

#include "util.hpp"
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

using std::vector;
using graph::Edge;
using graph::Graph;

namespace sssp {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

vector<double> bellmanFord(const Graph &graph, size_t source) {
  if (source > graph.num_vertices) {
    throw std::invalid_argument("Source id cannot exceed num_vertices");
  }

  // All vertices (except the source) are initially considered to be infinitely
  // far away
  auto distances = vector<double>(graph.num_vertices, kInfinity);
  distances[source] = 0.0;

  // Iteratively relax edges
  for (size_t iter = 0; iter < graph.num_vertices; iter++) {
    for (const Edge &edge : graph.getAllEdges()) {
      // "Relax" this edge (does this edge represent a shorter path?)
      if (distances.at(edge.src) + edge.cost < distances.at(edge.dest)) {
        distances[edge.dest] = distances.at(edge.src) + edge.cost;
      }
    }
  }

  // Check for negative-weight cycles
  for (const Edge &edge : graph.getAllEdges()) {
    if (distances.at(edge.src) + edge.cost < distances.at(edge.dest)) {
      throw std::domain_error("Graph contains a negative weight cycle.");
    }
  }

  return distances;
}

void bellmanFordWorker(const graph::Graph &graph, vector<double> &distances,
                       const util::Range &range, util::Barrier &barrier,
                       std::mutex &lock) {
  // Iteratively relax edges
  const auto &all_edges = graph.getAllEdges();
  auto localDists = vector<double>(distances);
  for (size_t iter = 0; iter < graph.num_vertices; iter++) {
    // relax edges
    for (size_t edge_id = range.start; edge_id < range.end; edge_id++) {
      // Relax this edge, ensuring mutual exclusion around access to distances
      const auto &edge = all_edges.at(edge_id);
      // std::lock_guard<std::mutex> guard{lock}; // lock released at end of scope
      if (localDists.at(edge.src) + edge.cost < localDists.at(edge.dest)) {
        distances[edge.dest] = localDists.at(edge.src) + edge.cost;
      }
    }
    barrier.wait();
    
    // read and copy distances
    for (size_t i = 0; i < graph.num_vertices; i++) {
      localDists[i] = distances.at(i);
    }
    barrier.wait();
  }
}

vector<double> bellmanFordParallel(const graph::Graph &graph, size_t source,
                                   size_t num_threads) {
  if (source > graph.num_vertices) {
    throw std::invalid_argument("Source id cannot exceed num_vertices");
  }

  auto partition = util::partition(graph.getNumEdges(), num_threads);
  auto threads = vector<std::thread>{};
  util::Barrier barrier{num_threads};
  std::mutex lock{};

  // Distances structure setup (non-source vertices initially considered
  // infinitely far away)
  auto distances = vector<double>(graph.num_vertices, kInfinity);
  distances[source] = 0.0;

  // Delegate to the worker function
  for (size_t i = 0; i < num_threads; i++) {
    threads.emplace_back(&bellmanFordWorker, std::cref(graph),
                         std::ref(distances), std::cref(partition.at(i)),
                         std::ref(barrier), std::ref(lock));
  }
  for (auto &thread : threads) {
    thread.join();
  }

  return distances;
}

} /* namespace sssp */
