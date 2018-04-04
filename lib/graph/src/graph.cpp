#include "graph.hpp"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <cstdlib>
#include <iostream>
#include <stdio.h>

using std::string;
using std::vector;

namespace graph {

Graph::Graph(size_t num_vertices)
    : num_vertices(num_vertices), adjacency_list(num_vertices) {}

void Graph::addEdge(size_t src, size_t dest, size_t cost) {
  adjacency_list[src].push_back({src, dest, cost});
  adjacency_list[dest].push_back({dest, src, cost});
}

vector<size_t> Graph::getNeighbors(size_t src) const {
  auto num_neighbors = adjacency_list[src].size();
  std::vector<size_t> res(num_neighbors);
  for (size_t i = 0; i < num_neighbors; i++) {
    res[i] = adjacency_list[src][i].dest;
  }
  return res;
}

size_t Graph::cost(size_t src, size_t dest) const {
  return adjacency_list.at(src).at(dest).cost;
}

string Graph::toString() const {
  std::string str;
  str += std::to_string(num_vertices) + "\n";
  for (size_t vertex = 0; vertex < num_vertices; vertex++) {
    const auto &edges = adjacency_list[vertex];
    for (const Edge &e : edges) {
      str += std::to_string(e.src) + " " + std::to_string(e.dest) + " " +
             std::to_string(e.cost) + "\n";
    }
  }
  return str;
}

Graph Graph::generateGraph(size_t num_vertices, size_t num_edges, int seed) {
  std::srand(seed);
  auto graph = Graph{num_vertices};

  if (num_edges < num_vertices - 1) {
    std::cerr << "Graph generation failed: number of vertices (" << num_vertices
              << ") was less than or equal to number of edges" << num_edges
              << "." << std::endl;
    return graph;
  }

  for (size_t i = 1; i < num_vertices; i++) {
    size_t src = std::rand() % i;
    size_t cost = 1 + (std::rand() % kMaxCost);
    graph.addEdge(src, i, cost);
  }

  for (size_t i = num_vertices - 1; i < num_edges; i++) {
    size_t src, dest;
    do {
      src = std::rand() % num_vertices;
      dest = std::rand() % num_vertices;
    } while (src == dest);
    size_t cost = 1 + (std::rand() % kMaxCost);
    graph.addEdge(src, dest, cost);
  }

  return graph;
}

} /* namespace graph */
