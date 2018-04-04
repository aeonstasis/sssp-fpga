#include "graph.hpp"

#include <stdio.h>
#include <algorithm>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

using std::string;
using std::vector;

namespace graph {

void checkBounds(const vector<size_t> &input, size_t max) {
  if (std::any_of(input.begin(), input.end(),
                  [max](const auto &val) { return val >= max; })) {
    throw std::invalid_argument("Specified index out of range");
  }
}

Graph::Graph(size_t num_vertices)
    : num_vertices(num_vertices), adjacency_list(num_vertices), all_edges(0) {}

void Graph::addEdge(size_t src, size_t dest, size_t cost) {
  checkBounds({src, dest}, num_vertices);
  adjacency_list[src].push_back({src, dest, cost});
  adjacency_list[dest].push_back({dest, src, cost});
}

const std::vector<Edge> &Graph::getEdges(size_t src) const {
  return adjacency_list.at(src);
}

const std::vector<Edge> &Graph::getAllEdges() {
  if (all_edges.size()) return all_edges;

  for (size_t vertex = 0; vertex < num_vertices; vertex++) {
    vector<Edge> edges = getEdges(vertex);
    all_edges.insert(std::end(all_edges), std::begin(edges), std::end(edges));
  }
  return all_edges;
}

vector<size_t> Graph::getNeighbors(size_t src) const {
  checkBounds({src}, num_vertices);
  auto neighbors = vector<size_t>{};
  for (const auto &edge : adjacency_list.at(src)) {
    if (edge.src == src) {
      neighbors.push_back(edge.dest);
    }
  }
  return neighbors;
}

size_t Graph::cost(size_t src, size_t dest) const {
  checkBounds({src, dest}, num_vertices);
  const auto &vertex_list = adjacency_list.at(src);
  auto edge =
      std::find_if(vertex_list.begin(), vertex_list.end(),
                   [dest](const auto &edge) { return edge.dest == dest; });
  if (edge == vertex_list.end()) {
    throw std::invalid_argument("Specified edge doesn't exist");
  }
  return (*edge).cost;
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

void Graph::save(const std::string &output_path) const {
  auto out_stream = std::ofstream{output_path};
  boost::archive::text_oarchive oa{out_stream};
  oa << *this;
}

void Graph::load(const std::string &input_path) {
  auto in_stream = std::ifstream{input_path};
  boost::archive::text_iarchive ia{in_stream};
  ia >> *this;
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

namespace boost {
namespace serialization {
template <class Archive>
void serialize(Archive &ar, graph::Edge &edge, const unsigned int) {
  ar &edge.src &edge.dest &edge.cost;
}

template <class Archive>
void serialize(Archive &ar, graph::Graph &graph, const unsigned int) {
  ar &graph.num_vertices &graph.adjacency_list;
}
} /* namespace serialization */
} /* namespace boost */
