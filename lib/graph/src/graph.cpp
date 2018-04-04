#include "graph.hpp"

#include <algorithm>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <stdio.h>

using std::string;
using std::vector;

namespace graph {

constexpr double kEdgeWeightMin = -100.0;
constexpr double kEdgeWeightMax = 100.0;

void checkBounds(const vector<size_t> &input, size_t max) {
  if (std::any_of(input.begin(), input.end(),
                  [max](const auto &val) { return val >= max; })) {
    throw std::invalid_argument("Specified index out of range");
  }
}

Graph::Graph(size_t num_vertices)
    : num_vertices(num_vertices), adjacency_list(num_vertices), all_edges() {}

Graph::Graph(const std::string &filename)
    : num_vertices(), adjacency_list(), all_edges() {
  auto file = std::ifstream{filename};
  auto line = std::string{};

  // Read number of vertices
  std::getline(file, line);
  num_vertices = static_cast<size_t>(std::stoi(line));
  adjacency_list.resize(num_vertices);

  // Read in each edge
  size_t src;
  size_t dest;
  double cost;
  for (; std::getline(file, line);) {
    auto stream = std::stringstream{line};
    stream >> src >> dest >> cost;
    addEdge(src, dest, cost);
  }
}

void Graph::addEdge(size_t src, size_t dest, double cost) {
  checkBounds({src, dest}, num_vertices);
  adjacency_list[src].push_back({src, dest, cost});
  adjacency_list[dest].push_back({dest, src, cost});
  all_edges.push_back({src, dest, cost});
  all_edges.push_back({dest, src, cost});
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

double Graph::cost(size_t src, size_t dest) const {
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

void Graph::saveBinary(const std::string &output_path) const {
  auto out_stream = std::ofstream{output_path};
  boost::archive::binary_oarchive oa{out_stream};
  oa << *this;
}

void Graph::loadBinary(const std::string &input_path) {
  auto in_stream = std::ifstream{input_path};
  boost::archive::binary_iarchive ia{in_stream};
  ia >> *this;
}

Graph Graph::generateGraph(size_t num_vertices, size_t num_edges) {
  std::random_device seed;
  auto engine = std::default_random_engine{seed()};
  auto weight_dist =
      std::uniform_real_distribution<double>{kEdgeWeightMin, kEdgeWeightMax};
  auto vertex_dist = std::uniform_int_distribution<size_t>{0, num_vertices - 1};

  auto graph = Graph{num_vertices};
  if (num_edges < num_vertices - 1) {
    auto buffer = std::stringstream{};
    buffer << "Graph generation failed: number of vertices (" << num_vertices
           << ") was less than or equal to number of edges" << num_edges << "."
           << std::endl;
    throw std::invalid_argument(buffer.str());
  }

  size_t src, dest;
  for (size_t i = 0; i < num_edges; i++) {
    do {
      src = vertex_dist(engine);
      dest = vertex_dist(engine);
    } while (src == dest);
    double cost = weight_dist(engine);
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
