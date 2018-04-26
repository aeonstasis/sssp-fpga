#include "graph.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>

using std::string;
using std::vector;

namespace graph {

constexpr double kEdgeWeightMin = 0.0;
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
    addSingleEdge(src, dest, cost);
  }
}

void Graph::addEdge(size_t src, size_t dest, double cost) {
  checkBounds({src, dest}, num_vertices);
  adjacency_list[src].push_back({src, dest, cost});
  adjacency_list[dest].push_back({dest, src, cost});
  all_edges.push_back({src, dest, cost});
  all_edges.push_back({dest, src, cost});
}

void Graph::addSingleEdge(size_t src, size_t dest, double cost) {
  checkBounds({src, dest}, num_vertices);
  adjacency_list[src].push_back({src, dest, cost});
  all_edges.push_back({src, dest, cost});
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

void Graph::saveToFile(const std::string &output_path) const {
  auto out_stream = std::ofstream{output_path};
  out_stream << toString();
}

std::vector<string> split(string str, char sep) {
  std::istringstream ss(str);
  std::vector<string> tokens;
  while(ss) {
    string s;
    if(!std::getline(ss, s, sep)) break;
    tokens.push_back(s);
  }
  return tokens;
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
