#pragma once

#include <cstddef>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace graph {

struct Edge {
  size_t src;
  size_t dest;
  double cost;
};

class Graph {
public:
  size_t num_vertices;
  std::vector<std::vector<Edge>> adjacency_list;

  Graph(size_t num_vertices);

  // Create a graph instance from a text file with the following format:
  //
  // num_vertices\n
  // src_id dest_id cost\n
  // ...
  // src_id dest_id cost\n
  Graph(const std::string &filename);
  void addEdge(size_t src, size_t dest, double cost);
  const std::vector<Edge> &getEdges(size_t src) const;
  const std::vector<Edge> &getAllEdges() const { return all_edges; }
  std::vector<size_t> getNeighbors(size_t src) const;
  double cost(size_t src, size_t dest) const;
  std::string toString() const;

  // Serialize a Graph instance to or from a file using a Boost binary archive
  void saveBinary(const std::string &output_path) const;
  void loadBinary(const std::string &input_path);

  static Graph generateGraph(size_t num_vertices, size_t num_edges);

private:
  std::vector<Edge> all_edges;
};
} /* namespace graph */
