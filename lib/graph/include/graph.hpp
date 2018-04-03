#pragma once
#include <cstddef>
#include <string>
#include <vector>

#define MAX_COST 100

struct Edge {
  size_t src;
  size_t dest;
  int cost;

  Edge(){};
  Edge(size_t src, size_t dest, int cost);
};

class Graph {
public:
  size_t num_vertices;
  std::vector<Edge> edges;

  Graph(size_t num_vertices);
  void addEdge(size_t src, size_t dest, int cost);
  std::string toString() const;

  static Graph generateGraph(size_t num_vertices, size_t num_edges, int seed);
};
