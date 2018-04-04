#include "graph.hpp"
#include <stdio.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <cstdlib>
#include <iostream>

Edge::Edge(size_t src, size_t dest, int cost)
    : src(src), dest(dest), cost(cost) {}

Graph::Graph(size_t num_vertices)
    : num_vertices(num_vertices), adjacency_list(num_vertices) {}

void Graph::addEdge(size_t src, size_t dest, int cost) {
  Edge e1 = Edge(src, dest, cost);
  Edge e2 = Edge(dest, src, cost);
  adjacency_list[src].push_back(e1);
  adjacency_list[dest].push_back(e2);
}

std::vector<int> Graph::getNeighbors(size_t src) {
  int num_neighbors = adjacency_list[src].size();
  std::vector<int> res(num_neighbors);
  for(int i = 0; i < num_neighbors; i++) {
    res[i] = adjacency_list[src][i].dest;
  }
  return res;
}

std::string Graph::toString() const {
  std::string str;
  str += std::to_string(num_vertices) + "\n";
  for (size_t vertex = 0; vertex < num_vertices; vertex++) {
    const auto& edges = adjacency_list[vertex];
    for (const Edge& e : edges) {
      str += std::to_string(e.src) + " " + std::to_string(e.dest) + " " +
             std::to_string(e.cost) + "\n";
    }
  }
  return str;
}

Graph Graph::generateGraph(size_t num_vertices, size_t num_edges, int seed) {
  std::srand(seed);
  Graph graph(num_vertices);

  if (num_edges < num_vertices - 1) {
    std::cerr << "Graph generation failed: number of vertices (" << num_vertices
              << ") was less than or equal to number of edges" << num_edges
              << "." << std::endl;
    return graph;
  }

  for (size_t i = 1; i < num_vertices; i++) {
    size_t src = std::rand() % i;
    int cost = 1 + (std::rand() % MAX_COST);
    graph.addEdge(src, i, cost);
  }

  for (size_t i = num_vertices - 1; i < num_edges; i++) {
    size_t src, dest;
    do {
      src = std::rand() % num_vertices;
      dest = std::rand() % num_vertices;
    } while (src == dest);
    int cost = 1 + (std::rand() % MAX_COST);
    graph.addEdge(src, dest, cost);
  }

  return graph;
}

