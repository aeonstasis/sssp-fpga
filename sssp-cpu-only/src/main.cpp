#include "bellmanford.hpp"
#include "docopt.h"
#include "graph.hpp"
#include <iostream>
#include <thread>

using graph::Graph;

int main(int argc, const char *argv[]) {
  // const Graph graph = Graph::generateGraph(10, 20, 12345);
  Graph graph(4);
  graph.addEdge(0, 1, 10);
  graph.addEdge(1, 2, 10);
  graph.addEdge(2, 3, 10);
  graph.addEdge(0, 2, 1);
  graph.addEdge(1, 3, 1);

  size_t source = 0;
  auto bell_dist = sssp::bellmanFord(graph, source);
  for (size_t i = 0; i < bell_dist.size(); i++) {
    std::cout << source << " -> " << i << ": " << bell_dist.at(i) << std::endl;
  }

  auto thread =
      std::thread{[graph]() { std::cout << graph.toString() << std::endl; }};
  thread.join();
  return 0;
}
