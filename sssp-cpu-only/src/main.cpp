#include "bellman_ford.hpp"
#include "docopt.h"
#include "graph.hpp"
#include <iostream>
#include <map>

static const char USAGE[] =
    R"(Run sequential or multithreaded single-source shortest paths using Bellman-Ford.
    )";

using graph::Graph;

void printArgs(const std::map<std::string, docopt::value> &args) {
  std::cout << "{" << std::endl;
  for (const auto &arg : args) {
    std::cout << "  " << arg.first << ": " << arg.second << std::endl;
  }
  std::cout << "}" << std::endl;
}

int main(int argc, const char *argv[]) {
  auto args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true);
  printArgs(args);

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
  return 0;
}
