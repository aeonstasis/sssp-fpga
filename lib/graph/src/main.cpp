#include "graph.hpp"
#include "docopt.h"
#include <iostream>

static const char USAGE[] = R"(
  Concurrency Final Project.

  Graph generator utility that writes graph binaries.

  Usage:
    graph_gen --vertices=<vertices> --edges=<edges> --output=<path>

  Options:
    --vertices=<vertices>   Number of vertices
    --edges=<edges>         Number of edges
    --output=<path>         Path to output file
)";

using graph::Graph;

int main(int argc, const char *argv[]) {
  auto args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true);
  auto graph = Graph::generateGraph(
      args.at("--vertices").asLong(),
      args.at("--edges").asLong());
  graph.saveBinary(args.at("--output").asString());
  std::cout << "Graph written successfully." << std::endl;
}
