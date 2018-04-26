#include "bellman_ford.h"
#include "docopt.h"
#include "graph.hpp"
#include <chrono>
#include <iostream>
#include <map>

static const char USAGE[] =
    R"(Concurrency Final Project.

  Run GPU accelerated single-source shortest paths using Bellman-Ford.

  Usage:
    sssp_cpu_only --input=<input> --source=<source>

  Options:
    -h --help           Show this screen.
    --input=<input>     String-valued path to an input graph text file.
    --source=<source>   Integer-valued source id, must be less than number of vertices. [default: 0]
)";

using graph::Graph;

void printArgs(const std::map<std::string, docopt::value> &args) {
  std::cout << "{" << std::endl;
  for (const auto &arg : args) {
    std::cout << "  " << arg.first << ": " << arg.second << std::endl;
  }
  std::cout << "}" << std::endl;
}

void printOutput(const std::vector<double> &paths,
                 const float duration) {
  std::cout << "Shortest paths to each vertex: " << std::endl;
  for (size_t i = 0; i < paths.size(); i++) {
    std::cout << "  " << i << ": " << paths.at(i) << std::endl;
  }

  std::cout
      << "Elapsed runtime: "
      << duration
      << "ms" << std::endl;
}

int main(int argc, const char *argv[]) {
  auto args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true);
  printArgs(args);

  auto graph = Graph{args.at("--input").asString()};
  auto source = args.at("--source").asLong();

  auto output = bellmanFordCUDA(graph, source);
  printOutput(output.distances, output.elapsed);

  return 0;
}
