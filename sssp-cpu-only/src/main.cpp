#include "bellman_ford.hpp"
#include "docopt.h"
#include "graph.hpp"
#include <chrono>
#include <iostream>
#include <map>

static const char USAGE[] =
    R"(Concurrency Final Project.

  Run sequential or multithreaded single-source shortest paths using Bellman-Ford.

  Usage:
    sssp_cpu_only --input=<input> --source=<source> [--workers=<workers>]

  Options:
    -h --help           Show this screen.
    --input=<input>     String-valued path to an input graph text file.
    --source=<source>   Integer-valued source id, must be less than number of vertices. [default: 0]
    --workers=<workers> Integer-valued number of threads.
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
                 const std::chrono::duration<double> &duration) {
  std::cout << "Shortest paths to each vertex: " << std::endl;
  for (size_t i = 0; i < paths.size(); i++) {
    std::cout << "  " << i << ": " << paths.at(i) << std::endl;
  }

  std::cout
      << "Elapsed runtime: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()
      << "ms" << std::endl;
}

int main(int argc, const char *argv[]) {
  auto args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true);
  auto num_workers = args.at("--workers");
  if (num_workers && num_workers.asLong() <= 0) {
    throw std::invalid_argument("Must specify at least one worker!");
  }
  printArgs(args);

  auto graph = Graph{args.at("--input").asString()};
  auto source = args.at("--source").asLong();

  auto start = std::chrono::high_resolution_clock::now();
  auto paths = (!num_workers)
                   ? sssp::bellmanFord(graph, source)
                   : sssp::bellmanFordParallel(graph, source, num_workers.asLong());
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  printOutput(paths, elapsed);

  return 0;
}
