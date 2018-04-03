#include "docopt.h"
#include "graph.hpp"
#include <iostream>
#include <thread>

int main(int argc, const char *argv[]) {
  const Graph graph = Graph::generateGraph(10, 20, 12345);
  auto thread =
      std::thread{[graph]() { std::cout << graph.toString() << std::endl; }};
  thread.join();
  return 0;
}
