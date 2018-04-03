#include <iostream>
#include <thread>
#include "graph.hpp"

int main(int argc, const char *argv[]) {
  auto dummy = Graph{};
  auto thread = std::thread{[]() { std::cout << "Hello world!" << std::endl; }};
  thread.join();
  return 0;
}
