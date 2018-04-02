#include <iostream>
#include <thread>

int main(int argc, const char *argv[]) {
  auto thread = std::thread{[]() { std::cout << "Hello world!" << std::endl; }};
  thread.join();
  return 0;
}
