#pragma once

#include <condition_variable>
#include <mutex>
#include <vector>

namespace util {

// Used to delineate the boundaries for a thread's work when partitioning a
// dataset.
struct Range {
  size_t start;
  size_t end;
};

// Splits a integral range as evenly as possible among some number of threads,
// returning the indices each thread should work with.
std::vector<Range> partition(size_t size, size_t num_threads);

// Helper synchronization class since std::barrier is still experimental.
class Barrier {
public:
  explicit Barrier(size_t count)
      : threshold_(count), count_(count), generation_(0) {}
  void wait();

private:
  std::mutex mutex_;
  std::condition_variable cond_;
  size_t threshold_;
  size_t count_;
  size_t generation_;
};

} /* namespace util */
