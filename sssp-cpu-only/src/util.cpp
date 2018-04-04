#include "util.hpp"

using std::vector;

namespace util {

vector<Range> partition(size_t size, size_t num_threads) {
  auto ranges = vector<Range>{};

  auto step_size = size / num_threads;
  auto get_extra = size % num_threads;

  size_t start = 0;
  size_t end = step_size;

  // Calculate the range for each thread
  for (size_t i = 0; i < num_threads; i++) {
    // Some threads are assigned additional work beyond minimum
    if (i < get_extra) {
      end++;
    } else if (i == num_threads - 1) {
      end = size;
    }
    ranges.push_back({start, end});

    // Advance forward
    start = end;
    end = start + step_size;
  }

  return ranges;
}

void Barrier::wait() {
  std::unique_lock<std::mutex> lock{mutex_};
  auto current_gen = generation_;
  if (--count_ == 0) {
    generation_++;
    count_ = threshold_;
    cond_.notify_all();
  } else {
    cond_.wait(lock,
               [this, current_gen]() { return current_gen != generation_; });
  }
}

} /* namespace util */
