/**
  CS 378 Concurrency Lab 4
  main.cpp
  Purpose: Entry point for running kmeans.

  @author Aaron Zou
  @version 1.0 02/13/18
*/

#include "docopt.h"
#include "kmeans.h"
#include "util.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <pthread.h>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;
using std::shared_ptr;
using util::DataSet;
using util::Point;
using util::label_t;
using util::range_t;
using locks::LockType;
using util::SyncType;

constexpr char USAGE[] =
    R"(Overview: K-Means (CS 378 Concurrency Lab 2).

  This program illustrates ways of speeding up kmeans through threading and
  the use of barriers and other synchronization primitives.

  Usage:
    kmeans cpu --input=<input> --clusters=<clusters> [--iterations=<iters>]
      [--threshold=<threshold>]
    kmeans cpu (--coarse | --fine) --input=<input> --clusters=<clusters>
      --workers=<workers> (--mutex | --spin) [--iterations=<iters>]
      [--threshold=<threshold>]
    kmeans cuda --input=<input> --clusters=<clusters> [--privatize | --extra]
      [--iterations=<iters>] [--threshold=<threshold>]
    kmeans (-h | --help)

  Options:
    -h --help                  Show this screen.
    --input=<input>            String-valued path to an input file.
    --iterations=<iters>       Integer-valued maximum number of iterations. [default: 0]
    --threshold=<threshold>    Floating-point threshold for convergence. [default: 0.0000001]
    --workers=<workers>        Integer-valued number of threads.
    --clusters=<clusters>      Integer number of clusters to find.
    --coarse                   Coarse synchronization.
    --fine                     Privatized updates per thread.
    --mutex                    Use a mutex to lock access to shared state.
    --spin                     Use a spinlock to lock access to shared state.
    --privatize                Set to true to have private updates.
    --extra                    Set to true to enable additional optimizations.
)";

// Global variables
size_t iters = 0;
size_t MAX_ITERS = 0;
double THRESHOLD = 0.0;
pthread_barrier_t barrier;
util::SyncType syncType = util::SyncType::NONE;

// 80 bytes
struct ThreadArg {
  const int tid;         // thread id (assumed strictly increasing)
  const range_t range;   // indices into the dataset this thread handles
  const range_t k_range; // range used for partioning operating on centroids
  const shared_ptr<const DataSet> data; // dataset being referenced
  vector<label_t> &labels;              // labels being operated on
  vector<Point> &centroids;             // centroids being operated on
  vector<size_t> &counts;               // count summations for each centroid
};

/**
  Parallel work performed by a single thread.
*/
void *workerThread(void *_arg) {
  const auto &args = *static_cast<ThreadArg *>(_arg);
  auto old_centroids = vector<Point>{};
  bool converged = false;

  do {
    // Ensure all threads are done checking termination condition before
    // nearest_centroids() is called
    pthread_barrier_wait(&barrier);
    old_centroids = args.centroids;
    if (args.tid == 0) {
      iters++;
    }

    args.data->nearest_centroids(args.centroids, args.labels, args.range);

    // Ensure threads are done reading from centroids
    pthread_barrier_wait(&barrier);
    args.data->sum_labeled_centroids(args.centroids, args.labels, args.counts,
                                     args.range, syncType);

    // Ensure threads are done adding points to centroids
    pthread_barrier_wait(&barrier);
    args.data->normalize_centroids(args.centroids, args.counts, args.k_range);

    // Ensure threads have completed work on centroids
    pthread_barrier_wait(&barrier);
    converged = util::converged(args.centroids, old_centroids, THRESHOLD).first;
  } while ((MAX_ITERS == 0 || iters < MAX_ITERS) && !converged);
  pthread_exit(nullptr);
}

/**
  Multi-threaded kmeans implementation.

  @param data - list of points in dataset
  @param k - number of clusters to find
  @param num_workers - number of threads to use
  @return KmeansOutput object
*/
vector<Point> kmeans_multi(shared_ptr<const DataSet> data, size_t k,
                           int num_workers) {
  const size_t num_features = data->num_features();
  auto centroids = util::random_centroids(k, num_features);
  auto counts = vector<size_t>(k, 0);
  auto labels = vector<label_t>(data->num_points(), 0);
  auto partition = util::partition(data->num_points(), num_workers);
  auto k_partition = util::partition(k, num_workers);

  // Create threads and arguments to pass
  auto threads = vector<pthread_t>(static_cast<size_t>(num_workers));
  auto args = vector<ThreadArg>{};
  for (int i = 0; i < num_workers; i++) {
    args.push_back(ThreadArg{i, partition[i], k_partition[i], data, labels,
                             centroids, counts});
  }

  // Start threads then join with them
  for (int i = 0; i < num_workers; i++) {
    int rc = pthread_create(&threads.data()[i], nullptr, workerThread,
                            static_cast<void *>(&(args.data()[i])));
    if (rc != 0) {
      std::cout << "pthread_create failed: " << std::strerror(rc) << std::endl;
    }
  }
  for (int i = 0; i < num_workers; i++) {
    int rc = pthread_join(threads[i], nullptr);
    if (rc != 0) {
      std::cout << "pthread_join failed: " << std::strerror(rc) << std::endl;
    }
  }

  return centroids;
}

/**
  Single-threaded kmeans implementation.

  @param data - list of points in dataset
  @param k - number of clusters to find
  @return { num_iters - number of iterations to converge, centroids - detected
  clusters}
*/
vector<Point> kmeans(shared_ptr<const DataSet> data, size_t k) {
  auto num_features = data->num_features();
  auto centroids = util::random_centroids(k, num_features);
  auto labels = vector<label_t>(data->num_points(), 0);
  auto counts = vector<size_t>(k, 0);
  auto old_centroids = vector<Point>{};
  bool converged = false;

  do {
    old_centroids = centroids;
    iters++;
    data->nearest_centroids(centroids, labels, data->max_range());
    data->sum_labeled_centroids(centroids, labels, counts, data->max_range(),
                                util::SyncType::NONE);
    data->normalize_centroids(centroids, counts, range_t{0, k});
    auto output = util::converged(centroids, old_centroids, THRESHOLD);
    converged = output.first;
  } while ((MAX_ITERS == 0 || iters < MAX_ITERS) && !converged);

  return centroids;
}

// Output used by graphing script
void printOutput(const vector<Point> &output,
                 const std::chrono::duration<double> &duration) {
  std::cout << "Converged in " << iters << " iterations (max=" << MAX_ITERS
            << ")" << std::endl;
  std::cout
      << "Parallel work completed in: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()
      << "ms" << std::endl;
  for (size_t i = 0; i < output.size(); i++) {
    std::cout << "Cluster " << i << " center: " << output[i] << std::endl;
  }
}

void printOutput(const vector<double> &raw, size_t k, size_t dim, size_t iter,
                 float elapsed) {
  std::cout << "Converged in " << iter << " iterations (max=" << MAX_ITERS
            << ")" << std::endl;
  std::cout << "Parallel work completed in: " << elapsed << "ms" << std::endl;
  size_t count = 0;
  for (size_t i = 0; i < k; i++) {
    auto point =
        vector<double>(raw.begin() + (i * dim), raw.begin() + ((i + 1) * dim));
    std::cout << "Cluster " << count++ << " center: " << Point(point)
              << std::endl;
  }
}

void cpu_kmeans(const std::map<std::string, docopt::value> &args) {
  size_t k = args.at("--clusters").asLong();
  if (!args.at(string("--workers"))) {
    auto data = DataSet::make_dataset(args.at("--input").asString());

    auto start = std::chrono::high_resolution_clock::now();
    auto output = kmeans(data, k);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;

    printOutput(output, elapsed);
  } else {
    // Multithreaded case
    auto num_workers = args.at("--workers").asLong();
    auto lType = args.at("--mutex").asBool() ? LockType::MUTEX : LockType::SPIN;
    syncType = args.at("--coarse").asBool() ? SyncType::COARSE : SyncType::FINE;
    auto data = DataSet::make_dataset(args.at("--input").asString(), lType);
    int rc = pthread_barrier_init(&barrier, nullptr, num_workers);
    if (rc != 0) {
      std::cerr << "pthread_barrier_init: " << std::strerror(rc) << std::endl;
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto output = kmeans_multi(data, k, num_workers);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;

    printOutput(output, elapsed);

    // Free resource
    rc = pthread_barrier_destroy(&barrier);
    if (rc != 0) {
      std::cerr << "pthread_barrier_destroy: " << std::strerror(rc)
                << std::endl;
    }
  }
}

int main(int argc, const char *argv[]) {
  // Process command-line arguments
  auto args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true);
  THRESHOLD = std::stod(args.at("--threshold").asString());
  MAX_ITERS = args.at("--iterations").asLong();
  std::srand(std::time(0));

  // Distinguish single- and multi-threaded cases
  auto data = DataSet::make_dataset(args.at("--input").asString());
  if (args.at("cpu").asBool()) {
    cpu_kmeans(args);
  } else {
    size_t k = args.at("--clusters").asLong();
    auto output = cuda_kmeans(
        data->make_raw(), data->num_features(), k, MAX_ITERS, THRESHOLD,
        args.at("--privatize").asBool(), args.at("--extra").asBool());
    printOutput(output.centroids, k, data->num_features(), output.iter,
                output.elapsed);
  }

  return 0;
}
