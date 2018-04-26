#pragma once

#include "locks.hpp"
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace util {

// Type aliases
using label_t = size_t;
using range_t = std::pair<size_t, size_t>;

/**
  Represents a point in n-space and supports useful operations.
*/
struct Point final {
  // Create a zero-vector of length dim
  explicit Point(size_t dim) : _point(std::vector<double>(dim, 0.0)) {}

  // Create a vector with the values in point
  explicit Point(const std::vector<double> &point) : _point(point) {}

  // Create a vector with random values in [0, 1)
  static Point random(size_t dim);

  // Convenience vector operations
  double dist(const Point &other) const;
  Point &operator+=(const Point &other);
  Point &operator/=(size_t divisor);
  friend std::ostream &operator<<(std::ostream &os, const Point &obj);
  friend bool operator==(const Point &p1, const Point &p2);
  size_t dim() const { return _point.size(); }

  // Data
  std::vector<double> _point;
};

std::ostream &operator<<(std::ostream &os, const Point &obj);
inline bool operator==(const Point &p1, const Point &p2) {
  return p1._point == p2._point;
}

/**
  Generates k random centroids with values in [0, 1).

  @param num_features - dimensionality of each centroid
  @param k - number of centroids
  @return list of centroids (std::vector<Point>)
*/
std::vector<Point> random_centroids(int k, size_t num_features);

/**
  Check how much centroids and old_centroids differ.
*/
std::pair<bool, double> converged(const std::vector<Point> &centroids,
                                  const std::vector<Point> &old_centroids,
                                  double threshold);

/**
  Helper to divide the data range among the threads and handle uneven partition.
  Last thread gets the remaining work.
  Each range has an inclusive `start` and exclusive `end`: [start, end)
*/
std::vector<range_t> partition(size_t size, int num_threads);

// Synchronization strategy
enum class SyncType { COARSE, FINE, NONE };

/**
  This class represents a collection of points and operations on them
  relating to centroids.
*/
class DataSet final {
public:
  explicit DataSet(const std::vector<Point> &points, locks::LockType lockType)
      : _points(points), _num_features(points[0].dim()), _mutex(), _spinlock(),
        _type(lockType) {}

  /**
    Factory that creates a DataSet from a file following the generate.py format.

    @param filename - path to file of correct format
    @return new DataSet instance
  */
  static std::shared_ptr<const DataSet>
  make_dataset(const std::string &filename,
               locks::LockType type = locks::LockType::NONE);

  inline range_t max_range() const { return range_t{0, _points.size()}; }
  inline size_t num_features() const { return _num_features; }
  inline size_t num_points() const { return _points.size(); }
  std::vector<double> make_raw() const;

  /**
    Set an index for each data point indicating closest centroid.

    @param centroids - list of centroids
    @param labels - list of indices into centroids indicating closest one to
    each data point
    @param range - which data points to process
  */
  void nearest_centroids(const std::vector<Point> &centroids,
                         std::vector<label_t> &labels, range_t range) const;

  /**
    Accumulate matching data points to corresponding centroids.
  */
  void sum_labeled_centroids(std::vector<Point> &centroids,
                             const std::vector<label_t> &labels,
                             std::vector<size_t> &counts, range_t range,
                             SyncType type) const;

  /**
    Divide centroids by counts of contributing points, handle empty cluster.
  */
  void normalize_centroids(std::vector<Point> &centroids,
                           std::vector<label_t> &counts, range_t k_range) const;

private:
  void lock() const;
  void unlock() const;

private:
  std::vector<Point> _points;
  size_t _num_features;
  mutable locks::Mutex _mutex;
  mutable locks::Spinlock _spinlock;
  locks::LockType _type;
};
} /* namespace util */
