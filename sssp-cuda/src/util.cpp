#include "util.h"

#include "pthread.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>

using std::vector;

namespace util {

vector<Point> random_centroids(int k, size_t num_features) {
  auto centroids = vector<Point>{};
  for (int i = 0; i < k; i++) {
    centroids.push_back(Point::random(num_features));
  }
  return centroids;
}

Point Point::random(size_t dim) {
  auto point = vector<double>(dim);
  std::generate(point.begin(), point.end(),
                []() { return static_cast<double>(std::rand()) / RAND_MAX; });
  return Point{point};
}

double Point::dist(const Point &other) const {
  if (this->dim() != other.dim()) {
    throw std::invalid_argument(
        "Point::dist(): two points must have same dimensionality!");
  }
  double result = 0.0f;
  for (size_t i = 0; i < this->dim(); i++) {
    result += std::pow(this->_point[i] - other._point[i], 2);
  }
  return std::sqrt(result);
}

Point &Point::operator+=(const Point &other) {
  if (this->dim() != other.dim()) {
    throw std::invalid_argument(
        "Point::operator+=(): two points must have same dimensionality!");
  }
  for (size_t i = 0; i < this->dim(); i++) {
    this->_point[i] += other._point[i];
  }
  return *this;
}

Point &Point::operator/=(size_t divisor) {
  if (divisor == 0) {
    throw std::invalid_argument("Point::operator/=(): divide by zero error!");
  }
  for (auto &coord : this->_point) {
    coord /= divisor;
  }
  return *this;
}

std::ostream &operator<<(std::ostream &os, const util::Point &obj) {
  os << "<";
  for (size_t i = 0; i < obj._point.size(); i++) {
    if (i > 0) {
      os << ", ";
    }
    os << obj._point[i];
  }
  os << ">";
  return os;
}

std::pair<bool, double> converged(const vector<Point> &centroids,
                                  const vector<Point> &old_centroids,
                                  double threshold) {
  // Check if the most movement for a given centroid is less than threshold
  auto delta_dists = vector<double>{};
  std::transform(centroids.begin(), centroids.end(), old_centroids.begin(),
                 std::back_inserter(delta_dists),
                 [](const auto &point1, const auto &point2) {
                   return point1.dist(point2);
                 });
  auto max_change = *std::max_element(delta_dists.begin(), delta_dists.end());
  return {max_change <= threshold, max_change};
}

vector<range_t> partition(size_t size, int num_threads) {
  auto ranges = vector<range_t>{};

  auto step_size = size / num_threads;
  int get_extra = size % num_threads;

  auto start = 0;
  auto end = step_size;

  for (int i = 0; i < num_threads; i++) {
    // Some threads are assigned additional work beyond minimum
    if (i < get_extra) {
      end++;
    } else if (i == num_threads - 1) {
      end = size;
    }
    ranges.push_back(range_t{start, end});

    // Take a "step" forward
    start = end;
    end = start + step_size;
  }
  return ranges;
}

void DataSet::nearest_centroids(const vector<Point> &centroids,
                                vector<label_t> &labels, range_t range) const {
  // No synchronization necessary, disjoint read-only intervals of centroids
  for (size_t i = range.first; i < range.second; i++) {
    // Calculate distance from current point to each centroid
    auto dists = vector<double>{};
    std::transform(
        centroids.begin(), centroids.end(), std::back_inserter(dists),
        [this, i](const auto &centroid) { return _points[i].dist(centroid); });

    // Store index of closest centroid
    labels[i] = std::distance(dists.begin(),
                              std::min_element(dists.begin(), dists.end()));
  }
}

void DataSet::sum_labeled_centroids(vector<Point> &centroids,
                                    const vector<label_t> &labels,
                                    vector<size_t> &counts, range_t range,
                                    SyncType type) const {
  if (type == SyncType::FINE) {
    // Increment local variables first, then bulk update shared state
    size_t k = centroids.size();
    auto local_centroids = vector<Point>{k, Point{num_features()}};
    auto local_counts = vector<size_t>(k);

    for (size_t i = range.first; i < range.second; i++) {
      auto label = labels[i];
      local_centroids[label] += this->_points[i];
      local_counts[label]++;
    }

    for (size_t i = 0; i < k; i++) {
      this->lock();
      if (counts[i] == 0) {
        centroids[i] = local_centroids[i];
      } else {
        centroids[i] += local_centroids[i];
      }
      counts[i] += local_counts[i];
      this->unlock();
    }
  } else {
    // Add each point to corresponding centroid
    for (size_t i = range.first; i < range.second; i++) {
      auto label = labels[i];
      this->lock();
      if (counts[label] == 0) {
        centroids[label] = this->_points[i];
      } else {
        centroids[label] += this->_points[i];
      }
      counts[label]++;
      this->unlock();
    }
  }
}

void DataSet::normalize_centroids(vector<Point> &centroids,
                                  vector<label_t> &counts,
                                  range_t k_range) const {
  // Divide by number of points and handle case where no points are assigned
  for (size_t i = k_range.first; i < k_range.second; i++) {
    if (counts[i] > 0) {
      centroids[i] /= counts[i];
    } else {
      // Assign a random point to this centroid
      auto index = std::rand() % this->_points.size();
      centroids[i] = this->_points[index];
    }
    // Always reset count
    counts[i] = 0;
  }
}

void DataSet::lock() const {
  if (_type == locks::LockType::MUTEX) {
    _mutex.lock();
  } else if (_type == locks::LockType::SPIN) {
    _spinlock.lock();
  } else {
    return;
  }
}

void DataSet::unlock() const {
  if (_type == locks::LockType::MUTEX) {
    _mutex.unlock();
  } else if (_type == locks::LockType::SPIN) {
    _spinlock.unlock();
  } else {
    return;
  }
}

vector<double> DataSet::make_raw() const {
  auto raw_output = vector<double>{};
  for (const auto &point : _points) {
    raw_output.insert(raw_output.end(), point._point.begin(),
                      point._point.end());
  }
  return raw_output;
}

std::shared_ptr<const DataSet>
DataSet::make_dataset(const std::string &filename, locks::LockType lockType) {
  auto file = std::ifstream{filename};
  auto line = std::string{};
  auto points = vector<Point>{};
  size_t num_dims = 0;

  // Get number of points
  std::getline(file, line);
  int num_points = std::stoi(line);
  points.reserve(num_points);
  if (num_points < 1) {
    throw std::invalid_argument("File must have at least one point!");
  }

  // Read each line in as a n-dim point
  for (int i = 0; i < num_points; i++) {
    std::getline(file, line);
    auto stream = std::stringstream{line};
    auto point = vector<double>{};

    // Discard point index (line number)
    int index;
    stream >> index;

    double coord;
    while (stream >> coord) {
      point.push_back(coord);
    }
    if (num_dims == 0) {
      num_dims = point.size();
    } else if (point.size() != num_dims) {
      throw std::invalid_argument(
          "Points must all have the same number of dimensions!");
    }
    points.push_back(Point{point});
  }

  return std::make_shared<const DataSet>(points, lockType);
}
} /* namespace util */
