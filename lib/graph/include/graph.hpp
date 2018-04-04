#pragma once

#include <cstddef>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace graph {

constexpr size_t kMaxCost = 100;

struct Edge {
  size_t src;
  size_t dest;
  size_t cost;
};

class Graph {
public:
  size_t num_vertices;
  std::vector<std::vector<Edge>> adjacency_list;
  mutable std::vector<Edge> all_edges;

  Graph(size_t num_vertices);
  void addEdge(size_t src, size_t dest, size_t cost);
  const std::vector<Edge>& getEdges(size_t src) const;
  const std::vector<Edge>& getAllEdges() const;
  std::vector<size_t> getNeighbors(size_t src) const;
  inline std::vector<Edge> getEdges(size_t src) const {
    return adjacency_list.at(src);
  }
  size_t cost(size_t src, size_t dest) const;
  std::string toString() const;

  void save(const std::string &output_path) const;
  void load(const std::string &input_path);

  static Graph generateGraph(size_t num_vertices, size_t num_edges, int seed);
};

class GraphPriorityQueue {
public:
  GraphPriorityQueue(size_t num_vertices)
      : priorities_(num_vertices), queue_() {}
  inline void add(size_t id, double priority) {
    priorities_[id] = priority;
    queue_.push({id, priorities_.at(id)});
  }
  inline bool empty() const { return queue_.empty(); }
  inline size_t remove_min() {
    auto node = queue_.top();
    queue_.pop();
    return node.id;
  }
  inline void decrease_priority(size_t id, double priority) {
    priorities_[id] = std::min(priorities_.at(id), priority);
  }

private:
  struct Node {
    size_t id;
    double *priority;
    Node(size_t id, double &priority) : id(id), priority(&priority) {}
    bool operator<(const Node &rhs) const { return *priority < *rhs.priority; }
  };

  std::vector<double> priorities_;
  std::priority_queue<Node> queue_;
};

} /* namespace graph */
