#include "graph.hpp"

#include "gtest/gtest.h"
#include <stdexcept>

using graph::Edge;
using graph::Graph;

TEST(GraphTest, AddEdge) {
  auto graph = Graph{3};
  graph.addEdge(0, 1, 3);
  EXPECT_EQ(graph.cost(0, 1), 3);
  EXPECT_EQ(graph.cost(1, 0), 3);
}

TEST(GraphTest, OutOfRange) {
  auto graph = Graph{0};
  ASSERT_THROW(graph.cost(0, 1), std::invalid_argument);
}

TEST(GraphTest, CostInvalidArg) {
  auto graph = Graph{3};
  graph.addEdge(0, 2, 3);
  ASSERT_THROW(graph.cost(1, 2), std::invalid_argument);
}

TEST(GraphTest, GetNeighbors) {}
