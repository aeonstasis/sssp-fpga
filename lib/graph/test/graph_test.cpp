#include "graph.hpp"

#include "gtest/gtest.h"
#include <cstdio>
#include <cstdlib>
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

TEST(GraphTest, GetNeighbors) {
  auto graph = Graph{3};
  graph.addEdge(0, 1, 2);
  graph.addEdge(0, 2, 3);
  auto neighbors = graph.getNeighbors(0);
  auto expected = std::vector<size_t>{{1, 2}};
  EXPECT_EQ(neighbors, expected);
}

TEST(GraphTest, GetNeighborsEmpty) {
  auto graph = Graph{3};
  auto neighbors = graph.getNeighbors(0);
  auto expected = std::vector<size_t>{};
  EXPECT_EQ(neighbors, expected);
}

TEST(GraphTest, SaveAndLoad) {
  auto graph = Graph{3};
  graph.addEdge(0, 1, 2);
  graph.addEdge(0, 2, 3);
  graph.addEdge(1, 2, 4);

  // TODO: properly use tmpfile() later
  auto filename = "/tmp/gtest_graph_test.bin";
  Graph new_graph{0};
  try {
    graph.saveToFile(filename);
    new_graph = Graph(filename);
  } catch (const std::exception &e) {
    std::remove(filename);
    FAIL() << "IO error encountered";
  }

  EXPECT_EQ(new_graph.num_vertices, graph.num_vertices);
  EXPECT_EQ(new_graph.cost(0, 1), 2);
  EXPECT_EQ(new_graph.cost(0, 2), 3);
  EXPECT_EQ(new_graph.cost(1, 2), 4);
  auto expected = std::vector<size_t>{{1, 2}};
  EXPECT_EQ(new_graph.getNumEdges(), graph.getNumEdges());
  EXPECT_EQ(new_graph.getNeighbors(0), expected);

  // std::remove(filename);
}
