#include "util.hpp"
#include "graph.hpp"
#include "bellman_ford.hpp"

#include "gtest/gtest.h"

using graph::Graph;
using sssp::bellmanFord;
using sssp::bellmanFordParallel;

class BellmanFordTest : public ::testing::Test {
  protected:
    BellmanFordTest() : the_graph(4) {}
    virtual void SetUp() {
      the_graph.addEdge(0, 1, 10);
      the_graph.addEdge(1, 2, 20);
      the_graph.addEdge(2, 3, 30);
      the_graph.addEdge(0, 2, 1);
      the_graph.addEdge(1, 3, 1);
    }
  Graph the_graph;
};

TEST_F(BellmanFordTest, Sequential) {
  auto dists0 = bellmanFord(the_graph, 0);
  EXPECT_EQ(0, dists0.at(0));  // self distance should be 0
  EXPECT_EQ(10, dists0.at(1));
  EXPECT_EQ(1, dists0.at(2));
  EXPECT_EQ(11, dists0.at(3));

  auto dists3 = bellmanFord(the_graph, 3);
  EXPECT_EQ(11, dists3.at(0));
  EXPECT_EQ(1, dists3.at(1));
  EXPECT_EQ(12, dists3.at(2));
  EXPECT_EQ(0, dists3.at(3));
}

TEST_F(BellmanFordTest, Parallel) {
  auto dists0 = bellmanFordParallel(the_graph, 0, 4);
  EXPECT_EQ(0, dists0.at(0));  // self distance should be 0
  EXPECT_EQ(10, dists0.at(1));
  EXPECT_EQ(1, dists0.at(2));
  EXPECT_EQ(11, dists0.at(3));

  auto dists3 = bellmanFordParallel(the_graph, 3, 4);
  EXPECT_EQ(11, dists3.at(0));
  EXPECT_EQ(1, dists3.at(1));
  EXPECT_EQ(12, dists3.at(2));
  EXPECT_EQ(0, dists3.at(3));
}
