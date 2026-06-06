#include <gtest/gtest.h>
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "nav2_teb_controller/g2o_types/edge_time_optimal.h"

using namespace nav2_teb_controller;

// Test that default construction sets measurement to 0
TEST(EdgeTimeOptimal, DefaultConstructor)
{
  EdgeTimeOptimal edge;
  EXPECT_DOUBLE_EQ(edge.measurement(), 0.0);
}

// Test that computeError() returns the dt value from the vertex
TEST(EdgeTimeOptimal, ComputeErrorReturnsDt)
{
  EdgeTimeOptimal edge;
  VertexTimeDiff* v = new VertexTimeDiff();
  v->setEstimate(0.5);  // set dt = 0.5s

  edge.setVertex(0, v);
  edge.computeError();

  EXPECT_DOUBLE_EQ(edge.error()[0], 0.5);

  delete v;
}

// Test that a zero dt gives zero error
TEST(EdgeTimeOptimal, ZeroDtGivesZeroError)
{
  EdgeTimeOptimal edge;
  VertexTimeDiff* v = new VertexTimeDiff();
  v->setEstimate(0.0);

  edge.setVertex(0, v);
  edge.computeError();

  EXPECT_DOUBLE_EQ(edge.error()[0], 0.0);

  delete v;
}

// Test negative dt (should still propagate without crash)
TEST(EdgeTimeOptimal, NegativeDtPropagates)
{
  EdgeTimeOptimal edge;
  VertexTimeDiff* v = new VertexTimeDiff();
  v->setEstimate(-0.1);

  edge.setVertex(0, v);
  edge.computeError();

  EXPECT_DOUBLE_EQ(edge.error()[0], -0.1);

  delete v;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}