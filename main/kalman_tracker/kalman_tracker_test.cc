#include "main/kalman_tracker/kalman_tracker.h"

#include "gtest/gtest.h"

namespace Tracker {

class KalmanVelocityTrackerTest : public ::testing::Test {};
class LinearAssignmentTest : public ::testing::Test {};

TEST_F(KalmanVelocityTrackerTest, VelocityEmptyInitializationSuccess) { Tracker::KalmanVelocityTracker tracker(); }

TEST_F(KalmanVelocityTrackerTest, VelocityStateInitializationSuccess) {
  Tracker::StateVector init_bbox(4, 1.0);
  Tracker::KalmanVelocityTracker tracker(init_bbox);

  auto state_bbox = tracker.get_state_bbox();

  EXPECT_FLOAT_EQ(state_bbox.at(0), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(1), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(2), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(3), 1);
}

TEST_F(LinearAssignmentTest, LinearAssignmentGraphInitializationSuccess) {
  const int num_nodes = 2;
  const int num_arcs = 2;
  Tracker::Graph graph(num_nodes, num_arcs);
}

}  // namespace Tracker
