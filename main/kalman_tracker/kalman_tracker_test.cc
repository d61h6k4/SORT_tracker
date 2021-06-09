#include "main/kalman_tracker/kalman_tracker.h"
#include "gtest/gtest.h"

namespace Tracker {

class SortTrackerTest : public ::testing::Test {};
class KalmanVelocityTrackerTest : public ::testing::Test {};
class LinearAssignmentTest : public ::testing::Test {};

TEST_F(KalmanVelocityTrackerTest, VelocityEmptyInitializationSuccess) {
  Tracker::KalmanVelocityTracker tracker();
}

//TEST_F(KalmanAccelerationTrackerTest, AccelerationEmptyInitializationSuccess) {
//  Tracker::KalmanAccelerationTracker<10, 4> tracker();
//}

TEST_F(KalmanVelocityTrackerTest, VelocityStateInitializationSuccess) {
  Tracker::StateVector init_state(4, 1.0);
  Tracker::KalmanVelocityTracker tracker(init_state);
}

//TEST_F(KalmanAccelerationTrackerTest, AccelerationStateInitializationSuccess) {
//  Tracker::StateVector init_state(4, 1.0);
//  Tracker::KalmanAccelerationTracker<10, 4> tracker(init_state);
//}

TEST_F(SortTrackerTest, StateToBboxConversionSuccess) {
  Tracker::StateVector state(4);

    state.at(0) = 10;
    state.at(1) = 10;
    state.at(2) = 50;
    state.at(3) = 2;

  Tracker::StateVector bbox = Tracker::get_bbox_from_state(state);

  EXPECT_FLOAT_EQ(bbox.at(0), 5.0);
  EXPECT_FLOAT_EQ(bbox.at(1), 7.5);
  EXPECT_FLOAT_EQ(bbox.at(2), 10);
  EXPECT_FLOAT_EQ(bbox.at(3), 5);
}

TEST_F(SortTrackerTest, BboxToStateConversionSuccess) {
  Tracker::StateVector bbox(4);

  bbox.at(0) = 0;
  bbox.at(1) = 0;
  bbox.at(2) = 40;
  bbox.at(3) = 20;

  Tracker::StateVector state = Tracker::get_state_from_bbox(bbox);

  EXPECT_FLOAT_EQ(state.at(0), 20);
  EXPECT_FLOAT_EQ(state.at(1), 10);
  EXPECT_FLOAT_EQ(state.at(2), 800);
  EXPECT_FLOAT_EQ(state.at(3), 2);
}

TEST_F(LinearAssignmentTest, LinearAssignmentGraphInitializationSuccess) {
    const int num_nodes = 2;
    const int num_arcs = 2;
    Tracker::Graph graph(num_nodes, num_arcs);
}

}  // namespace Tracker

