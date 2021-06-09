#include "main/kalman_tracker/kalman_tracker.h"
#include "gtest/gtest.h"

namespace Tracker {

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


TEST_F(LinearAssignmentTest, LinearAssignmentGraphInitializationSuccess) {
    const int num_nodes = 2;
    const int num_arcs = 2;
    Tracker::Graph graph(num_nodes, num_arcs);
}

}  // namespace Tracker

