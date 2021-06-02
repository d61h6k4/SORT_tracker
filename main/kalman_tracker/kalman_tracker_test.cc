#include "main/kalman_tracker/kalman_tracker.h"
#include "gtest/gtest.h"

namespace SortTracker {

class SortTrackerTest : public ::testing::Test {};
class LinearAssignmentTest : public ::testing::Test {};

TEST_F(SortTrackerTest, VelocityEmptyInitializationSuccess) {
  SortTracker::KalmanVelocityTracker tracker();
}

//TEST_F(SortTrackerTest, AccelerationEmptyInitializationSuccess) {
//  SortTracker::KalmanAccelerationTracker<10, 4> tracker();
//}

TEST_F(SortTrackerTest, VelocityStateInitializationSuccess) {
  SortTracker::StateVector init_state(4, 1.0);
  SortTracker::KalmanVelocityTracker tracker(init_state);
}

//TEST_F(SortTrackerTest, AccelerationStateInitializationSuccess) {
//  SortTracker::StateVector init_state(4, 1.0);
//  SortTracker::KalmanAccelerationTracker<10, 4> tracker(init_state);
//}

TEST_F(LinearAssignmentTest, LinearAssignmentGraphInitializationSuccess) {
    const int num_nodes = 2;
    const int num_arcs = 2;
    SortTracker::Graph graph(num_nodes, num_arcs);
}

}  // namespace SortTracker

