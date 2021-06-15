#include "kalman_tracker.h"

namespace Tracker {

// Correct step
StateVector KalmanVelocityTracker::update(const StateVector& observed_bbox) {
  time_since_update_ = 0;
  history_.clear();
  hits_ += 1;
  hit_streak_ += 1;

  // create new measurement matrix
  auto observed_state = get_state_from_bbox(observed_bbox);
  measurement_ = cv::Mat(dim_measure_, 1, CV_32F, observed_state.data());

  // update
  auto data = filter_.correct(measurement_);

  StateVector corrected_state;
  for (int i = 0; i < dim_measure_; ++i) {
    corrected_state.at(i) = data.at<float>(i, 0);
  }

  auto corrected_bbox = get_bbox_from_state(corrected_state);

  return corrected_bbox;
}

// Predict step
StateVector KalmanVelocityTracker::predict() {
  auto prediction = filter_.predict();
  age_ += 1;

  if (time_since_update_ > 0) {
    hit_streak_ = 0;
  }

  time_since_update_ += 1;

  StateVector state;
  for (int i = 0; i < dim_measure_; ++i) {
    state.at(i) = prediction.at<float>(i, 0);
  }

  auto predict_box = get_bbox_from_state(state);

  history_.push_back(predict_box);
  return history_.back();
}

// Return the current state vector
StateVector KalmanVelocityTracker::get_state_bbox() {
  cv::Mat s = filter_.statePost;
  StateVector state = {s.at<float>(0, 0), s.at<float>(1, 0), s.at<float>(2, 0), s.at<float>(3, 0)};
  return get_bbox_from_state(state);
}

// Solve linear assignment
void SortTracker::solve_assignment_(const cv::Mat& cost_matrix) {
  // Define a num_nodes / 2 by num_nodes / 2 assignment problem:
  const int num_nodes = 2;
  const int num_arcs = 2;
  //    const int num_left_nodes = num_nodes / 2;
  Graph graph(num_nodes, num_arcs);
  //    std::vector arc_costs(num_arcs);
  //    for (int arc = 0; arc < num_arcs; ++arc) {
  //        const int arc_tail = ...   // must be in [0, num_left_nodes)
  //        const int arc_head = ...   // must be in [num_left_nodes, num_nodes)
  //        graph.AddArc(arc_tail, arc_head);
  //        arc_costs[arc] = ...
  //    }
  //    // Build the StaticGraph. You can skip this step by using a ListGraph<>
  //    // instead, but then the ComputeAssignment() below will be slower. It is
  //    // okay if your graph is small and performance is not critical though.
  //    {
  //        std::vector arc_permutation;
  //        graph.Build(&arc_permutation);
  //        util::Permute(arc_permutation, &arc_costs);
  //    }
  //    // Construct the LinearSumAssignment.
  //    ::operations_research::LinearSumAssignment a(graph, num_left_nodes);
  //    for (int arc = 0; arc < num_arcs; ++arc) {
  //        // You can also replace 'arc_costs[arc]' by something like
  //        //   ComputeArcCost(permutation.empty() ? arc : permutation[arc])
  //        // if you don't want to store the costs in arc_costs to save memory.
  //        a.SetArcCost(arc, arc_costs[arc]);
  //    }
  //    // Compute the optimum assignment.
  //    bool success = a.ComputeAssignment();
  //    // Retrieve the cost of the optimum assignment.
  //    operations_research::CostValue optimum_cost = a.GetCost();
  //    // Retrieve the node-node correspondence of the optimum assignment and
  //    the
  //    // cost of each node pairing.
  //    for (int left_node = 0; left_node < num_left_nodes; ++left_node) {
  //        const int right_node = a.GetMate(left_node);
  //        operations_research::CostValue node_pair_cost =
  //                a.GetAssignmentCost(left_node);
  //    }
}

}  // namespace Tracker
