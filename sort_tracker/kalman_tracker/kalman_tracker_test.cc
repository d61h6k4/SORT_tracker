#include "sort_tracker/kalman_tracker/kalman_tracker.h"

#include "gtest/gtest.h"
#include "opencv2/core/mat.hpp"

namespace Tracker {

class KalmanVelocityTrackerTest : public ::testing::Test {};
class LinearAssignmentTest : public ::testing::Test {};
class SortTrackerTest : public ::testing::Test {};

TEST_F(KalmanVelocityTrackerTest, VelocityEmptyInitializationSuccess) { Tracker::KalmanVelocityTracker tracker(0); }

TEST_F(KalmanVelocityTrackerTest, VelocityStateBboxInitializationSuccess) {
  Tracker::BboxVector init_bbox = {1, 1, 1, 1};
  Tracker::KalmanVelocityTracker tracker(init_bbox, 0);

  auto state_bbox = tracker.get_state_bbox();

  EXPECT_FLOAT_EQ(state_bbox.at(0), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(1), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(2), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(3), 1);

  auto prediction = tracker.predict();
  EXPECT_EQ(prediction.size(), 4);
}

TEST_F(KalmanVelocityTrackerTest, VelocityStateDetectionInitializationSuccess) {
  Tracker::DetectionVector init_det = {1, 1, 1, 1, 1.0, 0, -1};
  Tracker::KalmanVelocityTracker tracker(init_det, 0);

  auto state_bbox = tracker.get_state_bbox();

  EXPECT_FLOAT_EQ(state_bbox.at(0), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(1), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(2), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(3), 1);

  auto prediction = tracker.predict();
  EXPECT_EQ(prediction.size(), 4);
}

TEST_F(KalmanVelocityTrackerTest, VelocityKalmanFilterTrackProgressSuccess) {
  Tracker::DetectionVector init_bbox = {100, 100, 10, 10, 1.0, 0, -1};
  Tracker::KalmanVelocityTracker tracker(init_bbox, 0);

  BboxVector res_prediction;
  DetectionVector res_update;
  DetectionVector input = init_bbox;

  for (int i = 0; i < 50; ++i) {
    res_prediction = tracker.predict();

    input.at(0) -= 2;
    input.at(1) -= 2;
    res_update = tracker.update(input);
  }

  EXPECT_NEAR(res_prediction.at(0), 0, 1e-3);
  EXPECT_NEAR(res_prediction.at(1), 0, 1e-3);
  EXPECT_NEAR(res_prediction.at(2), 10, 1e-3);
  EXPECT_NEAR(res_prediction.at(3), 10, 1e-3);
}

TEST_F(SortTrackerTest, SortTrackerMatchingSuccess) {
  Tracker::SortTracker sort_tracker(20, 1, 1, 0.1);

  std::vector<Tracker::DetectionVector> first_detections = {{0, 0, 20, 20, 0.9, 0, -1}, {100, 50, 40, 20, 0.8, 0, -1}};
  auto first_results = sort_tracker.update(first_detections);

  std::vector<Tracker::DetectionVector> second_detections = {{2, 2, 18, 22, 0.8, 0, -1}};
  auto second_results = sort_tracker.update(second_detections);

  EXPECT_EQ(second_results.size(), 1);

  auto result_bbox = second_results.at(0);
  EXPECT_FLOAT_EQ(result_bbox.at(4), 0);

  EXPECT_NEAR(result_bbox.at(0), 2, 2);
  EXPECT_NEAR(result_bbox.at(1), 2, 2);
  EXPECT_NEAR(result_bbox.at(2), 18, 2);
  EXPECT_NEAR(result_bbox.at(3), 22, 2);
}

TEST_F(SortTrackerTest, SortTrackerDeleteOldTrackersSuccess) {
  int max_age = 5;
  int min_hits = 1;
  Tracker::SortTracker sort_tracker(max_age, min_hits, 1, 0.1);

  std::vector<Tracker::DetectionVector> first_detections = {{0, 0, 20, 20, 0.9, 0, -1}, {100, 50, 40, 20, 0.8, 0, -1}};
  std::vector<Tracker::DetectionVector> empty_detections;

  // initialize trackers with min_hits
  for (int i = 0; i < min_hits + 1; ++i) {
    sort_tracker.update(first_detections);
  }

  // initialized trackers are stored for at most max_age frames
  for (int i = 0; i < max_age; ++i) {
    auto non_empty_updates = sort_tracker.update(empty_detections);
    EXPECT_EQ(non_empty_updates.size(), 2);
  }

  // after max_age empty updates all trackers shall be deleted
  auto empty_updates = sort_tracker.update(empty_detections);
  EXPECT_EQ(empty_updates.size(), 0);
}

TEST_F(LinearAssignmentTest, LinearAssignmentGraphInitializationSuccess) {
  // clang-format off
  std::vector<float> cost_data{
      2, 4, 5,
      4, 1, 6,
      7, 8, 2
  };
  // clang-format on
  cv::Mat cost_matrix = cv::Mat(3, 3, CV_32F, cost_data.data());

  const int num_nodes = cost_matrix.cols + cost_matrix.rows;
  const int num_arcs = cost_matrix.cols * cost_matrix.rows;
  const int num_left_nodes = cost_matrix.rows;

  Graph graph(num_nodes, num_arcs);
  std::vector<float> arc_costs(num_arcs);

  for (int i = 0; i < cost_matrix.rows; ++i) {
    for (int j = 0; j < cost_matrix.cols; ++j) {
      graph.AddArc(i, j + cost_matrix.rows);
      arc_costs[i * cost_matrix.rows + j] = cost_matrix.at<float>(i, j);
    }
  }

  graph.Build();

  // Construct the LinearSumAssignment.
  operations_research::LinearSumAssignment<Graph> a(graph, num_left_nodes);
  for (int arc = 0; arc < num_arcs; ++arc) {
    a.SetArcCost(arc, arc_costs[arc]);
  }
  // Compute the optimum assignment and retrieve the cost
  bool success = a.ComputeAssignment();
  operations_research::CostValue optimum_cost = a.GetCost();

  // Retrieve the node-node correspondence of the optimum assignment and the cost of each node pairing.
  std::vector<int> result(num_left_nodes);
  for (int left_node = 0; left_node < num_left_nodes; ++left_node) {
    result.at(left_node) = a.GetMate(left_node);
  }

  EXPECT_TRUE(success);
  EXPECT_EQ(optimum_cost, 5);

  EXPECT_EQ(result.size(), 3);
  EXPECT_EQ(result.at(0), 3);
  EXPECT_EQ(result.at(1), 4);
  EXPECT_EQ(result.at(2), 5);
}

TEST_F(LinearAssignmentTest, HungarianInitializationSuccess) {
  // clang-format off
  std::vector <std::vector<double>> cost_data{
      {4.1, 2.2, 5.4},
      {4, 2.1, 1.0},
  };
  // clang-format on
  absl::flat_hash_map<int, int> rows_cols_assignment;
  absl::flat_hash_map<int, int> cols_rows_assignment;
  operations_research::MinimizeLinearAssignment(cost_data, &rows_cols_assignment, &cols_rows_assignment);

  // check correct number of matches
  EXPECT_EQ(rows_cols_assignment.size(), 2);
  EXPECT_EQ(cols_rows_assignment.size(), 2);

  for (const auto& item : rows_cols_assignment) {
    EXPECT_EQ(item.first + 1, item.second);
  }

  for (const auto& item : cols_rows_assignment) {
    EXPECT_EQ(item.first, item.second + 1);
  }
}

TEST_F(LinearAssignmentTest, HungarianRuntimeComplexitySuccess) {
  // Build shifted diagonal matching, with random costs in [0, default_cost) for correct positions
  int m = 1000;
  int n = 1200;
  int default_cost = 100;
  std::vector<std::vector<double>> cost_data(m, std::vector<double>(n, default_cost));

  for (int i = 0; i < m; ++i) {
    cost_data[i][i + 1] = std::rand() % default_cost;
  }

  // solve and check diagonal shift by 1 element
  absl::flat_hash_map<int, int> rows_cols_assignment;
  absl::flat_hash_map<int, int> cols_rows_assignment;
  operations_research::MinimizeLinearAssignment(cost_data, &rows_cols_assignment, &cols_rows_assignment);

  for (const auto& item : rows_cols_assignment) {
    EXPECT_EQ(item.first + 1, item.second);
  }
}

}  // namespace Tracker
