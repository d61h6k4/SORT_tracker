#include "main/kalman_tracker/kalman_tracker.h"

#include "gtest/gtest.h"
#include "opencv2/video/tracking.hpp"

namespace Tracker {

class KalmanVelocityTrackerTest : public ::testing::Test {};
class LinearAssignmentTest : public ::testing::Test {};
class SortTrackerTest : public ::testing::Test {};

TEST_F(KalmanVelocityTrackerTest, VelocityEmptyInitializationSuccess) { Tracker::KalmanVelocityTracker tracker(0); }

TEST_F(KalmanVelocityTrackerTest, VelocityStateInitializationSuccess) {
  Tracker::StateVector init_bbox = {1, 1, 1, 1};
  Tracker::KalmanVelocityTracker tracker(init_bbox, 0);

  auto state_bbox = tracker.get_state_bbox();

  EXPECT_FLOAT_EQ(state_bbox.at(0), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(1), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(2), 1);
  EXPECT_FLOAT_EQ(state_bbox.at(3), 1);
}

// TEST_F(SortTrackerTest, SortTrackerPredictionSuccess) {
//  Tracker::SortTracker sort_tracker(20, 1, 0.1);
//
//  std::vector<Tracker::StateVector> first_detections = {{0, 0, 20, 20}, {100, 50, 40, 20}};
//  auto first_results = sort_tracker.update(first_detections);
//
//  for (const auto& item : first_results) {
//    for (int i = 0; i < item.size(); ++i) {
//      std::cout << item.at(i) << " ";
//    }
//    std::cout << std::endl;
//  }
//
//  std::vector<Tracker::StateVector> second_detections = {{2, 2, 18, 28}};
//  auto second_results = sort_tracker.update(second_detections);
//
//  for (const auto& item : first_results) {
//    for (int i = 0; i < item.size(); ++i) {
//      std::cout << item.at(i) << " ";
//    }
//    std::cout << std::endl;
//  }

//  auto result_bbox = second_results.at(0);
//  EXPECT_FLOAT_EQ(result_bbox.at(0), 2);
//  EXPECT_FLOAT_EQ(result_bbox.at(1), 2);
//  EXPECT_FLOAT_EQ(result_bbox.at(2), 18);
//  EXPECT_FLOAT_EQ(result_bbox.at(3), 28);
//}

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
