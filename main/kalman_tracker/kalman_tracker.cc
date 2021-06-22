#include "kalman_tracker.h"

namespace Tracker {

// Correct step
StateVector KalmanVelocityTracker::update(const StateVector& observed_bbox) {
  time_since_update = 0;
  history_.clear();
  hits += 1;
  hit_streak += 1;

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
  age += 1;

  if (time_since_update > 0) {
    hit_streak = 0;
  }

  time_since_update += 1;

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

// SortTracker constructor
SortTracker::SortTracker(int max_age, int min_hits, float iou_threshold) {
  frame_count_ = 0;
  tracker_count_ = 0;

  max_age_ = max_age;
  min_hits_ = min_hits;
  iou_threshold_ = iou_threshold;
}

// SortTracker update
std::vector<StateVector> SortTracker::update(const std::vector<StateVector>& detections) {
  std::vector<StateVector> predictions;
  for (size_t i = 0; i < trackers_.size(); ++i) {
    auto prediction = trackers_.at(i).predict();
    predictions.push_back(prediction);
  }

  // calculate IOU
  auto iou_matrix = calculate_pairwise_iou(detections, predictions);
  std::cout << iou_matrix.rows << " " << iou_matrix.cols << std::endl;

  // negate cost matrix for minimization task
  auto cost_matrix = (-1) * iou_matrix;
  auto assignment_indices = solve_assignment_(cost_matrix);

  //
  for (auto i = 0; i < assignment_indices.size(); ++i) {
    std::cout << "assignment" << assignment_indices.at(i) << std::endl;
  }

  // update matched trackers with detections, create new tracker otherwise
  for (auto i = 0; i < detections.size(); ++i) {
    int match_idx = assignment_indices.at(i);
    if (assignment_indices.size() == 0 || match_idx == -1) {
      KalmanVelocityTracker new_tracker = KalmanVelocityTracker(detections.at(i), tracker_count_++);
      trackers_.push_back(new_tracker);
    } else {
      auto temp = trackers_.at(match_idx).update(detections.at(i));

      for (auto k = 0; k < temp.size(); ++k) {
        std::cout << temp.at(k) << " ";
      }
    }
  }

  std::cout << "lol4" << std::endl;
  //
  std::vector<StateVector> result;
  auto it = trackers_.begin();
  while (it != trackers_.end()) {
    auto tracker = *it;

    bool is_initialized = tracker.hits >= min_hits_ || frame_count_ <= min_hits_;
    bool is_valid = is_initialized && (tracker.time_since_update <= max_age_);

    if (is_valid) {
      auto current_state = tracker.get_state_bbox();
      result.push_back(current_state);
    }

    //     TODO: fix deleting data structure
    //    if (tracker.time_since_update > max_age_){
    //      trackers_.erase(it);
    //    }
    //    else {
    //      ++it;
    //    }
    it++;
  }

  return result;
}

// Solve linear assignment
std::vector<int> SortTracker::solve_assignment_(const cv::Mat& cost_matrix) {
  std::vector<std::vector<double>> cost_data(cost_matrix.rows, std::vector<double>(cost_matrix.cols));

  for (int i = 0; i < cost_matrix.rows; i++) {
    for (int j = 0; j < cost_matrix.cols; j++) {
      cost_data[i][j] = static_cast<double>(cost_matrix.at<float>(i, j));
    }
  }

  absl::flat_hash_map<int, int> rows_cols_assignment;
  absl::flat_hash_map<int, int> cols_rows_assignment;
  operations_research::MinimizeLinearAssignment(cost_data, &rows_cols_assignment, &cols_rows_assignment);

  // Retrieve correspondences
  std::vector<int> result(cost_matrix.rows, -1);
  for (const auto& item : rows_cols_assignment) {
    result[item.first] = item.second;
  }

  return result;
}

}  // namespace Tracker
