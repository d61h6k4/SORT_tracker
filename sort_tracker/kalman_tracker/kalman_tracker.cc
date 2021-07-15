#include "kalman_tracker.h"

#include <algorithm>

namespace Tracker {

// Correct step
DetectionVector KalmanVelocityTracker::update(const DetectionVector& detection) {
  time_since_update_ = 0;
  hits_ += 1;
  hit_streak_ += 1;
  class_id_ = detection.at(5);

  // create new measurement matrix
  auto observed_bbox = get_bbox_from_detection(detection);
  auto observed_state = get_state_from_bbox(observed_bbox);
  measurement_ = cv::Mat(dim_measure_, 1, CV_32F, observed_state.data());

  // update
  auto data = filter_.correct(measurement_);

  StateVector corrected_state;
  for (int i = 0; i < dim_measure_; ++i) {
    corrected_state.at(i) = data.at<float>(i, 0);
  }

  auto corrected_bbox = get_bbox_from_state(corrected_state);

  DetectionVector result = {
      corrected_bbox.at(0),          corrected_bbox.at(1),         corrected_bbox.at(2), corrected_bbox.at(3), 1.0,
      static_cast<float>(class_id_), static_cast<float>(track_id_)};

  return result;
}

// Predict step
BboxVector KalmanVelocityTracker::predict() {
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

  return predict_box;
}

// Return the current state vector
DetectionVector KalmanVelocityTracker::get_state_bbox() const {
  cv::Mat state_post = filter_.statePost;
  StateVector state;

  for (int i = 0; i < dim_state_; ++i) {
    state.at(i) = state_post.at<float>(i, 0);
  }

  auto bbox = get_bbox_from_state(state);

  DetectionVector result = {bbox.at(0),
                            bbox.at(1),
                            bbox.at(2),
                            bbox.at(3),
                            1.0,
                            static_cast<float>(class_id_),
                            static_cast<float>(track_id_)};

  return result;
}

// SortTracker constructor
SortTracker::SortTracker(int max_age, int min_hits, int num_init_frames, float iou_threshold) {
  frame_count_ = 0;
  tracker_count_ = 0;

  max_age_ = max_age;
  min_hits_ = min_hits;
  num_init_frames_ = num_init_frames;
  iou_threshold_ = iou_threshold;
}

// SortTracker update
std::vector<DetectionVector> SortTracker::update(const std::vector<DetectionVector>& detections) {
  // update time step and all trackers' internal time steps
  frame_count_++;

  std::vector<BboxVector> predicted_bboxes;
  for (size_t i = 0; i < trackers_.size(); ++i) {
    auto prediction = trackers_.at(i).predict();
    predicted_bboxes.push_back(prediction);
  }

  std::vector<BboxVector> detected_bboxes;
  std::transform(detections.begin(), detections.end(), std::back_inserter(detected_bboxes),
                 [](const DetectionVector& x) -> BboxVector { return get_bbox_from_detection(x); });

  std::vector<int> assignment_indices(detected_bboxes.size(), -1);
  bool is_iou_valid = detected_bboxes.size() != 0 && predicted_bboxes.size() != 0;
  if (is_iou_valid) {
    // calculate negated iou matrix and solve matching problem
    cv::Mat iou_matrix = calculate_pairwise_iou(detected_bboxes, predicted_bboxes);
    cv::threshold(iou_matrix, iou_matrix, iou_threshold_, 1.0, cv::THRESH_TOZERO);

    auto cost_matrix = (-1) * iou_matrix;
    assignment_indices = solve_assignment_(cost_matrix);

    for (int i = 0; i < iou_matrix.rows; ++i) {
      if (iou_matrix.at<float>(i, assignment_indices.at(i)) <= iou_threshold_) {
        assignment_indices.at(i) = -1;
      }
    }
  }

  // update matched trackers with corresponding detections, create new tracker otherwise
  for (size_t i = 0; i < detections.size(); ++i) {
    int match_idx = assignment_indices.at(i);

    if (assignment_indices.size() == 0 || match_idx == -1) {
      KalmanVelocityTracker new_tracker = KalmanVelocityTracker(detections.at(i), tracker_count_++);
      trackers_.push_back(new_tracker);
    } else {
      trackers_.at(match_idx).update(detections.at(i));
    }
  }

  // construct result - bboxes with corresponding tracker id
  std::vector<DetectionVector> result;
  std::for_each(trackers_.begin(), trackers_.end(), [this, &result](const KalmanVelocityTracker& tracker) {
    // at the beginning also consider tracks that are at ongoing init
    bool is_initialized = tracker.hits_ >= min_hits_ || frame_count_ <= num_init_frames_;
    bool is_valid = is_initialized && (tracker.time_since_update_ <= max_age_);

    if (is_valid) {
      auto current_state = tracker.get_state_bbox();
      DetectionVector temp;

      std::copy(current_state.begin(), current_state.end(), temp.begin());
      temp.back() = static_cast<float>(tracker.track_id_);
      result.push_back(temp);
    }
  });

  auto remove_cond = [this](const KalmanVelocityTracker& x) { return x.time_since_update_ > max_age_; };
  trackers_.erase(std::remove_if(trackers_.begin(), trackers_.end(), remove_cond), trackers_.end());

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
