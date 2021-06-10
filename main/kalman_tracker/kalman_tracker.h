#pragma once

#ifndef SORT_TRACKER_KALMAN_FILTER_H
#define SORT_TRACKER_KALMAN_FILTER_H

#include <vector>

#include "main/utils/tracker_helper.h"
#include "opencv2/video/tracking.hpp"
#include "ortools/graph/graph.h"
#include "ortools/graph/linear_assignment.h"

namespace Tracker {

using StateVector = std::vector<float>;
// Choose a graph implementation (recommended StaticGraph<>).
using Graph = util::StaticGraph<>;

// Class for constant velocity motion model
class KalmanVelocityTracker {
 public:
  KalmanVelocityTracker() {
    time_since_update_ = 0;
    hits_ = 0;
    hit_streak_ = 0;
    age_ = 0;
    id_ = tracker_count;
  }
  KalmanVelocityTracker(const StateVector& init_bbox) {
    time_since_update_ = 0;
    hits_ = 0;
    hit_streak_ = 0;
    age_ = 0;
    id_ = tracker_count;

    filter_ = cv::KalmanFilter(DIM_STATE, DIM_MEASURE);
    measurement_matrix_ = cv::Mat::zeros(DIM_STATE, 1, CV_32F);

    // clang-format off
    std::vector<float> transition_data{
      1, 0, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 1,
      0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 1
    };

    std::vector<float> measurement_uncertainty_data{
      1e1, 0, 0, 0, 0, 0, 0,
      0, 1e1, 0, 0, 0, 0, 0,
      0, 0, 1e1, 0, 0, 0, 0,
      0, 0, 0, 1e1, 0, 0, 0,
      0, 0, 0, 0, 1e4, 0, 0,
      0, 0, 0, 0, 0, 1e4, 0,
      0, 0, 0, 0, 0, 0, 1e4
    };

    std::vector<float> process_noise_data{
      1e0, 0, 0, 0, 0, 0, 0,
      0, 1e0, 0, 0, 0, 0, 0,
      0, 0, 1e0, 0, 0, 0, 0,
      0, 0, 0, 1e1, 0, 0, 0,
      0, 0, 0, 0, 1e-2, 0, 0,
      0, 0, 0, 0, 0, 1e-2, 0,
      0, 0, 0, 0, 0, 0, 1e-4
    };

    std::vector<float> state_uncertainty_data{
      1e0, 0, 0, 0,
      0, 1e0, 0, 0,
      0, 0, 1e1, 0,
      0, 0, 0, 1e1
    };

    // clang-format on
    cv::Mat transition_matrix_init(DIM_STATE, DIM_STATE, CV_32F, transition_data.data());
    filter_.transitionMatrix = transition_matrix_init;

    filter_.measurementMatrix = measurement_matrix_;

    // process uncertainty matrix Q
    cv::Mat process_noise_init(DIM_STATE, DIM_STATE, CV_32F, process_noise_data.data());
    filter_.processNoiseCov = process_noise_init;

    // state uncertainty matrix R
    cv::Mat state_uncertainty_init(DIM_MEASURE, DIM_MEASURE, CV_32F, state_uncertainty_data.data());
    filter_.measurementNoiseCov = state_uncertainty_init;

    // uncertainty covariance matrix P
    cv::Mat measurement_noise_init(DIM_STATE, DIM_STATE, CV_32F, measurement_uncertainty_data.data());
    filter_.errorCovPost = measurement_noise_init;

    // copy initial state to kalman filter initial state matrix
    auto init_state = get_state_from_bbox(init_bbox);
    for (auto i = 0; i < DIM_MEASURE; ++i) {
      filter_.statePost.at<float>(i, 0) = init_state.at(i);
    }
  }

  static int tracker_count;

  StateVector get_state();
  StateVector predict();
  void update(const StateVector& observation);

 private:
  int DIM_STATE = 7;
  int DIM_MEASURE = 4;

  int time_since_update_;
  int hits_;
  int hit_streak_;
  int age_;
  int id_;

  cv::KalmanFilter filter_;
  cv::Mat measurement_matrix_;
  std::vector<StateVector> history_;
};

// class for unification of trackers and linear assignment solver
class SortTracker {
 public:
  StateVector update(const StateVector& observation);

 private:
  std::vector<KalmanVelocityTracker> trackers_;

  void solve_assignment_(const cv::Mat& cost_matrix);
};

}  // namespace Tracker

#endif  // SORT_TRACKER_KALMAN_FILTER_H
