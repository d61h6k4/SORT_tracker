#pragma once

#include <vector>

#include "main/utils/tracker_helper.h"
#include "opencv2/video/tracking.hpp"
#include "ortools/graph/graph.h"
#include "ortools/graph/linear_assignment.h"

namespace Tracker {

// Choose a graph implementation (recommended StaticGraph<>).
using Graph = util::StaticGraph<>;

// Class for constant velocity motion model
class KalmanVelocityTracker {
 public:
  KalmanVelocityTracker(int id) {
    time_since_update_ = 0;
    hits_ = 0;
    hit_streak_ = 0;
    age_ = 0;
    id_ = id;

    filter_.init(dim_state_, dim_measure_);

    // clang-format off
    std::vector<float> measurement_matrix{
        1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0
    };

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
        0, 0, 0, 1e0, 0, 0, 0,
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

    cv::Mat measurement_matrix_init(dim_measure_, dim_state_, CV_32F, measurement_matrix.data());
    filter_.measurementMatrix = measurement_matrix_init;

    // transition matrix F
    cv::Mat transition_matrix_init(dim_state_, dim_state_, CV_32F, transition_data.data());
    filter_.transitionMatrix = transition_matrix_init;

    // process uncertainty matrix Q
    cv::Mat process_noise_init(dim_state_, dim_state_, CV_32F, process_noise_data.data());
    filter_.processNoiseCov = process_noise_init;

    // state uncertainty matrix R
    cv::Mat state_uncertainty_init(dim_measure_, dim_measure_, CV_32F, state_uncertainty_data.data());
    filter_.measurementNoiseCov = state_uncertainty_init;

    // uncertainty covariance matrix P
    cv::Mat measurement_noise_init(dim_state_, dim_state_, CV_32F, measurement_uncertainty_data.data());
    filter_.errorCovPost = measurement_noise_init;
  }

  // create kalman filter with initial bbox state
  KalmanVelocityTracker(const StateVector& init_bbox, int id) : KalmanVelocityTracker(id) {
    auto init_state = get_state_from_bbox(init_bbox);
    for (int i = 0; i < dim_measure_; ++i) {
      filter_.statePost.at<float>(i) = init_state.at(i);
    }
  }

  StateVector get_state_bbox();
  StateVector predict();
  StateVector update(const StateVector& observation);

 private:
  const int dim_state_ = 7;
  const int dim_measure_ = 4;

  int time_since_update_;
  int hits_;
  int hit_streak_;
  int age_;
  int id_;

  cv::KalmanFilter filter_;
  cv::Mat measurement_;
  std::vector<StateVector> history_;
};

// class for unification of trackers and linear assignment solver
class SortTracker {
 public:
  StateVector update(const StateVector& detections);

 private:
  void solve_assignment_(const cv::Mat& cost_matrix);

 private:
  std::vector<KalmanVelocityTracker> trackers_;

  int tracker_count_ = 0;
};

}  // namespace Tracker
