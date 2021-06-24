#pragma once

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "main/utils/tracker_helper.h"
#include "opencv2/video/tracking.hpp"
#include "ortools/algorithms/hungarian.h"
#include "ortools/graph/graph.h"
#include "ortools/graph/linear_assignment.h"

namespace Tracker {

// Choose a graph implementation (recommended StaticGraph<>).
using Graph = util::ListGraph<int, int>;

// Class for constant velocity motion model
class KalmanVelocityTracker {
 public:
  KalmanVelocityTracker(int current_id) {
    time_since_update = 0;
    hits = 0;
    hit_streak = 0;
    age = 0;
    id = current_id;

    filter_.init(dim_state_, dim_measure_, 0);

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

    std::vector<float> error_covariance_data{
        1e0, 0, 0, 0, 0, 0, 0,
        0, 1e0, 0, 0, 0, 0, 0,
        0, 0, 1e0, 0, 0, 0, 0,
        0, 0, 0, 1e0, 0, 0, 0,
        0, 0, 0, 0, 1e1, 0, 0,
        0, 0, 0, 0, 0, 1e1, 0,
        0, 0, 0, 0, 0, 0, 1e1
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

    std::vector<float> observation_noise_data{
        1e0, 0, 0, 0,
        0, 1e0, 0, 0,
        0, 0, 1e1, 0,
        0, 0, 0, 1e1
    };
    // clang-format on

    cv::Mat measurement_matrix_init(dim_measure_, dim_state_, CV_32F, measurement_matrix.data());
    measurement_matrix_init.copyTo(filter_.measurementMatrix);

    // transition matrix F
    cv::Mat transition_matrix_init(dim_state_, dim_state_, CV_32F, transition_data.data());
    transition_matrix_init.copyTo(filter_.transitionMatrix);

    // process uncertainty matrix Q
    cv::Mat process_noise_init(dim_state_, dim_state_, CV_32F, process_noise_data.data());
    process_noise_init.copyTo(filter_.processNoiseCov);

    // state uncertainty matrix R
    cv::Mat observation_noise_init(dim_measure_, dim_measure_, CV_32F, observation_noise_data.data());
    observation_noise_init.copyTo(filter_.measurementNoiseCov);

    // uncertainty covariance matrix P
    cv::Mat error_covariance_init(dim_state_, dim_state_, CV_32F, error_covariance_data.data());
    error_covariance_init.copyTo(filter_.errorCovPost);
  }

  // create kalman filter with initial bbox state
  KalmanVelocityTracker(const BboxVector& init_bbox, int id) : KalmanVelocityTracker(id) {
    auto init_state = get_state_from_bbox(init_bbox);
    for (int i = 0; i < dim_measure_; ++i) {
      filter_.statePost.at<float>(i, 0) = init_state.at(i);
    }
  }

  BboxVector get_state_bbox() const;
  BboxVector predict();
  BboxVector update(const BboxVector& observation);

 public:
  int time_since_update;
  int hits;
  int hit_streak;
  int age;
  int id;

 private:
  int dim_state_ = 7;
  int dim_measure_ = 4;

  cv::KalmanFilter filter_;
  cv::Mat measurement_;
};

// class for unification of trackers and linear assignment solver
class SortTracker {
 public:
  SortTracker(int max_age = 5, int min_hits = 1, int num_init_frames = 5, float iou_threshold = 0.1);
  std::vector<BboxVectorWithId> update(const std::vector<BboxVector>& detections);

 private:
  std::vector<int> solve_assignment_(const cv::Mat& cost_matrix);

 private:
  std::vector<KalmanVelocityTracker> trackers_;

  int frame_count_;
  int tracker_count_;

  int max_age_;
  int min_hits_;
  int num_init_frames_;
  float iou_threshold_;
};

}  // namespace Tracker
