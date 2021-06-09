#pragma once

#ifndef SORT_TRACKER_KALMAN_FILTER_H
#define SORT_TRACKER_KALMAN_FILTER_H

#include <vector>
#include "ortools/graph/graph.h"
#include "ortools/graph/linear_assignment.h"
#include "opencv2/video/tracking.hpp"
#include "main/utils/tracker_helper.h"

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
    KalmanVelocityTracker(const StateVector & init_bbox) {
        time_since_update_ = 0;
        hits_ = 0;
        hit_streak_ = 0;
        age_ = 0;
        id_ = tracker_count;

        filter_ = cv::KalmanFilter(DIM_STATE, DIM_MEASURE);
        measurement_matrix_ = cv::Mat::zeros(DIM_STATE, 1, CV_32F);

        std::vector<float> mat_data {
            1, 0, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 1
        };

        cv::Mat transition_matrix_init(DIM_STATE, DIM_STATE, CV_32F, mat_data.data());
        filter_.transitionMatrix = transition_matrix_init;

        cv::setIdentity(filter_.measurementMatrix);
        cv::setIdentity(filter_.processNoiseCov, cv::Scalar::all(1e-2));
        cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(filter_.errorCovPost, cv::Scalar::all(1));

        auto init_state = get_state_from_bbox(init_bbox);

        // copy intial state to kalman filter initial state matrix
        for (auto i = 0; i < DIM_MEASURE; ++i){
          filter_.statePost.at<float>(i, 0) = init_state.at(i);
        }

    }
    ~KalmanVelocityTracker() {
        history_.clear();
    }

    static int tracker_count;

    StateVector get_state();
    StateVector predict();
    void update(const StateVector & observation);

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
    std::vector <StateVector> history_;
};

    // class for unification of trackers and linear assignment solver
class SortTracker {
public:
    StateVector update(const StateVector & observation);

private:
    std::vector<KalmanVelocityTracker> trackers_;

    void solve_assignment_(const cv::Mat & cost_matrix);
};

} // namespace SortTracker

#endif //SORT_TRACKER_KALMAN_FILTER_H
