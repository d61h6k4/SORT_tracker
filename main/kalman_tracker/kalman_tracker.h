#pragma once

#ifndef SORT_TRACKER_KALMAN_FILTER_H
#define SORT_TRACKER_KALMAN_FILTER_H

#include <vector>
#include "ortools/graph/graph.h"
#include "ortools/graph/linear_assignment.h"
#include "opencv2/video/tracking.hpp"

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
    KalmanVelocityTracker(const StateVector & init_state) {
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

        // initialize state vector with bounding box in [cx,cy,s,r] style
        //    filter_.statePost.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
        //    filter_.statePost.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
        //    filter_.statePost.at<float>(2, 0) = stateMat.area();
        //    filter_.statePost.at<float>(3, 0) = stateMat.width / stateMat.height;

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

    void solve_assignment_(const cv::Mat & cost_matrix);
};

    // class for unification of trackers and linear assignment solver
class SortTracker {
public:
    StateVector update(const StateVector & observation);

private:
    std::vector<KalmanVelocityTracker> trackers_;
};

StateVector get_bbox_from_state(const StateVector &);
StateVector get_state_from_bbox(const StateVector &);

} // namespace SortTracker

#endif //SORT_TRACKER_KALMAN_FILTER_H
