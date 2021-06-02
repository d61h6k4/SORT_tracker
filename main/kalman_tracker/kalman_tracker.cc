#include "kalman_tracker.h"

namespace SortTracker {

int KalmanVelocityTracker::tracker_count = 0;

void KalmanVelocityTracker::update(const StateVector & observation) {
    time_since_update_ = 0;
    history_.clear();
    hits_ += 1;
    hit_streak_ += 1;

    // measurement
//    measurement.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
//    measurement.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
//    measurement.at<float>(2, 0) = stateMat.area();
//    measurement.at<float>(3, 0) = stateMat.width / stateMat.height;

    // update
    filter_.correct(measurement_matrix_);
}

// Predict the estimated bounding box.
StateVector KalmanVelocityTracker::predict() {
    cv::Mat p = filter_.predict();
    age_ += 1;

    if (time_since_update_ > 0)
        hit_streak_ = 0;
    time_since_update_ += 1;

    auto predict_box = get_rect_xysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

    history_.push_back(predict_box);
    return history_.back();
}


// Solve linear assignment
    void KalmanVelocityTracker::solve_assignment_(const cv::Mat & cost_matrix) {

        // Define a num_nodes / 2 by num_nodes / 2 assignment problem:
        const int num_nodes = 2;
        const int num_arcs = 2;
//    const int num_left_nodes = num_nodes / 2;
        Graph graph(num_nodes, num_arcs);
//    std::vector arc_costs(num_arcs);
////    for (int arc = 0; arc < num_arcs; ++arc) {
////        const int arc_tail = ...   // must be in [0, num_left_nodes)
////        const int arc_head = ...   // must be in [num_left_nodes, num_nodes)
////        graph.AddArc(arc_tail, arc_head);
////        arc_costs[arc] = ...
////    }
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
//    // Retrieve the node-node correspondence of the optimum assignment and the
//    // cost of each node pairing.
//    for (int left_node = 0; left_node < num_left_nodes; ++left_node) {
//        const int right_node = a.GetMate(left_node);
//        operations_research::CostValue node_pair_cost =
//                a.GetAssignmentCost(left_node);
//    }
    }

// Return the current state vector
StateVector KalmanVelocityTracker::get_state() {
    cv::Mat s = filter_.statePost;
    return get_rect_xysr(s.at<float>(0, 0), s.at<float>(1, 0), s.at<float>(2, 0), s.at<float>(3, 0));
}

// Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style.
StateVector KalmanVelocityTracker::get_rect_xysr(float cx, float cy, float s, float r) {
    float w = std::sqrt(s * r);
    float h = s / w;
    float x = (cx - w / 2);
    float y = (cy - h / 2);

    if (x < 0 && cx > 0)
        x = 0;
    if (y < 0 && cy > 0)
        y = 0;

    StateVector result {x, y, w, h};

    return result;
}





}
