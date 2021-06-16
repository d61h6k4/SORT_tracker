#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "opencv2/highgui/highgui.hpp"

namespace Tracker {
using StateVector = std::array<float, 4>;

StateVector get_bbox_from_state(const StateVector&);
StateVector get_state_from_bbox(const StateVector&);

float calculate_iou(const StateVector&, const StateVector&);
cv::Mat calculate_pairwise_iou(const std::vector<StateVector>&, const std::vector<StateVector>&);
}  // namespace Tracker
