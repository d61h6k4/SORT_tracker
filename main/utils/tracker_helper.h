#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "opencv2/highgui/highgui.hpp"

namespace Tracker {
using BboxVector = std::array<float, 4>;
using BboxVectorWithId = std::array<float, 5>;
using StateVector = std::array<float, 7>;

BboxVector get_bbox_from_state(const StateVector&);
StateVector get_state_from_bbox(const BboxVector&);

float calculate_iou(const BboxVector&, const BboxVector&);
cv::Mat calculate_pairwise_iou(const std::vector<BboxVector>&, const std::vector<BboxVector>&);
}  // namespace Tracker
