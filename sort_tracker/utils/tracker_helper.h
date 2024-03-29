#pragma once

#include <array>
#include <vector>

#include "opencv2/core/mat.hpp"

namespace Tracker {
using BboxVector = std::array<float, 4>;       // (x, y, w, h)
using DetectionVector = std::array<float, 7>;  // (x, y, w, h, score, class_id, track_id)
using StateVector = std::array<float, 7>;      // (cx, cy, area, scale_ratio_w/h, cx', cy', area')

BboxVector get_bbox_from_detection(const DetectionVector&);
BboxVector get_bbox_from_state(const StateVector&);
StateVector get_state_from_bbox(const BboxVector&);

float calculate_iou(const BboxVector&, const BboxVector&);
cv::Mat calculate_pairwise_iou(const std::vector<BboxVector>&, const std::vector<BboxVector>&);
}  // namespace Tracker
