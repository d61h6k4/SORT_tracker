#include "tracker_helper.h"

#include <algorithm>

namespace Tracker {

// Convert bounding box [x,y,w,h,score,class_id,track_id] to [x,y,w,h] style vector.
BboxVector get_bbox_from_detection(const DetectionVector & det) {;
  BboxVector result;
  std::copy(det.begin(), det.begin() + 4, result.begin());

  return result;
}

// Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style vector.
BboxVector get_bbox_from_state(const StateVector& vec) {
  float w = std::sqrt(vec.at(2) * vec.at(3));
  float h = vec.at(2) / w;
  float x = (vec.at(0) - w / 2);
  float y = (vec.at(1) - h / 2);

  if (x < 0 && vec.at(0) > 0) x = 0;
  if (y < 0 && vec.at(1) > 0) y = 0;

  BboxVector result{x, y, w, h};

  return result;
}

// Convert bounding box from [x,y,w,h] to [cx,cy,s,r] style.
StateVector get_state_from_bbox(const BboxVector& bbox) {
  float center_x = bbox.at(0) + bbox.at(2) / 2;
  float center_y = bbox.at(1) + bbox.at(3) / 2;
  float area = bbox.at(2) * bbox.at(3);
  float ratio = bbox.at(2) / bbox.at(3);

  StateVector result{center_x, center_y, area, ratio, 0, 0, 0};

  return result;
}

// calculate IoU between two axis-aligned bboxes
float calculate_iou(const BboxVector& bbox_1, const BboxVector& bbox_2) {
  float x_left = std::max(bbox_1.at(0), bbox_2.at(0));
  float y_top = std::max(bbox_1.at(1), bbox_2.at(1));
  float x_right = std::min(bbox_1.at(0) + bbox_1.at(2), bbox_2.at(0) + bbox_2.at(2));
  float y_bottom = std::min(bbox_1.at(1) + bbox_1.at(3), bbox_2.at(1) + bbox_2.at(3));

  if (x_right < x_left || y_bottom < y_top) {
    return .0f;
  }

  float intersection = (x_right - x_left) * (y_bottom - y_top);
  float area_1 = bbox_1.at(2) * bbox_1.at(3);
  float area_2 = bbox_2.at(2) * bbox_2.at(3);

  float iou = intersection / (area_1 + area_2 - intersection + 1e-6);
  return iou;
}

cv::Mat calculate_pairwise_iou(const std::vector<BboxVector>& first, const std::vector<BboxVector>& second) {
  int n = first.size();
  int m = second.size();

  cv::Mat result = cv::Mat(n, m, CV_32F);

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      auto iou_i_j = calculate_iou(first.at(i), second.at(j));
      result.at<float>(i, j) = iou_i_j;
    }
  }

  return result;
}

}  // namespace Tracker