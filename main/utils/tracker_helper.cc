#include "tracker_helper.h"

namespace Tracker {

// Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style vector.
StateVector get_bbox_from_state(const StateVector &vec) {
  float w = std::sqrt(vec.at(2) * vec.at(3));
  float h = vec.at(2) / w;
  float x = (vec.at(0) - w / 2);
  float y = (vec.at(1) - h / 2);

  if (x < 0 && vec.at(0) > 0)
    x = 0;
  if (y < 0 && vec.at(1) > 0)
    y = 0;

  StateVector result{x, y, w, h};

  return result;
}

// Convert bounding box from [x,y,w,h] to [cx,cy,s,r] style.
StateVector get_state_from_bbox(const StateVector &bbox) {
  float center_x = bbox.at(0) + bbox.at(2) / 2;
  float center_y = bbox.at(1) + bbox.at(3) / 2;
  float area = bbox.at(2) * bbox.at(3);
  float ratio = bbox.at(2) / bbox.at(3);

  StateVector result{center_x, center_y, area, ratio};

  return result;
}

}