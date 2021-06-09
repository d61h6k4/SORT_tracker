#pragma once

#ifndef SORT_TRACKER_TRACKER_HELPER_H
#define SORT_TRACKER_TRACKER_HELPER_H

#include <vector>
#include <cmath>

namespace Tracker{
  using StateVector = std::vector<float>;

  StateVector get_bbox_from_state(const StateVector &);
  StateVector get_state_from_bbox(const StateVector &);
}


#endif //SORT_TRACKER_TRACKER_HELPER_H
