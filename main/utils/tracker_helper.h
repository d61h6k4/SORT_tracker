#pragma once

#ifndef SORT_TRACKER_TRACKER_HELPER_H
#define SORT_TRACKER_TRACKER_HELPER_H

#include <cmath>
#include <vector>

namespace Tracker {
using StateVector = std::vector<float>;

StateVector get_bbox_from_state(const StateVector&);
StateVector get_state_from_bbox(const StateVector&);
}  // namespace Tracker

#endif  // SORT_TRACKER_TRACKER_HELPER_H
