#include "main/utils/tracker_helper.h"

#include "gtest/gtest.h"

class SortTrackerTest : public ::testing::Test {};

TEST_F(SortTrackerTest, StateToBboxConversionSuccess) {
  Tracker::StateVector state = {10, 10, 50, 2};

  Tracker::StateVector bbox = Tracker::get_bbox_from_state(state);

  EXPECT_FLOAT_EQ(bbox.at(0), 5.0);
  EXPECT_FLOAT_EQ(bbox.at(1), 7.5);
  EXPECT_FLOAT_EQ(bbox.at(2), 10);
  EXPECT_FLOAT_EQ(bbox.at(3), 5);
}

TEST_F(SortTrackerTest, BboxToStateConversionSuccess) {
  Tracker::StateVector bbox = {0, 0, 40, 20};

  Tracker::StateVector state = Tracker::get_state_from_bbox(bbox);

  EXPECT_FLOAT_EQ(state.at(0), 20);
  EXPECT_FLOAT_EQ(state.at(1), 10);
  EXPECT_FLOAT_EQ(state.at(2), 800);
  EXPECT_FLOAT_EQ(state.at(3), 2);
}