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

TEST_F(SortTrackerTest, IoUCalculationSuccess) {
  Tracker::StateVector bbox_1 = {0, 0, 40, 20};
  Tracker::StateVector bbox_2 = {0, 0, 20, 10};

  float iou_1 = Tracker::calculate_iou(bbox_1, bbox_2);
  float iou_2 = Tracker::calculate_iou(bbox_2, bbox_1);

  EXPECT_FLOAT_EQ(iou_1, 0.25);
  EXPECT_FLOAT_EQ(iou_1, iou_2);
}

TEST_F(SortTrackerTest, ZeroIoUCalculationSuccess) {
  Tracker::StateVector bbox_1 = {0, 0, 40, 20};
  Tracker::StateVector bbox_2 = {100, 200, 10, 10};

  float iou_1 = Tracker::calculate_iou(bbox_1, bbox_2);
  float iou_2 = Tracker::calculate_iou(bbox_2, bbox_1);

  EXPECT_FLOAT_EQ(iou_1, 0.0);
  EXPECT_FLOAT_EQ(iou_1, iou_2);
}

TEST_F(SortTrackerTest, MatrixIouCalculationSuccess) {
  std::vector<Tracker::StateVector> bboxes_1 = {{0, 0, 20, 20}, {100, 50, 40, 20}};
  std::vector<Tracker::StateVector> bboxes_2 = {{10, 0, 10, 20}};

  cv::Mat result = Tracker::calculate_pairwise_iou(bboxes_1, bboxes_2);

  EXPECT_FLOAT_EQ(result.at<float>(0, 0), 0.5);
  EXPECT_FLOAT_EQ(result.at<float>(1, 0), 0);
}