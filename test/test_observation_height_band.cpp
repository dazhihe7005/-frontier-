#include "mine_uav_control/observation_height_band.hpp"

#include <gtest/gtest.h>

TEST(ObservationHeightBand, RisingConnectedPassageRemainsEligible) {
  const double home_z = 1.43;
  const double voxel = 0.25;
  EXPECT_TRUE(mine_uav_control::inObservationHeightBand(
      4.16, home_z, 0.0, 7.0, voxel));
  EXPECT_FALSE(mine_uav_control::inObservationHeightBand(
      9.0, home_z, 0.0, 7.0, voxel));
  EXPECT_FALSE(mine_uav_control::inObservationHeightBand(
      1.0, home_z, 0.0, 7.0, voxel));
}

TEST(ObservationHeightBand, RejectsInvalidInput) {
  EXPECT_FALSE(mine_uav_control::inObservationHeightBand(
      NAN, 0.0, 0.0, 7.0, 0.25));
  EXPECT_FALSE(mine_uav_control::inObservationHeightBand(
      2.0, 0.0, 0.0, 7.0, 0.0));
}
