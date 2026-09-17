#include <gtest/gtest.h>
#include "mine_uav_control/anisotropic_clearance.hpp"

TEST(AnisotropicClearance, WallAtFlightHeightUsesHorizontalRadius) {
  EXPECT_TRUE(mine_uav_control::insideAnisotropicClearance(3, 0, 0, 0.5, 1.8, 0.4));
  EXPECT_FALSE(mine_uav_control::insideAnisotropicClearance(5, 0, 0, 0.5, 1.8, 0.4));
}

TEST(AnisotropicClearance, GroundBelowBodyDoesNotBlockHorizontalReachability) {
  EXPECT_FALSE(mine_uav_control::insideAnisotropicClearance(0, 0, -2, 0.5, 1.8, 0.4));
  EXPECT_TRUE(mine_uav_control::insideAnisotropicClearance(0, 0, -1, 0.5, 1.8, 0.4));
}
