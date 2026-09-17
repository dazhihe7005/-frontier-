#include <gtest/gtest.h>
#include "mine_uav_control/outbound_goal_contract.hpp"

TEST(OutboundGoalContract, RejectsEntranceRearFrontierDuringOutboundFlight) {
  EXPECT_FALSE(mine_uav_control::outboundProgressAllowed(13.4, 3.25, 2.0));
  EXPECT_FALSE(mine_uav_control::outboundProgressAllowed(13.4, -12.75, 2.0));
}

TEST(OutboundGoalContract, KeepsSidewaysAndSmallBacktrackAvailable) {
  EXPECT_TRUE(mine_uav_control::outboundProgressAllowed(13.4, 13.4, 2.0));
  EXPECT_TRUE(mine_uav_control::outboundProgressAllowed(13.4, 12.0, 2.0));
  EXPECT_TRUE(mine_uav_control::outboundProgressAllowed(13.4, 20.0, 2.0));
}
