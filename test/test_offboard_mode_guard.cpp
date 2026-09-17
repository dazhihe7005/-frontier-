#include <gtest/gtest.h>

#include "mine_uav_control/offboard_mode_guard.hpp"

using mine_uav_control::externalOffboardExit;

TEST(OffboardModeGuard, ManualTakeoverRevokesReentry) {
  EXPECT_TRUE(externalOffboardExit("OFFBOARD", "POSCTL", "OFFBOARD", true,
                                  false));
  EXPECT_TRUE(externalOffboardExit("OFFBOARD", "MANUAL", "OFFBOARD", true,
                                  false));
}

TEST(OffboardModeGuard, BridgeRequestedExitIsNotManualTakeover) {
  EXPECT_FALSE(externalOffboardExit("OFFBOARD", "POSCTL", "OFFBOARD", true,
                                   true));
  EXPECT_FALSE(externalOffboardExit("POSCTL", "OFFBOARD", "OFFBOARD", true,
                                   false));
}

TEST(OffboardModeGuard, PrestreamAndUnmanagedModesDoNotLatch) {
  EXPECT_FALSE(externalOffboardExit("POSCTL", "POSCTL", "OFFBOARD", true,
                                   false));
  EXPECT_FALSE(externalOffboardExit("OFFBOARD", "POSCTL", "OFFBOARD", false,
                                   false));
  EXPECT_FALSE(externalOffboardExit("OFFBOARD", "OFFBOARD", "OFFBOARD", true,
                                   false));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
