#include <algorithm>

#include <gtest/gtest.h>

#include "mine_uav_control/shaft_mission.hpp"

using mine_uav_control::ShaftMission;

TEST(ShaftMission, RequiresIndependentDepthAndFreshRange) {
  ShaftMission mission({});
  ShaftMission::Input in;
  in.enabled = true;
  in.dt = 0.1;
  in.range_fresh = true;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kIdle);
  in.depth_valid = true;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kDescending);
  in.enabled = false;
  mission.step(in);
  in.enabled = true;
  in.range_fresh = false;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kIdle);
  in.range_fresh = true;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kDescending);
  in.range_fresh = false;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kFault);
}

TEST(ShaftMission, BottomConfirmationThenReturnToCapturedEntrance) {
  ShaftMission mission({});
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.dt = 0.1;
  in.depth = 10.0;
  EXPECT_LT(mission.step(in).vertical_speed_enu, 0.0);
  in.depth = 11.0;
  in.bottom_range = 1.5;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kDescending);
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kDescending);
  EXPECT_GT(mission.step(in).vertical_speed_enu, 0.0);
  in.bottom_range = INFINITY;
  in.depth = 10.4;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kComplete);
  EXPECT_FALSE(mission.step(in).command_valid);
  in.depth_valid = false;
  in.range_fresh = false;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kComplete);
}

TEST(ShaftMission, StaleSensorAndDepthJumpFailClosed) {
  ShaftMission mission({});
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.dt = 0.1;
  mission.step(in);
  in.range_fresh = false;
  EXPECT_FALSE(mission.step(in).command_valid);
  in.enabled = false;
  mission.step(in);
  in.enabled = true;
  in.range_fresh = true;
  mission.step(in);
  in.depth = 5.0;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kFault);
}

TEST(ShaftMission, DepthLimitDoesNotPretendReturnIsPossible) {
  ShaftMission::Config config;
  config.max_depth = 2.0;
  ShaftMission mission(config);
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.dt = 0.1;
  mission.step(in);
  in.depth = 2.1;
  EXPECT_EQ(mission.step(in).state, ShaftMission::State::kFault);
}

TEST(ShaftMission, SimulatedFourHundredTwentyMeterShaftCycle) {
  ShaftMission mission({});
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.range_max = 30.0;
  in.dt = 0.05;
  constexpr double shaft_bottom = 420.0;
  double simulated_depth = 0.0;
  double deepest = 0.0;
  bool saw_return = false;
  for (int i = 0; i < 36000; ++i) {
    in.depth = simulated_depth;
    const double distance = shaft_bottom - simulated_depth;
    in.bottom_range = distance <= in.range_max ? distance : INFINITY;
    const auto out = mission.step(in);
    if (out.state == ShaftMission::State::kReturning) {
      saw_return = true;
    }
    if (out.state == ShaftMission::State::kComplete) {
      EXPECT_TRUE(saw_return);
      EXPECT_GE(deepest, 417.0);
      EXPECT_LT(deepest, shaft_bottom - 0.5);
      EXPECT_LE(simulated_depth, 0.5);
      return;
    }
    ASSERT_NE(out.state, ShaftMission::State::kFault);
    simulated_depth += -out.vertical_speed_enu * in.dt;
    deepest = std::max(deepest, simulated_depth);
  }
  FAIL() << "420 m shaft cycle did not complete within the simulated duration";
}

TEST(ShaftMission, InvalidConfigurationFailsBeforeMotion) {
  ShaftMission::Config config;
  config.bottom_stop_margin = config.bottom_trigger;
  ShaftMission mission(config);
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.dt = 0.1;
  const auto out = mission.step(in);
  EXPECT_EQ(out.state, ShaftMission::State::kFault);
  EXPECT_FALSE(out.command_valid);
}
