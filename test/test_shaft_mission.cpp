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

TEST(ShaftMission, FiveMeterLaserThresholdBrakesBeforeReversing) {
  ShaftMission mission({});
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.dt = 0.1;
  in.depth = 10.0;
  in.bottom_range = INFINITY;

  ShaftMission::Output out;
  for (int i = 0; i < 20; ++i) {
    out = mission.step(in);
  }
  ASSERT_EQ(out.state, ShaftMission::State::kDescending);
  ASSERT_NEAR(out.vertical_speed_enu, -0.5, 1e-9);

  in.depth = 11.0;
  in.bottom_range = 5.01;
  out = mission.step(in);
  EXPECT_EQ(out.state, ShaftMission::State::kDescending);
  EXPECT_NEAR(out.vertical_speed_enu, -0.5, 1e-9);

  in.bottom_range = 5.0;
  double previous_speed = out.vertical_speed_enu;
  out = mission.step(in);
  EXPECT_EQ(out.state, ShaftMission::State::kDescending);
  EXPECT_GT(out.vertical_speed_enu, previous_speed);
  EXPECT_LE(out.vertical_speed_enu, 0.0);
  EXPECT_LE(std::abs(out.vertical_speed_enu - previous_speed), 0.05 + 1e-9);

  bool saw_return = false;
  bool saw_positive_climb = false;
  for (int i = 0; i < 30; ++i) {
    previous_speed = out.vertical_speed_enu;
    out = mission.step(in);
    EXPECT_LE(std::abs(out.vertical_speed_enu - previous_speed), 0.05 + 1e-9);
    saw_return = saw_return || out.state == ShaftMission::State::kReturning;
    saw_positive_climb = saw_positive_climb || out.vertical_speed_enu > 0.0;
  }
  EXPECT_TRUE(saw_return);
  EXPECT_TRUE(saw_positive_climb);
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
      EXPECT_GE(deepest, 415.0);
      EXPECT_LT(deepest, shaft_bottom - 4.0);
      EXPECT_LE(simulated_depth, 0.5);
      return;
    }
    ASSERT_NE(out.state, ShaftMission::State::kFault);
    simulated_depth += -out.vertical_speed_enu * in.dt;
    deepest = std::max(deepest, simulated_depth);
  }
  FAIL() << "420 m shaft cycle did not complete within the simulated duration";
}

TEST(ShaftMission, SimulatedFiveHundredMeterOpticalFlowEightMeterRangeCycle) {
  ShaftMission::Config config;
  config.max_depth = 505.0;
  config.max_duration = 2400.0;
  ShaftMission mission(config);
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.range_min = 0.3;
  in.range_max = 8.0;
  in.dt = 0.05;
  constexpr double shaft_bottom = 500.0;
  double true_depth = 0.0;
  double flow_depth = 0.0;
  double deepest = 0.0;
  bool saw_saturated_range = false;
  bool saw_resolved_range = false;
  bool saw_return = false;
  for (int i = 0; i < 48000; ++i) {
    in.depth = flow_depth;
    const double remaining = shaft_bottom - true_depth;
    in.bottom_range = remaining <= in.range_max ? remaining : in.range_max;
    saw_saturated_range = saw_saturated_range || in.bottom_range == in.range_max;
    saw_resolved_range = saw_resolved_range || in.bottom_range < in.range_max;
    const auto out = mission.step(in);
    if (out.state == ShaftMission::State::kReturning) saw_return = true;
    if (out.state == ShaftMission::State::kComplete) {
      EXPECT_TRUE(saw_saturated_range);
      EXPECT_TRUE(saw_resolved_range);
      EXPECT_TRUE(saw_return);
      EXPECT_GT(deepest, 494.0);
      EXPECT_LT(deepest, 496.0);
      EXPECT_LE(std::abs(true_depth), 0.5);
      return;
    }
    ASSERT_NE(out.state, ShaftMission::State::kFault);
    const double depth_delta = -out.vertical_speed_enu * in.dt;
    true_depth += depth_delta;
    flow_depth += depth_delta;
    deepest = std::max(deepest, true_depth);
  }
  FAIL() << "500 m optical-flow shaft cycle did not complete";
}

TEST(ShaftMission, OpticalFlowBiasCanHideEntranceErrorWithoutAbsoluteZ) {
  ShaftMission::Config config;
  config.max_depth = 505.0;
  config.max_duration = 2400.0;
  ShaftMission mission(config);
  ShaftMission::Input in;
  in.enabled = true;
  in.depth_valid = true;
  in.range_fresh = true;
  in.range_min = 0.3;
  in.range_max = 8.0;
  in.dt = 0.05;
  constexpr double shaft_bottom = 500.0;
  constexpr double flow_bias_mps = 0.002;
  double true_depth = 0.0;
  double flow_depth = 0.0;
  bool saw_return = false;
  for (int i = 0; i < 48000; ++i) {
    in.depth = flow_depth;
    const double remaining = shaft_bottom - true_depth;
    in.bottom_range = remaining <= in.range_max ? remaining : in.range_max;
    const auto out = mission.step(in);
    if (out.state == ShaftMission::State::kReturning) saw_return = true;
    if (out.state == ShaftMission::State::kComplete) {
      EXPECT_TRUE(saw_return);
      EXPECT_LE(flow_depth, config.entrance_tolerance);
      EXPECT_LT(true_depth, -1.0)
          << "With no absolute Z source, a persistent optical-flow bias should "
             "remain physically unobservable to the mission state machine";
      return;
    }
    ASSERT_NE(out.state, ShaftMission::State::kFault);
    const double depth_delta = -out.vertical_speed_enu * in.dt;
    true_depth += depth_delta;
    flow_depth += depth_delta + flow_bias_mps * in.dt;
  }
  FAIL() << "biased 500 m optical-flow cycle did not reach a terminal state";
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
