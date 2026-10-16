#include "mine_uav_control/five_meter_avoidance_logic.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(FiveMeterAvoidanceLogic, BuildsGoalAlongCapturedHeading) {
  const auto east = mine_uav_control::makeForwardGoal(1.0, 2.0, 1.5, 0.0, 5.0);
  EXPECT_NEAR(east.x, 6.0, 1e-9);
  EXPECT_NEAR(east.y, 2.0, 1e-9);
  EXPECT_NEAR(east.z, 1.5, 1e-9);

  const auto north = mine_uav_control::makeForwardGoal(
      -2.0, 3.0, 0.8, kPi / 2.0, 5.0);
  EXPECT_NEAR(north.x, -2.0, 1e-9);
  EXPECT_NEAR(north.y, 8.0, 1e-9);
  EXPECT_NEAR(north.z, 0.8, 1e-9);
}

TEST(FiveMeterAvoidanceLogic, RequiresBothPositionAndLowSpeed) {
  const auto goal = mine_uav_control::makeForwardGoal(0.0, 0.0, 1.5, 0.0, 5.0);
  EXPECT_TRUE(mine_uav_control::goalSettled(
      goal, 4.8, 0.0, 1.5, 0.1, 0.0, 0.0, 0.35, 0.2));
  EXPECT_FALSE(mine_uav_control::goalSettled(
      goal, 4.8, 0.0, 1.5, 0.3, 0.0, 0.0, 0.35, 0.2));
  EXPECT_FALSE(mine_uav_control::goalSettled(
      goal, 4.5, 0.0, 1.5, 0.0, 0.0, 0.0, 0.35, 0.2));
}

TEST(FiveMeterAvoidanceLogic, RequiresStableHoverBeforeHandoff) {
  EXPECT_TRUE(mine_uav_control::hoverStable(0.08, 0.04, 0.10, 0.05));
  EXPECT_TRUE(mine_uav_control::hoverStable(0.10, -0.05, 0.10, 0.05));
  EXPECT_FALSE(mine_uav_control::hoverStable(0.11, 0.01, 0.10, 0.05));
  EXPECT_FALSE(mine_uav_control::hoverStable(0.08, 0.06, 0.10, 0.05));
}

TEST(FiveMeterAvoidanceLogic, NormalizesCapturedYaw) {
  const auto goal = mine_uav_control::makeForwardGoal(
      0.0, 0.0, 0.0, 5.0 * kPi, 1.0);
  EXPECT_NEAR(std::abs(goal.yaw), kPi, 1e-9);
  EXPECT_NEAR(goal.x, -1.0, 1e-9);
  EXPECT_NEAR(goal.y, 0.0, 1e-9);
}

TEST(FiveMeterAvoidanceLogic, PullsHorizontalGoalBackThroughPitchAlignment) {
  const auto alignment = mine_uav_control::quaternionFromRpy(
      0.0, 25.0 * kPi / 180.0, 0.0);
  const auto goal = mine_uav_control::makeOutputHorizontalForwardGoal(
      1.0, 2.0, 3.0, 0.0, 5.0, alignment);

  EXPECT_NEAR(goal.x, 1.0 + 5.0 * std::cos(25.0 * kPi / 180.0), 1e-9);
  EXPECT_NEAR(goal.y, 2.0, 1e-9);
  EXPECT_NEAR(goal.z, 3.0 + 5.0 * std::sin(25.0 * kPi / 180.0), 1e-9);

  const auto output_displacement = mine_uav_control::rotateVector(
      alignment, mine_uav_control::Vector3(
          goal.x - 1.0, goal.y - 2.0, goal.z - 3.0));
  EXPECT_NEAR(output_displacement.x, 5.0, 1e-9);
  EXPECT_NEAR(output_displacement.y, 0.0, 1e-9);
  EXPECT_NEAR(output_displacement.z, 0.0, 1e-9);
}

TEST(FiveMeterAvoidanceLogic, PreservesAlignedHorizontalHeading) {
  const auto alignment = mine_uav_control::multiply(
      mine_uav_control::quaternionFromRpy(0.0, 25.0 * kPi / 180.0, 0.0),
      mine_uav_control::quaternionFromRpy(0.0, 0.0, -0.4));
  const double raw_yaw = 0.7;
  const auto goal = mine_uav_control::makeOutputHorizontalForwardGoal(
      0.0, 0.0, 0.0, raw_yaw, 5.0, alignment);
  const auto output_displacement = mine_uav_control::rotateVector(
      alignment, mine_uav_control::Vector3(goal.x, goal.y, goal.z));
  const auto output_raw_heading = mine_uav_control::rotateVector(
      alignment, mine_uav_control::Vector3(
          std::cos(raw_yaw), std::sin(raw_yaw), 0.0));

  EXPECT_NEAR(std::hypot(output_displacement.x, output_displacement.y),
              5.0, 1e-9);
  EXPECT_NEAR(output_displacement.z, 0.0, 1e-9);
  EXPECT_NEAR(std::atan2(output_displacement.y, output_displacement.x),
              std::atan2(output_raw_heading.y, output_raw_heading.x), 1e-9);
  EXPECT_NEAR(goal.yaw, raw_yaw, 1e-9);
}

}  // namespace
