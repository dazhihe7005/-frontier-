#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "mine_uav_control/arc_trajectory.hpp"

namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

void expectNear(double actual, double expected, double tolerance = kEpsilon) {
  EXPECT_NEAR(actual, expected, tolerance);
}

}  // namespace

TEST(ArcTrajectoryTest, CounterClockwiseStartsAlongInitialYaw) {
  mine_uav_control::Pose2D start{0.0, 0.0, 2.0, 0.0};
  mine_uav_control::ArcConfig config;
  config.radius = 5.0;
  config.speed = 1.0;
  config.arc_angle_deg = 90.0;
  config.direction = 1;

  const mine_uav_control::ArcTrajectory trajectory(start, config);
  ASSERT_TRUE(trajectory.valid()) << trajectory.error();

  const auto initial = trajectory.sample(0.0);
  expectNear(initial.x, 0.0);
  expectNear(initial.y, 0.0);
  expectNear(initial.z, 2.0);
  expectNear(initial.vx, 1.0);
  expectNear(initial.vy, 0.0);
  expectNear(initial.vz, 0.0);
  expectNear(initial.yaw, 0.0);

  const auto midpoint = trajectory.sample(trajectory.duration() / 2.0);
  expectNear(midpoint.x, 5.0 * std::sin(kPi / 4.0));
  expectNear(midpoint.y, 5.0 * (1.0 - std::cos(kPi / 4.0)));
  expectNear(midpoint.z, 2.0);
  expectNear(midpoint.vx, std::cos(kPi / 4.0));
  expectNear(midpoint.vy, std::sin(kPi / 4.0));
  expectNear(midpoint.yaw, kPi / 4.0);
}

TEST(ArcTrajectoryTest, ClockwiseUsesRightHandCircleAndTangentVelocity) {
  mine_uav_control::Pose2D start{0.0, 0.0, 3.0, 0.0};
  mine_uav_control::ArcConfig config;
  config.radius = 5.0;
  config.speed = 2.0;
  config.arc_angle_deg = 90.0;
  config.direction = -1;

  const mine_uav_control::ArcTrajectory trajectory(start, config);
  ASSERT_TRUE(trajectory.valid()) << trajectory.error();

  const auto endpoint = trajectory.sample(trajectory.duration());
  expectNear(endpoint.x, 5.0);
  expectNear(endpoint.y, -5.0);
  expectNear(endpoint.z, 3.0);
  expectNear(endpoint.vx, 0.0);
  expectNear(endpoint.vy, 0.0);
  expectNear(endpoint.vz, 0.0);
  expectNear(endpoint.yaw, -kPi / 2.0);
  EXPECT_TRUE(endpoint.finished);
}

TEST(ArcTrajectoryTest, HoldsFinalPositionAndYawAfterArcCompletes) {
  mine_uav_control::Pose2D start{1.0, -2.0, 4.0, kPi - 0.1};
  mine_uav_control::ArcConfig config;
  config.radius = 2.0;
  config.speed = 1.0;
  config.arc_angle_deg = 180.0;
  config.direction = 1;

  const mine_uav_control::ArcTrajectory trajectory(start, config);
  ASSERT_TRUE(trajectory.valid()) << trajectory.error();

  const auto at_end = trajectory.sample(trajectory.duration());
  const auto after_end = trajectory.sample(trajectory.duration() + 100.0);
  expectNear(after_end.x, at_end.x);
  expectNear(after_end.y, at_end.y);
  expectNear(after_end.z, at_end.z);
  expectNear(after_end.vx, 0.0);
  expectNear(after_end.vy, 0.0);
  expectNear(after_end.vz, 0.0);
  expectNear(after_end.yaw, at_end.yaw);
  EXPECT_TRUE(after_end.finished);
  EXPECT_GE(after_end.yaw, -kPi);
  EXPECT_LE(after_end.yaw, kPi);
}

TEST(ArcTrajectoryTest, WrapsYawIntoMinusPiToPi) {
  expectNear(mine_uav_control::ArcTrajectory::wrapYaw(3.0 * kPi), kPi);
  expectNear(mine_uav_control::ArcTrajectory::wrapYaw(-3.0 * kPi), -kPi);
  EXPECT_GE(mine_uav_control::ArcTrajectory::wrapYaw(11.0), -kPi);
  EXPECT_LE(mine_uav_control::ArcTrajectory::wrapYaw(11.0), kPi);
}

TEST(ArcTrajectoryTest, RejectsInvalidConfiguration) {
  mine_uav_control::Pose2D start{0.0, 0.0, 0.0, 0.0};

  mine_uav_control::ArcConfig config;
  config.radius = 0.0;
  EXPECT_FALSE(mine_uav_control::ArcTrajectory(start, config).valid());

  config.radius = 1.0;
  config.speed = -1.0;
  EXPECT_FALSE(mine_uav_control::ArcTrajectory(start, config).valid());

  config.speed = 1.0;
  config.direction = 0;
  EXPECT_FALSE(mine_uav_control::ArcTrajectory(start, config).valid());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
