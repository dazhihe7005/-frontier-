#include <gtest/gtest.h>

#include "mine_uav_control/obstacle_safety.hpp"

namespace {

using mine_uav_control::ObstaclePoint;

TEST(ObstacleSafety, LiveCylinderDoesNotTradeHeightForHorizontalDistance) {
  const std::vector<ObstaclePoint> points{{0.6, 0.0, 0.45}};
  const auto result = mine_uav_control::liveHorizontalCylinderClearance(
      points, {0.0, 0.0, 0.0}, 0.8, 0.5);
  ASSERT_TRUE(result.violated);
  EXPECT_NEAR(result.horizontal_distance, 0.6, 1e-9);
  EXPECT_NEAR(result.vertical_distance, 0.45, 1e-9);
}

TEST(ObstacleSafety, GroundPlaneIsExcludedButObstacleTopStillStopsVehicle) {
  const ObstaclePoint vehicle{0.0, 0.0, 0.5};
  EXPECT_FALSE(mine_uav_control::liveHorizontalCylinderClearance(
      {{0.2, 0.0, -0.05}}, vehicle, 0.8, 0.5, 0.0).violated);
  EXPECT_TRUE(mine_uav_control::liveHorizontalCylinderClearance(
      {{0.6, 0.0, 0.05}}, vehicle, 0.8, 0.5, 0.0).violated);
}

TEST(ObstacleSafety, IncompleteSurfaceCannotQualifyCoverage) {
  const std::vector<ObstaclePoint> points{
      {1.0, 0.0, -0.1}, {1.0, 0.1, 0.0}, {1.0, 0.2, 0.1}};
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 2.0, 1.5,
      10, 0.6, 0.6);
  EXPECT_TRUE(result.obstacle_nearby);
  EXPECT_FALSE(result.complete);
}

TEST(ObstacleSafety, DenseSpanningSurfaceQualifiesCoverage) {
  std::vector<ObstaclePoint> points;
  for (int y = -4; y <= 4; ++y) {
    for (int z = -4; z <= 4; ++z) {
      points.push_back({1.5, 0.1 * y, 0.1 * z});
    }
  }
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 2.0, 1.5,
      30, 0.6, 0.6);
  EXPECT_TRUE(result.obstacle_nearby);
  EXPECT_TRUE(result.complete);
  EXPECT_GE(result.point_count, 30u);
}

}  // namespace
