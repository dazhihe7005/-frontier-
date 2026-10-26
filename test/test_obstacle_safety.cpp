#include <gtest/gtest.h>

#include "mine_uav_control/obstacle_safety.hpp"

namespace {

using mine_uav_control::ObstaclePoint;
using mine_uav_control::updateSurfaceCoverageReadiness;

TEST(ObstacleSafety, LiveCylinderDoesNotTradeHeightForHorizontalDistance) {
  const std::vector<ObstaclePoint> points{{0.6, 0.0, 0.25}};
  const auto result = mine_uav_control::liveHorizontalCylinderClearance(
      points, {0.0, 0.0, 0.0}, 0.8, 0.5, 0.5);
  ASSERT_TRUE(result.violated);
  EXPECT_NEAR(result.horizontal_distance, 0.6, 1e-9);
  EXPECT_NEAR(result.vertical_distance, 0.25, 1e-9);

  EXPECT_FALSE(mine_uav_control::liveHorizontalCylinderClearance(
      {{0.1, 0.0, 0.501}}, {0.0, 0.0, 0.0}, 0.8, 0.5, 0.5).violated);

  EXPECT_TRUE(mine_uav_control::liveHorizontalCylinderClearance(
      {{0.6, 0.0, -0.49}}, {0.0, 0.0, 0.0}, 0.8, 0.5, 0.5).violated);
  EXPECT_FALSE(mine_uav_control::liveHorizontalCylinderClearance(
      {{0.1, 0.0, -0.501}}, {0.0, 0.0, 0.0}, 0.8, 0.5, 0.5).violated);
}

TEST(ObstacleSafety, GroundPlaneIsExcludedButObstacleTopStillStopsVehicle) {
  const ObstaclePoint vehicle{0.0, 0.0, 0.5};
  EXPECT_FALSE(mine_uav_control::liveHorizontalCylinderClearance(
      {{0.2, 0.0, -0.05}}, vehicle, 0.8, 0.5, 0.5, 0.0).violated);
  EXPECT_TRUE(mine_uav_control::liveHorizontalCylinderClearance(
      {{0.6, 0.0, 0.20}}, vehicle, 0.8, 0.5, 0.5, 0.0).violated);
}

TEST(ObstacleSafety, OneFrameSpeckleNeverBecomesPersistentObstacle) {
  mine_uav_control::PersistentObstacleMap map;
  map.configure(0.1, 100, 2, 3);

  // Multiple points in the same voxel and same packet count as one scan.
  map.observeFrame({{1.01, 0.01, 0.01}, {1.02, 0.02, 0.02}});
  EXPECT_TRUE(map.confirmedPoints().empty());
  map.observeFrame({});
  map.observeFrame({});
  map.observeFrame({});
  EXPECT_TRUE(map.confirmedPoints().empty());
}

TEST(ObstacleSafety, RepeatedSurfacePersistsThroughOcclusion) {
  mine_uav_control::PersistentObstacleMap map;
  map.configure(0.1, 100, 2, 3);

  map.observeFrame({{1.01, 0.01, 0.01}});
  map.observeFrame({{1.04, 0.04, 0.04}});
  ASSERT_EQ(map.confirmedSize(), 1u);
  ASSERT_EQ(map.confirmedPoints().size(), 1u);

  for (int frame = 0; frame < 10; ++frame) map.observeFrame({});
  EXPECT_EQ(map.confirmedSize(), 1u);
}

TEST(ObstacleSafety, HitsOutsideWindowDoNotConfirmNoise) {
  mine_uav_control::PersistentObstacleMap map;
  map.configure(0.1, 100, 2, 3);

  map.observeFrame({{1.01, 0.01, 0.01}});
  map.observeFrame({});
  map.observeFrame({});
  map.observeFrame({{1.01, 0.01, 0.01}});
  EXPECT_TRUE(map.confirmedPoints().empty());
}

TEST(ObstacleSafety, IncompleteSurfaceCannotQualifyCoverage) {
  const std::vector<ObstaclePoint> points{
      {1.0, 0.0, -0.1}, {1.0, 0.1, 0.0}, {1.0, 0.2, 0.1}};
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 2.0, 0.7, 0.7,
      10, 0.6, 0.0);
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
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 2.0, 0.7, 0.7,
      30, 0.6, 0.0);
  EXPECT_TRUE(result.obstacle_nearby);
  EXPECT_TRUE(result.complete);
  EXPECT_GE(result.point_count, 30u);
}

TEST(ObstacleSafety, FlightSizedPanelQualifiesCoverageAndKeepsNearestPoint) {
  std::vector<ObstaclePoint> points;
  for (int y = -3; y <= 3; ++y) {
    for (int z = -3; z <= 3; ++z) {
      points.push_back({1.0, 0.1 * y, 0.1 * z});
    }
  }
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 2.0, 0.7, 0.7,
      20, 0.5, 0.5);
  EXPECT_TRUE(result.obstacle_nearby);
  EXPECT_TRUE(result.complete);
  EXPECT_NEAR(result.horizontal_span, 0.6, 1e-9);
  EXPECT_NEAR(result.vertical_span, 0.6, 1e-9);
  EXPECT_NEAR(result.nearest_horizontal, 1.0, 1e-9);
  EXPECT_NEAR(result.nearest_obstacle.x, 1.0, 1e-9);
}

TEST(ObstacleSafety, DenseNarrowVerticalStructureQualifiesCoverage) {
  std::vector<ObstaclePoint> points;
  for (int x = 0; x <= 3; ++x) {
    for (int z = -20; z <= 20; ++z) {
      points.push_back({1.5 + 0.1 * x, 0.0, 0.03 * z});
    }
  }
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 1.0, 0.7, 0.7,
      30, 0.5, 0.0, -10.0, 120, 0.2, 0.8);
  EXPECT_FALSE(result.broad_surface_complete);
  EXPECT_TRUE(result.narrow_vertical_surface_complete);
  EXPECT_TRUE(result.complete);
}

TEST(ObstacleSafety, SparseNarrowVerticalStructureRemainsIncomplete) {
  std::vector<ObstaclePoint> points;
  for (int z = -20; z <= 20; ++z) {
    points.push_back({1.5, 0.0, 0.03 * z});
  }
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 1.0, 0.7, 0.7,
      30, 0.5, 0.0, -10.0, 120, 0.2, 0.8);
  EXPECT_FALSE(result.narrow_vertical_surface_complete);
  EXPECT_FALSE(result.complete);
}

TEST(ObstacleSafety, DenseShortStripRemainsIncomplete) {
  std::vector<ObstaclePoint> points;
  for (int x = 0; x <= 3; ++x) {
    for (int z = -5; z <= 5; ++z) {
      for (int repeat = 0; repeat < 4; ++repeat) {
        points.push_back({1.5 + 0.1 * x, 0.0, 0.03 * z});
      }
    }
  }
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 1.0, 0.7, 0.7,
      30, 0.5, 0.0, -10.0, 120, 0.2, 0.8);
  EXPECT_FALSE(result.narrow_vertical_surface_complete);
  EXPECT_FALSE(result.complete);
}

TEST(ObstacleSafety, CeilingOutsideRelativeSafetyBandIsNotCoverage) {
  std::vector<ObstaclePoint> ceiling;
  for (int x = 1; x <= 10; ++x) {
    for (int y = -4; y <= 4; ++y) {
      ceiling.push_back({0.2 * x, 0.1 * y, 1.4});
    }
  }
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      ceiling, {0.0, 0.0, 0.5}, 1.0, 0.0, 2.5, 2.0, 0.7, 0.7,
      30, 0.6, 0.0);
  EXPECT_FALSE(result.obstacle_nearby);
  EXPECT_FALSE(result.complete);
}

TEST(ObstacleSafety, LowHorizontalSurfaceDoesNotNeedVerticalSpan) {
  std::vector<ObstaclePoint> low_surface;
  for (int x = 5; x <= 15; ++x) {
    for (int y = -4; y <= 4; ++y) {
      low_surface.push_back({0.1 * x, 0.1 * y, 0.2});
    }
  }
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      low_surface, {0.0, 0.0, 0.5}, 1.0, 0.0, 2.5, 2.0, 0.7, 0.7,
      30, 0.6, 0.0);
  EXPECT_TRUE(result.obstacle_nearby);
  EXPECT_TRUE(result.complete);
  EXPECT_NEAR(result.vertical_span, 0.0, 1e-9);
}

TEST(ObstacleSafety, BriefObstacleHoldsThenClearsWithoutFault) {
  mine_uav_control::TransientObstacleMonitor monitor;
  monitor.configure(3.0, 3);

  EXPECT_EQ(monitor.update(true, 10.0),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(true, 11.0),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(false, 11.1),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(false, 11.2),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(false, 11.3),
            mine_uav_control::ObstacleMonitorAction::kClear);
  EXPECT_FALSE(monitor.active());
}

TEST(ObstacleSafety, ContinuousObstacleBecomesPersistentAtThreeSeconds) {
  mine_uav_control::TransientObstacleMonitor monitor;
  monitor.configure(3.0, 3);

  EXPECT_EQ(monitor.update(true, 20.0),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(true, 22.99),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(true, 23.0),
            mine_uav_control::ObstacleMonitorAction::kPersistent);
  EXPECT_TRUE(monitor.active());
}

TEST(ObstacleSafety, PersistentPlanningZoneObstacleDoesNotExitOffboard) {
  EXPECT_FALSE(
      mine_uav_control::persistentHardIntrusionRequiresVerifiedReturn(
          mine_uav_control::ObstacleMonitorAction::kPersistent, false));
  EXPECT_FALSE(
      mine_uav_control::persistentHardIntrusionRequiresVerifiedReturn(
          mine_uav_control::ObstacleMonitorAction::kHold, true));
}

TEST(ObstacleSafety, PersistentHardIntrusionRequestsVerifiedReturn) {
  EXPECT_TRUE(
      mine_uav_control::persistentHardIntrusionRequiresVerifiedReturn(
          mine_uav_control::ObstacleMonitorAction::kPersistent, true));
}

TEST(ObstacleSafety, EveryPersistentReplanMustMoveAwayFromObstacle) {
  EXPECT_TRUE(
      mine_uav_control::persistentObstacleReplanRequiresAwayMotion(
          mine_uav_control::ObstacleMonitorAction::kPersistent));
  EXPECT_FALSE(
      mine_uav_control::persistentObstacleReplanRequiresAwayMotion(
          mine_uav_control::ObstacleMonitorAction::kHold));
  EXPECT_FALSE(
      mine_uav_control::persistentObstacleReplanRequiresAwayMotion(
          mine_uav_control::ObstacleMonitorAction::kClear));
}

TEST(ObstacleSafety, HardObstacleEscapeMustIncreaseClearance) {
  const mine_uav_control::ObstaclePoint vehicle{0.0, 0.0, 1.0};
  const mine_uav_control::ObstaclePoint obstacle{0.5, 0.0, 1.0};
  EXPECT_TRUE(mine_uav_control::plannedMotionIncreasesHorizontalClearance(
      vehicle, obstacle, {-0.1, 0.0, 1.0}, -0.1, 0.0));
  EXPECT_FALSE(mine_uav_control::plannedMotionIncreasesHorizontalClearance(
      vehicle, obstacle, {0.1, 0.0, 1.0}, 0.1, 0.0));
  EXPECT_FALSE(mine_uav_control::plannedMotionIncreasesHorizontalClearance(
      vehicle, obstacle, {0.0, 0.0, 1.0}, 0.0, 0.0));
}

TEST(ObstacleSafety, VerifiedReturnMayLeaveSoftZoneOnlyWhileMovingAway) {
  const ObstaclePoint vehicle{0.0, 0.0, 1.0};
  const ObstaclePoint wall{0.0, -0.79, 1.0};
  const ObstaclePoint away_target{-0.2, 0.2, 1.0};
  EXPECT_TRUE(verifiedReturnSoftZoneEscapeAllowed(
      true, true, true, false, vehicle, wall, away_target, -0.1, 0.1));
  EXPECT_FALSE(verifiedReturnSoftZoneEscapeAllowed(
      true, true, true, true, vehicle, wall, away_target, -0.1, 0.1));
  EXPECT_FALSE(verifiedReturnSoftZoneEscapeAllowed(
      false, true, true, false, vehicle, wall, away_target, -0.1, 0.1));
  EXPECT_FALSE(verifiedReturnSoftZoneEscapeAllowed(
      true, true, true, false, vehicle, wall,
      {0.0, -0.2, 1.0}, 0.0, -0.1));
}

TEST(ObstacleSafety, SurfaceCoverageReadinessHasBidirectionalHysteresis) {
  int complete_streak = 0;
  int incomplete_streak = 0;
  bool ready = true;
  EXPECT_TRUE(updateSurfaceCoverageReadiness(
      true, false, 3, 2, &complete_streak, &incomplete_streak, &ready));
  EXPECT_FALSE(updateSurfaceCoverageReadiness(
      true, false, 3, 2, &complete_streak, &incomplete_streak, &ready));
  EXPECT_FALSE(updateSurfaceCoverageReadiness(
      true, true, 3, 2, &complete_streak, &incomplete_streak, &ready));
  EXPECT_FALSE(updateSurfaceCoverageReadiness(
      true, true, 3, 2, &complete_streak, &incomplete_streak, &ready));
  EXPECT_TRUE(updateSurfaceCoverageReadiness(
      true, true, 3, 2, &complete_streak, &incomplete_streak, &ready));
  EXPECT_TRUE(updateSurfaceCoverageReadiness(
      true, false, 3, 2, &complete_streak, &incomplete_streak, &ready));
  EXPECT_FALSE(updateSurfaceCoverageReadiness(
      true, false, 3, 2, &complete_streak, &incomplete_streak, &ready));
  EXPECT_TRUE(updateSurfaceCoverageReadiness(
      false, false, 3, 2, &complete_streak, &incomplete_streak, &ready));
}

TEST(ObstacleSafety, SparseClearPacketDoesNotReleaseHold) {
  mine_uav_control::TransientObstacleMonitor monitor;
  monitor.configure(3.0, 2);

  EXPECT_EQ(monitor.update(true, 30.0),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(false, 30.1),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(true, 30.2),
            mine_uav_control::ObstacleMonitorAction::kHold);
  EXPECT_EQ(monitor.update(true, 33.0),
            mine_uav_control::ObstacleMonitorAction::kPersistent);
}

}  // namespace
