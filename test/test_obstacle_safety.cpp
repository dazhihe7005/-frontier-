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

TEST(ObstacleSafety, DynamicClearanceMatchesPublishedEmergencyBrake) {
  EXPECT_NEAR(mine_uav_control::obstacleEmergencyBrakeDuration(
                  0.64, 1.75, 0.25),
              1.5 * 0.64 / 1.75, 1e-12);
  EXPECT_NEAR(mine_uav_control::obstacleEmergencyStoppingDistance(
                  0.64, 1.75, 0.25),
              0.5 * 0.64 * (1.5 * 0.64 / 1.75), 1e-12);
  EXPECT_NEAR(mine_uav_control::requiredDynamicObstacleClearance(
                  0.65, 0.8, 0.64, 0.64, 0.25, 1.75, 0.10, 0.25),
              0.65 + 0.10 + 0.64 * 0.25 +
                  0.75 * 0.64 * 0.64 / 1.75,
              1e-12);
}

TEST(ObstacleSafety, TonightSecondFlightWouldBrakeBeforeOldPointEightMetres) {
  const ObstaclePoint vehicle{0.0, 0.0, 0.5};
  const auto result =
      mine_uav_control::liveDynamicHorizontalCylinderClearance(
          {{1.08, 0.0, 0.5}}, vehicle,
          0.64, 0.0, 0.0, 0.72, 0.0, 0.0, true,
          0.65, 0.8, 1.7, 0.25, 1.75, 0.10, 0.25,
          0.5, 0.5, 0.0);
  EXPECT_TRUE(result.violated);
  EXPECT_GT(result.required_horizontal_clearance, 1.08);
  EXPECT_NEAR(result.closing_speed, 0.72, 1e-12);
  EXPECT_FALSE(mine_uav_control::liveHorizontalCylinderClearance(
      {{1.08, 0.0, 0.5}}, vehicle, 0.8, 0.5, 0.5, 0.0).violated);
}

TEST(ObstacleSafety, DynamicMonitorIgnoresExtraDistanceWhileMovingAway) {
  const ObstaclePoint vehicle{0.0, 0.0, 0.5};
  const auto result =
      mine_uav_control::liveDynamicHorizontalCylinderClearance(
          {{1.0, 0.0, 0.5}}, vehicle,
          -1.0, 0.0, 0.0, -0.8, 0.0, 0.0, true,
          0.65, 0.8, 1.7, 0.25, 1.75, 0.10, 0.25,
          0.5, 0.5, 0.0);
  EXPECT_FALSE(result.violated);
  EXPECT_NEAR(result.required_horizontal_clearance, 0.8, 1e-12);
  EXPECT_NEAR(result.closing_speed, 0.0, 1e-12);
}

TEST(ObstacleSafety, TangentialSpeedLengthensClosingStopBeforeWall) {
  const ObstaclePoint vehicle{0.0, 0.0, 0.5};
  const auto result =
      mine_uav_control::liveDynamicHorizontalCylinderClearance(
          {{0.85, 0.0, 0.5}}, vehicle,
          0.2, 1.0, 0.0, 0.2, 1.0, 0.0, true,
          0.65, 0.8, 1.7, 0.25, 1.75, 0.10, 0.25,
          0.5, 0.5, 0.0);
  ASSERT_TRUE(result.violated);
  EXPECT_NEAR(result.closing_speed, 0.2, 1e-12);
  EXPECT_GT(result.required_horizontal_clearance, 0.85);
  // A model that incorrectly used only the 0.2 m/s closing component for
  // brake duration would remain below this wall distance.
  EXPECT_LT(mine_uav_control::requiredDynamicObstacleClearance(
                0.65, 0.8, 0.2, 0.2, 0.25, 1.75, 0.10, 0.25),
            0.85);
}

TEST(ObstacleSafety, DynamicMonitorPrioritizesFartherWallInFlightDirection) {
  const ObstaclePoint vehicle{0.0, 0.0, 0.5};
  const auto result =
      mine_uav_control::liveDynamicHorizontalCylinderClearance(
          {{-0.7, 0.0, 0.5}, {1.2, 0.0, 0.5}}, vehicle,
          1.0, 0.0, 0.0, 1.0, 0.0, 0.0, true,
          0.65, 0.8, 1.7, 0.25, 1.75, 0.10, 0.25,
          0.5, 0.5, 0.0);
  ASSERT_TRUE(result.violated);
  EXPECT_NEAR(result.obstacle.x, 1.2, 1e-12);
  EXPECT_GT(result.risk_margin, 0.2);
}

TEST(ObstacleSafety, HardIntrusionAlwaysOverridesFartherDynamicThreat) {
  const ObstaclePoint vehicle{0.0, 0.0, 0.5};
  const auto result =
      mine_uav_control::liveDynamicHorizontalCylinderClearance(
          {{-0.50, 0.0, 0.5}, {1.1, 0.0, 0.5}}, vehicle,
          1.0, 0.0, 0.0, 1.0, 0.0, 0.0, true,
          0.65, 0.8, 1.7, 0.25, 1.75, 0.10, 0.25,
          0.5, 0.5, 0.0);
  ASSERT_TRUE(result.violated);
  EXPECT_TRUE(result.hard_clearance_violated);
  EXPECT_NEAR(result.obstacle.x, -0.50, 1e-12);
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

TEST(ObstacleSafety, ConfirmedNarrowSurfaceCanSurviveTonightPointDropout) {
  std::vector<ObstaclePoint> points;
  for (int x = 0; x <= 2; ++x) {
    for (int z = -8; z <= 8; ++z) {
      points.push_back({1.5 + 0.1 * x, 0.0, 0.05 * z});
    }
  }
  ASSERT_EQ(points.size(), 51u);
  const auto result = mine_uav_control::evaluateForwardObstacleCoverage(
      points, {0.0, 0.0, 0.0}, 1.0, 0.0, 2.5, 1.0, 0.7, 0.7,
      30, 0.5, 0.0, -10.0, 120, 0.2, 0.8,
      45, 0.1, 0.7);
  EXPECT_FALSE(result.narrow_vertical_surface_complete);
  EXPECT_TRUE(result.narrow_vertical_surface_retainable);
  EXPECT_FALSE(result.complete);
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
      mine_uav_control::hardIntrusionRequiresImmediateVerifiedReturn(false));
}

TEST(ObstacleSafety, FirstHardIntrusionRequestsVerifiedReturn) {
  EXPECT_TRUE(
      mine_uav_control::hardIntrusionRequiresImmediateVerifiedReturn(true));
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
  EXPECT_FALSE(verifiedReturnSoftZoneEscapeAllowed(
      true, true, true, false, vehicle, wall,
      away_target, -0.1, 0.1, 0.15));
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
