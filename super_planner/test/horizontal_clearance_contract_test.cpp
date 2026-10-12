#include <gtest/gtest.h>

#include <super_core/horizontal_clearance_contract.h>

namespace {

TEST(HorizontalClearanceContract, InsetsWallByExactHorizontalMargin) {
  Eigen::Vector4d plane(1.0, 0.0, 0.0, -2.0), inset;
  const Eigen::Vector3d seed(0.5, 0.0, 1.5);
  ASSERT_TRUE(super_planner::insetObstaclePlaneHorizontal(
      plane, 1.0, seed, seed, inset));
  EXPECT_NEAR(inset(3), -1.0, 1e-12);
  EXPECT_NEAR(inset.head<3>().dot(Eigen::Vector3d(1.0, 0.0, 1.5)) +
                  inset(3), 0.0, 1e-12);
}

TEST(HorizontalClearanceContract, DoesNotMoveVerticalObstaclePlane) {
  Eigen::Vector4d plane(0.0, 0.0, -1.0, 0.4), inset;
  const Eigen::Vector3d seed(0.0, 0.0, 1.5);
  ASSERT_TRUE(super_planner::insetObstaclePlaneHorizontal(
      plane, 1.0, seed, seed, inset));
  EXPECT_TRUE(inset.isApprox(plane));
}

TEST(HorizontalClearanceContract, InsetsObliquePlaneByHorizontalSupport) {
  Eigen::Vector4d plane(0.6, 0.0, 0.8, -2.0), inset;
  const Eigen::Vector3d seed(0.0, 0.0, 1.0);
  ASSERT_TRUE(super_planner::insetObstaclePlaneHorizontal(
      plane, 1.0, seed, seed, inset));
  EXPECT_NEAR(inset(3), -1.4, 1e-12);
}

TEST(HorizontalClearanceContract, RejectsSeedCutByRequiredMargin) {
  Eigen::Vector4d plane(1.0, 0.0, 0.0, -2.0), inset;
  EXPECT_FALSE(super_planner::insetObstaclePlaneHorizontal(
      plane, 1.0, Eigen::Vector3d(1.1, 0.0, 1.5),
      Eigen::Vector3d(0.5, 0.0, 1.5), inset));
}

}  // namespace
