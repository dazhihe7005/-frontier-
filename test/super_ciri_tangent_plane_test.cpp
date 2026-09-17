#include <gtest/gtest.h>

#include <super_core/ciri.h>

namespace super_planner {

class CiriTangentPlaneTestPeer {
 public:
  static bool tangent(const Eigen::Vector3d& obstacle,
                      double radius,
                      const Eigen::Vector3d& endpoint,
                      const Eigen::Vector3d& ellipsoid_center,
                      Eigen::Vector4d& plane) {
    return CIRI::findTangentPlaneOfSphere(obstacle, radius, endpoint,
                                         ellipsoid_center, plane);
  }
};

TEST(CiriTangentPlane, VerticalTangencyWithOffsetSeedIsFinite) {
  // Reproduced from the maze SITL nonfinite diagnostic at sim t=18.024 s.
  const Eigen::Vector3d obstacle(6.7, -0.1, 1.1);
  const Eigen::Vector3d endpoint(6.7, -0.1, 1.5);
  const Eigen::Vector3d ellipsoid_center(6.3, -0.1, 1.5);
  Eigen::Vector4d plane;
  ASSERT_TRUE(CiriTangentPlaneTestPeer::tangent(
      obstacle, 0.4, endpoint, ellipsoid_center, plane));
  EXPECT_TRUE(plane.allFinite());
  EXPECT_NEAR(plane.head<3>().norm(), 1.0, 1e-9);
  EXPECT_NEAR(plane.head<3>().dot(endpoint) + plane(3), 0.0, 1e-8);
  EXPECT_GE(plane.head<3>().dot(obstacle) + plane(3), 0.4 - 1e-8);
  EXPECT_LE(plane.head<3>().dot(ellipsoid_center) + plane(3), 1e-8);
}

TEST(CiriTangentPlane, VerticalTangencyWithCoincidentSeedIsFinite) {
  const Eigen::Vector3d obstacle(0.0, 0.0, 0.0);
  const Eigen::Vector3d endpoint(0.0, 0.0, 0.4);
  Eigen::Vector4d plane;
  ASSERT_TRUE(CiriTangentPlaneTestPeer::tangent(
      obstacle, 0.4, endpoint, endpoint, plane));
  EXPECT_TRUE(plane.allFinite());
}

TEST(CiriTangentPlane, RejectsEndpointInsideObstacleSphere) {
  const Eigen::Vector3d obstacle(0.0, 0.0, 0.0);
  const Eigen::Vector3d endpoint(0.0, 0.0, 0.35);
  const Eigen::Vector3d seed(0.2, 0.0, 0.35);
  Eigen::Vector4d plane;
  EXPECT_FALSE(CiriTangentPlaneTestPeer::tangent(
      obstacle, 0.4, endpoint, seed, plane));
}

}  // namespace super_planner
