#include <gtest/gtest.h>

#include <limits>

#include <traj_opt/guide_time_initialization.h>

namespace {

using super_utils::Mat3Df;
using super_utils::VecDf;

TEST(GuideTimeInitialization,
     RaisesDegenerateNonzeroSegmentToVelocityBound) {
    Mat3Df path(3, 3);
    path.col(0) << 0.0, 0.0, 0.0;
    path.col(1) << 1.0, 0.0, 0.0;
    path.col(2) << 1.4, 0.0, 0.0;
    VecDf times(2);
    times << 1.0, 0.008;

    ASSERT_TRUE(traj_opt::applyGeometricTimeLowerBound(path, 1.0, times));

    EXPECT_DOUBLE_EQ(times(0), 1.0);
    EXPECT_NEAR(times(1), 0.4, 1.0e-12);
}

TEST(GuideTimeInitialization, PreservesLongerGuideDuration) {
    Mat3Df path(3, 2);
    path.col(0) << 0.0, 0.0, 0.0;
    path.col(1) << 0.4, 0.0, 0.0;
    VecDf times(1);
    times << 0.75;

    ASSERT_TRUE(traj_opt::applyGeometricTimeLowerBound(path, 1.0, times));

    EXPECT_DOUBLE_EQ(times(0), 0.75);
}

TEST(GuideTimeInitialization, DoesNotInflateZeroLengthSegment) {
    Mat3Df path(3, 2);
    path.col(0) << 2.0, -1.0, 0.5;
    path.col(1) = path.col(0);
    VecDf times(1);
    times << 0.008;

    ASSERT_TRUE(traj_opt::applyGeometricTimeLowerBound(path, 1.0, times));

    EXPECT_DOUBLE_EQ(times(0), 0.008);
}

TEST(GuideTimeInitialization, RejectsNonFinitePath) {
    Mat3Df path(3, 2);
    path.setZero();
    path(0, 1) = std::numeric_limits<double>::quiet_NaN();
    VecDf times(1);
    times << 0.5;

    EXPECT_FALSE(traj_opt::applyGeometricTimeLowerBound(path, 1.0, times));
}

TEST(GuideTimeInitialization, RejectsNonFiniteTime) {
    Mat3Df path(3, 2);
    path.setZero();
    VecDf times(1);
    times << std::numeric_limits<double>::infinity();

    EXPECT_FALSE(traj_opt::applyGeometricTimeLowerBound(path, 1.0, times));
}

TEST(GuideTimeInitialization, RejectsNegativeTime) {
    Mat3Df path(3, 2);
    path.setZero();
    path(0, 1) = 0.4;
    VecDf times(1);
    times << -0.1;

    EXPECT_FALSE(traj_opt::applyGeometricTimeLowerBound(path, 1.0, times));
}

TEST(GuideTimeInitialization, RejectsInvalidVelocityAndDimensions) {
    Mat3Df path(3, 2);
    path.setZero();
    VecDf times(1);
    times << 0.5;

    EXPECT_FALSE(traj_opt::applyGeometricTimeLowerBound(path, 0.0, times));

    Mat3Df mismatched_path(3, 3);
    mismatched_path.setZero();
    EXPECT_FALSE(traj_opt::applyGeometricTimeLowerBound(
            mismatched_path, 1.0, times));
}

}  // namespace

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
