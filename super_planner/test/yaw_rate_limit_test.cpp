#include <gtest/gtest.h>

#include <cmath>

#include <traj_opt/yaw_traj_opt.h>

TEST(YawRateLimit, ClipsHalfTurnToFitShortPositionLeg) {
    geometry_utils::Trajectory position;
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 4);
    coefficients(0, 2) = 0.8;
    position.emplace_back(3.0, coefficients);

    super_utils::Vec4f initial = super_utils::Vec4f::Zero();
    super_utils::Vec4f requested = super_utils::Vec4f::Zero();
    requested(0) = M_PI;
    geometry_utils::Trajectory yaw;
    traj_opt::YawTrajOpt optimizer(0.6);

    ASSERT_TRUE(optimizer.optimize(initial, requested, position, yaw,
                                   3, false, false));
    EXPECT_LE(yaw.getMaxVelRate(), 0.6001);
    EXPECT_GT(yaw.getPos(3.0).x(), 0.0);
    EXPECT_LT(yaw.getPos(3.0).x(), M_PI);
}

TEST(YawRateLimit, VeryShortReturnLegStillProducesABoundedCommand) {
    geometry_utils::Trajectory position;
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 4);
    coefficients(0, 2) = 0.8;
    position.emplace_back(0.02, coefficients);
    super_utils::Vec4f initial = super_utils::Vec4f::Zero();
    super_utils::Vec4f requested = super_utils::Vec4f::Zero();
    requested(0) = M_PI;
    geometry_utils::Trajectory yaw;
    traj_opt::YawTrajOpt optimizer(0.6);
    ASSERT_TRUE(optimizer.optimize(initial, requested, position, yaw,
                                   3, false, false));
    EXPECT_LE(yaw.getMaxVelRate(), 0.6001);
    EXPECT_GE(yaw.getPos(0.02).x(), 0.0);
    EXPECT_LT(yaw.getPos(0.02).x(), M_PI);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
