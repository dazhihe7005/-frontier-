#include <gtest/gtest.h>

#include <limits>

#include <traj_opt/line_search_plateau.h>

TEST(LineSearchPlateau, AcceptsFiniteNearStationaryCandidate) {
    EXPECT_TRUE(traj_opt::acceptableLineSearchPlateau(
        35053.5, 0.0066, 0.0128, 0.8));
    EXPECT_TRUE(traj_opt::acceptableLineSearchPlateau(
        126491.8, 0.0918, 0.0024, 0.8));
}

TEST(LineSearchPlateau, RejectsInaccurateOrUnsafeCandidate) {
    EXPECT_FALSE(traj_opt::acceptableLineSearchPlateau(
        35053.5, 1.0, 0.0128, 0.8));
    EXPECT_FALSE(traj_opt::acceptableLineSearchPlateau(
        35053.5, 0.0066, 0.1, 0.8));
    EXPECT_FALSE(traj_opt::acceptableLineSearchPlateau(
        std::numeric_limits<double>::infinity(), 0.0066, 0.0128, 0.8));
    EXPECT_FALSE(traj_opt::acceptableLineSearchPlateau(
        35053.5, std::numeric_limits<double>::quiet_NaN(), 0.0128, 0.8));
}
