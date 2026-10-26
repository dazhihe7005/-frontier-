#include <gtest/gtest.h>

#include "mine_uav_control/exploration_mission_profile.hpp"

namespace mine_uav_control {
namespace {

TEST(ExplorationMissionProfile, ParsesOnlyNamedOperationalProfiles) {
  ExplorationMissionProfile profile = ExplorationMissionProfile::kTunnel;
  EXPECT_TRUE(parseExplorationMissionProfile("goaf", &profile));
  EXPECT_EQ(ExplorationMissionProfile::kGoaf, profile);
  EXPECT_TRUE(parseExplorationMissionProfile("tunnel", &profile));
  EXPECT_EQ(ExplorationMissionProfile::kTunnel, profile);
  EXPECT_FALSE(parseExplorationMissionProfile("", &profile));
  EXPECT_FALSE(parseExplorationMissionProfile("rescan", &profile));
  EXPECT_FALSE(parseExplorationMissionProfile("tunnel", nullptr));
}

TEST(ExplorationMissionProfile, SharesNormalReturnAndIsolatesCorridorGuidance) {
  const auto goaf = explorationMissionCapabilities(
      ExplorationMissionProfile::kGoaf);
  const auto tunnel = explorationMissionCapabilities(
      ExplorationMissionProfile::kTunnel);

  EXPECT_FALSE(goaf.corridor_guidance);
  EXPECT_TRUE(tunnel.corridor_guidance);
  EXPECT_EQ(ExplorationReturnBehavior::kNormal, goaf.return_behavior);
  EXPECT_EQ(ExplorationReturnBehavior::kNormal, tunnel.return_behavior);
}

}  // namespace
}  // namespace mine_uav_control
