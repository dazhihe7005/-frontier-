#pragma once

#include <string>

namespace mine_uav_control {

// Mission identity and optional guidance are deliberately separate from the
// normal exploration/return state machine. Both profiles retain the captured
// entry heading, forward-priority frontiers, end-wall/map-closure checks,
// connected-turn continuation, and verified return-home behavior.
enum class ExplorationMissionProfile {
  kGoaf,
  kTunnel,
};

// The only implemented return behavior is the existing verified return home.
// A future "rescan while returning" policy belongs behind this boundary and
// must not be mixed into ordinary outbound exploration goal selection.
enum class ExplorationReturnBehavior {
  kNormal,
};

struct ExplorationMissionCapabilities {
  bool corridor_guidance{false};
  ExplorationReturnBehavior return_behavior{
      ExplorationReturnBehavior::kNormal};
};

inline bool parseExplorationMissionProfile(
    const std::string& value, ExplorationMissionProfile* profile) {
  if (profile == nullptr) return false;
  if (value == "goaf") {
    *profile = ExplorationMissionProfile::kGoaf;
    return true;
  }
  if (value == "tunnel") {
    *profile = ExplorationMissionProfile::kTunnel;
    return true;
  }
  return false;
}

inline const char* explorationMissionProfileName(
    ExplorationMissionProfile profile) {
  switch (profile) {
    case ExplorationMissionProfile::kTunnel:
      return "tunnel";
    case ExplorationMissionProfile::kGoaf:
    default:
      return "goaf";
  }
}

inline ExplorationMissionCapabilities explorationMissionCapabilities(
    ExplorationMissionProfile profile) {
  ExplorationMissionCapabilities capabilities;
  capabilities.corridor_guidance =
      profile == ExplorationMissionProfile::kTunnel;
  return capabilities;
}

}  // namespace mine_uav_control
