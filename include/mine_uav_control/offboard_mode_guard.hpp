#pragma once

#include <string>

namespace mine_uav_control {

// A transition away from an observed, managed OFFBOARD session is a pilot/PX4
// takeover unless this bridge itself requested the exit. Do not re-enter
// OFFBOARD until the automatic-enable switch has been reset.
inline bool externalOffboardExit(const std::string& previous_mode,
                                 const std::string& current_mode,
                                 const std::string& offboard_mode,
                                 bool offboard_owned,
                                 bool exit_requested) {
  return offboard_owned && !exit_requested &&
         previous_mode == offboard_mode && current_mode != offboard_mode;
}

}  // namespace mine_uav_control
