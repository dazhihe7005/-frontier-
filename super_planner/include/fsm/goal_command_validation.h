#pragma once

#include <cmath>
#include <cstdint>

#include <super_planner/GoalCommand.h>

namespace fsm {

inline bool validateGoalCommand(const super_planner::GoalCommand& command,
                                uint64_t active_goal_id) {
  if (command.command == super_planner::GoalCommand::CANCEL_GOAL) {
    return command.goal_id == 0 || command.goal_id == active_goal_id;
  }
  if (command.command != super_planner::GoalCommand::SET_GOAL ||
      command.goal_id == 0) {
    return false;
  }
  const auto& p = command.goal.position;
  const auto& q = command.goal.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w)) {
    return false;
  }
  const double norm_squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return norm_squared > 1e-12;
}

}  // namespace fsm
