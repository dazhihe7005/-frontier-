#pragma once

#include <algorithm>
#include <cmath>

namespace mine_uav_control {

inline bool outboundProgressAllowed(double current_progress,
                                    double candidate_progress,
                                    double max_backtrack) {
  return std::isfinite(current_progress) && std::isfinite(candidate_progress) &&
         std::isfinite(max_backtrack) && max_backtrack >= 0.0 &&
         candidate_progress >= std::max(0.0, current_progress - max_backtrack);
}

}  // namespace mine_uav_control
