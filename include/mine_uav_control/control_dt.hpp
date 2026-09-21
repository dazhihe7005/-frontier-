#pragma once

#include <cmath>

namespace mine_uav_control {

// Preserve real callback timing whenever it is usable. ROS simulated time can
// occasionally deliver two timer callbacks with the same current_real stamp;
// in that single case use the timer's configured/expected period instead.
// Negative, non-finite, or long positive intervals are deliberately preserved
// so ShaftMission's fail-closed dt validation can reject them.
inline double task2ControlDt(double actual_dt, double expected_dt) {
  if (std::isfinite(actual_dt) && actual_dt > 1e-9) {
    return actual_dt;
  }
  if (std::isfinite(actual_dt) && actual_dt >= 0.0 && actual_dt <= 1e-9 &&
      std::isfinite(expected_dt) && expected_dt > 0.0) {
    return expected_dt;
  }
  return actual_dt;
}

}  // namespace mine_uav_control
