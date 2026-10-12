#pragma once

#include <algorithm>
#include <cmath>

namespace traj_opt {

// L-BFGS can exhaust its Wolfe search after the large time and velocity
// gradients nearly cancel. Only a finite, nearly stationary candidate with
// a small speed violation may continue to the ordinary constraint checks.
inline bool acceptableLineSearchPlateau(double cost, double gradient_inf,
                                        double speed_squared_violation,
                                        double max_vel) {
    return std::isfinite(cost) && cost >= 0.0 &&
           std::isfinite(gradient_inf) && gradient_inf >= 0.0 &&
           std::isfinite(speed_squared_violation) &&
           std::isfinite(max_vel) && max_vel > 0.0 &&
           gradient_inf <= 1.0e-5 * std::max(1.0, cost) &&
           speed_squared_violation <= 0.05 * max_vel * max_vel;
}

}  // namespace traj_opt
