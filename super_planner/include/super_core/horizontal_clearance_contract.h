#pragma once

#include <cmath>
#include <Eigen/Core>

namespace super_planner {

// Inset only an obstacle half-space.  The support function of a horizontal
// disk of radius extra is extra * ||n_xy||; vertical flight bounds are not
// contracted.  An infeasible seed is rejected before optimization.
inline bool insetObstaclePlaneHorizontal(
    const Eigen::Vector4d& plane, double extra,
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    Eigen::Vector4d& inset) {
    if (!plane.allFinite() || !start.allFinite() || !end.allFinite() ||
        !std::isfinite(extra) || extra < 0.0) return false;
    inset = plane;
    inset(3) += extra * plane.head<2>().norm();
    if (!inset.allFinite()) return false;
    return inset.head<3>().dot(start) + inset(3) <= 1e-8 &&
           inset.head<3>().dot(end) + inset(3) <= 1e-8;
}

}  // namespace super_planner
