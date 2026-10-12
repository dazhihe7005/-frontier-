#include <traj_opt/guide_time_initialization.h>

#include <algorithm>
#include <cmath>

namespace traj_opt {

bool applyGeometricTimeLowerBound(const super_utils::Mat3Df &path,
                                  const double max_vel,
                                  super_utils::VecDf &times) {
    if (path.cols() != times.size() + 1) {
        return false;
    }
    if (!(max_vel > 0.0) || !std::isfinite(max_vel)
        || !path.allFinite() || !times.allFinite()
        || (times.array() < 0.0).any()) {
        return false;
    }

    for (Eigen::Index i = 0; i < times.size(); ++i) {
        const double distance = (path.col(i + 1) - path.col(i)).norm();
        const double velocity_limited_time = distance / max_vel;
        times(i) = std::max(times(i), velocity_limited_time);
    }
    return times.allFinite();
}

}  // namespace traj_opt
