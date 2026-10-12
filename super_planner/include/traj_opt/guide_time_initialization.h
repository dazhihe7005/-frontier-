#pragma once

#include <utils/header/type_utils.hpp>

namespace traj_opt {

bool applyGeometricTimeLowerBound(const super_utils::Mat3Df &path,
                                  double max_vel,
                                  super_utils::VecDf &times);

}  // namespace traj_opt
