#pragma once

#include <cmath>

namespace mine_uav_control {

// Conservative voxel-centre test for a horizontal wall stand-off and a
// separately bounded vertical body. Voxel half-widths cover discretization.
inline bool insideAnisotropicClearance(int dx, int dy, int dz,
                                       double resolution,
                                       double horizontal_radius,
                                       double vertical_clearance_below,
                                       double vertical_clearance_above) {
  const double xy = std::hypot(dx * resolution, dy * resolution);
  const double z = dz * resolution;
  return xy <= horizontal_radius + resolution * 0.7071067811865476 &&
         z >= -vertical_clearance_below - resolution * 0.5 &&
         z <= vertical_clearance_above + resolution * 0.5;
}

inline bool insideAnisotropicClearance(int dx, int dy, int dz,
                                       double resolution,
                                       double horizontal_radius,
                                       double vertical_radius) {
  return insideAnisotropicClearance(
      dx, dy, dz, resolution, horizontal_radius,
      vertical_radius, vertical_radius);
}

}  // namespace mine_uav_control
