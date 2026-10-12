#pragma once

#include <cmath>

namespace mine_uav_control {

// Grid centres can lie half a voxel outside a physical height boundary.
// Apply that quantization margin to the configured observation range, not to
// the launch-height cruise target: the surveyed tunnel rises along its axis.
inline bool inObservationHeightBand(double point_z, double home_z,
                                    double min_height, double max_height,
                                    double voxel_resolution) {
  if (!std::isfinite(point_z) || !std::isfinite(home_z) ||
      !std::isfinite(min_height) || !std::isfinite(max_height) ||
      !std::isfinite(voxel_resolution) || voxel_resolution <= 0.0 ||
      max_height < min_height) {
    return false;
  }
  const double relative_height = point_z - home_z;
  const double margin = 0.5 * voxel_resolution;
  return relative_height >= min_height - margin &&
         relative_height <= max_height + margin;
}

}  // namespace mine_uav_control
