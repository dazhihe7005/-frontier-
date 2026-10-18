#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mine_uav_control {

struct CorridorCenterEstimate {
  bool valid{false};
  double center_lateral{0.0};
  double width{0.0};
  double left_wall_lateral{0.0};
  double right_wall_lateral{0.0};
};

struct CorridorPreviewEstimate {
  bool valid{false};
  double target_lateral{0.0};
  double lookahead{0.0};
  double near_width{0.0};
  double minimum_width{0.0};
  double center_shift{0.0};
  std::size_t slices{0};
};

inline double median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) return upper;
  std::nth_element(values.begin(), values.begin() + middle - 1,
                   values.begin() + middle);
  return 0.5 * (values[middle - 1] + upper);
}

// Estimate the corridor medial axis from robust left/right wall slices. The
// two medians must straddle the aircraft, and explicit centering is enabled
// only for a physically passable but bounded-width corridor.
inline CorridorCenterEstimate estimateCorridorCenter(
    const std::vector<double>& left_wall_laterals,
    const std::vector<double>& right_wall_laterals,
    double vehicle_lateral,
    std::size_t minimum_points_per_wall,
    double minimum_width,
    double maximum_width) {
  CorridorCenterEstimate estimate;
  if (left_wall_laterals.size() < minimum_points_per_wall ||
      right_wall_laterals.size() < minimum_points_per_wall ||
      !std::isfinite(vehicle_lateral) || !std::isfinite(minimum_width) ||
      !std::isfinite(maximum_width) || minimum_width <= 0.0 ||
      maximum_width < minimum_width) {
    return estimate;
  }
  estimate.left_wall_lateral = median(left_wall_laterals);
  estimate.right_wall_lateral = median(right_wall_laterals);
  estimate.width =
      estimate.left_wall_lateral - estimate.right_wall_lateral;
  estimate.center_lateral = 0.5 *
      (estimate.left_wall_lateral + estimate.right_wall_lateral);
  estimate.valid = std::isfinite(estimate.left_wall_lateral) &&
      std::isfinite(estimate.right_wall_lateral) &&
      estimate.left_wall_lateral > vehicle_lateral &&
      estimate.right_wall_lateral < vehicle_lateral &&
      estimate.width >= minimum_width && estimate.width <= maximum_width;
  return estimate;
}

// Search from the filtered medial axis outward. This keeps the center a soft
// preference: all offsets inside the configured search band remain available
// when the centered segment is blocked.
inline std::vector<double> centeredLateralOffsets(
    double preferred_lateral, double search_half_width, double step) {
  std::vector<double> offsets;
  if (!std::isfinite(preferred_lateral) ||
      !std::isfinite(search_half_width) || search_half_width < 0.0 ||
      !std::isfinite(step) || step <= 0.0) {
    return offsets;
  }
  const double center = std::max(
      -search_half_width,
      std::min(preferred_lateral, search_half_width));
  const auto append_unique = [&offsets](double value) {
    for (const double existing : offsets) {
      if (std::abs(existing - value) <= 1e-9) return;
    }
    offsets.push_back(value);
  };
  append_unique(center);
  for (double delta = step;
       delta <= 2.0 * search_half_width + step + 1e-9;
       delta += step) {
    if (center + delta <= search_half_width + 1e-9) {
      append_unique(center + delta);
    }
    if (center - delta >= -search_half_width - 1e-9) {
      append_unique(center - delta);
    }
  }
  append_unique(search_half_width);
  append_unique(-search_half_width);
  return offsets;
}

// Find a steering-continuous route through longitudinal free-space slices.
// Unlike a single pair of wall medians, this retains where a passage narrows
// and where its centre moves. The selected cell at the furthest connected
// slice becomes an early lateral preference for the next short goal.
inline CorridorPreviewEstimate estimateCorridorPreview(
    const std::vector<std::vector<bool>>& passable,
    double first_forward, double forward_step,
    double lateral_min, double lateral_step,
    double current_lateral, double max_lateral_slope,
    std::size_t minimum_slices) {
  CorridorPreviewEstimate estimate;
  if (passable.empty() || passable.front().empty() ||
      !std::isfinite(first_forward) || first_forward <= 0.0 ||
      !std::isfinite(forward_step) || forward_step <= 0.0 ||
      !std::isfinite(lateral_min) ||
      !std::isfinite(lateral_step) || lateral_step <= 0.0 ||
      !std::isfinite(current_lateral) ||
      !std::isfinite(max_lateral_slope) || max_lateral_slope <= 0.0 ||
      minimum_slices == 0) {
    return estimate;
  }
  const std::size_t columns = passable.front().size();
  for (const auto& slice : passable) {
    if (slice.size() != columns) return estimate;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> previous_cost(columns, infinity);
  std::vector<double> current_cost(columns, infinity);
  std::vector<std::vector<int>> predecessor(
      passable.size(), std::vector<int>(columns, -1));
  std::vector<std::vector<double>> run_width(
      passable.size(), std::vector<double>(columns, 0.0));
  std::vector<std::vector<double>> edge_clearance(
      passable.size(), std::vector<double>(columns, 0.0));

  for (std::size_t row = 0; row < passable.size(); ++row) {
    std::size_t begin = 0;
    while (begin < columns) {
      if (!passable[row][begin]) {
        ++begin;
        continue;
      }
      std::size_t end = begin;
      while (end + 1 < columns && passable[row][end + 1]) ++end;
      const double width = (end - begin + 1) * lateral_step;
      for (std::size_t column = begin; column <= end; ++column) {
        run_width[row][column] = width;
        edge_clearance[row][column] = lateral_step *
            (0.5 + std::min(column - begin, end - column));
      }
      begin = end + 1;
    }
  }

  std::size_t reached_slices = 0;
  for (std::size_t row = 0; row < passable.size(); ++row) {
    std::fill(current_cost.begin(), current_cost.end(), infinity);
    const double allowed_change = max_lateral_slope *
        (row == 0 ? first_forward : forward_step) + 1e-9;
    bool row_reachable = false;
    for (std::size_t column = 0; column < columns; ++column) {
      if (!passable[row][column]) continue;
      const double lateral = lateral_min + column * lateral_step;
      double best_cost = infinity;
      int best_predecessor = -1;
      if (row == 0) {
        if (std::abs(lateral - current_lateral) <= allowed_change) {
          best_cost = std::abs(lateral - current_lateral);
        }
      } else {
        for (std::size_t prior = 0; prior < columns; ++prior) {
          if (!std::isfinite(previous_cost[prior])) continue;
          const double prior_lateral = lateral_min + prior * lateral_step;
          const double change = std::abs(lateral - prior_lateral);
          if (change > allowed_change) continue;
          const double candidate = previous_cost[prior] + change;
          if (candidate < best_cost) {
            best_cost = candidate;
            best_predecessor = static_cast<int>(prior);
          }
        }
      }
      if (!std::isfinite(best_cost)) continue;
      // Prefer the middle of a connected free run without making boundary
      // distance a hard constraint. Steering distance remains the main cost.
      current_cost[column] = best_cost - 0.15 * edge_clearance[row][column];
      predecessor[row][column] = best_predecessor;
      row_reachable = true;
    }
    if (!row_reachable) break;
    previous_cost.swap(current_cost);
    reached_slices = row + 1;
  }
  if (reached_slices < minimum_slices) return estimate;

  const std::size_t last_row = reached_slices - 1;
  std::size_t selected = 0;
  for (std::size_t column = 1; column < columns; ++column) {
    if (previous_cost[column] < previous_cost[selected]) selected = column;
  }
  if (!std::isfinite(previous_cost[selected])) return estimate;

  estimate.minimum_width = std::numeric_limits<double>::infinity();
  int column = static_cast<int>(selected);
  for (std::size_t reverse = reached_slices; reverse > 0; --reverse) {
    const std::size_t row = reverse - 1;
    if (column < 0) return CorridorPreviewEstimate{};
    const std::size_t path_column = static_cast<std::size_t>(column);
    if (row == 0) estimate.near_width = run_width[row][path_column];
    estimate.minimum_width = std::min(
        estimate.minimum_width, run_width[row][path_column]);
    column = predecessor[row][path_column];
  }
  estimate.valid = true;
  estimate.target_lateral = lateral_min + selected * lateral_step;
  estimate.lookahead = first_forward + last_row * forward_step;
  estimate.center_shift = estimate.target_lateral - current_lateral;
  estimate.slices = reached_slices;
  return estimate;
}

}  // namespace mine_uav_control
