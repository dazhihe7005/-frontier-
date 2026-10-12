#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <data_structure/base/trajectory.h>

namespace super_planner {

inline double binomial(int n, int k) {
    double result = 1.0;
    for (int i = 1; i <= k; ++i) {
        result *= static_cast<double>(n - k + i) / i;
    }
    return result;
}

inline bool bernsteinInside(const std::vector<double>& controls,
                            double min_z, double max_z, int depth) {
    constexpr double kRoundoff = 1e-9;
    if (std::all_of(controls.begin(), controls.end(), [&](double value) {
            return std::isfinite(value) && value >= min_z - kRoundoff &&
                   value <= max_z + kRoundoff;
        })) return true;
    if (controls.front() < min_z - kRoundoff || controls.front() > max_z + kRoundoff ||
        controls.back() < min_z - kRoundoff || controls.back() > max_z + kRoundoff ||
        depth == 0) return false;

    // de Casteljau subdivision. Each child control hull still encloses the
    // entire polynomial over its half-interval; no time sampling is assumed.
    const int n = static_cast<int>(controls.size());
    std::vector<double> left(n), right(n), row = controls;
    left[0] = row[0];
    right[n - 1] = row[n - 1];
    for (int level = 1; level < n; ++level) {
        for (int i = 0; i < n - level; ++i) {
            row[i] = 0.5 * (row[i] + row[i + 1]);
        }
        left[level] = row[0];
        right[n - level - 1] = row[n - level - 1];
    }
    return bernsteinInside(left, min_z, max_z, depth - 1) &&
           bernsteinInside(right, min_z, max_z, depth - 1);
}

inline bool polynomialInsideFlightZ(const Eigen::MatrixXd& matrix,
                                    double duration, double min_z,
                                    double max_z) {
    const int n = static_cast<int>(matrix.cols()) - 1;
    if (!(std::isfinite(min_z) && std::isfinite(max_z) && min_z < max_z) ||
        matrix.rows() < 3 || n < 0 || n > 20 ||
        !std::isfinite(duration) || duration <= 0.0) return false;
    std::vector<double> controls(n + 1, 0.0);
    double time_power = 1.0;
    for (int j = 0; j <= n; ++j) {
        const double power_coefficient = matrix(2, n - j) * time_power;
        if (!std::isfinite(power_coefficient)) return false;
        for (int k = j; k <= n; ++k) {
            controls[k] += power_coefficient * binomial(k, j) /
                           binomial(n, j);
        }
        time_power *= duration;
    }
    if (!std::all_of(controls.begin(), controls.end(),
                     [](double value) { return std::isfinite(value); })) return false;
    return bernsteinInside(controls, min_z, max_z, 14);
}

inline bool trajectoryInsideFlightZ(const geometry_utils::Trajectory& trajectory,
                                    double min_z, double max_z) {
    if (trajectory.empty()) return false;
    for (const auto& piece : trajectory) {
        if (!polynomialInsideFlightZ(piece.getCoeffMat(), piece.getDuration(),
                                     min_z, max_z)) return false;
    }
    return true;
}

}  // namespace super_planner
