#!/usr/bin/env python3
"""Audit vertical command continuity, full SUPER polynomials and hold drift.

This is a diagnostic for task-one SITL bags, not a flight-safety certificate.
PositionCommand samples cannot prove behavior between samples. Record
/planning_cmd/poly_traj to additionally inspect every committed polynomial.
"""

import argparse
import bisect
import json
import math

import numpy as np
import rosbag


def polynomial_extrema(coefficients, duration):
    """Return exact-in-polynomial candidate min/max from ends and dz/dt roots.

    SUPER Piece::getPos evaluates the stored coefficients in descending order.
    Roots are obtained numerically, so near-real and boundary roots use a small
    tolerance. Invalid pieces are rejected rather than silently skipped.
    """
    if not coefficients or not math.isfinite(duration) or duration <= 0:
        raise ValueError("invalid polynomial piece")
    coefficients = np.asarray(coefficients, dtype=float)
    if not np.isfinite(coefficients).all():
        raise ValueError("non-finite polynomial coefficient")
    candidates = [0.0, duration]
    derivative = np.polyder(coefficients)
    for root in np.roots(derivative):
        if abs(root.imag) <= 1e-7 * max(1.0, abs(root.real)):
            time = float(root.real)
            if -1e-8 <= time <= duration + 1e-8:
                candidates.append(min(duration, max(0.0, time)))
    values = [(float(np.polyval(coefficients, t)), t) for t in candidates]
    return min(values), max(values)


def inspect_polynomial(message, alignment_z=None,
                       min_height=-2.0, max_height=1.8):
    degree = message.order_pos
    piece_count = message.piece_num_pos
    if (piece_count <= 0 or len(message.time_pos) != piece_count or
            len(message.coef_pos_z) != piece_count * (degree + 1)):
        raise ValueError("inconsistent polynomial message lengths")
    minimum = (math.inf, None)
    maximum = (-math.inf, None)
    max_piece_step = 0.0
    previous_end = None
    offset = 0.0
    height_violations = 0
    for index, duration in enumerate(message.time_pos):
        start = index * (degree + 1)
        coefficients = message.coef_pos_z[start:start + degree + 1]
        low, high = polynomial_extrema(coefficients, duration)
        if low[0] < minimum[0]:
            minimum = (low[0], offset + low[1])
        if high[0] > maximum[0]:
            maximum = (high[0], offset + high[1])
        piece_start = float(np.polyval(coefficients, 0.0))
        piece_end = float(np.polyval(coefficients, duration))
        if previous_end is not None:
            max_piece_step = max(max_piece_step, abs(piece_start - previous_end))
        previous_end = piece_end
        offset += duration
        if alignment_z is not None and (
                low[0] + alignment_z < min_height or
                high[0] + alignment_z > max_height):
            height_violations += 1
    return minimum, maximum, max_piece_step, height_violations


def nearest_pose(poses, times, time):
    if not poses:
        return None
    index = bisect.bisect_left(times, time)
    candidates = poses[max(0, index - 1):min(len(poses), index + 1)]
    return min(candidates, key=lambda pose: abs(pose[0] - time))


def audit(path, jump_threshold, alignment_z, min_height, max_height):
    commands, poses, polynomials, statuses, goal_heights = [], [], [], [], []
    with rosbag.Bag(path) as bag:
        for topic, message, receipt in bag.read_messages(topics=[
            "/planning/pos_cmd", "/planning_cmd/poly_traj",
            "/mavros/local_position/pose", "/mine_uav/exploration/status",
            "/mine_uav/super/goal_command",
        ]):
            receipt_time = receipt.to_sec()
            header = getattr(message, "header", None)
            stamp = (header.stamp.to_sec() if header is not None and
                     header.stamp.to_sec() > 0 else receipt_time)
            if topic == "/planning/pos_cmd" and message.trajectory_flag in (1, 2):
                commands.append((stamp, message.position.z,
                                 message.velocity.z, message.acceleration.z,
                                 receipt_time))
            elif topic.endswith("/pose"):
                poses.append((stamp, message.pose.position.x,
                              message.pose.position.y, message.pose.position.z))
            elif topic.endswith("poly_traj") and message.type & 2:
                polynomials.append((stamp, message))
            elif topic.endswith("/status"):
                statuses.append((receipt_time, message.data))
            elif topic.endswith("goal_command") and message.command == 1:
                goal_heights.append(message.goal.position.z)

    sample_min = min(commands, key=lambda item: item[1]) if commands else None
    sample_max = max(commands, key=lambda item: item[1]) if commands else None
    valid_steps = []
    residual_steps = []
    gaps = []
    pose_times = [pose[0] for pose in poses]
    status_times = [status[0] for status in statuses]
    for previous, current in zip(commands, commands[1:]):
        dt = current[0] - previous[0]
        if dt > 0.1:
            before = nearest_pose(poses, pose_times, previous[0])
            after = nearest_pose(poses, pose_times, current[0])
            phase_index = bisect.bisect_right(status_times, previous[4]) - 1
            gaps.append({
                "start_s": previous[0], "end_s": current[0], "duration_s": dt,
                "status_at_start": statuses[phase_index][1] if phase_index >= 0 else None,
                "actual_dx_m": after[1] - before[1] if before and after else None,
                "actual_dz_m": after[3] - before[3] if before and after else None,
                "actual_z_start_m": before[3] if before else None,
                "actual_z_end_m": after[3] if after else None,
            })
        if 0 < dt <= 0.06:
            valid_steps.append((abs(current[1] - previous[1]), current[0], dt))
            expected_dz = 0.5 * (previous[2] + current[2]) * dt
            residual_steps.append((abs(current[1] - previous[1] - expected_dz),
                                   current[0], dt))

    polynomial_min = (math.inf, None)
    polynomial_max = (-math.inf, None)
    max_piece_step = 0.0
    max_trajectory_z_span = 0.0
    poly_violating_pieces = 0
    malformed = 0
    labels = set()
    unique_violations = {}
    distinct_polynomial_times = []
    for receipt, message in polynomials:
        labels.add(message.header.frame_id)
        try:
            low, high, step, violations = inspect_polynomial(
                message, alignment_z, min_height, max_height)
        except ValueError:
            malformed += 1
            continue
        if low[0] < polynomial_min[0]:
            polynomial_min = (low[0], message.start_WT_pos.to_sec() + low[1])
        if high[0] > polynomial_max[0]:
            polynomial_max = (high[0], message.start_WT_pos.to_sec() + high[1])
        max_piece_step = max(max_piece_step, step)
        max_trajectory_z_span = max(max_trajectory_z_span, high[0] - low[0])
        poly_violating_pieces += violations
        start = message.start_WT_pos.to_sec()
        if not distinct_polynomial_times or start != distinct_polynomial_times[-1][0]:
            distinct_polynomial_times.append((start, receipt))
        if alignment_z is not None and violations:
            unique_violations.setdefault(start, {
                "published_at_s": receipt,
                "planned_peak_z_m": high[0],
                "planned_peak_time_s": start + high[1],
                "planned_trough_z_m": low[0],
                "planned_trough_time_s": start + low[1],
            })

    for event in unique_violations.values():
        index = bisect.bisect_right(
            [item[1] for item in distinct_polynomial_times],
            event["published_at_s"])
        event["next_distinct_polynomial_at_s"] = (
            distinct_polynomial_times[index][1]
            if index < len(distinct_polynomial_times) else None)
        next_time = event["next_distinct_polynomial_at_s"]
        event["peak_preempted_before_time"] = (
            next_time is not None and next_time < event["planned_peak_time_s"])

    return {
        "bag": path,
        "height_units": "m; planner camera_init, PX4 local only after alignment",
        "alignment_z_m": alignment_z,
        "px4_local_height_fence_m": [min_height, max_height]
        if alignment_z is not None else None,
        "sampled_command_count": len(commands),
        "sampled_min_z_m": sample_min[1] if sample_min else None,
        "sampled_max_z_m": sample_max[1] if sample_max else None,
        "max_adjacent_z_step_m": max(valid_steps, default=(0.0, None, None)),
        "adjacent_z_steps_over_threshold": sum(
            step[0] > jump_threshold for step in valid_steps),
        "max_adjacent_z_kinematic_residual_m": max(
            residual_steps, default=(0.0, None, None)),
        "adjacent_z_residuals_over_threshold": sum(
            step[0] > jump_threshold for step in residual_steps),
        "max_command_abs_vz_m_s": max((abs(c[2]) for c in commands), default=None),
        "max_command_abs_az_m_s2": max((abs(c[3]) for c in commands), default=None),
        "set_goal_z_range_m": [min(goal_heights), max(goal_heights)]
        if goal_heights else None,
        "command_gaps": gaps,
        "committed_polynomial_count": len(polynomials),
        "malformed_polynomial_count": malformed,
        "polynomial_frame_labels": sorted(labels),
        "polynomial_min_z_m_and_time": polynomial_min if polynomials and
        malformed < len(polynomials) else None,
        "polynomial_max_z_m_and_time": polynomial_max if polynomials and
        malformed < len(polynomials) else None,
        "max_internal_piece_boundary_z_step_m": max_piece_step if polynomials else None,
        "max_one_committed_trajectory_z_span_m": max_trajectory_z_span
        if polynomials else None,
        "polynomial_pieces_outside_px4_z_fence": poly_violating_pieces
        if alignment_z is not None and polynomials else None,
        "unique_polynomial_fence_violations": list(unique_violations.values())
        if alignment_z is not None and polynomials else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--jump-threshold-m", type=float, default=0.05,
                        help="Diagnostic step threshold, not a safety limit")
    parser.add_argument("--alignment-z", type=float,
                        help="Explicit planner-to-PX4 local z offset for fence audit")
    parser.add_argument("--min-height", type=float, default=-2.0)
    parser.add_argument("--max-height", type=float, default=1.8)
    args = parser.parse_args()
    if args.jump_threshold_m <= 0 or not math.isfinite(args.jump_threshold_m):
        parser.error("jump threshold must be positive and finite")
    if args.alignment_z is not None and not math.isfinite(args.alignment_z):
        parser.error("alignment z must be finite")
    if not all(map(math.isfinite, (args.min_height, args.max_height))) or \
            args.min_height >= args.max_height:
        parser.error("height limits must be finite and increasing")
    print(json.dumps(audit(args.bag, args.jump_threshold_m, args.alignment_z,
                           args.min_height, args.max_height),
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
