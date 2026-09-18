#!/usr/bin/env python3

"""Audit an isolated shaft PX4/Gazebo SITL bag; not a real-flight verifier."""

import argparse
import json
import math
import sys

import rosbag


def analyze(path, bottom_top=-20.85, shaft_half_width=5.0, vehicle_radius=0.4):
    statuses = []
    modes = []
    first_active_xy = None
    max_xy_deviation = 0.0
    min_bottom_margin = math.inf
    min_side_margin = math.inf
    deepest_world_z = math.inf
    deepest_relative = -math.inf
    depth_at_completion = None
    latest_depth = None
    latest_bottom_range = None
    latest_vertical_velocity = None
    bottom_min_event = None
    max_range_alignment_error = 0.0
    active = False
    complete = False
    fault = False
    disarmed_while_active = False
    pad_seen = False
    pad_gone_after_seen = False
    position_setpoint_count = 0
    latest_world_z = None
    world_z_at_fault = None
    deepest_world_z_after_fault = math.inf
    fault_time = None
    last_world_time = None
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[
            "/mine_uav/shaft/status", "/mine_uav/shaft/relative_depth_m",
            "/gazebo/model_states", "/mavros/state",
            "/mavros/setpoint_raw/local", "/mine_uav/shaft/bottom_range",
            "/mavros/local_position/odom",
        ]):
            t = round(stamp.to_sec(), 3)
            if topic == "/mine_uav/shaft/relative_depth_m":
                latest_depth = msg.data
            elif topic == "/mine_uav/shaft/bottom_range":
                latest_bottom_range = msg.range
            elif topic == "/mavros/local_position/odom":
                latest_vertical_velocity = msg.twist.twist.linear.z
            elif topic == "/mine_uav/shaft/status":
                if not statuses or statuses[-1]["state"] != msg.data:
                    statuses.append({"time": t, "state": msg.data,
                                     "relative_depth": latest_depth,
                                     "bottom_range": latest_bottom_range,
                                     "vertical_velocity_enu": latest_vertical_velocity})
                active = msg.data in ("DESCENDING", "RETURNING")
                if msg.data == "COMPLETE":
                    complete = True
                    if depth_at_completion is None:
                        depth_at_completion = latest_depth
                if msg.data.startswith("FAULT"):
                    fault = True
                    if world_z_at_fault is None:
                        world_z_at_fault = latest_world_z
                        fault_time = t
            elif topic == "/mavros/state":
                if not modes or modes[-1]["mode"] != msg.mode or \
                        modes[-1]["armed"] != msg.armed:
                    modes.append({"time": t, "mode": msg.mode,
                                  "armed": msg.armed})
                if active and not msg.armed:
                    disarmed_while_active = True
            elif topic == "/mavros/setpoint_raw/local":
                position_setpoint_count += 1
            elif topic == "/gazebo/model_states":
                names = msg.name
                if "shaft_launch_pad" in names:
                    pad_seen = True
                elif pad_seen:
                    pad_gone_after_seen = True
                if "iris" not in names:
                    continue
                pose = msg.pose[names.index("iris")].position
                latest_world_z = pose.z
                last_world_time = t
                if fault:
                    deepest_world_z_after_fault = min(
                        deepest_world_z_after_fault, pose.z)
                if not active:
                    continue
                if first_active_xy is None:
                    first_active_xy = (pose.x, pose.y)
                max_xy_deviation = max(
                    max_xy_deviation,
                    math.hypot(pose.x - first_active_xy[0],
                               pose.y - first_active_xy[1]))
                bottom_margin = pose.z - bottom_top - vehicle_radius
                if latest_bottom_range is not None and math.isfinite(
                        latest_bottom_range):
                    max_range_alignment_error = max(
                        max_range_alignment_error,
                        abs(latest_bottom_range - (pose.z - bottom_top)))
                if bottom_margin < min_bottom_margin:
                    min_bottom_margin = bottom_margin
                    bottom_min_event = {"time": t, "state": statuses[-1]["state"],
                                        "bottom_range": latest_bottom_range,
                                        "vertical_velocity_enu": latest_vertical_velocity,
                                        "world_z": pose.z}
                min_side_margin = min(
                    min_side_margin,
                    shaft_half_width - abs(pose.x) - vehicle_radius,
                    shaft_half_width - abs(pose.y) - vehicle_radius)
                deepest_world_z = min(deepest_world_z, pose.z)
                if latest_depth is not None:
                    deepest_relative = max(deepest_relative, latest_depth)
    return {
        "statuses": statuses,
        "modes": modes,
        "complete": complete,
        "fault": fault,
        "world_z_at_fault": world_z_at_fault,
        "post_fault_extra_descent": (
            world_z_at_fault - deepest_world_z_after_fault
            if world_z_at_fault is not None and
            math.isfinite(deepest_world_z_after_fault) else None),
        "fault_to_loiter_seconds": next(
            (mode["time"] - status["time"]
             for status in statuses if status["state"].startswith("FAULT")
             for mode in modes if mode["time"] >= status["time"]
             and mode["mode"] == "AUTO.LOITER"), None),
        "post_fault_observation_seconds": (
            last_world_time - fault_time
            if last_world_time is not None and fault_time is not None else None),
        "disarmed_while_active": disarmed_while_active,
        "pad_seen": pad_seen,
        "pad_gone_after_seen": pad_gone_after_seen,
        "deepest_world_z": deepest_world_z,
        "deepest_relative_depth": deepest_relative,
        "depth_at_completion": depth_at_completion,
        "min_bottom_margin": min_bottom_margin,
        "bottom_min_event": bottom_min_event,
        "max_range_alignment_error": max_range_alignment_error,
        "min_side_margin": min_side_margin,
        "max_xy_deviation": max_xy_deviation,
        "position_setpoint_count": position_setpoint_count,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--bottom-top", type=float, default=-20.85)
    parser.add_argument("--shaft-half-width", type=float, default=5.0)
    parser.add_argument("--vehicle-radius", type=float, default=0.4)
    parser.add_argument("--require-complete", action="store_true")
    parser.add_argument("--require-fault", action="store_true")
    parser.add_argument("--max-fault-to-loiter", type=float)
    parser.add_argument("--max-post-fault-descent", type=float)
    parser.add_argument("--min-post-fault-observation", type=float)
    parser.add_argument("--min-bottom-margin", type=float)
    parser.add_argument("--max-xy-deviation", type=float)
    parser.add_argument("--max-range-alignment-error", type=float)
    args = parser.parse_args()
    result = analyze(args.bag, args.bottom_top,
                     args.shaft_half_width, args.vehicle_radius)
    print(json.dumps(result,
                     indent=2, allow_nan=False))
    failures = []
    if args.require_complete and not result["complete"]:
        failures.append("not complete")
    if args.require_fault and not result["fault"]:
        failures.append("no fault observed")
    if args.max_fault_to_loiter is not None and (
            result["fault_to_loiter_seconds"] is None or
            result["fault_to_loiter_seconds"] > args.max_fault_to_loiter):
        failures.append("fault-to-LOITER above threshold or absent")
    if args.max_post_fault_descent is not None and (
            result["post_fault_extra_descent"] is None or
            result["post_fault_extra_descent"] > args.max_post_fault_descent):
        failures.append("post-fault descent above threshold or absent")
    if args.min_post_fault_observation is not None and (
            result["post_fault_observation_seconds"] is None or
            result["post_fault_observation_seconds"] < args.min_post_fault_observation):
        failures.append("post-fault observation too short or absent")
    if args.min_bottom_margin is not None and \
            result["min_bottom_margin"] < args.min_bottom_margin:
        failures.append("bottom margin below threshold")
    if args.max_xy_deviation is not None and \
            result["max_xy_deviation"] > args.max_xy_deviation:
        failures.append("XY deviation above threshold")
    if args.max_range_alignment_error is not None and \
            result["max_range_alignment_error"] > args.max_range_alignment_error:
        failures.append("range/world alignment error above threshold")
    if failures:
        print("FAIL: " + ", ".join(failures), file=sys.stderr)
        sys.exit(1)
