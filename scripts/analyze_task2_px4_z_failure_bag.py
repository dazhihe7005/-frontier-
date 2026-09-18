#!/usr/bin/env python3

"""Audit command withdrawal after PX4 SITL loses vertical position.

This measures a fail-closed NUC response, not safe hover or return. A mode
change to LOITER/LAND without valid Z is not evidence of safe recovery.
"""

import argparse
import json
import sys

import rosbag


def analyze(path, bottom_top=-20.85, vehicle_radius=0.4):
    saw_valid_z = False
    invalid_at = None
    injected_at = None
    ready_false_at = None
    mode_exit_at = None
    task_hold_at = None
    last_mode = None
    final_mode = None
    final_connected = None
    post_invalid_setpoints = 0
    last_setpoint_at = None
    world_at_invalid = None
    latest_world = None
    last_world_at = None
    min_world_after_invalid = None
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[
            "/mavros/estimator_status", "/mavros/state",
            "/mavros/setpoint_raw/local", "/mine_uav/task2/command_ready",
            "/mine_uav/mission/active_task",
            "/mine_uav/sitl/px4_z_failure_status", "/gazebo/model_states",
        ]):
            t = stamp.to_sec()
            if topic == "/gazebo/model_states" and "iris" in msg.name:
                latest_world = msg.pose[msg.name.index("iris")].position.z
                last_world_at = t
                if invalid_at is not None:
                    min_world_after_invalid = min(
                        latest_world if min_world_after_invalid is None
                        else min_world_after_invalid, latest_world)
            elif topic == "/mavros/estimator_status":
                valid_z = (msg.pos_vert_abs_status_flag or
                           msg.pos_vert_agl_status_flag)
                if valid_z:
                    saw_valid_z = True
                elif saw_valid_z and invalid_at is None:
                    invalid_at = t
                    world_at_invalid = latest_world
            elif topic == "/mine_uav/sitl/px4_z_failure_status":
                if msg.data == "INJECTED_BARO_GPS_OFF":
                    injected_at = t
            elif topic == "/mavros/setpoint_raw/local":
                last_setpoint_at = t
                if invalid_at is not None:
                    post_invalid_setpoints += 1
            elif topic == "/mine_uav/task2/command_ready":
                if invalid_at is not None and ready_false_at is None and \
                        not msg.data:
                    ready_false_at = t
            elif topic == "/mine_uav/mission/active_task":
                if invalid_at is not None and task_hold_at is None and \
                        msg.data == 0:
                    task_hold_at = t
            elif topic == "/mavros/state":
                final_mode = msg.mode
                final_connected = msg.connected
                if invalid_at is not None and mode_exit_at is None and \
                        last_mode == "OFFBOARD" and msg.mode != "OFFBOARD":
                    mode_exit_at = t
                last_mode = msg.mode

    def delay(event_time):
        return round(event_time - invalid_at, 3) if \
            event_time is not None and invalid_at is not None else None

    return {
        "injection_confirmed_in_bag": injected_at is not None,
        "injection_time": round(injected_at, 3) if injected_at else None,
        "first_px4_z_invalid_time": round(invalid_at, 3) if invalid_at else None,
        "ready_withdraw_delay_s": delay(ready_false_at),
        "mode_exit_delay_s": delay(mode_exit_at),
        "task_hold_delay_s": delay(task_hold_at),
        "post_invalid_setpoint_count": post_invalid_setpoints,
        "last_setpoint_time": round(last_setpoint_at, 3)
        if last_setpoint_at is not None else None,
        "world_z_at_invalid": world_at_invalid,
        "minimum_world_z_after_invalid": min_world_after_invalid,
        "minimum_body_outside_bottom_margin_after_invalid": (
            min_world_after_invalid - bottom_top - vehicle_radius
            if min_world_after_invalid is not None else None),
        "post_invalid_world_observation_s": delay(last_world_at),
        "final_px4_mode": final_mode,
        "final_px4_connected": final_connected,
        "safe_no_z_recovery_proven": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--bottom-top", type=float, default=-20.85)
    parser.add_argument("--vehicle-radius", type=float, default=0.4)
    parser.add_argument("--require-injection", action="store_true")
    parser.add_argument("--max-ready-withdraw-delay", type=float)
    parser.add_argument("--max-post-invalid-setpoints", type=int)
    args = parser.parse_args()
    result = analyze(args.bag, args.bottom_top, args.vehicle_radius)
    print(json.dumps(result, indent=2))
    failures = []
    if result["first_px4_z_invalid_time"] is None:
        failures.append("no PX4 Z-valid-to-invalid transition observed")
    if args.require_injection and not result["injection_confirmed_in_bag"]:
        failures.append("no confirmed baro+GPS injection event")
    if args.max_ready_withdraw_delay is not None and (
            result["ready_withdraw_delay_s"] is None or
            result["ready_withdraw_delay_s"] > args.max_ready_withdraw_delay):
        failures.append("command readiness not withdrawn in time")
    if args.max_post_invalid_setpoints is not None and \
            result["post_invalid_setpoint_count"] > \
            args.max_post_invalid_setpoints:
        failures.append("setpoints continued after PX4 Z became invalid")
    if failures:
        print("FAIL: " + "; ".join(failures), file=sys.stderr)
        sys.exit(1)
