#!/usr/bin/env python3

"""Audit SITL-only depth disagreement command withdrawal, not flight safety."""

import argparse
import json
import sys

import rosbag


def analyze(path):
    first_ready = None
    ready_withdrawn = None
    last_setpoint = None
    first_bias = None
    loiter = None
    setpoints_after_withdrawal = 0
    world_at_withdrawal = None
    lowest_world_after_withdrawal = None
    last_world = None
    latest_world = None
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[
            "/mine_uav/task2/command_ready",
            "/mine_uav/sitl/shaft_depth_bias_m",
            "/mavros/setpoint_raw/local",
            "/mavros/state",
            "/gazebo/model_states",
        ]):
            t = stamp.to_sec()
            if topic == "/mine_uav/sitl/shaft_depth_bias_m":
                if first_bias is None and msg.data >= 1.0:
                    first_bias = t
            elif topic == "/mine_uav/task2/command_ready":
                if msg.data and first_ready is None:
                    first_ready = t
                elif not msg.data and first_ready is not None and ready_withdrawn is None:
                    ready_withdrawn = t
                    world_at_withdrawal = latest_world
            elif topic == "/mavros/setpoint_raw/local":
                last_setpoint = t
                if ready_withdrawn is not None:
                    setpoints_after_withdrawal += 1
            elif topic == "/mavros/state":
                if ready_withdrawn is not None and msg.mode == "AUTO.LOITER" and loiter is None:
                    loiter = t
            elif topic == "/gazebo/model_states" and "iris" in msg.name:
                latest_world = msg.pose[msg.name.index("iris")].position.z
                if ready_withdrawn is not None:
                    last_world = t
                    if lowest_world_after_withdrawal is None:
                        lowest_world_after_withdrawal = latest_world
                    else:
                        lowest_world_after_withdrawal = min(
                            lowest_world_after_withdrawal, latest_world)
    return {
        "first_ready_s": first_ready,
        "first_bias_at_least_1m_s": first_bias,
        "ready_withdrawn_s": ready_withdrawn,
        "last_setpoint_s": last_setpoint,
        "setpoints_after_withdrawal": setpoints_after_withdrawal,
        "loiter_s": loiter,
        "post_withdrawal_observation_s": (
            last_world - ready_withdrawn
            if last_world is not None and ready_withdrawn is not None else None),
        "post_withdrawal_extra_descent_m": (
            world_at_withdrawal - lowest_world_after_withdrawal
            if world_at_withdrawal is not None and
            lowest_world_after_withdrawal is not None else None),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--max-withdrawal-delay", type=float, default=0.5)
    parser.add_argument("--min-post-observation", type=float, default=10.0)
    args = parser.parse_args()
    result = analyze(args.bag)
    print(json.dumps(result, indent=2))
    failures = []
    if result["first_ready_s"] is None or result["first_bias_at_least_1m_s"] is None:
        failures.append("no active mission or no 1 m injected bias")
    if result["ready_withdrawn_s"] is None:
        failures.append("router did not withdraw command readiness")
    if (result["first_bias_at_least_1m_s"] is not None and
            result["ready_withdrawn_s"] is not None and
            abs(result["ready_withdrawn_s"] - result["first_bias_at_least_1m_s"]) >
            args.max_withdrawal_delay):
        failures.append("withdrawal not close to disagreement threshold")
    if result["setpoints_after_withdrawal"]:
        failures.append("setpoints continued after withdrawal")
    if result["loiter_s"] is None:
        failures.append("no observed PX4 AUTO.LOITER")
    if (result["post_withdrawal_observation_s"] is None or
            result["post_withdrawal_observation_s"] < args.min_post_observation):
        failures.append("post-withdrawal observation too short")
    if failures:
        print("FAIL: " + "; ".join(failures), file=sys.stderr)
        sys.exit(1)
