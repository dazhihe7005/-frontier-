#!/usr/bin/env python3

"""Audit a simulated shaft mode exit; MAVROS mode_sent is not confirmation."""

import argparse
import json
import sys

import rosbag


def analyze(path, expected_mode="AUTO.LOITER"):
    events = []
    last_mode = None
    last_shaft = None
    seen_active = False
    seen_active_offboard = False
    confirmed_at = None
    hold_at = None
    ready_false_at = None
    offboard_reentry = False
    setpoints_after_exit = 0
    end_time = None
    confirmed_injector = False
    topics = [
        "/mavros/state", "/mine_uav/mission/active_task",
        "/mine_uav/shaft/status", "/mine_uav/task2/command_ready",
        "/mine_uav/sitl/shaft_takeover_status",
        "/mavros/setpoint_raw/local", "/mavros/setpoint_position/local",
        "/clock",
    ]
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec()
            end_time = t
            if topic == "/mine_uav/sitl/shaft_takeover_status":
                if msg.data == "CONFIRMED:" + expected_mode:
                    confirmed_injector = True
                events.append({"time": round(t, 3), "event": msg.data})
            elif topic == "/mine_uav/shaft/status":
                if msg.data != last_shaft:
                    events.append({"time": round(t, 3), "shaft": msg.data})
                    last_shaft = msg.data
                if msg.data == "DESCENDING":
                    seen_active = True
            elif topic == "/mavros/state":
                if seen_active and msg.mode == "OFFBOARD":
                    seen_active_offboard = True
                if confirmed_at is None and seen_active_offboard and \
                        last_mode == "OFFBOARD" and msg.mode == expected_mode:
                    confirmed_at = t
                if msg.mode != last_mode:
                    events.append({"time": round(t, 3), "px4_mode": msg.mode})
                    last_mode = msg.mode
                if confirmed_at is not None and t > confirmed_at and \
                        msg.mode == "OFFBOARD":
                    offboard_reentry = True
            elif topic == "/mine_uav/mission/active_task":
                if confirmed_at is not None and msg.data == 0 and hold_at is None:
                    hold_at = t
            elif topic == "/mine_uav/task2/command_ready":
                if confirmed_at is not None and not msg.data and ready_false_at is None:
                    ready_false_at = t
            elif topic in ("/mavros/setpoint_raw/local",
                           "/mavros/setpoint_position/local"):
                if confirmed_at is not None:
                    setpoints_after_exit += 1
    observation = (end_time - confirmed_at
                   if end_time is not None and confirmed_at is not None else None)
    return {
        "expected_mode": expected_mode,
        "saw_descent": seen_active,
        "saw_offboard_during_descent": seen_active_offboard,
        "confirmed_by_px4_state": confirmed_at is not None,
        "confirmed_by_injector": confirmed_injector,
        "hold_delay_s": round(hold_at - confirmed_at, 3)
        if hold_at is not None else None,
        "command_not_ready_delay_s": round(ready_false_at - confirmed_at, 3)
        if ready_false_at is not None else None,
        "offboard_reentry": offboard_reentry,
        "setpoints_after_exit": setpoints_after_exit,
        "post_exit_observation_s": round(observation, 3)
        if observation is not None else None,
        "events": events,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--expected-mode", default="AUTO.LOITER")
    parser.add_argument("--min-observation", type=float, default=10.0)
    parser.add_argument("--max-hold-delay", type=float, default=1.0)
    args = parser.parse_args()
    result = analyze(args.bag, args.expected_mode)
    print(json.dumps(result, indent=2))
    failures = []
    if not result["saw_descent"]:
        failures.append("shaft never descended")
    if not result["saw_offboard_during_descent"]:
        failures.append("no OFFBOARD state observed during descent")
    if not result["confirmed_by_px4_state"] or not result["confirmed_by_injector"]:
        failures.append("requested mode not confirmed by both PX4 and injector")
    if result["hold_delay_s"] is None or \
            result["hold_delay_s"] > args.max_hold_delay:
        failures.append("scheduler did not withdraw task promptly")
    if result["command_not_ready_delay_s"] is None:
        failures.append("task2 command remained ready")
    if result["offboard_reentry"] or result["setpoints_after_exit"]:
        failures.append("OFFBOARD or a setpoint reappeared after takeover")
    if result["post_exit_observation_s"] is None or \
            result["post_exit_observation_s"] < args.min_observation:
        failures.append("post-exit observation too short")
    if failures:
        print("FAIL: " + "; ".join(failures), file=sys.stderr)
        sys.exit(1)
