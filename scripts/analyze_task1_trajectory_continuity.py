#!/usr/bin/env python3
"""Audit task-one goal handovers and adjacent SUPER/PX4 velocity samples.

Only adjacent samples no more than 60 ms apart are compared. Longer gaps are
reported separately; a hard CANCEL during wall confirmation can intentionally
leave the vehicle holding without any SUPER command.
"""

import argparse
import bisect
import json
import math

import rosbag


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def sample_metrics(samples, velocity_start, position_start=None):
    result = {
        "samples": len(samples),
        "max_adjacent_velocity_delta_m_s": 0.0,
        "max_adjacent_position_delta_m": 0.0 if position_start is not None else None,
        "velocity_steps_over_0_2_m_s": 0,
        "gaps_over_0_1_s": 0,
        "longest_gap_s": 0.0,
        "large_step_times": [],
    }
    for previous, current in zip(samples, samples[1:]):
        dt = current[0] - previous[0]
        if dt > 0.1:
            result["gaps_over_0_1_s"] += 1
            result["longest_gap_s"] = max(result["longest_gap_s"], dt)
        if not 0 < dt <= 0.06:
            continue
        dv = distance(previous[velocity_start:velocity_start + 3],
                      current[velocity_start:velocity_start + 3])
        result["max_adjacent_velocity_delta_m_s"] = max(
            result["max_adjacent_velocity_delta_m_s"], dv)
        if position_start is not None:
            dp = distance(previous[position_start:position_start + 3],
                          current[position_start:position_start + 3])
            result["max_adjacent_position_delta_m"] = max(
                result["max_adjacent_position_delta_m"], dp)
        if dv > 0.2:
            result["velocity_steps_over_0_2_m_s"] += 1
            result["large_step_times"].append(current[0])
    return result


def analyze(path):
    goals, commands, actual, statuses, bridge = [], [], [], [], []
    with rosbag.Bag(path) as bag:
        for topic, message, stamp in bag.read_messages(topics=[
            "/mine_uav/super/goal_command", "/planning/pos_cmd",
            "/mavros/local_position/velocity_local",
            "/mine_uav/exploration/status", "/mine_uav/task1/command_status",
        ]):
            time = stamp.to_sec()
            if topic.endswith("goal_command"):
                goals.append((time, message.command, message.goal_id,
                              message.reason))
            elif topic == "/planning/pos_cmd":
                commands.append((time, message.position.x, message.position.y,
                                 message.position.z, message.velocity.x,
                                 message.velocity.y, message.velocity.z))
            elif topic.endswith("velocity_local"):
                actual.append((time, message.twist.linear.x,
                               message.twist.linear.y, message.twist.linear.z))
            elif topic.endswith("exploration/status"):
                statuses.append(message.data)
            else:
                bridge.append(message.data)

    planned = sample_metrics(commands, 4, 1)
    flown = sample_metrics(actual, 1)
    set_goals = [event for event in goals if event[1] == 1]
    set_times = [event[0] for event in set_goals]
    near_goal_steps = []
    for stamp in planned["large_step_times"]:
        index = bisect.bisect_right(set_times, stamp) - 1
        if index >= 0 and 0 <= stamp - set_times[index] <= 0.1:
            near_goal_steps.append({
                "seconds_after_set": stamp - set_times[index],
                "goal_reason": set_goals[index][3],
            })
    planned["large_steps_within_0_1_s_of_goal_set"] = near_goal_steps
    del planned["large_step_times"]
    del flown["large_step_times"]

    active = 0
    direct_retargets = 0
    for _, command, goal_id, _ in goals:
        if command == 1:
            if active:
                direct_retargets += 1
            active = goal_id
        elif command == 2 and (goal_id == 0 or goal_id == active):
            active = 0
    return {
        "bag": path,
        "set_goals": len(set_goals),
        "cancel_goals": len(goals) - len(set_goals),
        "direct_retargets": direct_retargets,
        "final_exploration_status": statuses[-1] if statuses else None,
        "bridge_fault_statuses": sorted(set(s for s in bridge if any(
            marker in s for marker in ("FAULT", "INVALID", "GEOFENCE",
                                         "SPEED_LIMIT", "ACCELERATION_LIMIT")))),
        "planned": planned,
        "actual": flown,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="ROS1 bag with goal, command and PX4 topics")
    args = parser.parse_args()
    print(json.dumps(analyze(args.bag), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
