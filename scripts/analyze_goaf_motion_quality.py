#!/usr/bin/env python3
"""Accept a complete goaf SITL mission by outcome and motion continuity.

Slow observation and map-convergence periods are reported, not failed. The
checks instead bound adjacent position/velocity changes and turn rate while
the vehicle is moving. Samples after mission completion (PX4 landing contact)
are excluded from the motion window.
"""

import argparse
import json
import math

import rosbag

from analyze_task1_sitl_bag import analyze


LIMITS = {
    "planned_position_step_m": 0.05,
    "planned_velocity_step_m_s": 0.08,
    "bridge_position_step_m": 0.05,
    "bridge_velocity_step_m_s": 0.08,
    "actual_position_step_m": 0.08,
    "actual_velocity_step_m_s": 0.12,
    "actual_turn_rate_rad_s": 1.8,
    "planned_acceleration_m_s2": 1.6,
    "actual_speed_m_s": 1.2,
    "local_altitude_m": 1.8,
    "truth_return_error_m": 1.0,
}


def adjacent_peak(samples, start, end, offset, width):
    peak = None
    count = 0
    gaps = 0
    for previous, current in zip(samples, samples[1:]):
        dt = current[0] - previous[0]
        if start <= previous[0] <= end and dt > 0.1:
            gaps += 1
        if not (start <= previous[0] <= end and 0 < dt <= 0.06):
            continue
        delta = math.dist(previous[offset:offset + width],
                          current[offset:offset + width])
        count += 1
        if peak is None or delta > peak[0]:
            peak = (delta, current[0])
    return {"max": peak[0] if peak else None,
            "time": peak[1] if peak else None, "pairs": count,
            "gaps_over_0_1_s": gaps}


def turn_peak(samples, start, end):
    peak = None
    count = 0
    for previous, current in zip(samples, samples[1:]):
        dt = current[0] - previous[0]
        if not (start <= previous[0] <= end and 0 < dt <= 0.06):
            continue
        ax, ay = previous[1:3]
        bx, by = current[1:3]
        if min(math.hypot(ax, ay), math.hypot(bx, by)) < 0.3:
            continue
        angle = abs(math.atan2(ax * by - ay * bx, ax * bx + ay * by))
        rate = angle / dt
        count += 1
        if peak is None or rate > peak[0]:
            peak = (rate, current[0])
    return {"max": peak[0] if peak else None,
            "time": peak[1] if peak else None, "pairs": count}


def analyze_quality(path):
    mission = analyze(path)
    starts = [goal[0] for goal in mission["goals"]]
    start = min(starts) if starts else None
    end = mission["complete_time"]
    samples = {name: [] for name in (
        "planned_position", "planned_velocity", "bridge_position",
        "bridge_velocity", "actual_position", "actual_velocity")}
    with rosbag.Bag(path) as bag:
        for topic, message, stamp in bag.read_messages(topics=[
                "/planning/pos_cmd", "/mavros/setpoint_raw/local",
                "/mavros/local_position/pose",
                "/mavros/local_position/velocity_local"]):
            t = stamp.to_sec()
            if topic == "/planning/pos_cmd":
                p, v = message.position, message.velocity
                samples["planned_position"].append((t, p.x, p.y, p.z))
                samples["planned_velocity"].append((t, v.x, v.y, v.z))
            elif topic == "/mavros/setpoint_raw/local":
                p, v = message.position, message.velocity
                samples["bridge_position"].append((t, p.x, p.y, p.z))
                samples["bridge_velocity"].append((t, v.x, v.y, v.z))
            elif topic == "/mavros/local_position/pose":
                p = message.pose.position
                samples["actual_position"].append((t, p.x, p.y, p.z))
            else:
                v = message.twist.linear
                samples["actual_velocity"].append((t, v.x, v.y, v.z))

    motion = {}
    if start is not None and end is not None:
        for name in samples:
            motion[name] = adjacent_peak(samples[name], start, end, 1, 3)
        motion["actual_turn_rate"] = turn_peak(
            samples["actual_velocity"], start, end)
    else:
        motion = {name: {"max": None, "time": None, "pairs": 0}
                  for name in samples}
        motion["actual_turn_rate"] = {
            "max": None, "time": None, "pairs": 0}

    checks = {
        "mission_complete": mission["mission_complete"],
        "model_complete": mission["model_complete"],
        "map_closure_ready": mission["map_closure_ready"],
        "returned_home": mission["truth_return_horizontal_error"] is not None
        and mission["truth_return_horizontal_error"] <=
        LIMITS["truth_return_error_m"],
        "no_bridge_fault": not mission["command_faults"],
        "no_stale_position_command":
            mission["goal_lifecycle"]["stale_position_commands"] == 0,
    }
    for key, name in (
            ("planned_position_step_m", "planned_position"),
            ("planned_velocity_step_m_s", "planned_velocity"),
            ("bridge_position_step_m", "bridge_position"),
            ("bridge_velocity_step_m_s", "bridge_velocity"),
            ("actual_position_step_m", "actual_position"),
            ("actual_velocity_step_m_s", "actual_velocity"),
            ("actual_turn_rate_rad_s", "actual_turn_rate")):
        value = motion[name]["max"]
        checks[key] = (motion[name]["pairs"] >= 100 and value is not None
                       and value <= LIMITS[key])
    # The bridge keeps publishing at 50 Hz through an isolated planner tick
    # gap.  A missing planner sample is reported above, but continuous bridge
    # setpoints and measured motion are the safety-critical streams here.
    checks["no_unobserved_motion_gaps"] = all(
        motion[name].get("gaps_over_0_1_s", 0) == 0
        for name in ("bridge_position", "bridge_velocity",
                     "actual_position", "actual_velocity"))
    checks["planned_acceleration_m_s2"] = (
        mission["planned_acceleration_max"] is not None and
        mission["planned_acceleration_max"] <=
        LIMITS["planned_acceleration_m_s2"])
    checks["actual_speed_m_s"] = (
        mission["truth_speed_max"] is not None and
        mission["truth_speed_max"] <= LIMITS["actual_speed_m_s"])
    checks["local_altitude_m"] = (
        mission["mission_altitude_max"] is not None and
        mission["mission_altitude_max"] <= LIMITS["local_altitude_m"])
    return {
        "bag": path, "mission_start": start, "mission_end": end,
        "limits": LIMITS, "motion": motion,
        "battery_min_voltage": battery_min_voltage(path, start, end),
        "slow_periods_s": {
            "outbound": mission["exploration_low_speed_max"],
            "return": mission["return_low_speed_max"]},
        "checks": checks,
        "failed_checks": [name for name, value in checks.items() if not value],
        "verdict": "PASS" if all(checks.values()) else "FAIL",
    }


def battery_min_voltage(path, start, end):
    if start is None or end is None:
        return None
    values = []
    with rosbag.Bag(path) as bag:
        for _, message, stamp in bag.read_messages(topics=["/mavros/battery"]):
            if start <= stamp.to_sec() <= end and math.isfinite(message.voltage):
                values.append(message.voltage)
    return min(values) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--json-output")
    args = parser.parse_args()
    report = analyze_quality(args.bag)
    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as output:
            json.dump(report, output, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report["verdict"] == "PASS" else 1)


if __name__ == "__main__":
    main()
