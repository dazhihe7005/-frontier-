#!/usr/bin/env python3

"""Audit a full-stack Task-1 SITL managed-hold trajectory-clock injection."""

import argparse
import json
import math
import sys

import rosbag


TOPICS = [
    "/mine_uav/sitl/trajectory_sync_injection_status",
    "/mine_uav/super/execution_enable",
    "/mine_uav/task1/command_status",
    "/mine_uav/task1/command_fault",
    "/planning/pos_cmd",
    "/mavros/state",
]


def distance(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def position(message):
    return (message.position.x, message.position.y, message.position.z)


def analyze(path):
    injection = []
    authorization = []
    commands = []
    bridge_status = []
    faults = []
    modes = []
    with rosbag.Bag(path, "r") as bag:
        for topic, message, receipt_time in bag.read_messages(topics=TOPICS):
            stamp = receipt_time.to_sec()
            if topic.endswith("trajectory_sync_injection_status"):
                injection.append((stamp, message.data))
            elif topic.endswith("execution_enable"):
                authorization.append((stamp, bool(message.data)))
            elif topic.endswith("command_status"):
                bridge_status.append((stamp, message.data))
            elif topic.endswith("command_fault"):
                faults.append((stamp, message.data))
            elif topic == "/planning/pos_cmd":
                commands.append((stamp, position(message), message.trajectory_id))
            elif topic == "/mavros/state":
                modes.append((stamp, message.mode))

    start = next((stamp for stamp, state in injection
                  if state == "INJECTING"), None)
    cleared = next((stamp for stamp, state in injection
                    if state == "CLEARED" and
                    (start is None or stamp >= start)), None)
    false_edge = next((stamp for stamp, value in authorization
                       if start is not None and stamp >= start and not value), None)
    true_edge = next((stamp for stamp, value in authorization
                      if false_edge is not None and stamp > false_edge and value), None)

    paused_commands = []
    before_pause = None
    after_resume = None
    if false_edge is not None and true_edge is not None:
        paused_commands = [item for item in commands
                           if false_edge <= item[0] < true_edge]
        before = [item for item in commands if item[0] <= false_edge]
        after = [item for item in commands if item[0] >= true_edge]
        before_pause = before[-1] if before else None
        after_resume = after[0] if after else None

    reference_spread = None
    if paused_commands:
        anchor = paused_commands[0][1]
        reference_spread = max(distance(anchor, item[1])
                               for item in paused_commands)
    resume_step = None
    if paused_commands and after_resume:
        resume_step = distance(paused_commands[-1][1], after_resume[1])

    monitoring_seen = any(
        start is not None and stamp >= start and
        (cleared is None or stamp <= cleared + 2.0) and
        status.startswith("OBSTACLE_MONITORING_")
        for stamp, status in bridge_status)
    relevant_faults = [value for stamp, value in faults
                       if value and start is not None and stamp >= start and
                       (true_edge is None or stamp <= true_edge + 5.0)]
    premature_mode_exits = [mode for stamp, mode in modes
                            if start is not None and stamp >= start and
                            (true_edge is None or stamp <= true_edge + 1.0) and
                            mode != "OFFBOARD"]

    failures = []
    if start is None or cleared is None:
        failures.append("synthetic obstacle did not complete")
    if false_edge is None:
        failures.append("SUPER execution authorization never fell")
    if true_edge is None:
        failures.append("SUPER execution authorization never resumed")
    if not monitoring_seen:
        failures.append("bridge never entered obstacle monitoring")
    if len(paused_commands) < 3:
        failures.append("insufficient paused SUPER command samples")
    if reference_spread is None or reference_spread > 0.03:
        failures.append("SUPER reference advanced while execution was blocked")
    if resume_step is None or resume_step > 0.08:
        failures.append("SUPER reference jumped when execution resumed")
    if relevant_faults:
        failures.append("bridge faulted after the managed hold")
    if premature_mode_exits:
        failures.append("PX4 left OFFBOARD during the managed hold")

    return {
        "verdict": "PASS" if not failures else "FAIL",
        "failures": failures,
        "injection_start_s": start,
        "injection_cleared_s": cleared,
        "authorization_false_s": false_edge,
        "authorization_resumed_s": true_edge,
        "paused_command_samples": len(paused_commands),
        "paused_reference_spread_m": reference_spread,
        "resume_reference_step_m": resume_step,
        "monitoring_status_seen": monitoring_seen,
        "faults_after_injection": relevant_faults,
        "premature_mode_exits": premature_mode_exits,
        "trajectory_id_before_pause": before_pause[2] if before_pause else None,
        "trajectory_id_after_resume": after_resume[2] if after_resume else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--json-output")
    args = parser.parse_args()
    report = analyze(args.bag)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
