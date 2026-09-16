#!/usr/bin/env python3

"""Evaluate a task-one PX4 SITL rosbag against repeatable acceptance limits."""

import argparse
import bisect
from collections import Counter
import json
import math
import sys

import rosbag


TOPICS = [
    "/mine_uav/super/goal_command",
    "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local",
    "/mavros/state",
    "/mine_uav/exploration/finished",
    "/mine_uav/exploration/model_coverage",
    "/mine_uav/exploration/status",
    "/mine_uav/task1/command_status",
    "/Odometry",
    "/planning/pos_cmd",
    "/rosout_agg",
]

FAULT_MARKERS = (
    "FAULT",
    "INVALID",
    "GEOFENCE",
    "SPEED_LIMIT",
    "ACCELERATION_LIMIT",
    "POSITION_LIMIT",
)

SUPER_FAILURE_CATEGORY_MARKERS = (
    ("backup_optimization", "[BackOpt] Opt failed"),
    ("main_optimization", "[ExpOpt] Opt failed"),
    ("corridor_generation", "GeneratePolytopeFromLine failed"),
    ("backup_trajectory_return", "generateBackupTrajectory return"),
    ("main_trajectory_return", "GenerateExpTrajectory failed"),
    ("other_minco", "Minco exp_traj opt failed"),
)


class ClockMapper:
    """Map rosbag receipt time to Gazebo simulation time when /clock exists."""

    def __init__(self, samples):
        self.receipt = [sample[0] for sample in samples]
        self.simulation = [sample[1] for sample in samples]

    @property
    def available(self):
        return bool(self.receipt)

    def map(self, receipt_time):
        if not self.receipt:
            return receipt_time
        index = bisect.bisect_right(self.receipt, receipt_time) - 1
        if index < 0:
            return self.simulation[0]
        if index >= len(self.receipt) - 1:
            return self.simulation[-1]
        left_receipt = self.receipt[index]
        right_receipt = self.receipt[index + 1]
        if right_receipt <= left_receipt:
            return self.simulation[index]
        fraction = (receipt_time - left_receipt) / (right_receipt - left_receipt)
        return self.simulation[index] + fraction * (
            self.simulation[index + 1] - self.simulation[index]
        )


def vector_norm(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = int(fraction * (len(ordered) - 1))
    return ordered[index]


def mean(values):
    return sum(values) / len(values) if values else None


def rate(stamps):
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return None
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def longest_below(samples, threshold):
    longest = 0.0
    start = None
    for stamp, value in samples:
        if value < threshold and start is None:
            start = stamp
        elif value >= threshold and start is not None:
            longest = max(longest, stamp - start)
            start = None
    if start is not None and samples:
        longest = max(longest, samples[-1][0] - start)
    return longest


def analyze_goal_lifecycle_events(events, grace=0.02):
    """Count outputs after a matching cancel and before the next SET.

    Events are (receipt_time, kind, goal_id) in bag order. The only tolerated
    post-cancel command is one already in flight within the 20 ms grace.
    PlanFromRest logs never have grace: starting a new plan after cancellation
    is precisely the regression this metric is meant to expose.
    """
    active_goal_id = None
    cancel_time = None
    grace_used = False
    stale_commands = 0
    stale_plans = 0
    cancel_count = 0
    ignored_stale_cancels = 0
    silence_latency = 0.0
    for stamp, kind, goal_id in events:
        if kind == "set":
            if goal_id:
                active_goal_id = goal_id
                cancel_time = None
                grace_used = False
        elif kind == "cancel":
            if active_goal_id is not None and (
                goal_id == 0 or goal_id == active_goal_id
            ):
                active_goal_id = None
                cancel_time = stamp
                grace_used = False
                cancel_count += 1
            elif goal_id != 0:
                ignored_stale_cancels += 1
        elif kind in ("pos_cmd", "plan_from_rest") and cancel_time is not None:
            silence_latency = max(silence_latency, stamp - cancel_time)
            if kind == "plan_from_rest":
                stale_plans += 1
            elif stamp - cancel_time >= grace or grace_used:
                stale_commands += 1
            else:
                grace_used = True
    return {
        "active_goal_id": active_goal_id,
        "cancel_count": cancel_count,
        "ignored_stale_cancels": ignored_stale_cancels,
        "stale_position_commands": stale_commands,
        "stale_plan_from_rest_calls": stale_plans,
        "cancel_to_silence_latency": silence_latency,
    }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Analyze task-one SITL completion, safety and fluidity"
    )
    parser.add_argument("bag", help="task-one rosbag path")
    parser.add_argument("--json-output", help="optional JSON report path")
    parser.add_argument(
        "--profile", choices=("legacy_fast", "lab_1m"), default="legacy_fast",
        help="lab_1m evaluates moving phases at the configured 1 m/s ceiling",
    )
    parser.add_argument("--min-median-speed", type=float)
    parser.add_argument("--max-low-speed-duration", type=float)
    parser.add_argument("--max-cross-track", type=float, default=1.5)
    parser.add_argument("--max-mission-altitude", type=float,
                        help="maximum local-frame z during task-one mission")
    parser.add_argument("--max-return-error", type=float, default=1.0)
    parser.add_argument("--max-planned-speed", type=float, default=2.05)
    parser.add_argument("--max-planned-acceleration", type=float, default=1.60)
    parser.add_argument("--max-actual-speed", type=float, default=3.0)
    parser.add_argument("--min-odom-rate", type=float, default=5.0)
    parser.add_argument("--min-path-length", type=float, default=45.0)
    parser.add_argument("--max-offboard-duration", type=float)
    parser.add_argument(
        "--max-super-failures",
        type=int,
        default=-1,
        help="fail above this /rosout_agg warning count; -1 only reports it",
    )
    arguments = parser.parse_args()
    defaults = {
        "legacy_fast": (1.5, 1.0, 90.0),
        "lab_1m": (0.7, 1.5, 180.0),
    }
    median_speed, low_duration, offboard_duration = defaults[arguments.profile]
    if arguments.min_median_speed is None:
        arguments.min_median_speed = median_speed
    if arguments.max_low_speed_duration is None:
        arguments.max_low_speed_duration = low_duration
    if arguments.max_offboard_duration is None:
        arguments.max_offboard_duration = offboard_duration
    if arguments.max_mission_altitude is None and arguments.profile == "lab_1m":
        arguments.max_mission_altitude = 1.8
    return arguments


def analyze(path):
    clock_samples = []
    with rosbag.Bag(path) as bag:
        for _topic, message, bag_stamp in bag.read_messages(topics=["/clock"]):
            clock_samples.append((bag_stamp.to_sec(), message.clock.to_sec()))
    clock_mapper = ClockMapper(clock_samples)

    poses = []
    velocities = []
    planned_speeds = []
    planned_accelerations = []
    goals = []
    state_changes = []
    statuses = []
    coverage = []
    command_faults = []
    odom_stamps = []
    super_failures = []
    super_failure_breakdown = Counter()
    lifecycle_events = []
    lifecycle_command_count = 0
    rosout_message_count = 0
    finished_true = False
    last_state = None

    with rosbag.Bag(path) as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=TOPICS):
            stamp = clock_mapper.map(bag_stamp.to_sec())
            if topic == "/mavros/local_position/pose":
                point = message.pose.position
                poses.append((stamp, point.x, point.y, point.z))
            elif topic == "/mavros/local_position/velocity_local":
                linear = message.twist.linear
                velocities.append(
                    (stamp, vector_norm(linear.x, linear.y, linear.z))
                )
            elif topic == "/planning/pos_cmd":
                lifecycle_events.append((stamp, "pos_cmd", 0))
                velocity = message.velocity
                acceleration = message.acceleration
                planned_speeds.append(
                    (stamp, vector_norm(velocity.x, velocity.y, velocity.z))
                )
                planned_accelerations.append(
                    (
                        stamp,
                        vector_norm(
                            acceleration.x, acceleration.y, acceleration.z
                        ),
                    )
                )
            elif topic == "/mine_uav/super/goal_command":
                lifecycle_command_count += 1
                if message.command == message.SET_GOAL:
                    point = message.goal.position
                    goals.append((stamp, point.x, point.y, point.z))
                    lifecycle_events.append((stamp, "set", message.goal_id))
                elif message.command == message.CANCEL_GOAL:
                    lifecycle_events.append((stamp, "cancel", message.goal_id))
            elif topic == "/mavros/state":
                state = (message.mode, bool(message.armed))
                if state != last_state:
                    state_changes.append((stamp, state[0], state[1]))
                    last_state = state
            elif topic == "/mine_uav/exploration/status":
                if not statuses or statuses[-1][1] != message.data:
                    statuses.append((stamp, message.data))
            elif topic == "/mine_uav/exploration/model_coverage":
                coverage.append((stamp, message.data))
            elif topic == "/mine_uav/exploration/finished":
                finished_true = finished_true or bool(message.data)
            elif topic == "/mine_uav/task1/command_status":
                if any(marker in message.data for marker in FAULT_MARKERS):
                    entry = (stamp, message.data)
                    if not command_faults or command_faults[-1][1] != message.data:
                        command_faults.append(entry)
            elif topic == "/Odometry":
                odom_stamps.append(stamp)
            elif topic == "/rosout_agg":
                rosout_message_count += 1
                text = getattr(message, "msg", "")
                if "PlanFromRest" in text:
                    lifecycle_events.append((stamp, "plan_from_rest", 0))
                for category, marker in SUPER_FAILURE_CATEGORY_MARKERS:
                    if marker in text:
                        super_failures.append((stamp, text))
                        super_failure_breakdown[category] += 1
                        break

    offboard_start = next(
        (stamp for stamp, mode, _armed in state_changes if mode == "OFFBOARD"),
        None,
    )
    loiter_after_offboard = None
    if offboard_start is not None:
        loiter_after_offboard = next(
            (
                stamp
                for stamp, mode, _armed in state_changes
                if stamp > offboard_start and mode == "AUTO.LOITER"
            ),
            None,
        )
    complete_time = next(
        (stamp for stamp, text in statuses if text.startswith("COMPLETE")), None
    )
    active_end = loiter_after_offboard or complete_time

    active_poses = []
    active_velocities = []
    if offboard_start is not None and active_end is not None:
        active_poses = [
            sample for sample in poses if offboard_start <= sample[0] <= active_end
        ]
        steady_start = offboard_start + 2.0
        steady_end = max(steady_start, active_end - 2.0)
        active_velocities = [
            sample
            for sample in velocities
            if steady_start <= sample[0] <= steady_end
        ]

    path_length = 0.0
    for previous, current in zip(active_poses, active_poses[1:]):
        path_length += vector_norm(
            current[1] - previous[1],
            current[2] - previous[2],
            current[3] - previous[3],
        )

    return_error = None
    cross_track_max = None
    if active_poses:
        home_x, home_y = active_poses[0][1], active_poses[0][2]
        final_x, final_y = active_poses[-1][1], active_poses[-1][2]
        return_error = math.hypot(final_x - home_x, final_y - home_y)
        forward_goal = next(
            (
                goal
                for goal in goals
                if math.hypot(goal[1] - home_x, goal[2] - home_y) > 2.0
            ),
            None,
        )
        if forward_goal is not None:
            heading = math.atan2(forward_goal[2] - home_y, forward_goal[1] - home_x)
            sine = math.sin(heading)
            cosine = math.cos(heading)
            cross_track_max = max(
                abs(-sine * (sample[1] - home_x) + cosine * (sample[2] - home_y))
                for sample in active_poses
            )

    speed_values = [value for _stamp, value in active_velocities]
    first_goal_time = goals[0][0] if goals else None
    mission_heights = [
        sample[3] for sample in active_poses
        if first_goal_time is not None and sample[0] >= first_goal_time
    ]
    closure_wait_time = next(
        (stamp for stamp, text in statuses
         if first_goal_time is not None and stamp > first_goal_time
         and text.startswith("WAIT_MAP_CLOSURE")), None
    )
    return_start_time = next(
        (stamp for stamp, text in statuses if text.startswith("RETURNING")), None
    )
    exploration_velocities = [
        sample for sample in velocities
        if first_goal_time is not None and closure_wait_time is not None
        and first_goal_time + 2.0 <= sample[0] < closure_wait_time
    ]
    return_velocities = [
        sample for sample in velocities
        if return_start_time is not None and complete_time is not None
        and return_start_time + 2.0 <= sample[0] <= complete_time - 2.0
    ]
    planned_speed_values = [value for _stamp, value in planned_speeds]
    planned_acceleration_values = [value for _stamp, value in planned_accelerations]
    final_coverage = coverage[-1][1] if coverage else ""
    lifecycle = analyze_goal_lifecycle_events(lifecycle_events)

    return {
        "bag": path,
        "time_basis": "gazebo_clock" if clock_mapper.available else "bag_receipt",
        "entered_offboard": offboard_start is not None,
        "offboard_start": offboard_start,
        "offboard_end": active_end,
        "offboard_duration": (
            active_end - offboard_start
            if offboard_start is not None and active_end is not None
            else None
        ),
        "entered_auto_loiter_after_offboard": loiter_after_offboard is not None,
        "mission_complete": complete_time is not None and finished_true,
        "complete_time": complete_time,
        "map_closure_ready": "map_closure=ready" in final_coverage,
        "three_wall_ready": "three_wall=ready" in final_coverage,
        "final_coverage": final_coverage,
        "goal_count": len(goals),
        "goals": goals,
        "goal_lifecycle_available": lifecycle_command_count > 0,
        "goal_lifecycle": lifecycle,
        "path_length": path_length,
        "return_horizontal_error": return_error,
        "cross_track_max": cross_track_max,
        "mission_altitude_max": max(mission_heights) if mission_heights else None,
        "mission_altitude_min": min(mission_heights) if mission_heights else None,
        "speed_mean": mean(speed_values),
        "speed_median": percentile(speed_values, 0.5),
        "exploration_speed_median": percentile(
            [value for _stamp, value in exploration_velocities], 0.5
        ),
        "return_speed_median": percentile(
            [value for _stamp, value in return_velocities], 0.5
        ),
        "exploration_low_speed_max": longest_below(exploration_velocities, 0.2),
        "return_low_speed_max": longest_below(return_velocities, 0.2),
        "speed_p90": percentile(speed_values, 0.9),
        "speed_max": max(speed_values) if speed_values else None,
        "low_speed_fraction": (
            sum(value < 0.2 for value in speed_values) / len(speed_values)
            if speed_values
            else None
        ),
        "longest_speed_below_0_2": longest_below(active_velocities, 0.2),
        "planned_speed_max": (
            max(planned_speed_values) if planned_speed_values else None
        ),
        "planned_acceleration_max": (
            max(planned_acceleration_values)
            if planned_acceleration_values
            else None
        ),
        "odom_rate": rate(odom_stamps),
        "command_faults": command_faults,
        "rosout_available": rosout_message_count > 0,
        "rosout_message_count": rosout_message_count,
        "super_failure_log_count": len(super_failures),
        "super_failure_breakdown": dict(super_failure_breakdown),
        "super_failure_examples": super_failures[:10],
        "status_changes": statuses,
        "state_changes": state_changes,
    }


def evaluate(report, arguments):
    if arguments.profile == "lab_1m":
        phase_speeds = (
            report["exploration_speed_median"],
            report["return_speed_median"],
        )
        moving_speed_median = (
            min(phase_speeds) if all(value is not None for value in phase_speeds)
            else None
        )
        unexpected_low_speed = max(
            report["exploration_low_speed_max"], report["return_low_speed_max"]
        )
    else:
        moving_speed_median = report["speed_median"]
        unexpected_low_speed = report["longest_speed_below_0_2"]
    report["profile"] = arguments.profile
    checks = {
        "entered_offboard": report["entered_offboard"],
        "mission_complete": report["mission_complete"],
        "map_closure_ready": report["map_closure_ready"],
        "entered_auto_loiter_after_offboard": report[
            "entered_auto_loiter_after_offboard"
        ],
        "no_command_faults": not report["command_faults"],
        "at_least_three_goals": report["goal_count"] >= 3,
        "goal_lifecycle_recorded": report["goal_lifecycle_available"],
        "goal_cancel_observed": report["goal_lifecycle"]["cancel_count"] > 0,
        "no_stale_position_commands": report["goal_lifecycle"]["stale_position_commands"] == 0,
        "no_stale_plan_from_rest": report["goal_lifecycle"]["stale_plan_from_rest_calls"] == 0,
        "minimum_path_length": report["path_length"] >= arguments.min_path_length,
        "return_horizontal_error": report["return_horizontal_error"] is not None
        and report["return_horizontal_error"] <= arguments.max_return_error,
        "cross_track_limit": report["cross_track_max"] is not None
        and report["cross_track_max"] <= arguments.max_cross_track,
        "median_cruise_speed": moving_speed_median is not None
        and moving_speed_median >= arguments.min_median_speed,
        "low_speed_duration": unexpected_low_speed <= arguments.max_low_speed_duration,
        "actual_speed_limit": report["speed_max"] is not None
        and report["speed_max"] <= arguments.max_actual_speed,
        "odometry_rate": report["odom_rate"] is not None
        and report["odom_rate"] >= arguments.min_odom_rate,
        "planned_speed_limit": report["planned_speed_max"] is not None
        and report["planned_speed_max"] <= arguments.max_planned_speed,
        "planned_acceleration_limit": report["planned_acceleration_max"] is not None
        and report["planned_acceleration_max"]
        <= arguments.max_planned_acceleration,
        "offboard_duration": report["offboard_duration"] is not None
        and report["offboard_duration"] <= arguments.max_offboard_duration,
    }
    if arguments.max_mission_altitude is not None:
        checks["mission_altitude_limit"] = (
            report["mission_altitude_max"] is not None
            and report["mission_altitude_max"] <= arguments.max_mission_altitude
        )
    if arguments.max_super_failures >= 0:
        checks["super_failure_logs_available"] = report["rosout_available"]
        checks["super_failure_limit"] = report["rosout_available"] and (
            report["super_failure_log_count"] <= arguments.max_super_failures
        )
    report["checks"] = checks
    report["failed_checks"] = [name for name, passed in checks.items() if not passed]
    report["verdict"] = "PASS" if not report["failed_checks"] else "FAIL"
    return report


def format_value(value, suffix=""):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}{suffix}"
    return f"{value}{suffix}"


def print_summary(report):
    print(f"TASK1_SITL_ACCEPTANCE [{report['profile']}]: {report['verdict']}")
    print(
        "completion: offboard=%s complete=%s map_closure=%s auto_loiter=%s"
        % (
            report["entered_offboard"],
            report["mission_complete"],
            report["map_closure_ready"],
            report["entered_auto_loiter_after_offboard"],
        )
    )
    print(
        "motion: duration=%s path=%s return_error=%s cross_track=%s "
        "mission_z_max=%s"
        % (
            format_value(report["offboard_duration"], "s"),
            format_value(report["path_length"], "m"),
            format_value(report["return_horizontal_error"], "m"),
            format_value(report["cross_track_max"], "m"),
            format_value(report["mission_altitude_max"], "m"),
        )
    )
    print(
        "speed: mean=%s median=%s p90=%s max=%s low<0.2_longest=%s"
        % (
            format_value(report["speed_mean"], "m/s"),
            format_value(report["speed_median"], "m/s"),
            format_value(report["speed_p90"], "m/s"),
            format_value(report["speed_max"], "m/s"),
            format_value(report["longest_speed_below_0_2"], "s"),
        )
    )
    if report["profile"] == "lab_1m":
        print(
            "moving_phases: exploration_median=%s return_median=%s "
            "exploration_low<0.2=%s return_low<0.2=%s"
            % (
                format_value(report["exploration_speed_median"], "m/s"),
                format_value(report["return_speed_median"], "m/s"),
                format_value(report["exploration_low_speed_max"], "s"),
                format_value(report["return_low_speed_max"], "s"),
            )
        )
    super_failure_text = (
        str(report["super_failure_log_count"])
        if report["rosout_available"]
        else "not_recorded"
    )
    print(
        "planner: speed_max=%s acceleration_max=%s odom_rate=%s "
        "bridge_faults=%d super_failure_logs=%s"
        % (
            format_value(report["planned_speed_max"], "m/s"),
            format_value(report["planned_acceleration_max"], "m/s^2"),
            format_value(report["odom_rate"], "Hz"),
            len(report["command_faults"]),
            super_failure_text,
        )
    )
    lifecycle = report["goal_lifecycle"]
    print(
        "goal_lifecycle: recorded=%s cancels=%d stale_pos_cmd=%d "
        "stale_plan_from_rest=%d cancel_to_silence=%s"
        % (
            report["goal_lifecycle_available"],
            lifecycle["cancel_count"],
            lifecycle["stale_position_commands"],
            lifecycle["stale_plan_from_rest_calls"],
            format_value(lifecycle["cancel_to_silence_latency"], "s"),
        )
    )
    if report["super_failure_breakdown"]:
        print(
            "super_failure_breakdown: "
            + ", ".join(
                f"{name}={count}"
                for name, count in sorted(
                    report["super_failure_breakdown"].items()
                )
            )
        )
    if report["failed_checks"]:
        print("failed_checks: " + ", ".join(report["failed_checks"]))


def main():
    arguments = parse_arguments()
    try:
        report = evaluate(analyze(arguments.bag), arguments)
    except (OSError, rosbag.bag.ROSBagException) as error:
        print(f"Unable to analyze bag: {error}", file=sys.stderr)
        return 2

    print_summary(report)
    if arguments.json_output:
        with open(arguments.json_output, "w", encoding="utf-8") as output:
            json.dump(report, output, indent=2, ensure_ascii=False)
            output.write("\n")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
