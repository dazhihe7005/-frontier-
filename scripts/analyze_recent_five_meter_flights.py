#!/usr/bin/env python3
"""Plot SUPER route tracking and PX4 yaw response for real five-metre flights."""

import argparse
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as point_cloud2
from pyulog import ULog


def wrap_angle(value):
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def quaternion_yaw(w, x, y, z):
    return np.arctan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def ros_quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ulog_dataset(ulog, name):
    return next(dataset.data for dataset in ulog.data_list
                if dataset.name == name and dataset.multi_id == 0)


def state_transitions(status):
    times = np.asarray(status["timestamp"], dtype=float) / 1.0e6
    transitions = []
    previous = None
    for index, nav_state in enumerate(status["nav_state"]):
        nav_state = int(nav_state)
        if nav_state != previous:
            transitions.append((times[index], nav_state))
            previous = nav_state
    return transitions


def first_time(transitions, predicate, after=-math.inf):
    return next(time for time, state in transitions
                if time > after and predicate(state))


def read_ros_flight(path):
    active_time = None
    complete_time = None
    offboard_time = None
    offboard_exit_time = None
    goal = None
    planner = []
    bridge = []
    actual = []
    state_previous = None

    topics = [
        "/mine_uav/five_meter_test/status",
        "/mine_uav/super/goal_command",
        "/planning/pos_cmd",
        "/mine_uav/setpoint_cmd",
        "/mine_uav/task1/odometry",
        "/mavros/state",
    ]
    with rosbag.Bag(path) as bag:
        for topic, message, stamp in bag.read_messages(topics=topics):
            time = stamp.to_sec()
            if topic == "/mine_uav/five_meter_test/status":
                text = message.data
                if "state=ACTIVE" in text and active_time is None:
                    active_time = time
                if "state=COMPLETE" in text and complete_time is None:
                    complete_time = time
            elif topic == "/mine_uav/super/goal_command":
                if int(message.command) == 1 and goal is None:
                    goal = (message.goal.position.x, message.goal.position.y,
                            message.goal.position.z)
            elif topic == "/mavros/state":
                mode = message.mode
                if mode != state_previous:
                    if mode == "OFFBOARD" and offboard_time is None:
                        offboard_time = time
                    elif (state_previous == "OFFBOARD" and mode != "OFFBOARD"
                          and offboard_exit_time is None):
                        offboard_exit_time = time
                    state_previous = mode
            elif topic == "/planning/pos_cmd":
                if message.trajectory_flag in (1, 2):
                    planner.append((time, message.position.x, message.position.y,
                                    message.position.z, message.yaw,
                                    message.yaw_dot))
            elif topic == "/mine_uav/setpoint_cmd":
                bridge.append((time, message.position.x, message.position.y,
                               message.position.z, message.yaw,
                               message.yaw_rate))
            elif topic == "/mine_uav/task1/odometry":
                position = message.pose.pose.position
                actual.append((time, position.x, position.y, position.z,
                               ros_quaternion_yaw(
                                   message.pose.pose.orientation)))

    if active_time is None or complete_time is None or offboard_time is None:
        raise RuntimeError("bag does not contain a complete five-metre task")
    if offboard_exit_time is None:
        offboard_exit_time = complete_time

    def select(rows, start, stop):
        return np.asarray([row for row in rows if start <= row[0] <= stop],
                          dtype=float)

    planner = select(planner, active_time, complete_time)
    bridge = select(bridge, active_time, offboard_exit_time)
    actual = select(actual, active_time - 2.0, offboard_exit_time + 3.0)
    if planner.size == 0 or actual.size == 0 or bridge.size == 0:
        raise RuntimeError("bag is missing planner, bridge, or odometry samples")

    altitude = float(np.median(actual[:, 3]))
    obstacle_voxels = {}
    cloud_index = 0
    with rosbag.Bag(path) as bag:
        for _, cloud, stamp in bag.read_messages(
                topics=["/mine_uav/task1/cloud_registered"]):
            time = stamp.to_sec()
            if time < active_time or time > complete_time:
                continue
            cloud_index += 1
            if cloud_index % 4 != 0:
                continue
            for x, y, z in point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=True):
                if z < 0.0 or abs(z - altitude) >= 0.5:
                    continue
                key = (int(math.floor(x / 0.10)),
                       int(math.floor(y / 0.10)),
                       int(math.floor(z / 0.10)))
                obstacle_voxels[key] = (x, y, z)
    obstacles = np.asarray(list(obstacle_voxels.values()), dtype=float)
    return {
        "active_time": active_time,
        "complete_time": complete_time,
        "offboard_time": offboard_time,
        "offboard_exit_time": offboard_exit_time,
        "goal": goal,
        "planner": planner,
        "bridge": bridge,
        "actual": actual,
        "obstacles": obstacles,
    }


def read_ulog_flight(path):
    ulog = ULog(path)
    status = ulog_dataset(ulog, "vehicle_status")
    transitions = state_transitions(status)
    offboard_time = first_time(transitions, lambda state: state == 14)
    exit_time = first_time(transitions, lambda state: state != 14,
                           after=offboard_time)

    attitude = ulog_dataset(ulog, "vehicle_attitude")
    attitude_sp = ulog_dataset(ulog, "vehicle_attitude_setpoint")
    angular_velocity = ulog_dataset(ulog, "vehicle_angular_velocity")
    rates_sp = ulog_dataset(ulog, "vehicle_rates_setpoint")
    rate_status = ulog_dataset(ulog, "rate_ctrl_status")
    manual = ulog_dataset(ulog, "manual_control_setpoint")
    allocator = ulog_dataset(ulog, "control_allocator_status")
    local_position = ulog_dataset(ulog, "vehicle_local_position")

    attitude_time = np.asarray(attitude["timestamp"], dtype=float) / 1.0e6
    actual_yaw = quaternion_yaw(
        *[np.asarray(attitude[f"q[{index}]"], dtype=float)
          for index in range(4)])
    sp_time = np.asarray(attitude_sp["timestamp"], dtype=float) / 1.0e6
    desired_yaw = quaternion_yaw(
        *[np.asarray(attitude_sp[f"q_d[{index}]"], dtype=float)
          for index in range(4)])
    interpolated_sp = np.interp(attitude_time, sp_time,
                                np.unwrap(desired_yaw))
    yaw_error = wrap_angle(interpolated_sp - actual_yaw)

    rate_time = np.asarray(angular_velocity["timestamp"], dtype=float) / 1.0e6
    actual_yaw_rate = np.asarray(angular_velocity["xyz[2]"], dtype=float)
    desired_yaw_rate = np.interp(
        rate_time, np.asarray(rates_sp["timestamp"], dtype=float) / 1.0e6,
        np.asarray(rates_sp["yaw"], dtype=float))

    def interval_stats(start, stop):
        attitude_mask = ((attitude_time >= start) & (attitude_time < stop))
        rate_mask = (rate_time >= start) & (rate_time < stop)
        error = yaw_error[attitude_mask]
        rate_error = (desired_yaw_rate[rate_mask] -
                      actual_yaw_rate[rate_mask])
        return {
            "yaw_actual_peak_to_peak_deg": float(np.degrees(np.ptp(
                np.unwrap(actual_yaw[attitude_mask])))),
            "yaw_setpoint_peak_to_peak_deg": float(np.degrees(np.ptp(
                np.unwrap(interpolated_sp[attitude_mask])))),
            "yaw_error_rms_deg": float(np.degrees(np.sqrt(np.mean(
                error * error)))),
            "yaw_error_max_deg": float(np.degrees(np.max(np.abs(error)))),
            "yaw_rate_error_rms_deg_s": float(np.degrees(np.sqrt(np.mean(
                rate_error * rate_error)))),
            "yaw_rate_error_max_deg_s": float(np.degrees(np.max(
                np.abs(rate_error)))),
        }

    integral_time = np.asarray(rate_status["timestamp"], dtype=float) / 1.0e6
    offboard_integral_mask = ((integral_time >= offboard_time) &
                              (integral_time < exit_time))
    manual_time = np.asarray(manual["timestamp"], dtype=float) / 1.0e6
    post_manual_mask = ((manual_time >= exit_time) &
                        (manual_time < exit_time + 3.0))
    allocator_time = np.asarray(allocator["timestamp"], dtype=float) / 1.0e6
    allocator_mask = ((allocator_time >= offboard_time) &
                      (allocator_time < exit_time))

    reset_counters = {
        "attitude": sorted(set(int(value) for value in
                               attitude["quat_reset_counter"])),
        "heading": sorted(set(int(value) for value in
                              local_position["heading_reset_counter"])),
        "xy": sorted(set(int(value) for value in
                         local_position["xy_reset_counter"])),
    }
    return {
        "offboard_time": offboard_time,
        "exit_time": exit_time,
        "attitude_time": attitude_time,
        "actual_yaw": actual_yaw,
        "sp_time": sp_time,
        "desired_yaw": desired_yaw,
        "manual_time": manual_time,
        "manual_yaw": np.asarray(manual["yaw"], dtype=float),
        "pre5": interval_stats(offboard_time - 5.0, offboard_time),
        "offboard": interval_stats(offboard_time, exit_time),
        "post3": interval_stats(exit_time, min(exit_time + 3.0,
                                                attitude_time[-1])),
        "yaw_integrator_min": float(np.min(
            np.asarray(rate_status["yawspeed_integ"])[offboard_integral_mask])),
        "yaw_integrator_max": float(np.max(
            np.asarray(rate_status["yawspeed_integ"])[offboard_integral_mask])),
        "post_exit_rc_yaw_mean": float(np.mean(
            np.asarray(manual["yaw"])[post_manual_mask])),
        "post_exit_rc_yaw_max_abs": float(np.max(np.abs(
            np.asarray(manual["yaw"])[post_manual_mask]))),
        "allocator_torque_achieved": bool(np.all(
            np.asarray(allocator["torque_setpoint_achieved"])[allocator_mask])),
        "reset_counters": reset_counters,
        "parameters": {
            name: float(ulog.initial_parameters[name])
            for name in ("MC_YAW_P", "MC_YAWRATE_P", "MC_YAWRATE_I",
                         "MC_YAWRATE_D", "MC_YR_INT_LIM")
            if name in ulog.initial_parameters
        },
    }


def plot_routes(flights, labels, output_path):
    figure, axes = plt.subplots(1, len(flights), figsize=(7 * len(flights), 6),
                                squeeze=False)
    for axis, flight, label in zip(axes[0], flights, labels):
        obstacles = flight["obstacles"]
        if obstacles.size:
            axis.scatter(obstacles[:, 0], obstacles[:, 1], s=4,
                         color="0.55", alpha=0.32,
                         label="Obstacle points (relevant height)")
        planned = flight["planner"]
        actual = flight["actual"]
        axis.plot(planned[:, 1], planned[:, 2], color="#1565c0",
                  linewidth=2.0, label="SUPER command")
        axis.plot(actual[:, 1], actual[:, 2], color="#d32f2f",
                  linewidth=2.0, label="Actual odometry")
        axis.scatter([planned[0, 1]], [planned[0, 2]], marker="o", s=70,
                     color="#2e7d32", label="Start")
        if flight["goal"] is not None:
            axis.scatter([flight["goal"][0]], [flight["goal"][1]],
                         marker="*", s=130, color="#6a1b9a", label="Goal")
        axis.set_title(label)
        axis.set_xlabel("Task-frame x / m (forward)")
        axis.set_ylabel("Task-frame y / m (left +)")
        axis.grid(True, alpha=0.25)
        axis.axis("equal")
        axis.legend(loc="best", fontsize=8)
    figure.suptitle("Latest two real flights: SUPER route vs actual route")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_yaw(ros_flights, ulog_flights, labels, output_path):
    figure, axes = plt.subplots(len(labels), 1, figsize=(12, 5 * len(labels)),
                                squeeze=False)
    for axis, ros_flight, ulog_flight, label in zip(
            axes[:, 0], ros_flights, ulog_flights, labels):
        offboard = ulog_flight["offboard_time"]
        exit_time = ulog_flight["exit_time"]
        attitude_time = ulog_flight["attitude_time"] - offboard
        desired_time = ulog_flight["sp_time"] - offboard
        axis.plot(attitude_time, np.degrees(np.unwrap(
            ulog_flight["actual_yaw"])), color="#d32f2f", linewidth=1.6,
                  label="PX4 actual yaw")
        axis.plot(desired_time, np.degrees(np.unwrap(
            ulog_flight["desired_yaw"])), color="#1565c0", linewidth=1.6,
                  label="PX4 attitude yaw setpoint")
        bridge = ros_flight["bridge"]
        ros_offboard = ros_flight["offboard_time"]
        # MAVROS converts ROS ENU yaw into PX4 NED yaw as pi/2-yaw. Plot the
        # converted value so it can be compared directly with the ULog.
        bridge_yaw_ned = np.pi / 2.0 - bridge[:, 4]
        axis.plot(bridge[:, 0] - ros_offboard,
                  np.degrees(np.unwrap(bridge_yaw_ned)), color="#6a1b9a",
                  linewidth=1.0, alpha=0.8,
                  label="Bridge yaw command (PX4/NED)")
        axis.axvspan(0.0, exit_time - offboard, color="#90caf9", alpha=0.12,
                     label="OFFBOARD")
        axis.axvline(0.0, color="0.2", linestyle="--", linewidth=1.0)
        axis.axvline(exit_time - offboard, color="0.2", linestyle="--",
                     linewidth=1.0)
        axis.set_xlim(-5.0, exit_time - offboard + 5.0)
        axis.set_title(label)
        axis.set_xlabel("Time from OFFBOARD entry / s")
        axis.set_ylabel("Yaw / deg")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=8)
        rc_axis = axis.twinx()
        rc_axis.plot(ulog_flight["manual_time"] - offboard,
                     ulog_flight["manual_yaw"], color="#ef6c00",
                     linewidth=0.8, alpha=0.55, label="RC yaw input")
        rc_axis.set_ylabel("RC yaw input")
        rc_axis.set_ylim(-1.05, 1.05)
    figure.suptitle("Yaw command and response around autonomous flight")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", action="append", required=True)
    parser.add_argument("--ulog", action="append", required=True)
    parser.add_argument("--label", action="append", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    if not (len(args.bag) == len(args.ulog) == len(args.label)):
        parser.error("--bag, --ulog and --label counts must match")
    os.makedirs(args.output_dir, exist_ok=True)
    ros_flights = [read_ros_flight(path) for path in args.bag]
    ulog_flights = [read_ulog_flight(path) for path in args.ulog]
    plot_routes(ros_flights, args.label,
                os.path.join(args.output_dir, "latest_two_routes.png"))
    plot_yaw(ros_flights, ulog_flights, args.label,
             os.path.join(args.output_dir, "latest_two_yaw.png"))
    summaries = []
    for label, ros_flight, ulog_flight in zip(
            args.label, ros_flights, ulog_flights):
        planned = ros_flight["planner"]
        actual = ros_flight["actual"]
        tracking_mask = ((actual[:, 0] >= planned[0, 0]) &
                         (actual[:, 0] <= planned[-1, 0]))
        tracking_actual = actual[tracking_mask]
        tracking_reference = np.column_stack([
            np.interp(tracking_actual[:, 0], planned[:, 0], planned[:, axis])
            for axis in (1, 2, 3)
        ])
        tracking_error = tracking_actual[:, 1:4] - tracking_reference
        horizontal_error = np.linalg.norm(tracking_error[:, :2], axis=1)
        bridge_yaw_rate_deg_s = np.degrees(ros_flight["bridge"][:, 5])
        obstacle_clearance = {}
        obstacles = ros_flight["obstacles"]
        if obstacles.size:
            for route_name, route in (("planner", planned[:, 1:3]),
                                      ("actual", tracking_actual[:, 1:3])):
                chunk_minima = []
                for start in range(0, len(route), 100):
                    delta = (route[start:start + 100, None, :] -
                             obstacles[None, :, :2])
                    chunk_minima.append(float(np.min(np.linalg.norm(
                        delta, axis=2))))
                obstacle_clearance[route_name + "_min_horizontal_m"] = min(
                    chunk_minima)
        summaries.append({
            "label": label,
            "task_duration_s": (ros_flight["complete_time"] -
                                ros_flight["active_time"]),
            "offboard_duration_s": (ulog_flight["exit_time"] -
                                    ulog_flight["offboard_time"]),
            "planner_bounds_xyz_m": {
                "min": np.min(planned[:, 1:4], axis=0).tolist(),
                "max": np.max(planned[:, 1:4], axis=0).tolist(),
            },
            "actual_bounds_xyz_m": {
                "min": np.min(actual[:, 1:4], axis=0).tolist(),
                "max": np.max(actual[:, 1:4], axis=0).tolist(),
            },
            "tracking_error_m": {
                "horizontal_rms": float(np.sqrt(np.mean(
                    horizontal_error * horizontal_error))),
                "horizontal_max": float(np.max(horizontal_error)),
                "vertical_rms": float(np.sqrt(np.mean(
                    tracking_error[:, 2] * tracking_error[:, 2]))),
                "vertical_max_abs": float(np.max(np.abs(
                    tracking_error[:, 2]))),
            },
            "observed_obstacle_clearance": obstacle_clearance,
            "super_bridge_yaw_command": {
                "peak_to_peak_deg": float(np.degrees(np.ptp(np.unwrap(
                    ros_flight["bridge"][:, 4])))),
                "yaw_rate_rms_deg_s": float(np.sqrt(np.mean(
                    bridge_yaw_rate_deg_s * bridge_yaw_rate_deg_s))),
                "yaw_rate_max_abs_deg_s": float(np.max(np.abs(
                    bridge_yaw_rate_deg_s))),
            },
            "pre5": ulog_flight["pre5"],
            "offboard": ulog_flight["offboard"],
            "post3": ulog_flight["post3"],
            "yaw_integrator_min": ulog_flight["yaw_integrator_min"],
            "yaw_integrator_max": ulog_flight["yaw_integrator_max"],
            "post_exit_rc_yaw_mean": ulog_flight["post_exit_rc_yaw_mean"],
            "post_exit_rc_yaw_max_abs":
                ulog_flight["post_exit_rc_yaw_max_abs"],
            "allocator_torque_achieved":
                ulog_flight["allocator_torque_achieved"],
            "reset_counters": ulog_flight["reset_counters"],
            "parameters": ulog_flight["parameters"],
        })
    with open(os.path.join(args.output_dir, "summary.json"), "w",
              encoding="utf-8") as stream:
        json.dump(summaries, stream, indent=2, ensure_ascii=False)
    print(json.dumps(summaries, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
