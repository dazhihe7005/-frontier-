#!/usr/bin/env python3
"""Observe and grade the isolated GBPlanner2 dynamic chain."""

import json
import math
import threading

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud, PointCloud2
from std_msgs.msg import Bool, Float32, String
from trajectory_msgs.msg import MultiDOFJointTrajectory


class Validator:
    TOPICS = {
        "raw_mid360": ("/mine_uav/sitl/mid360/points", PointCloud),
        "fastlio_input": ("/mine_uav/sitl/mid360/points2", PointCloud2),
        "fastlio_odometry": ("/Odometry", Odometry),
        "px4_vision_input": ("/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped),
        "planner_odometry": ("/mine_uav/gbplanner/planner_odometry", Odometry),
        "voxblox_input": ("/mine_uav/gbplanner/voxblox_points", PointCloud2),
        "voxblox_map": ("/gbplanner_node/tsdf_pointcloud", PointCloud2),
        "gbplanner_trajectory": ("/gbplanner/command/trajectory", MultiDOFJointTrajectory),
        "px4_setpoint": ("/mavros/setpoint_raw/local", PositionTarget),
    }

    def __init__(self):
        self.duration = max(20.0, float(rospy.get_param("~duration", 180.0)))
        self.observe_full_duration = bool(rospy.get_param(
            "~observe_full_duration", False))
        self.min_displacement = max(
            0.5, float(rospy.get_param("~min_displacement", 0.5)))
        self.counts = {key: 0 for key in self.TOPICS}
        self._lock = threading.Lock()
        self.states = set()
        self.executor_states = set()
        self.mission_states = set()
        self.first_position = None
        self.max_displacement = 0.0
        self.trajectory_points = 0
        self.setpoints_finite = True
        self.min_clearance = float("inf")
        self.clearance_violations = 0
        self.min_spatial_clearance = float("inf")
        self.spatial_clearance_violations = 0
        self.max_tracking_error = 0.0
        self.max_actual_speed = 0.0
        self.max_command_speed = 0.0
        self.min_flight_z = float("inf")
        self.max_flight_z = -float("inf")
        self.vision_false_samples = 0
        self._vision_seen_healthy = False
        self._flight_active = False
        self.blocked_events = 0
        self.recovery_events = 0
        self._last_blocked = False
        self._last_recovery = False
        self.latest_local_position = None
        self.latest_truth_position = None
        self.latest_fastlio_position = None
        self.local_reference = None
        self.truth_reference = None
        self.fastlio_reference = None
        self.max_truth_relative_error = 0.0
        self.max_fastlio_truth_relative_error = 0.0
        self.min_clearance_state = None
        for key, (topic, message_type) in self.TOPICS.items():
            rospy.Subscriber(topic, message_type,
                             lambda msg, name=key: self._topic_cb(name, msg),
                             queue_size=50)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=20)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=50)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped,
                         self._velocity_cb, queue_size=50)
        rospy.Subscriber("/mine_uav/gbplanner/horizontal_clearance", Float32,
                         self._clearance_cb, queue_size=50)
        rospy.Subscriber("/mine_uav/gbplanner/spatial_clearance", Float32,
                         self._spatial_clearance_cb, queue_size=50)
        rospy.Subscriber("/mine_uav/gbplanner/tracking_error", Float32,
                         self._tracking_cb, queue_size=50)
        rospy.Subscriber("/mine_uav/gbplanner/vision_healthy", Bool,
                         self._vision_cb, queue_size=20)
        rospy.Subscriber("/mine_uav/gbplanner/execution_blocked", Bool,
                         self._blocked_cb, queue_size=20)
        rospy.Subscriber("/mine_uav/gbplanner/recovery_active", Bool,
                         self._recovery_cb, queue_size=20)
        rospy.Subscriber("/mine_uav/gbplanner/executor_status", String,
                         lambda msg: self.executor_states.add(msg.data), queue_size=10)
        rospy.Subscriber("/mine_uav/gbplanner/mission_status", String,
                         lambda msg: self.mission_states.add(msg.data), queue_size=10)
        # Gazebo truth is diagnostic-only and never enters localization,
        # planning, trajectory execution or any flight-control setpoint.
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._truth_cb, queue_size=10)

    def _topic_cb(self, name, message):
        with self._lock:
            self.counts[name] += 1
            if name == "gbplanner_trajectory":
                self.trajectory_points = max(self.trajectory_points,
                                             len(message.points))
            elif name == "px4_setpoint":
                p = message.position
                self.setpoints_finite &= all(math.isfinite(v) for v in
                                              (p.x, p.y, p.z, message.yaw))
                velocity = message.velocity
                speed = math.sqrt(velocity.x**2 + velocity.y**2 +
                                  velocity.z**2)
                self.max_command_speed = max(self.max_command_speed, speed)
            elif name == "fastlio_odometry":
                p = message.pose.pose.position
                self.latest_fastlio_position = (p.x, p.y, p.z)
                self._update_truth_error()

    def _state_cb(self, message):
        with self._lock:
            if message.connected:
                self.states.add("CONNECTED")
            if message.armed:
                self.states.add("ARMED")
            if message.mode == "OFFBOARD":
                self.states.add("OFFBOARD")
            self._flight_active = bool(message.armed and
                                       message.mode == "OFFBOARD")

    def _pose_cb(self, message):
        p = message.pose.position
        current = (p.x, p.y, p.z)
        with self._lock:
            self.latest_local_position = current
            if self.first_position is None:
                self.first_position = current
            displacement = math.sqrt(sum((current[i]-self.first_position[i])**2
                                         for i in range(3)))
            self.max_displacement = max(self.max_displacement, displacement)
            self.min_flight_z = min(self.min_flight_z, p.z)
            self.max_flight_z = max(self.max_flight_z, p.z)
            self._update_truth_error()

    def _truth_cb(self, message):
        try:
            index = message.name.index("iris")
        except ValueError:
            return
        p = message.pose[index].position
        with self._lock:
            self.latest_truth_position = (p.x, p.y, p.z)
            self._update_truth_error()

    def _update_truth_error(self):
        if self.latest_local_position is None or self.latest_truth_position is None:
            return
        if self.local_reference is None:
            self.local_reference = self.latest_local_position
            self.truth_reference = self.latest_truth_position
        local_delta = tuple(self.latest_local_position[i]-self.local_reference[i]
                            for i in range(3))
        truth_delta = tuple(self.latest_truth_position[i]-self.truth_reference[i]
                            for i in range(3))
        error = math.sqrt(sum((local_delta[i]-truth_delta[i])**2
                              for i in range(3)))
        self.max_truth_relative_error = max(
            self.max_truth_relative_error, error)
        if self.latest_fastlio_position is not None:
            if self.fastlio_reference is None:
                self.fastlio_reference = self.latest_fastlio_position
            fastlio_delta = tuple(
                self.latest_fastlio_position[i]-self.fastlio_reference[i]
                for i in range(3))
            fastlio_error = math.sqrt(sum(
                (fastlio_delta[i]-truth_delta[i])**2 for i in range(3)))
            self.max_fastlio_truth_relative_error = max(
                self.max_fastlio_truth_relative_error, fastlio_error)

    def _velocity_cb(self, message):
        velocity = message.twist.linear
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        with self._lock:
            self.max_actual_speed = max(self.max_actual_speed, speed)

    def _clearance_cb(self, message):
        if not math.isfinite(message.data):
            return
        with self._lock:
            if message.data < self.min_clearance:
                self.min_clearance = message.data
                self.min_clearance_state = {
                    "local": self.latest_local_position,
                    "fastlio": self.latest_fastlio_position,
                    "truth": self.latest_truth_position,
                }
            if message.data < 1.0:
                self.clearance_violations += 1

    def _tracking_cb(self, message):
        if math.isfinite(message.data):
            with self._lock:
                self.max_tracking_error = max(
                    self.max_tracking_error, message.data)

    def _spatial_clearance_cb(self, message):
        if not math.isfinite(message.data):
            return
        with self._lock:
            self.min_spatial_clearance = min(
                self.min_spatial_clearance, message.data)
            if message.data < 1.0:
                self.spatial_clearance_violations += 1

    def _vision_cb(self, message):
        with self._lock:
            if message.data:
                self._vision_seen_healthy = True
            elif self._flight_active and self._vision_seen_healthy:
                # Startup is intentionally unhealthy until FAST-LIO2 has
                # initialized.  Continuity is a flight-time property, so do
                # not turn that correct preflight gate into a test failure.
                self.vision_false_samples += 1

    def _blocked_cb(self, message):
        with self._lock:
            value = bool(message.data)
            if value and not self._last_blocked:
                self.blocked_events += 1
            self._last_blocked = value

    def _recovery_cb(self, message):
        with self._lock:
            value = bool(message.data)
            if value and not self._last_recovery:
                self.recovery_events += 1
            self._last_recovery = value

    def run(self):
        started = rospy.Time.now()
        rate = rospy.Rate(2)
        interrupted = False
        try:
            while not rospy.is_shutdown() and \
                    (rospy.Time.now()-started).to_sec() < self.duration:
                with self._lock:
                    complete = (self.counts["gbplanner_trajectory"] >= 1 and
                                self.counts["px4_setpoint"] >= 100 and
                                self.max_displacement >= 1.0 and
                                "OFFBOARD" in self.states)
                if complete and not self.observe_full_duration:
                    break
                rate.sleep()
        except rospy.ROSInterruptException:
            interrupted = True
        elapsed = max(0.0, (rospy.Time.now()-started).to_sec())
        observation_complete = (
            not interrupted and
            (not self.observe_full_duration or elapsed >= self.duration))
        with self._lock:
            checks = {
                "observation_complete": observation_complete,
                "mid360_to_fastlio": self.counts["raw_mid360"] >= 5 and
                                      self.counts["fastlio_input"] >= 5,
                "fastlio_running": self.counts["fastlio_odometry"] >= 5,
                "fastlio_to_px4": self.counts["px4_vision_input"] >= 10,
                "fastlio_to_gbplanner": self.counts["planner_odometry"] >= 5 and
                                         self.counts["voxblox_input"] >= 5,
                "voxblox_integrated": self.counts["voxblox_map"] >= 1,
                "gbplanner_planned": self.counts["gbplanner_trajectory"] >= 1 and
                                     self.trajectory_points >= 2,
                "trajectory_to_px4": self.counts["px4_setpoint"] >= 50 and
                                     self.setpoints_finite,
                "px4_dynamic_flight": {"CONNECTED", "ARMED", "OFFBOARD"}.issubset(self.states) and
                                      self.max_displacement >= self.min_displacement,
                "executor_streaming": "STREAMING_TO_PX4" in self.executor_states,
                "hard_1m_clearance": math.isfinite(self.min_clearance) and
                                     self.min_clearance >= 1.0 and
                                     math.isfinite(self.min_spatial_clearance) and
                                     self.min_spatial_clearance >= 1.0,
                "vision_continuity": self.vision_false_samples == 0,
            }
            result = {
                "passed": all(checks.values()),
                "checks": checks,
                "counts": dict(self.counts),
                "px4_states_seen": sorted(self.states),
                "executor_states_seen": sorted(self.executor_states),
                "mission_states_seen": sorted(self.mission_states),
                "max_px4_displacement_m": round(self.max_displacement, 4),
                "max_trajectory_points": self.trajectory_points,
                "setpoints_finite": self.setpoints_finite,
                "min_horizontal_clearance_m": (
                    round(self.min_clearance, 4)
                    if math.isfinite(self.min_clearance) else None),
                "clearance_samples_below_1m": self.clearance_violations,
                "min_spatial_clearance_m": (
                    round(self.min_spatial_clearance, 4)
                    if math.isfinite(self.min_spatial_clearance) else None),
                "spatial_clearance_samples_below_1m": (
                    self.spatial_clearance_violations),
                "max_tracking_error_m": round(self.max_tracking_error, 4),
                "max_actual_speed_mps": round(self.max_actual_speed, 4),
                "max_command_speed_mps": round(self.max_command_speed, 4),
                "flight_z_range_m": [
                    round(self.min_flight_z, 4), round(self.max_flight_z, 4)]
                    if math.isfinite(self.min_flight_z) else None,
                "vision_unhealthy_samples": self.vision_false_samples,
                "blocked_events": self.blocked_events,
                "recovery_events": self.recovery_events,
                "observed_duration_s": round(elapsed, 3),
                "max_fastlio_px4_vs_truth_relative_error_m": round(
                    self.max_truth_relative_error, 4),
                "max_fastlio_vs_truth_relative_error_m": round(
                    self.max_fastlio_truth_relative_error, 4),
                "diagnostic_references": {
                    "local": self.local_reference,
                    "fastlio": self.fastlio_reference,
                    "truth": self.truth_reference,
                },
                "min_clearance_state": self.min_clearance_state,
            }
        print("GBPLANNER_FULL_CHAIN_RESULT=" + json.dumps(result, sort_keys=True))
        return result["passed"]


if __name__ == "__main__":
    rospy.init_node("gbplanner_full_chain_validator")
    raise SystemExit(0 if Validator().run() else 1)
