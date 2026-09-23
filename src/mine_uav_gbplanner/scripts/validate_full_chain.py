#!/usr/bin/env python3
"""Observe and grade the isolated GBPlanner2 dynamic chain."""

import json
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud, PointCloud2
from std_msgs.msg import String
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
        self.counts = {key: 0 for key in self.TOPICS}
        self._lock = threading.Lock()
        self.states = set()
        self.executor_states = set()
        self.mission_states = set()
        self.first_position = None
        self.max_displacement = 0.0
        self.trajectory_points = 0
        self.setpoints_finite = True
        for key, (topic, message_type) in self.TOPICS.items():
            rospy.Subscriber(topic, message_type,
                             lambda msg, name=key: self._topic_cb(name, msg),
                             queue_size=50)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=20)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=50)
        rospy.Subscriber("/mine_uav/gbplanner/executor_status", String,
                         lambda msg: self.executor_states.add(msg.data), queue_size=10)
        rospy.Subscriber("/mine_uav/gbplanner/mission_status", String,
                         lambda msg: self.mission_states.add(msg.data), queue_size=10)

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

    def _state_cb(self, message):
        with self._lock:
            if message.connected:
                self.states.add("CONNECTED")
            if message.armed:
                self.states.add("ARMED")
            if message.mode == "OFFBOARD":
                self.states.add("OFFBOARD")

    def _pose_cb(self, message):
        p = message.pose.position
        current = (p.x, p.y, p.z)
        with self._lock:
            if self.first_position is None:
                self.first_position = current
            displacement = math.sqrt(sum((current[i]-self.first_position[i])**2
                                         for i in range(3)))
            self.max_displacement = max(self.max_displacement, displacement)

    def run(self):
        started = rospy.Time.now()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown() and \
                (rospy.Time.now()-started).to_sec() < self.duration:
            with self._lock:
                complete = (self.counts["gbplanner_trajectory"] >= 1 and
                            self.counts["px4_setpoint"] >= 100 and
                            self.max_displacement >= 1.0 and
                            "OFFBOARD" in self.states)
            if complete:
                break
            rate.sleep()
        with self._lock:
            checks = {
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
                                      self.max_displacement >= 0.5,
                "executor_streaming": "STREAMING_TO_PX4" in self.executor_states,
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
            }
        print("GBPLANNER_FULL_CHAIN_RESULT=" + json.dumps(result, sort_keys=True))
        return result["passed"]


if __name__ == "__main__":
    rospy.init_node("gbplanner_full_chain_validator")
    raise SystemExit(0 if Validator().run() else 1)
