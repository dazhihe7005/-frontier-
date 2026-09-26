#!/usr/bin/env python3
"""Record plans projected into Gazebo truth for offline mesh diagnosis only.

This node publishes nothing. The pose pairing is approximate and must never
be used to alter a planner, safety controller, estimator, or PX4 setpoint.
"""

import csv
from datetime import datetime
import math
import os
from pathlib import Path
import threading

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory


def yaw_of(quaternion):
    x, y, z, w = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1.0e-8:
        raise ValueError("invalid orientation")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))


def project_point(point, odom_xyz, odom_yaw, truth_xyz, truth_yaw):
    """Diagnostic SE(2)+Z projection anchored at two current body poses."""
    delta = truth_yaw-odom_yaw
    cosine, sine = math.cos(delta), math.sin(delta)
    dx, dy = point[0]-odom_xyz[0], point[1]-odom_xyz[1]
    return (truth_xyz[0]+cosine*dx-sine*dy,
            truth_xyz[1]+sine*dx+cosine*dy,
            truth_xyz[2]+point[2]-odom_xyz[2])


def pose_tuple(pose):
    position = pose.position
    return ((position.x, position.y, position.z),
            yaw_of(pose.orientation))


class PlannedTrajectoryRecorder:
    def __init__(self):
        model_name = rospy.get_param("~model_name", "iris")
        self.model_name = model_name
        self.run_id = rospy.get_param("/run_id", "")
        self.max_anchor_age = max(0.05, float(rospy.get_param(
            "~max_anchor_age", 0.30)))
        output_dir = Path("/home/nuc/gbplanner2_isolated_ws/runtime/map_reference")
        default_name = "planned_trajectory_{}_{}.csv".format(
            datetime.now().strftime("%Y%m%d_%H%M%S_%f"), os.getpid())
        self.output = Path(rospy.get_param(
            "~output_file", str(output_dir / default_name)))
        self.raw_output = Path(rospy.get_param(
            "~raw_output_file", str(output_dir / default_name.replace(
                "planned_trajectory_", "raw_planner_path_"))))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.raw_output.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output.open("w", encoding="utf-8", newline="")
        self.raw_file = self.raw_output.open("w", encoding="utf-8", newline="")
        self.writer = csv.writer(self.file)
        self.raw_writer = csv.writer(self.raw_file)
        self.writer.writerow((
            "sim_time", "trajectory_index", "header_seq", "point_index",
            "point_time_s", "x", "y", "z", "raw_x", "raw_y", "raw_z",
            "truth_anchor_x", "truth_anchor_y", "truth_anchor_z",
            "odom_anchor_x", "odom_anchor_y", "odom_anchor_z",
            "yaw_delta_rad", "truth_age_s", "odom_age_s", "first_point_error_m",
            "ros_run_id"))
        self.raw_writer.writerow((
            "sim_time", "header_seq", "point_index", "x", "y", "z",
            "raw_x", "raw_y", "raw_z", "first_point_error_m",
            "truth_age_s", "odom_age_s", "ros_run_id"))
        self.lock = threading.Lock()
        self.truth = None
        self.odom = None
        self.trajectory_index = 0
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._truth_cb, queue_size=10)
        rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                         self._odom_cb, queue_size=10)
        rospy.Subscriber("/gbplanner/command/trajectory", MultiDOFJointTrajectory,
                         self._trajectory_cb, queue_size=10)
        rospy.Subscriber("/gbplanner/diagnostic_raw_path", PoseArray,
                         self._raw_path_cb, queue_size=10)
        rospy.on_shutdown(self._close)
        rospy.loginfo("Read-only planned trajectory projection: %s", self.output)
        rospy.loginfo("Read-only raw planner path projection: %s", self.raw_output)

    def _truth_cb(self, message):
        try:
            index = message.name.index(self.model_name)
            pose = pose_tuple(message.pose[index])
        except (ValueError, IndexError):
            return
        with self.lock:
            self.truth = (rospy.Time.now(), pose)

    def _odom_cb(self, message):
        try:
            pose = pose_tuple(message.pose.pose)
        except ValueError:
            return
        with self.lock:
            self.odom = (rospy.Time.now(), pose)

    def _trajectory_cb(self, message):
        now = rospy.Time.now()
        with self.lock:
            truth, odom = self.truth, self.odom
            self.trajectory_index += 1
            index = self.trajectory_index
        if truth is None or odom is None:
            return
        truth_age = (now-truth[0]).to_sec()
        odom_age = (now-odom[0]).to_sec()
        if not (0.0 <= truth_age <= self.max_anchor_age and
                0.0 <= odom_age <= self.max_anchor_age):
            rospy.logwarn_throttle(2.0, "Plan mesh audit skipped stale pose anchor")
            return
        truth_xyz, truth_yaw = truth[1]
        odom_xyz, odom_yaw = odom[1]
        if not message.points or not message.points[0].transforms:
            return
        first = message.points[0].transforms[0].translation
        first_error = math.dist((first.x, first.y, first.z), odom_xyz)
        for point_index, sample in enumerate(message.points):
            if not sample.transforms:
                continue
            position = sample.transforms[0].translation
            raw = (position.x, position.y, position.z)
            if not all(math.isfinite(value) for value in raw):
                continue
            projected = project_point(
                raw, odom_xyz, odom_yaw, truth_xyz, truth_yaw)
            self.writer.writerow((
                "{:.3f}".format(now.to_sec()), index, message.header.seq,
                point_index, "{:.3f}".format(sample.time_from_start.to_sec()),
                *("{:.5f}".format(value) for value in projected),
                *("{:.5f}".format(value) for value in raw),
                *("{:.5f}".format(value) for value in truth_xyz),
                *("{:.5f}".format(value) for value in odom_xyz),
                "{:.5f}".format(truth_yaw-odom_yaw),
                "{:.3f}".format(truth_age),
                "{:.3f}".format(odom_age),
                "{:.5f}".format(first_error), self.run_id))
        self.file.flush()

    def _raw_path_cb(self, message):
        now = rospy.Time.now()
        with self.lock:
            truth, odom = self.truth, self.odom
        if truth is None or odom is None or not message.poses:
            return
        truth_age = (now-truth[0]).to_sec()
        odom_age = (now-odom[0]).to_sec()
        if not (0.0 <= truth_age <= self.max_anchor_age and
                0.0 <= odom_age <= self.max_anchor_age):
            return
        truth_xyz, truth_yaw = truth[1]
        odom_xyz, odom_yaw = odom[1]
        first = message.poses[0].position
        first_error = math.dist((first.x, first.y, first.z), odom_xyz)
        for index, pose in enumerate(message.poses):
            point = pose.position
            raw = (point.x, point.y, point.z)
            if not all(math.isfinite(value) for value in raw):
                continue
            projected = project_point(
                raw, odom_xyz, odom_yaw, truth_xyz, truth_yaw)
            self.raw_writer.writerow((
                "{:.3f}".format(now.to_sec()), message.header.seq, index,
                *("{:.5f}".format(value) for value in projected),
                *("{:.5f}".format(value) for value in raw),
                "{:.5f}".format(first_error),
                "{:.3f}".format(truth_age), "{:.3f}".format(odom_age),
                self.run_id))
        self.raw_file.flush()

    def _close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()
        if not self.raw_file.closed:
            self.raw_file.flush()
            self.raw_file.close()


if __name__ == "__main__":
    rospy.init_node("diagnostic_planned_trajectory")
    PlannedTrajectoryRecorder()
    rospy.spin()
