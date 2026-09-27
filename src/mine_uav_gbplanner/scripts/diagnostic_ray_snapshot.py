#!/usr/bin/env python3
"""Capture one raw MID360 scan with diagnostic-only Gazebo truth pose.

Never publish truth, a map, or a control command. The CSV is used offline to
compare Gazebo ray returns with the exact collision meshes at the same pose.
"""

import csv
from collections import deque
from datetime import datetime
import math
import os
from pathlib import Path

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud


def rotate(q, p):
    x, y, z, w = q
    px, py, pz = p
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz,
            pz+w*tz+x*ty-y*tx)


def interpolate_pose(poses, stamp, max_gap):
    """Interpolate callback-timestamped truth at a scan's header stamp.

    ModelStates has no header, so the callback time is only an approximation.
    Refuse extrapolation and expose the bracket width in the CSV instead of
    silently treating the latest pose as the scan acquisition pose.
    """
    for earlier, later in zip(poses, poses[1:]):
        t0, p0, q0 = earlier
        t1, p1, q1 = later
        if not (t0 <= stamp <= t1):
            continue
        if t1 <= t0 or stamp-t0 > max_gap or t1-stamp > max_gap:
            return None
        fraction = (stamp-t0)/(t1-t0)
        position = tuple((1.0-fraction)*p0[i]+fraction*p1[i]
                         for i in range(3))
        if sum(q0[i]*q1[i] for i in range(4)) < 0.0:
            q1 = tuple(-v for v in q1)
        quaternion = tuple((1.0-fraction)*q0[i]+fraction*q1[i]
                           for i in range(4))
        norm = math.sqrt(sum(v*v for v in quaternion))
        if norm < 1.0e-8:
            return None
        return (position, tuple(v/norm for v in quaternion), t1-t0,
                max(stamp-t0, t1-stamp))
    return None


def allowed_world_x(body_x, world_x_range):
    """Optional spatial capture gate; no truth is published or fed to control."""
    return not world_x_range or world_x_range[0] <= body_x <= world_x_range[1]


class RaySnapshot:
    def __init__(self):
        self.target_time = float(rospy.get_param("~target_sim_time", 32.0))
        self.world_x_range = tuple(float(value) for value in rospy.get_param(
            "~world_x_range", []))
        if self.world_x_range and (len(self.world_x_range) != 2 or
                                   not all(math.isfinite(value)
                                           for value in self.world_x_range) or
                                   self.world_x_range[0] > self.world_x_range[1]):
            raise ValueError("world_x_range must be [minimum, maximum]")
        self.max_pose_age = float(rospy.get_param("~max_pose_age", 0.15))
        self.offset = tuple(float(value) for value in rospy.get_param(
            "~sensor_offset", [0.1315, 0.0, 0.223]))
        pitch = float(rospy.get_param("~sensor_pitch", 0.436332313))
        self.sensor_q = (0.0, math.sin(pitch/2.0), 0.0, math.cos(pitch/2.0))
        self.model_name = rospy.get_param("~model_name", "iris")
        self.run_id = rospy.get_param("/run_id", "")
        default = Path("/home/nuc/gbplanner2_isolated_ws/runtime/map_reference") / (
            "ray_snapshot_{}_{}.csv".format(
                datetime.now().strftime("%Y%m%d_%H%M%S_%f"), os.getpid()))
        self.output = Path(rospy.get_param("~output_file", str(default)))
        self.truth_poses = deque(maxlen=200)
        self.done = False
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._truth_cb, queue_size=10)
        rospy.Subscriber("/mine_uav/sitl/mid360/points", PointCloud,
                         self._cloud_cb, queue_size=2, buff_size=8*1024*1024)

    def _truth_cb(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            return
        pose = message.pose[index]
        p, q = pose.position, pose.orientation
        self.truth_poses.append((rospy.Time.now().to_sec(),
                                 (p.x, p.y, p.z),
                                 (q.x, q.y, q.z, q.w)))

    def _cloud_cb(self, message):
        if self.done or len(self.truth_poses) < 2:
            return
        now = rospy.Time.now().to_sec()
        scan_stamp = message.header.stamp.to_sec()
        if now < self.target_time or scan_stamp <= 0.0:
            return
        pose = interpolate_pose(list(self.truth_poses), scan_stamp,
                                self.max_pose_age)
        if pose is None:
            return
        body, body_q, bracket_width, pose_gap = pose
        if not allowed_world_x(body[0], self.world_x_range):
            return
        sensor_offset = rotate(body_q, self.offset)
        origin = tuple(body[i]+sensor_offset[i] for i in range(3))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with self.output.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(("sim_time", "truth_age_s", "ray_index",
                             "sensor_x", "sensor_y", "sensor_z",
                             "direction_x", "direction_y", "direction_z",
                             "measured_range_m", "ros_run_id",
                             "capture_sim_time", "truth_pose_bracket_s"))
            count = 0
            for index, point in enumerate(message.points):
                local = (point.x, point.y, point.z)
                if not all(math.isfinite(value) for value in local):
                    continue
                distance = math.sqrt(sum(value*value for value in local))
                if distance < 0.2:
                    continue
                body_ray = rotate(self.sensor_q, local)
                world_ray = rotate(body_q, body_ray)
                direction = tuple(value/distance for value in world_ray)
                writer.writerow(("{:.3f}".format(scan_stamp),
                                 "{:.3f}".format(pose_gap), index,
                                 *("{:.6f}".format(value) for value in origin),
                                 *("{:.8f}".format(value) for value in direction),
                                 "{:.5f}".format(distance), self.run_id,
                                 "{:.3f}".format(now),
                                 "{:.4f}".format(bracket_width)))
                count += 1
        self.done = True
        rospy.logwarn("Read-only ray snapshot: %d rays at scan %.3f s, capture %.3f s, pose bracket %.4f s -> %s",
                      count, scan_stamp, now, bracket_width, self.output)


if __name__ == "__main__":
    rospy.init_node("diagnostic_ray_snapshot")
    RaySnapshot()
    rospy.spin()
