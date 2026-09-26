#!/usr/bin/env python3
"""Capture one raw MID360 scan with diagnostic-only Gazebo truth pose.

Never publish truth, a map, or a control command. The CSV is used offline to
compare Gazebo ray returns with the exact collision meshes at the same pose.
"""

import csv
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


class RaySnapshot:
    def __init__(self):
        self.target_time = float(rospy.get_param("~target_sim_time", 32.0))
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
        self.truth = None
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
        self.truth = (rospy.Time.now().to_sec(),
                      (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _cloud_cb(self, message):
        if self.done or self.truth is None:
            return
        now = rospy.Time.now().to_sec()
        if now < self.target_time or abs(now-self.truth[0]) > self.max_pose_age:
            return
        stamp, body, body_q = self.truth
        sensor_offset = rotate(body_q, self.offset)
        origin = tuple(body[i]+sensor_offset[i] for i in range(3))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with self.output.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(("sim_time", "truth_age_s", "ray_index",
                             "sensor_x", "sensor_y", "sensor_z",
                             "direction_x", "direction_y", "direction_z",
                             "measured_range_m", "ros_run_id"))
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
                writer.writerow(("{:.3f}".format(now),
                                 "{:.3f}".format(now-stamp), index,
                                 *("{:.6f}".format(value) for value in origin),
                                 *("{:.8f}".format(value) for value in direction),
                                 "{:.5f}".format(distance), self.run_id))
                count += 1
        self.done = True
        rospy.logwarn("Read-only ray snapshot: %d rays at %.3f s -> %s",
                      count, now, self.output)


if __name__ == "__main__":
    rospy.init_node("diagnostic_ray_snapshot")
    RaySnapshot()
    rospy.spin()
