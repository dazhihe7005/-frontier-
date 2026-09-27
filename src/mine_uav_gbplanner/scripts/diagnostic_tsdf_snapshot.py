#!/usr/bin/env python3
"""Save one read-only local TSDF snapshot; never publish control data."""

import csv
from datetime import datetime
import math
import os
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class TsdfSnapshot:
    def __init__(self):
        single_target = float(rospy.get_param("~target_sim_time", 32.0))
        configured_targets = rospy.get_param("~target_sim_times", [])
        self.target_times = sorted(set(float(value) for value in
                                       (configured_targets or [single_target])))
        if not self.target_times or any(
                not math.isfinite(value) or value < 0.0
                for value in self.target_times):
            raise ValueError("target_sim_times must be finite nonnegative times")
        self.next_target = 0
        self.bounds = tuple(float(value) for value in rospy.get_param(
            "~bounds", [-3.0, 10.0, -6.0, 2.0, -1.0, 5.0]))
        self.planner_x_range = tuple(float(value) for value in rospy.get_param(
            "~planner_x_range", []))
        if self.planner_x_range and (len(self.planner_x_range) != 2 or
                                     self.planner_x_range[0] > self.planner_x_range[1]):
            raise ValueError("planner_x_range must be [minimum, maximum]")
        self.planner_x = None
        self.planner_rx = rospy.Time(0)
        self.run_id = rospy.get_param("/run_id", "")
        default = Path("/home/nuc/gbplanner2_isolated_ws/runtime/map_reference") / (
            "tsdf_snapshot_{}_{}.csv".format(
                datetime.now().strftime("%Y%m%d_%H%M%S_%f"), os.getpid()))
        self.output = Path(rospy.get_param("~output_file", str(default)))
        if self.planner_x_range:
            rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                             self._odom_cb, queue_size=10)
        rospy.Subscriber("/gbplanner_node/tsdf_pointcloud", PointCloud2,
                         self._cloud_cb, queue_size=1, buff_size=32*1024*1024)

    def _odom_cb(self, message):
        self.planner_x = message.pose.pose.position.x
        self.planner_rx = rospy.Time.now()

    def _cloud_cb(self, message):
        now = rospy.Time.now().to_sec()
        if (self.next_target >= len(self.target_times) or
                now < self.target_times[self.next_target]):
            return
        if self.planner_x_range and (
                self.planner_x is None or
                not self.planner_x_range[0] <= self.planner_x <= self.planner_x_range[1] or
                rospy.Time.now()-self.planner_rx > rospy.Duration(0.5)):
            return
        if not {"x", "y", "z", "intensity"}.issubset(
                {field.name for field in message.fields}):
            rospy.logerr("TSDF snapshot lacks x/y/z/intensity fields")
            self.next_target = len(self.target_times)
            return
        low_x, high_x, low_y, high_y, low_z, high_z = self.bounds
        target_time = self.target_times[self.next_target]
        output = (self.output if len(self.target_times) == 1 else
                  self.output.with_name("{}_target_{:.3f}{}".format(
                      self.output.stem, target_time, self.output.suffix)))
        output.parent.mkdir(parents=True, exist_ok=True)
        count = occupied = 0
        with output.open("w", encoding="utf-8", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(("sim_time", "frame_id", "x", "y", "z",
                             "tsdf_distance_m", "ros_run_id"))
            for x, y, z, distance in point_cloud2.read_points(
                    message, field_names=("x", "y", "z", "intensity"),
                    skip_nans=True):
                if not all(math.isfinite(value) for value in (x, y, z, distance)):
                    continue
                if not (low_x <= x <= high_x and low_y <= y <= high_y and
                        low_z <= z <= high_z):
                    continue
                writer.writerow(("{:.3f}".format(now), message.header.frame_id,
                                 "{:.5f}".format(x), "{:.5f}".format(y),
                                 "{:.5f}".format(z), "{:.5f}".format(distance),
                                 self.run_id))
                count += 1
                occupied += distance <= 0.2
        self.next_target += 1
        rospy.logwarn("Read-only local TSDF snapshot: target %.3f s, %d voxels, %d <=0.2 m at %.3f s, planner_x=%s -> %s",
                      target_time, count, occupied, now, self.planner_x, output)


if __name__ == "__main__":
    rospy.init_node("diagnostic_tsdf_snapshot")
    TsdfSnapshot()
    rospy.spin()
