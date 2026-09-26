#!/usr/bin/env python3
"""Save one read-only local TSDF snapshot; never publish control data."""

import csv
from datetime import datetime
import math
import os
from pathlib import Path

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class TsdfSnapshot:
    def __init__(self):
        self.target_time = float(rospy.get_param("~target_sim_time", 32.0))
        self.bounds = tuple(float(value) for value in rospy.get_param(
            "~bounds", [-1.0, 10.0, -6.0, 2.0, -1.0, 5.0]))
        self.run_id = rospy.get_param("/run_id", "")
        default = Path("/home/nuc/gbplanner2_isolated_ws/runtime/map_reference") / (
            "tsdf_snapshot_{}_{}.csv".format(
                datetime.now().strftime("%Y%m%d_%H%M%S_%f"), os.getpid()))
        self.output = Path(rospy.get_param("~output_file", str(default)))
        self.done = False
        rospy.Subscriber("/gbplanner_node/tsdf_pointcloud", PointCloud2,
                         self._cloud_cb, queue_size=1, buff_size=32*1024*1024)

    def _cloud_cb(self, message):
        now = rospy.Time.now().to_sec()
        if self.done or now < self.target_time:
            return
        if not {"x", "y", "z", "intensity"}.issubset(
                {field.name for field in message.fields}):
            rospy.logerr("TSDF snapshot lacks x/y/z/intensity fields")
            self.done = True
            return
        low_x, high_x, low_y, high_y, low_z, high_z = self.bounds
        self.output.parent.mkdir(parents=True, exist_ok=True)
        count = occupied = 0
        with self.output.open("w", encoding="utf-8", newline="") as target:
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
        self.done = True
        rospy.logwarn("Read-only local TSDF snapshot: %d voxels, %d <=0.2 m at %.3f s -> %s",
                      count, occupied, now, self.output)


if __name__ == "__main__":
    rospy.init_node("diagnostic_tsdf_snapshot")
    TsdfSnapshot()
    rospy.spin()
