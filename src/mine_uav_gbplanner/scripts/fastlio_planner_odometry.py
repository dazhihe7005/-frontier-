#!/usr/bin/env python3
"""Publish planner odometry with FAST-LIO2 XY/yaw and PX4 barometric Z.

The 1018 profile fuses external vision in XY only.  Matching the planning map
to PX4's vertical reference prevents independent lidar-Z drift from moving the
map through the vehicle; no Gazebo pose is used.
"""

import copy
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


class PlannerOdometry:
    def __init__(self):
        self.max_px4_age = float(rospy.get_param("~max_px4_age", 0.5))
        self._lock = threading.Lock()
        self._pose = None
        self._pose_rx = rospy.Time(0)
        self._armed = False
        self._offset = None
        self._pub = rospy.Publisher(
            "/mine_uav/gbplanner/planner_odometry", Odometry, queue_size=20)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=20)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/Odometry", Odometry, self._odom_cb, queue_size=20)

    def _pose_cb(self, message):
        with self._lock:
            self._pose = message
            self._pose_rx = rospy.Time.now()

    def _state_cb(self, message):
        with self._lock:
            self._armed = message.armed

    def _odom_cb(self, message):
        with self._lock:
            pose = self._pose
            pose_rx = self._pose_rx
            armed = self._armed
        if pose is None or (rospy.Time.now()-pose_rx).to_sec() > self.max_px4_age:
            rospy.logwarn_throttle(2.0, "Planner odometry waiting for PX4 barometric height")
            return
        lio_z = message.pose.pose.position.z
        px4_z = pose.pose.position.z
        if not math.isfinite(lio_z) or not math.isfinite(px4_z):
            return
        with self._lock:
            if self._offset is None or not armed:
                self._offset = lio_z-px4_z
            offset = self._offset
        output = copy.deepcopy(message)
        output.pose.pose.position.z = px4_z+offset
        output.child_frame_id = "base_link"
        self._pub.publish(output)


if __name__ == "__main__":
    rospy.init_node("fastlio_planner_odometry")
    PlannerOdometry()
    rospy.spin()
