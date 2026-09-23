#!/usr/bin/env python3
"""Expose GBPlanner decision timing and geometry in the live ROS log."""

import math
import threading

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from trajectory_msgs.msg import MultiDOFJointTrajectory


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


class DecisionMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._odom = None
        self._failures = 0
        rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                         self._odom_callback, queue_size=10)
        rospy.Subscriber("/gbplanner_status", Bool, self._status_callback,
                         queue_size=20)
        rospy.Subscriber("/gbplanner/command/trajectory",
                         MultiDOFJointTrajectory, self._trajectory_callback,
                         queue_size=5)

    def _odom_callback(self, message):
        with self._lock:
            self._odom = message

    def _status_callback(self, message):
        with self._lock:
            if message.data:
                self._failures = 0
                return
            self._failures += 1
            failures = self._failures
            odom = self._odom
        if failures == 1 or failures % 5 == 0:
            if odom is None:
                rospy.logwarn("GBPlanner empty decision x%d; odometry unavailable",
                              failures)
                return
            p = odom.pose.pose.position
            rospy.logwarn("GBPlanner empty decision x%d at [%.2f %.2f %.2f]",
                          failures, p.x, p.y, p.z)

    def _trajectory_callback(self, message):
        if len(message.points) < 2:
            return
        transforms = [point.transforms[0] for point in message.points
                      if point.transforms]
        if len(transforms) < 2:
            return
        length = 0.0
        for left, right in zip(transforms, transforms[1:]):
            length += math.sqrt(
                (right.translation.x-left.translation.x) ** 2 +
                (right.translation.y-left.translation.y) ** 2 +
                (right.translation.z-left.translation.z) ** 2)
        first, last = transforms[0], transforms[-1]
        duration = message.points[-1].time_from_start.to_sec()
        with self._lock:
            self._failures = 0
            odom = self._odom
        current_yaw = float("nan") if odom is None else \
            yaw_of(odom.pose.pose.orientation)
        rospy.loginfo(
            "GBPlanner decision seq=%d samples=%d length=%.2fm duration=%.2fs "
            "start=[%.2f %.2f %.2f] end=[%.2f %.2f %.2f] "
            "yaw_current=%.2f yaw_end=%.2f",
            message.header.seq, len(message.points), length, duration,
            first.translation.x, first.translation.y, first.translation.z,
            last.translation.x, last.translation.y, last.translation.z,
            current_yaw, yaw_of(last.rotation))


if __name__ == "__main__":
    rospy.init_node("gbplanner_decision_monitor")
    DecisionMonitor()
    rospy.spin()
