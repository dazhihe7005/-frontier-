#!/usr/bin/env python3

"""SITL-only Gazebo truth pose as external vision; never a real sensor."""

import math
import os
from urllib.parse import urlparse

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class SitlShaftVisionTruth:
    FCU_URL = "udp://:14540@localhost:14557"

    def __init__(self):
        self.vehicle_model = rospy.get_param("~vehicle_model", "iris_vision")
        self.allowed_master_port = int(
            rospy.get_param("~allowed_master_port", 11319))
        self.expected_fcu_url = rospy.get_param(
            "~expected_fcu_url", self.FCU_URL)
        self.entrance_world_z = float(rospy.get_param("~entrance_world_z", 0.25))
        self.stop_after_depth = float(rospy.get_param("~stop_after_depth", -1.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        if not math.isfinite(self.publish_rate) or self.publish_rate < 5.0:
            raise ValueError("publish_rate must be finite and at least 5 Hz")
        self.last_publish = rospy.Time(0)
        self.last_safety_check = rospy.Time(0)
        self.safe_link = False
        self.stopped = False
        self.pub = rospy.Publisher(
            "/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped,
            queue_size=10)
        self.active_pub = rospy.Publisher(
            "/mine_uav/sitl/vision_truth_active", Bool, queue_size=1,
            latch=True)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.on_models, queue_size=1)
        self.active_pub.publish(Bool(data=False))

    def sitl_only(self):
        try:
            port = urlparse(os.environ.get("ROS_MASTER_URI", "")).port
        except ValueError:
            return False
        return (port == self.allowed_master_port and
                rospy.get_param("/use_sim_time", False) and
                rospy.get_param("/mavros/fcu_url", "") ==
                self.expected_fcu_url)

    def on_models(self, message):
        now = rospy.Time.now()
        if (self.last_safety_check.is_zero() or
                (now - self.last_safety_check).to_sec() >= 0.5):
            self.safe_link = self.sitl_only()
            self.last_safety_check = now
        if not self.safe_link or self.stopped:
            return
        try:
            pose = message.pose[message.name.index(self.vehicle_model)]
        except ValueError:
            return
        p = pose.position
        q = pose.orientation
        if not all(math.isfinite(value) for value in
                   (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        if now.is_zero():
            return
        if self.stop_after_depth >= 0.0 and \
                self.entrance_world_z - p.z >= self.stop_after_depth:
            self.stopped = True
            self.active_pub.publish(Bool(data=False))
            rospy.logwarn("SITL vision-truth fault injection: stream stopped")
            return
        if (not self.last_publish.is_zero() and
                0.0 <= (now - self.last_publish).to_sec() <
                1.0 / self.publish_rate):
            return
        self.last_publish = now
        estimate = PoseWithCovarianceStamped()
        estimate.header.stamp = now
        estimate.header.frame_id = "odom"
        estimate.pose.pose = pose
        # Deliberately declared simulation uncertainty, not a real calibration.
        for index in (0, 7, 14):
            estimate.pose.covariance[index] = 0.05 ** 2
        for index in (21, 28, 35):
            estimate.pose.covariance[index] = 0.10 ** 2
        self.pub.publish(estimate)
        self.active_pub.publish(Bool(data=True))


if __name__ == "__main__":
    rospy.init_node("sitl_shaft_vision_truth")
    SitlShaftVisionTruth()
    rospy.spin()
