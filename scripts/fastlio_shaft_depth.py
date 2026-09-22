#!/usr/bin/env python3
"""Entrance-relative shaft depth derived only from FAST-LIO2 odometry."""

import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from mine_uav_control.msg import ShaftDepthEstimate


class FastlioShaftDepth:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.source_id = rospy.get_param("~source_id", "fastlio2_vertical_displacement")
        self.expected_frame = rospy.get_param("~expected_frame", "camera_init")
        self.base_sigma_m = float(rospy.get_param("~base_sigma_m", 0.08))
        self.sigma_per_meter = float(rospy.get_param("~sigma_per_meter", 0.0002))
        self.enabled = False
        self.origin_z = None
        self.publisher = rospy.Publisher(
            "/mine_uav/shaft/depth_estimate", ShaftDepthEstimate, queue_size=10
        )
        rospy.Subscriber(
            "/mine_uav/mission/shaft_enable", Bool, self.on_enable, queue_size=1
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odometry, queue_size=20)

    def on_enable(self, message):
        if message.data and not self.enabled:
            self.origin_z = None
        self.enabled = message.data

    def on_odometry(self, message):
        if not self.enabled:
            return
        if self.expected_frame and message.header.frame_id != self.expected_frame:
            rospy.logerr_throttle(
                2.0,
                "FAST-LIO shaft depth rejected frame %s (expected %s)",
                message.header.frame_id,
                self.expected_frame,
            )
            return
        z = message.pose.pose.position.z
        if not math.isfinite(z):
            return
        if self.origin_z is None:
            self.origin_z = z
        depth = self.origin_z - z
        covariance = message.pose.covariance[14]
        covariance_sigma = math.sqrt(covariance) if covariance > 0.0 else 0.0
        sigma = max(
            self.base_sigma_m + self.sigma_per_meter * abs(depth), covariance_sigma
        )
        estimate = ShaftDepthEstimate()
        estimate.header.stamp = (
            rospy.Time.now() if message.header.stamp.is_zero() else message.header.stamp
        )
        estimate.header.frame_id = self.expected_frame
        estimate.relative_depth_m = depth
        estimate.sigma_m = sigma
        estimate.valid = math.isfinite(depth) and math.isfinite(sigma)
        estimate.source_id = self.source_id
        self.publisher.publish(estimate)


if __name__ == "__main__":
    rospy.init_node("fastlio_shaft_depth")
    FastlioShaftDepth()
    rospy.spin()
