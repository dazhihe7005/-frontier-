#!/usr/bin/env python3
"""Publish entrance-relative Task-2 depth from qualified Omni-LIVO odometry."""

import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

from mine_uav_control.msg import ShaftDepthEstimate


class RelativeDepthCore:
    def __init__(self, expected_frame="task1_planning", max_sigma=0.25):
        self.expected_frame = str(expected_frame)
        self.max_sigma = float(max_sigma)
        self.enabled = False
        self.reference_z = None

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        if not enabled:
            self.reference_z = None
        self.enabled = enabled

    def evaluate(self, z, variance, frame_id, healthy):
        if not self.enabled:
            return False, 0.0, float("inf"), "disabled"
        if frame_id != self.expected_frame:
            return False, 0.0, float("inf"), "frame_mismatch"
        if not healthy:
            return False, 0.0, float("inf"), "localization_unhealthy"
        if not math.isfinite(z) or not math.isfinite(variance) or variance < 0.0:
            return False, 0.0, float("inf"), "invalid_odometry"
        sigma = math.sqrt(variance)
        if sigma > self.max_sigma:
            return False, 0.0, sigma, "depth_uncertainty"
        if self.reference_z is None:
            self.reference_z = z
        return True, self.reference_z - z, sigma, "ok"


class OmniLivoShaftDepthNode:
    def __init__(self):
        self.source_id = rospy.get_param(
            "~source_id", "omnilivo_planning_odometry"
        )
        self.timeout = float(rospy.get_param("~timeout", 0.5))
        self.health_timeout = float(rospy.get_param("~health_timeout", 0.5))
        self.core = RelativeDepthCore(
            expected_frame=rospy.get_param("~expected_frame", "task1_planning"),
            max_sigma=rospy.get_param("~max_sigma_m", 0.25),
        )
        if self.timeout <= 0.0 or self.health_timeout <= 0.0:
            raise ValueError("Task-2 depth timeouts must be positive")
        self.odometry = None
        self.odometry_received = None
        self.health = False
        self.health_received = None

        self.publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "/mine_uav/shaft/depth_estimate"),
            ShaftDepthEstimate,
            queue_size=10,
        )
        # Standard-message mirror for the downstream telemetry relay. Invalid
        # estimates are represented as NaN, so a stale last depth is never
        # forwarded as if it were current.
        self.telemetry_publisher = rospy.Publisher(
            rospy.get_param(
                "~telemetry_topic", "/mine_uav/shaft/relative_depth_m"
            ),
            Float32,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~enable_topic", "/mine_uav/mission/shaft_enable"),
            Bool,
            self.enable_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~odometry_topic", "/mine_uav/task1/odometry"),
            Odometry,
            self.odometry_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~health_topic", "/mine_uav/task1/vision_healthy"),
            Bool,
            self.health_callback,
            queue_size=2,
        )
        rate = max(1.0, float(rospy.get_param("~rate", 20.0)))
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_callback)

    def enable_callback(self, message):
        self.core.set_enabled(message.data)

    def odometry_callback(self, message):
        self.odometry = message
        self.odometry_received = time.monotonic()

    def health_callback(self, message):
        self.health = bool(message.data)
        self.health_received = time.monotonic()

    def timer_callback(self, _event):
        now = time.monotonic()
        message = ShaftDepthEstimate()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.core.expected_frame
        message.source_id = self.source_id
        message.relative_depth_m = 0.0
        message.sigma_m = float("inf")
        message.valid = False

        odometry_fresh = (
            self.odometry is not None
            and self.odometry_received is not None
            and now - self.odometry_received <= self.timeout
        )
        health_fresh = (
            self.health_received is not None
            and now - self.health_received <= self.health_timeout
        )
        if odometry_fresh:
            odometry = self.odometry
            valid, depth, sigma, _ = self.core.evaluate(
                odometry.pose.pose.position.z,
                odometry.pose.covariance[14],
                odometry.header.frame_id,
                self.health and health_fresh,
            )
            message.header.stamp = odometry.header.stamp
            message.header.frame_id = odometry.header.frame_id
            message.relative_depth_m = depth
            message.sigma_m = sigma
            message.valid = valid
        self.publisher.publish(message)
        self.telemetry_publisher.publish(
            Float32(data=message.relative_depth_m if message.valid else float("nan"))
        )


if __name__ == "__main__":
    rospy.init_node("omnilivo_shaft_depth")
    OmniLivoShaftDepthNode()
    rospy.spin()
