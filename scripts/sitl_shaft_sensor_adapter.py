#!/usr/bin/env python3

"""SITL-only world-truth depth with selectable truth or Gazebo ray range."""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from mine_uav_control.msg import ShaftDepthEstimate
from sensor_msgs.msg import Range
from std_msgs.msg import Float64


class SitlShaftSensorAdapter:
    def __init__(self):
        self.vehicle_model = rospy.get_param("~vehicle_model", "iris")
        self.bottom_top_world_z = float(
            rospy.get_param("~bottom_top_world_z", -20.85)
        )
        self.entrance_world_z = float(
            rospy.get_param("~entrance_world_z", 0.25)
        )
        self.max_range = float(rospy.get_param("~max_range", 30.0))
        self.depth_sigma_m = float(rospy.get_param("~depth_sigma_m", 0.02))
        self.publish_depth_estimate = bool(
            rospy.get_param("~publish_depth_estimate", True)
        )
        self.range_source = rospy.get_param("~range_source", "truth")
        if self.range_source not in ("truth", "gazebo"):
            raise ValueError("range_source must be 'truth' or 'gazebo'")
        # SITL-only fault injection: stop the independent bottom range stream
        # after reaching this entrance-relative depth. Negative disables it.
        self.stop_range_after_depth = float(
            rospy.get_param("~stop_range_after_depth", -1.0)
        )
        self.stop_depth_after_depth = float(
            rospy.get_param("~stop_depth_after_depth", -1.0)
        )
        self.bad_sigma_after_depth = float(
            rospy.get_param("~bad_sigma_after_depth", -1.0)
        )
        # Deliberately lie about depth while claiming good quality. This
        # exercises PX4/depth disagreement detection, never real hardware.
        self.depth_drift_after_depth = float(
            rospy.get_param("~depth_drift_after_depth", -1.0)
        )
        self.depth_drift_rate_mps = float(
            rospy.get_param("~depth_drift_rate_mps", 0.0)
        )
        if (not math.isfinite(self.depth_drift_rate_mps) or
                self.depth_drift_rate_mps < 0.0):
            raise ValueError("depth_drift_rate_mps must be finite and nonnegative")
        self.depth_drift_start = None
        self.last_publish = rospy.Time(0)
        self.pad_present = True
        self.latest_depth = None
        self.depth_pub = rospy.Publisher(
            "/mine_uav/shaft/relative_depth_m", Float64, queue_size=10
        )
        self.depth_estimate_pub = rospy.Publisher(
            "/mine_uav/shaft/depth_estimate", ShaftDepthEstimate, queue_size=10
        )
        self.depth_bias_pub = rospy.Publisher(
            "/mine_uav/sitl/shaft_depth_bias_m", Float64, queue_size=10
        )
        self.range_pub = rospy.Publisher(
            "/mine_uav/shaft/bottom_range", Range, queue_size=10
        )
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.on_models, queue_size=1
        )
        if self.range_source == "gazebo":
            rospy.Subscriber("/mine_uav/sitl/shaft_downward_range", Range,
                             self.on_gazebo_range, queue_size=10)
        rospy.logwarn("SITL shaft depth uses WORLD TRUTH; bottom range source=%s",
                      self.range_source)

    def on_gazebo_range(self, message):
        if not rospy.get_param("/use_sim_time", False):
            return
        if self.pad_present or self.latest_depth is None:
            return
        if (self.stop_range_after_depth >= 0.0 and
                self.latest_depth >= self.stop_range_after_depth):
            rospy.logwarn_once("SITL fault injection: bottom range stream stopped")
            return
        self.range_pub.publish(message)

    def injected_depth_bias(self, true_depth, now):
        if self.depth_drift_after_depth < 0.0 or self.depth_drift_rate_mps == 0.0:
            return 0.0
        if self.depth_drift_start is None:
            if true_depth < self.depth_drift_after_depth:
                return 0.0
            self.depth_drift_start = now
            rospy.logwarn("SITL fault injection: declared-good depth begins drifting")
        elapsed = max(0.0, (now - self.depth_drift_start).to_sec())
        return self.depth_drift_rate_mps * elapsed

    def on_models(self, message):
        if not rospy.get_param("/use_sim_time", False):
            rospy.logerr_throttle(2.0, "SITL shaft sensor adapter requires /use_sim_time")
            return
        # Do not misinterpret the temporary entrance platform as the bottom.
        # This also prevents motion before the platform is removed.
        self.pad_present = "shaft_launch_pad" in message.name
        if self.pad_present:
            return
        try:
            z = message.pose[message.name.index(self.vehicle_model)].position.z
        except ValueError:
            return
        if not math.isfinite(z):
            return
        now = rospy.Time.now()
        if not self.last_publish.is_zero() and (
            now - self.last_publish
        ).to_sec() < 1.0 / 30.0:
            return
        self.last_publish = now
        depth = self.entrance_world_z - z
        self.latest_depth = depth
        injected_bias = self.injected_depth_bias(depth, now)
        self.depth_bias_pub.publish(Float64(data=injected_bias))
        if self.publish_depth_estimate:
            if (self.stop_depth_after_depth < 0.0 or
                    depth < self.stop_depth_after_depth):
                self.depth_pub.publish(Float64(data=depth))
                estimate = ShaftDepthEstimate()
                estimate.header.stamp = now
                estimate.header.frame_id = "gazebo_world"
                estimate.relative_depth_m = depth + injected_bias
                estimate.sigma_m = (
                    1.0 if self.bad_sigma_after_depth >= 0.0 and
                    depth >= self.bad_sigma_after_depth else self.depth_sigma_m
                )
                estimate.valid = True
                estimate.source_id = "gazebo_world_truth"
                self.depth_estimate_pub.publish(estimate)
            else:
                rospy.logwarn_once("SITL fault injection: depth stream stopped")
        if self.range_source == "gazebo":
            return
        if (self.stop_range_after_depth >= 0.0 and
                depth >= self.stop_range_after_depth):
            rospy.logwarn_once(
                "SITL fault injection: bottom range stopped after %.2f m depth",
                self.stop_range_after_depth,
            )
            return
        remaining = z - self.bottom_top_world_z
        reading = Range()
        reading.header.stamp = now
        reading.header.frame_id = "sitl_downward_range"
        reading.radiation_type = Range.INFRARED
        reading.field_of_view = 0.1
        reading.min_range = 0.2
        reading.max_range = self.max_range
        reading.range = remaining if remaining <= self.max_range else float("inf")
        self.range_pub.publish(reading)


if __name__ == "__main__":
    rospy.init_node("sitl_shaft_sensor_adapter")
    SitlShaftSensorAdapter()
    rospy.spin()
