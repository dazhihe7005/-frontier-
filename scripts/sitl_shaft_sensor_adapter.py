#!/usr/bin/env python3

"""SITL-only ideal independent depth/range from Gazebo world geometry."""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Range
from std_msgs.msg import Float64


class SitlShaftSensorAdapter:
    def __init__(self):
        self.bottom_top_world_z = float(
            rospy.get_param("~bottom_top_world_z", -20.85)
        )
        self.entrance_world_z = float(
            rospy.get_param("~entrance_world_z", 0.25)
        )
        self.max_range = float(rospy.get_param("~max_range", 30.0))
        # SITL-only fault injection: stop the independent bottom range stream
        # after reaching this entrance-relative depth. Negative disables it.
        self.stop_range_after_depth = float(
            rospy.get_param("~stop_range_after_depth", -1.0)
        )
        self.last_publish = rospy.Time(0)
        self.depth_pub = rospy.Publisher(
            "/mine_uav/shaft/relative_depth_m", Float64, queue_size=10
        )
        self.range_pub = rospy.Publisher(
            "/mine_uav/shaft/bottom_range", Range, queue_size=10
        )
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.on_models, queue_size=10
        )
        rospy.logwarn("SITL shaft adapter uses Gazebo WORLD TRUTH, not a physical sensor")

    def on_models(self, message):
        if not rospy.get_param("/use_sim_time", False):
            rospy.logerr_throttle(2.0, "SITL shaft sensor adapter requires /use_sim_time")
            return
        # Do not misinterpret the temporary entrance platform as the bottom.
        # This also prevents motion before the platform is removed.
        if "shaft_launch_pad" in message.name:
            return
        try:
            z = message.pose[message.name.index("iris")].position.z
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
        self.depth_pub.publish(Float64(data=depth))
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
