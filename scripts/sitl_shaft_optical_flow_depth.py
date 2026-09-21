#!/usr/bin/env python3
"""SITL-only side optical-flow depth surrogate.

Uses only frame-to-frame vertical displacement from Gazebo truth to emulate
integrated side-looking optical flow. Absolute world Z is not published as the
task depth source. This is an interface/failure-model surrogate, not a camera
or optical-flow physics simulator.
"""

import math

import rospy
from gazebo_msgs.msg import ModelStates


def integrate_depth_delta(previous_z, current_z, dt, scale, drift_rate_mps):
    values = (previous_z, current_z, dt, scale, drift_rate_mps)
    if not all(math.isfinite(value) for value in values) or dt < 0.0:
        return None
    # Gazebo can deliver several ModelStates callbacks under the same /clock
    # stamp when simulation is accelerated. Spatial displacement is still real
    # and must be integrated even when dt == 0; only the drift term is zero.
    return (previous_z - current_z) * scale + drift_rate_mps * dt
from mine_uav_control.msg import ShaftDepthEstimate
from std_msgs.msg import Bool, Float64


class SitlSideFlowDepth:
    def __init__(self):
        self.vehicle_model = rospy.get_param("~vehicle_model", "iris")
        self.source_id = rospy.get_param(
            "~source_id", "sitl_side_optical_flow_integrated")
        self.scale = float(rospy.get_param("~scale", 1.0))
        self.drift_rate_mps = float(rospy.get_param("~drift_rate_mps", 0.0))
        self.base_sigma_m = float(rospy.get_param("~base_sigma_m", 0.05))
        self.sigma_per_meter = float(rospy.get_param("~sigma_per_meter", 0.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.enabled = False
        self.last_z = None
        self.last_stamp = None
        self.depth = 0.0
        self.last_publish = rospy.Time(0)
        self.pub = rospy.Publisher(
            "/mine_uav/shaft/depth_estimate", ShaftDepthEstimate, queue_size=10)
        self.error_pub = rospy.Publisher(
            "/mine_uav/sitl/side_flow_integrated_depth_m", Float64, queue_size=10)
        rospy.Subscriber(
            "/mine_uav/mission/shaft_enable", Bool, self.on_enable, queue_size=1)
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.on_models, queue_size=1)
        rospy.logwarn(
            "SITL side-flow depth surrogate active; this is not a real optical-flow sensor")

    def on_enable(self, message):
        if message.data and not self.enabled:
            self.depth = 0.0
            self.last_z = None
            self.last_stamp = None
            self.last_publish = rospy.Time(0)
        self.enabled = message.data

    def on_models(self, message):
        if not self.enabled or not rospy.get_param("/use_sim_time", False):
            return
        try:
            z = message.pose[message.name.index(self.vehicle_model)].position.z
        except ValueError:
            return
        if not math.isfinite(z):
            return
        now = rospy.Time.now()
        if self.last_z is None:
            self.last_z = z
            self.last_stamp = now
            return
        dt = (now - self.last_stamp).to_sec()
        delta = integrate_depth_delta(
            self.last_z, z, dt, self.scale, self.drift_rate_mps)
        if delta is None:
            # Simulation clock moved backwards. Rebase all time-dependent
            # throttles so a previous future timestamp cannot suppress the
            # depth stream indefinitely.
            self.last_z = z
            self.last_stamp = now
            self.last_publish = rospy.Time(0)
            return
        self.depth += delta
        self.last_z = z
        self.last_stamp = now
        if (not self.last_publish.is_zero() and
                (now - self.last_publish).to_sec() < 1.0 / self.publish_rate):
            return
        self.last_publish = now
        sigma = self.base_sigma_m + self.sigma_per_meter * abs(self.depth)
        estimate = ShaftDepthEstimate()
        estimate.header.stamp = now
        estimate.header.frame_id = "camera_init"
        estimate.relative_depth_m = self.depth
        estimate.sigma_m = sigma
        estimate.valid = math.isfinite(self.depth) and math.isfinite(sigma)
        estimate.source_id = self.source_id
        self.pub.publish(estimate)
        self.error_pub.publish(Float64(data=self.depth))


if __name__ == "__main__":
    rospy.init_node("sitl_shaft_optical_flow_depth")
    SitlSideFlowDepth()
    rospy.spin()
