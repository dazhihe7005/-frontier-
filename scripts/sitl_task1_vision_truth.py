#!/usr/bin/env python3

"""SITL-only Gazebo truth odometry for task-one control regressions.

This source is deliberately separate from Omni-LIVO planner odometry.  It is
only an external-vision A/B input that lets a control regression test SUPER,
the command bridge and PX4 without conflating an empty-world LIO degeneracy.
"""

import math
import os
from urllib.parse import urlparse

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def quaternion_conjugate(quaternion):
    return Quaternion(
        x=-quaternion.x, y=-quaternion.y, z=-quaternion.z,
        w=quaternion.w)


def quaternion_multiply(left, right):
    return Quaternion(
        x=left.w * right.x + left.x * right.w
          + left.y * right.z - left.z * right.y,
        y=left.w * right.y - left.x * right.z
          + left.y * right.w + left.z * right.x,
        z=left.w * right.z + left.x * right.y
          - left.y * right.x + left.z * right.w,
        w=left.w * right.w - left.x * right.x
          - left.y * right.y - left.z * right.z,
    )


def rotate_vector(rotation, vector):
    pure = Quaternion(x=vector.x, y=vector.y, z=vector.z, w=0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(rotation, pure), quaternion_conjugate(rotation))
    return Vector3(x=rotated.x, y=rotated.y, z=rotated.z)


class SitlTask1VisionTruth:
    FCU_URL = "udp://:14540@localhost:14557"

    def __init__(self):
        self.vehicle_model = rospy.get_param("~vehicle_model", "iris_xy_vision")
        self.allowed_master_port = int(rospy.get_param("~allowed_master_port", 11319))
        self.expected_fcu_url = rospy.get_param("~expected_fcu_url", self.FCU_URL)
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        if not math.isfinite(self.publish_rate) or self.publish_rate < 5.0:
            raise ValueError("publish_rate must be finite and at least 5 Hz")

        self.last_publish = rospy.Time(0)
        self.last_safety_check = rospy.Time(0)
        self.safe_link = False
        self.origin_position = None
        self.origin_from_world = None
        self.odometry_pub = rospy.Publisher(
            "/mine_uav/sitl/vision_truth_odometry", Odometry,
            queue_size=10)
        self.health_pub = rospy.Publisher(
            "/mine_uav/sitl/vision_truth_healthy", Bool,
            queue_size=1, latch=True)
        self.health_pub.publish(Bool(data=False))
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_callback,
            queue_size=1)

    def sitl_only(self):
        try:
            port = urlparse(os.environ.get("ROS_MASTER_URI", "")).port
        except ValueError:
            return False
        return (
            port == self.allowed_master_port
            and rospy.get_param("/use_sim_time", False)
            and rospy.get_param("/mavros/fcu_url", "") == self.expected_fcu_url
        )

    def model_callback(self, message):
        now = rospy.Time.now()
        if (
            self.last_safety_check.is_zero()
            or (now - self.last_safety_check).to_sec() >= 0.5
        ):
            self.safe_link = self.sitl_only()
            self.last_safety_check = now
            self.health_pub.publish(Bool(data=self.safe_link))
        if not self.safe_link or now.is_zero():
            return
        try:
            index = message.name.index(self.vehicle_model)
        except ValueError:
            rospy.logwarn_throttle(
                2.0, "SITL truth cannot find Gazebo model %s", self.vehicle_model)
            self.health_pub.publish(Bool(data=False))
            return
        if (
            not self.last_publish.is_zero()
            and 0.0 <= (now - self.last_publish).to_sec() < 1.0 / self.publish_rate
        ):
            return

        pose = message.pose[index]
        twist = message.twist[index]
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.health_pub.publish(Bool(data=False))
            return

        quaternion_norm = math.sqrt(
            pose.orientation.x ** 2 + pose.orientation.y ** 2
            + pose.orientation.z ** 2 + pose.orientation.w ** 2)
        if quaternion_norm < 1.0e-6:
            self.health_pub.publish(Bool(data=False))
            return
        world_orientation = Quaternion(
            x=pose.orientation.x / quaternion_norm,
            y=pose.orientation.y / quaternion_norm,
            z=pose.orientation.z / quaternion_norm,
            w=pose.orientation.w / quaternion_norm,
        )
        if self.origin_position is None:
            self.origin_position = Vector3(
                x=pose.position.x, y=pose.position.y, z=pose.position.z)
            self.origin_from_world = quaternion_conjugate(world_orientation)
            rospy.logwarn(
                "SITL task-one vision truth active with local origin "
                "[%.3f %.3f %.3f]; simulation only",
                pose.position.x, pose.position.y, pose.position.z)

        displacement = Vector3(
            x=pose.position.x - self.origin_position.x,
            y=pose.position.y - self.origin_position.y,
            z=pose.position.z - self.origin_position.z,
        )
        local_position = rotate_vector(self.origin_from_world, displacement)
        local_orientation = quaternion_multiply(
            self.origin_from_world, world_orientation)
        local_linear_velocity = rotate_vector(
            self.origin_from_world, twist.linear)
        local_angular_velocity = rotate_vector(
            self.origin_from_world, twist.angular)

        self.last_publish = now
        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = "camera_init"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = local_position.x
        odometry.pose.pose.position.y = local_position.y
        odometry.pose.pose.position.z = local_position.z
        odometry.pose.pose.orientation = local_orientation
        odometry.twist.twist.linear = local_linear_velocity
        odometry.twist.twist.angular = local_angular_velocity
        for covariance_index in (0, 7, 14):
            odometry.pose.covariance[covariance_index] = 0.05 ** 2
        for covariance_index in (21, 28, 35):
            odometry.pose.covariance[covariance_index] = 0.10 ** 2
        self.odometry_pub.publish(odometry)
        self.health_pub.publish(Bool(data=True))


if __name__ == "__main__":
    rospy.init_node("sitl_task1_vision_truth")
    SitlTask1VisionTruth()
    rospy.spin()
