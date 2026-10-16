#!/usr/bin/env python3
"""ROS regression for levelling Task 1 planner inputs before SUPER."""

import math
import threading
import time
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as point_cloud2
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class PlanningFrameTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._odom = None
        self._cloud = None
        self._alignment = None
        self._mission_pub = rospy.Publisher(
            "/mine_uav/mission/goaf_enable", Bool, queue_size=1, latch=True)
        self._alignment_pub = rospy.Publisher(
            "/mine_uav/task1/fastlio_to_px4_alignment",
            TransformStamped, queue_size=1, latch=True)
        self._sensor_rotation_pub = rospy.Publisher(
            "/mine_uav/task1/fastlio_sensor_from_body_rotation",
            QuaternionStamped, queue_size=1, latch=True)
        self._odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=2)
        self._cloud_pub = rospy.Publisher(
            "/cloud_registered", PointCloud2, queue_size=2)
        self._odom_sub = rospy.Subscriber(
            "/mine_uav/task1/odometry", Odometry, self._odom_callback)
        self._cloud_sub = rospy.Subscriber(
            "/mine_uav/task1/cloud_registered",
            PointCloud2, self._cloud_callback)
        self._planning_alignment_sub = rospy.Subscriber(
            "/mine_uav/task1/planning_to_px4_alignment",
            TransformStamped, self._alignment_callback)

    def tearDown(self):
        self._odom_sub.unregister()
        self._cloud_sub.unregister()
        self._planning_alignment_sub.unregister()

    def _odom_callback(self, message):
        with self._lock:
            self._odom = message

    def _cloud_callback(self, message):
        with self._lock:
            self._cloud = message

    def _alignment_callback(self, message):
        with self._lock:
            self._alignment = message

    def test_pitch_is_removed_before_planning(self):
        pitch = math.radians(25.0)
        half = 0.5 * pitch

        source_alignment = TransformStamped()
        source_alignment.header.frame_id = "odom"
        source_alignment.child_frame_id = "camera_init"
        source_alignment.transform.translation.x = 1.0
        source_alignment.transform.translation.y = 2.0
        source_alignment.transform.translation.z = 3.0
        source_alignment.transform.rotation.y = math.sin(half)
        source_alignment.transform.rotation.w = math.cos(half)

        sensor_rotation = QuaternionStamped()
        sensor_rotation.header.frame_id = "camera_init"
        sensor_rotation.quaternion.y = -math.sin(half)
        sensor_rotation.quaternion.w = math.cos(half)

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            source_alignment.header.stamp = stamp
            sensor_rotation.header.stamp = stamp
            self._alignment_pub.publish(source_alignment)
            self._sensor_rotation_pub.publish(sensor_rotation)
            self._mission_pub.publish(Bool(data=True))

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "camera_init"
            odom.pose.pose.position.x = 5.0 * math.cos(pitch)
            odom.pose.pose.position.z = 5.0 * math.sin(pitch)
            odom.pose.pose.orientation.w = 1.0
            self._odom_pub.publish(odom)
            cloud = point_cloud2.create_cloud_xyz32(
                odom.header,
                [(odom.pose.pose.position.x, 0.0,
                  odom.pose.pose.position.z)])
            self._cloud_pub.publish(cloud)

            with self._lock:
                complete = (self._odom is not None and
                            self._cloud is not None and
                            self._alignment is not None)
            if complete:
                break
            time.sleep(0.05)

        with self._lock:
            output_odom = self._odom
            output_cloud = self._cloud
            output_alignment = self._alignment
        self.assertIsNotNone(output_odom)
        self.assertIsNotNone(output_cloud)
        self.assertIsNotNone(output_alignment)
        self.assertEqual(output_odom.header.frame_id, "task1_planning")
        self.assertAlmostEqual(output_odom.pose.pose.position.x, 5.0, places=5)
        self.assertAlmostEqual(output_odom.pose.pose.position.y, 0.0, places=5)
        self.assertAlmostEqual(output_odom.pose.pose.position.z, 0.0, places=5)
        self.assertAlmostEqual(output_odom.pose.pose.orientation.x, 0.0, places=5)
        self.assertAlmostEqual(output_odom.pose.pose.orientation.y, 0.0, places=5)
        self.assertAlmostEqual(output_odom.pose.pose.orientation.z, 0.0, places=5)
        self.assertAlmostEqual(abs(output_odom.pose.pose.orientation.w), 1.0,
                               places=5)

        point = next(point_cloud2.read_points(
            output_cloud, field_names=("x", "y", "z"), skip_nans=False))
        self.assertEqual(output_cloud.header.frame_id, "task1_planning")
        self.assertAlmostEqual(point[0], 5.0, places=5)
        self.assertAlmostEqual(point[1], 0.0, places=5)
        self.assertAlmostEqual(point[2], 0.0, places=5)

        self.assertEqual(output_alignment.child_frame_id, "task1_planning")
        self.assertAlmostEqual(output_alignment.transform.translation.x, 1.0)
        self.assertAlmostEqual(output_alignment.transform.translation.y, 2.0)
        self.assertAlmostEqual(output_alignment.transform.translation.z, 3.0)
        self.assertAlmostEqual(output_alignment.transform.rotation.x, 0.0)
        self.assertAlmostEqual(output_alignment.transform.rotation.y, 0.0)
        self.assertAlmostEqual(output_alignment.transform.rotation.z, 0.0)
        self.assertAlmostEqual(output_alignment.transform.rotation.w, 1.0)


if __name__ == "__main__":
    rospy.init_node("test_task1_planning_frame")
    rostest.rosrun("mine_uav_control", "task1_planning_frame",
                   PlanningFrameTest)
