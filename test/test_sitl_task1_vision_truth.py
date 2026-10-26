#!/usr/bin/env python3

"""Task-one truth isolation and local-frame conversion regression tests."""

import importlib.util
import math
import pathlib
import unittest
from unittest.mock import MagicMock, patch

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist


ROOT = pathlib.Path(__file__).resolve().parents[1]
PATH = ROOT / "scripts" / "sitl_task1_vision_truth.py"
SPEC = importlib.util.spec_from_file_location("sitl_task1_vision_truth", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SitlTask1VisionTruthTest(unittest.TestCase):
    def make_adapter(self):
        adapter = object.__new__(MODULE.SitlTask1VisionTruth)
        adapter.vehicle_model = "iris_xy_vision"
        adapter.allowed_master_port = 11319
        adapter.expected_fcu_url = MODULE.SitlTask1VisionTruth.FCU_URL
        adapter.publish_rate = 30.0
        adapter.last_publish = rospy.Time(0)
        adapter.last_safety_check = rospy.Time(0)
        adapter.safe_link = False
        adapter.origin_position = None
        adapter.origin_from_world = None
        adapter.odometry_pub = MagicMock()
        adapter.health_pub = MagicMock()
        return adapter

    def test_rejects_production_master_and_real_fcu(self):
        adapter = self.make_adapter()
        with patch.dict(MODULE.os.environ,
                        {"ROS_MASTER_URI": "http://localhost:11312"}), \
                patch.object(rospy, "get_param", return_value=True):
            self.assertFalse(adapter.sitl_only())
        with patch.dict(MODULE.os.environ,
                        {"ROS_MASTER_URI": "http://localhost:11319"}), \
                patch.object(
                    rospy, "get_param",
                    side_effect=lambda key, default=None:
                    True if key == "/use_sim_time" else
                    "serial:///dev/ttyUSB0"):
            self.assertFalse(adapter.sitl_only())

    @staticmethod
    def model(x, y, z, yaw=0.0):
        message = ModelStates()
        message.name = ["iris_xy_vision"]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.z = math.sin(0.5 * yaw)
        pose.orientation.w = math.cos(0.5 * yaw)
        message.pose = [pose]
        message.twist = [Twist()]
        return message

    def test_world_pose_is_rebased_into_initial_body_frame(self):
        adapter = self.make_adapter()
        with patch.object(adapter, "sitl_only", return_value=True), \
                patch.object(rospy.Time, "now",
                             side_effect=[rospy.Time(10), rospy.Time(11)]):
            adapter.model_callback(self.model(-8.0, 2.0, 16.0, math.pi / 2.0))
            adapter.model_callback(self.model(-8.0, 3.0, 17.5, math.pi / 2.0))

        first, second = [call.args[0]
                         for call in adapter.odometry_pub.publish.call_args_list]
        self.assertAlmostEqual(first.pose.pose.position.x, 0.0, places=6)
        self.assertAlmostEqual(first.pose.pose.position.z, 0.0, places=6)
        # A +world-Y displacement is +local-X for an initial +90 degree yaw.
        self.assertAlmostEqual(second.pose.pose.position.x, 1.0, places=6)
        self.assertAlmostEqual(second.pose.pose.position.y, 0.0, places=6)
        self.assertAlmostEqual(second.pose.pose.position.z, 1.5, places=6)
        self.assertAlmostEqual(second.pose.pose.orientation.w, 1.0, places=6)
        self.assertEqual(second.header.frame_id, "camera_init")
        self.assertAlmostEqual(second.pose.covariance[14], 0.05 ** 2)

    def test_truth_mode_is_explicit_and_not_the_sitl_default(self):
        generic = (ROOT / "launch" / "task1_px4_sitl.launch").read_text()
        regression = (
            ROOT / "launch" / "task1_trajectory_sync_sitl.launch").read_text()
        self.assertIn(
            '<arg name="use_sitl_vision_truth" default="false"/>', generic)
        self.assertIn(
            '<arg name="use_sitl_vision_truth" value="true"/>', regression)


if __name__ == "__main__":
    unittest.main()
