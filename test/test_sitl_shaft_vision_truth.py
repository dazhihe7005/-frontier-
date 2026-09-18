#!/usr/bin/env python3

"""External-vision world truth is isolated from any real MAVROS master."""

import importlib.util
import pathlib
import unittest
from unittest.mock import MagicMock, patch

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose


PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sitl_shaft_vision_truth.py"
SPEC = importlib.util.spec_from_file_location("sitl_shaft_vision_truth", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SitlVisionTruthTest(unittest.TestCase):
    def test_rejects_real_master_or_fcu(self):
        with patch.dict(MODULE.os.environ, {"ROS_MASTER_URI": "http://localhost:11312"}), \
                patch.object(rospy, "get_param", return_value=True):
            self.assertFalse(MODULE.SitlShaftVisionTruth.sitl_only())
        with patch.dict(MODULE.os.environ, {"ROS_MASTER_URI": "http://localhost:11319"}), \
                patch.object(rospy, "get_param", side_effect=lambda key, default=None:
                             True if key == "/use_sim_time" else "serial:///dev/ttyUSB0"):
            self.assertFalse(MODULE.SitlShaftVisionTruth.sitl_only())

    def make_adapter(self, stop_after_depth=-1.0):
        adapter = object.__new__(MODULE.SitlShaftVisionTruth)
        adapter.vehicle_model = "iris_vision"
        adapter.entrance_world_z = 0.25
        adapter.stop_after_depth = stop_after_depth
        adapter.publish_rate = 30.0
        adapter.last_publish = rospy.Time(0)
        adapter.last_safety_check = rospy.Time(0)
        adapter.safe_link = False
        adapter.stopped = False
        adapter.pub = MagicMock()
        adapter.active_pub = MagicMock()
        return adapter

    def model(self, z):
        model = ModelStates()
        model.name = ["iris_vision"]
        pose = Pose()
        pose.position.z = z
        pose.orientation.w = 1.0
        model.pose = [pose]
        return model

    def test_only_simulated_truth_publishes_finite_pose(self):
        adapter = self.make_adapter()
        with patch.object(adapter, "sitl_only", return_value=True), \
                patch.object(rospy.Time, "now", return_value=rospy.Time(10)):
            adapter.on_models(self.model(1.0))
        estimate = adapter.pub.publish.call_args.args[0]
        self.assertEqual(estimate.header.frame_id, "odom")
        self.assertEqual(estimate.pose.pose.position.z, 1.0)
        self.assertAlmostEqual(estimate.pose.covariance[14], 0.05 ** 2)

    def test_vision_stream_stops_after_depth_threshold(self):
        adapter = self.make_adapter(stop_after_depth=6.0)
        with patch.object(adapter, "sitl_only", return_value=True), \
                patch.object(rospy.Time, "now", return_value=rospy.Time(10)):
            adapter.on_models(self.model(-6.0))
        self.assertTrue(adapter.stopped)
        adapter.pub.publish.assert_not_called()
        self.assertFalse(adapter.active_pub.publish.call_args.args[0].data)


if __name__ == "__main__":
    unittest.main()
