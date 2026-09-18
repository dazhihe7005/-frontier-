#!/usr/bin/env python3

"""SITL-only fault injection must be opt-in and ramp, not jump."""

import importlib.util
import pathlib
import unittest

import rospy


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sitl_shaft_sensor_adapter.py"
SPEC = importlib.util.spec_from_file_location("sitl_shaft_sensor_adapter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DepthDriftTest(unittest.TestCase):
    def make_adapter(self, after, rate):
        adapter = object.__new__(MODULE.SitlShaftSensorAdapter)
        adapter.depth_drift_after_depth = after
        adapter.depth_drift_rate_mps = rate
        adapter.depth_drift_start = None
        return adapter

    def test_default_disabled(self):
        adapter = self.make_adapter(-1.0, 0.0)
        self.assertEqual(adapter.injected_depth_bias(100.0, rospy.Time(10)), 0.0)

    def test_bias_starts_at_threshold_and_grows_continuously(self):
        adapter = self.make_adapter(6.0, 0.5)
        self.assertEqual(adapter.injected_depth_bias(5.9, rospy.Time(10)), 0.0)
        self.assertEqual(adapter.injected_depth_bias(6.0, rospy.Time(12)), 0.0)
        self.assertAlmostEqual(
            adapter.injected_depth_bias(6.5, rospy.Time(14)), 1.0)


if __name__ == "__main__":
    unittest.main()
