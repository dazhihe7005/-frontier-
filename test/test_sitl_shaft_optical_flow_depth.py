#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest

MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sitl_shaft_optical_flow_depth.py"
SPEC = importlib.util.spec_from_file_location("sitl_shaft_optical_flow_depth", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OpticalFlowDepthIntegrationTest(unittest.TestCase):
    def test_large_sim_time_gap_keeps_full_displacement(self):
        delta = MODULE.integrate_depth_delta(1.5, 0.9, 1.2, 1.0, 0.0)
        self.assertAlmostEqual(delta, 0.6)

    def test_drift_is_integrated_over_elapsed_sim_time(self):
        delta = MODULE.integrate_depth_delta(1.5, 1.0, 2.0, 1.0, 0.01)
        self.assertAlmostEqual(delta, 0.52)

    def test_zero_time_still_integrates_spatial_displacement(self):
        delta = MODULE.integrate_depth_delta(1.0, 0.9, 0.0, 1.0, 0.01)
        self.assertAlmostEqual(delta, 0.1)

    def test_negative_time_is_rejected(self):
        self.assertIsNone(MODULE.integrate_depth_delta(1.0, 0.9, -0.1, 1.0, 0.0))


if __name__ == "__main__":
    unittest.main()
