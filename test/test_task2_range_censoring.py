#!/usr/bin/env python3

"""Unit regression for the Gazebo range-limit audit semantics."""

import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "analyze_task2_sitl_bag.py"
SPEC = importlib.util.spec_from_file_location("analyze_task2_sitl_bag", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Task2RangeCensoringTest(unittest.TestCase):
    def test_saturated_max_range_is_not_a_distance_measurement(self):
        self.assertTrue(MODULE.is_saturated_range(30.0, 30.0))
        self.assertIsNone(MODULE.finite_range_alignment_error(30.0, 30.0, 46.8))

    def test_resolved_bottom_can_be_compared_with_world_truth(self):
        self.assertFalse(MODULE.is_saturated_range(17.0, 30.0))
        self.assertAlmostEqual(
            MODULE.finite_range_alignment_error(17.0, 30.0, 17.03), 0.03)

    def test_infinite_out_of_range_is_also_not_a_distance(self):
        self.assertFalse(MODULE.is_saturated_range(float("inf"), 30.0))
        self.assertIsNone(MODULE.finite_range_alignment_error(
            float("inf"), 30.0, 46.8))

    def test_circular_shaft_margin_uses_radial_distance(self):
        self.assertAlmostEqual(
            MODULE.side_margin(0.3, 0.4, 5.0, 0.5905, 2.5), 1.4095)

    def test_legacy_square_margin_is_unchanged(self):
        self.assertAlmostEqual(
            MODULE.side_margin(0.3, 0.4, 5.0, 0.4), 4.2)


if __name__ == "__main__":
    unittest.main()
