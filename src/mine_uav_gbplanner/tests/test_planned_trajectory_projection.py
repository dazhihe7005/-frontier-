#!/usr/bin/env python3
"""Check that truth projection is a diagnostic coordinate transform only."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/diagnostic_planned_trajectory.py"
SPEC = importlib.util.spec_from_file_location("diagnostic_planned_trajectory", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PlannedTrajectoryProjectionTest(unittest.TestCase):
    def test_translation_and_relative_height(self):
        result = MODULE.project_point(
            (12.0, 4.0, 7.0), (10.0, 5.0, 6.0), 0.0,
            (100.0, 200.0, 2.0), 0.0)
        self.assertEqual(result, (102.0, 199.0, 3.0))

    def test_relative_yaw_rotates_horizontal_displacement(self):
        result = MODULE.project_point(
            (11.0, 5.0, 6.0), (10.0, 5.0, 6.0), 0.0,
            (100.0, 200.0, 2.0), math.pi/2.0)
        self.assertAlmostEqual(result[0], 100.0)
        self.assertAlmostEqual(result[1], 201.0)
        self.assertAlmostEqual(result[2], 2.0)

    def test_odom_anchor_maps_exactly_to_truth_anchor(self):
        result = MODULE.project_point(
            (10.0, 5.0, 6.0), (10.0, 5.0, 6.0), -0.5,
            (100.0, 200.0, 2.0), 1.1)
        self.assertAlmostEqual(result[0], 100.0)
        self.assertAlmostEqual(result[1], 200.0)
        self.assertAlmostEqual(result[2], 2.0)


if __name__ == "__main__":
    unittest.main()
