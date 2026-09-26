#!/usr/bin/env python3
"""Protect the distinction between a side-wall return and a roof/floor return."""

import importlib.util
from pathlib import Path
import unittest


EXECUTOR = Path(__file__).resolve().parents[1] / "scripts/gbplanner_px4_executor.py"
SPEC = importlib.util.spec_from_file_location("gbplanner_px4_executor", EXECUTOR)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class VerticalHazardTest(unittest.TestCase):
    def setUp(self):
        self.executor = MODULE.GbplannerPx4Executor.__new__(
            MODULE.GbplannerPx4Executor)
        self.executor.proximity_z_max = 0.35
        self.executor.safety_radius = 1.0
        self.executor.vertical_avoidance_margin = 0.30

    def test_raised_side_wall_is_not_a_ceiling(self):
        # This return previously locked the UAV at x ~= 2.5 m for >100 s.
        self.assertFalse(self.executor._is_vertical_hazard(
            1.276, 0.356, True))

    def test_roof_and_floor_remain_protected(self):
        self.assertTrue(self.executor._is_vertical_hazard(1.276, 1.20, True))
        self.assertTrue(self.executor._is_vertical_hazard(1.276, -1.20, True))

    def test_clear_or_stale_return_does_not_trigger_escape(self):
        self.assertFalse(self.executor._is_vertical_hazard(1.40, 1.35, True))
        self.assertFalse(self.executor._is_vertical_hazard(1.276, 1.20, False))


if __name__ == "__main__":
    unittest.main()
