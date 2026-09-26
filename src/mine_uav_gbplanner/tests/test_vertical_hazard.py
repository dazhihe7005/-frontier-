#!/usr/bin/env python3
"""Protect the distinction between a side-wall return and a roof/floor return."""

import importlib.util
import math
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

    def test_opposite_floor_bounds_roof_escape(self):
        # A 0.35 m descent would violate the 1 m radius when the floor is
        # only 1.2 m below. The extra 0.05 m reserve limits it to 0.15 m.
        self.assertAlmostEqual(
            MODULE.vertical_escape_room(0.0, -1.20, 1.05), 0.15)
        self.assertEqual(
            MODULE.vertical_escape_room(0.0, -1.00, 1.05), 0.0)
        self.assertTrue(math.isinf(
            MODULE.vertical_escape_room(1.20, -0.10, 1.05)))

    def test_sidewall_does_not_latch_completed_vertical_escape(self):
        # Previous code used overall spatial clearance. In a narrow tunnel
        # a 1.3 m sidewall kept a roof escape latched indefinitely.
        self.assertFalse(MODULE.vertical_escape_pending(
            True, 0.02, 1.50, 1.25))
        self.assertTrue(MODULE.vertical_escape_pending(
            True, 0.02, 1.20, 1.25))
        self.assertTrue(MODULE.vertical_escape_pending(
            True, 0.12, 1.50, 1.25))

    def test_downward_blind_cone_brakes_before_floor(self):
        scale = MODULE.floor_descent_scale
        self.assertEqual(scale(1.20, 1.0, 0.25, 0.55), 0.0)
        self.assertAlmostEqual(scale(1.40, 1.0, 0.25, 0.55), 0.5)
        self.assertEqual(scale(1.60, 1.0, 0.25, 0.55), 1.0)

    def test_rising_floor_is_guarded_during_level_flight(self):
        required = MODULE.floor_guard_required
        self.assertTrue(required(1.34, 1.0, 0.0, 0.0))
        self.assertFalse(required(1.50, 1.0, 0.0, 0.0))
        self.assertTrue(required(1.50, 1.0, -0.04, 0.0))

    def test_new_floor_threat_reverses_old_roof_escape(self):
        direction = MODULE.vertical_escape_direction
        self.assertEqual(direction(True, True, 1.20, True, -1.0), 1.0)
        self.assertEqual(direction(False, True, 1.20, False, 0.0), -1.0)
        self.assertEqual(direction(False, True, -1.20, False, 0.0), 1.0)
        self.assertEqual(direction(False, False, float("nan"), False, 0.0), 0.0)

    def test_escape_direction_persists_after_floor_recovers(self):
        # A roof return becomes the nearest vertical point while the
        # floor-escape target is still being reached. Do not reverse then.
        direction = MODULE.vertical_escape_direction
        self.assertEqual(direction(False, False, 1.72, True, 1.0), 1.0)

    def test_recovery_cannot_bypass_floor_brake(self):
        stop = MODULE.should_proximity_stop
        self.assertTrue(stop(0.0))
        self.assertTrue(stop(0.10))
        self.assertFalse(stop(0.11))

    def test_recovery_directional_guard_stops_a_new_obstacle(self):
        self.assertEqual(self.executor._proximity_scale(
            1.50, 1.50, 0.30, 0.0, True, recovery_active=True), 0.0)
        self.assertLess(self.executor._proximity_scale(
            1.50, 1.50, 0.40, 0.0, True, recovery_active=True), 0.10)
        self.assertEqual(self.executor._proximity_scale(
            1.50, 1.50, 1.00, 0.0, True, recovery_active=True), 1.0)


if __name__ == "__main__":
    unittest.main()
