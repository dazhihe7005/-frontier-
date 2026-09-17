#!/usr/bin/env python3
"""Geometry and time-alignment checks for the maze clearance audit."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_maze_clearance.py"
SPEC = importlib.util.spec_from_file_location("analyze_maze_clearance", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MazeClearanceTest(unittest.TestCase):
    def test_box_distance_is_zero_inside_and_euclidean_at_corner(self):
        box = (1.0, 2.0, -1.0, 1.0)
        self.assertEqual(MODULE.horizontal_distance(1.5, 0.0, box), 0.0)
        self.assertAlmostEqual(MODULE.horizontal_distance(0.7, 1.4, box), 0.5)

    def test_world_geometry_uses_collision_box_and_spawn_offset(self):
        world = SCRIPT.parent.parent / "worlds" / "goaf_serpentine_maze.world"
        boxes = MODULE.baffles_from_world(world, -8.0)
        self.assertEqual(boxes["baffle_1_right_opening"],
                         (17.7, 18.3, -20.0, 6.0))
        self.assertEqual(boxes["baffle_2_left_opening"],
                         (27.7, 28.3, -6.0, 20.0))

    def test_nearest_command_matches_in_simulation_time(self):
        samples = [(1.0, 0.0, 0.0), (1.1, 0.5, 0.0), (1.2, 1.0, 0.0)]
        self.assertEqual(MODULE.nearest(samples, 1.08), samples[1])
        self.assertEqual(MODULE.nearest(samples, 0.0), samples[0])
        self.assertEqual(MODULE.nearest(samples, 2.0), samples[-1])

    def test_partial_run_cannot_pass_wall_clearance(self):
        report = {"max_local_x_m": 5.55, "mission_complete": False,
                  "baffles": {"baffle_1": {"planned_surface_margin_m": 11.0,
                                           "actual_surface_margin_m": 11.0}}}
        self.assertEqual(MODULE.acceptance_failures(report, 1.0, 1.0, 45.0, True),
                         ["insufficient_forward_progress", "mission_not_complete"])

    def test_close_wall_fails_even_after_completion(self):
        report = {"max_local_x_m": 47.0, "mission_complete": True,
                  "baffles": {"baffle_3": {"planned_surface_margin_m": 0.05,
                                           "actual_surface_margin_m": 0.13}}}
        self.assertEqual(MODULE.acceptance_failures(report, 1.0, 1.0, 45.0, True),
                         ["baffle_3:planned_margin", "baffle_3:actual_margin"])


if __name__ == "__main__":
    unittest.main()
