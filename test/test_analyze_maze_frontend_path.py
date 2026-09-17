#!/usr/bin/env python3
"""Exact segment-to-wall geometry checks for the maze guide-path audit."""

import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_maze_frontend_path.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("analyze_maze_frontend_path", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SegmentBoxDistanceTest(unittest.TestCase):
    BOX = (17.7, 18.3, -20.0, 6.0)

    def test_path_crossing_wall_is_zero(self):
        self.assertEqual(MODULE.segment_box_distance_xy(
            (17.0, 0.0), (19.0, 0.0), self.BOX), 0.0)

    def test_parallel_path_has_exact_standoff(self):
        self.assertAlmostEqual(MODULE.segment_box_distance_xy(
            (16.3, 5.3), (16.3, 5.7), self.BOX), 1.4)

    def test_diagonal_to_corner(self):
        self.assertAlmostEqual(MODULE.segment_box_distance_xy(
            (16.7, 7.0), (16.0, 8.0), self.BOX), 2 ** 0.5)

    def test_path_inside_wall_is_zero(self):
        self.assertEqual(MODULE.segment_box_distance_xy(
            (17.8, 0.0), (18.1, 0.0), self.BOX), 0.0)


if __name__ == "__main__":
    unittest.main()
