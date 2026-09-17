#!/usr/bin/env python3
"""Check point-to-segment geometry used in the failed-seed audit."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_corridor_seed_clearance.py"
SPEC = importlib.util.spec_from_file_location("analyze_corridor_seed_clearance", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SeedDistanceTest(unittest.TestCase):
    def test_projects_to_segment_interior(self):
        self.assertAlmostEqual(MODULE.point_segment_distance(
            (1.0, 1.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), 1.0)

    def test_clamps_past_endpoint(self):
        self.assertAlmostEqual(MODULE.point_segment_distance(
            (3.0, 4.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
                               math.sqrt(17.0))

    def test_zero_length_line(self):
        self.assertAlmostEqual(MODULE.point_segment_distance(
            (3.0, 4.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), 5.0)


if __name__ == "__main__":
    unittest.main()
