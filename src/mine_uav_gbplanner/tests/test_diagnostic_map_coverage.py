#!/usr/bin/env python3
"""Pure-Python tests for the read-only coverage geometry."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diagnostic_map_coverage.py"
spec = importlib.util.spec_from_file_location("diagnostic_map_coverage", SCRIPT)
coverage = importlib.util.module_from_spec(spec)
spec.loader.exec_module(coverage)


class CoverageGridTests(unittest.TestCase):
    def setUp(self):
        self.grid = coverage.CoverageGrid({
            "resolution_m": 1.0,
            "origin_xy_m": [0.0, 0.0],
            "reachable_cells": [[0, 0, 2.0], [1, 0, 2.0],
                                [2, 0, 2.0], [1, 1, 4.0]],
        })

    def test_ray_marks_only_height_compatible_cells(self):
        self.grid.trace_ray((0.5, 0.5, 2.0), (2.5, 0.5, 2.0), 10.0)
        self.assertEqual(self.grid.seen, {(0, 0), (1, 0), (2, 0)})
        self.assertEqual(self.grid.report(12.0)["visibility_fraction"], 0.75)
        self.assertFalse(self.grid.report(12.0)["complete"])
        self.assertEqual(self.grid.report(12.0)["largest_unseen_components"][0]["cells"], 1)

    def test_visited_and_visible_are_distinct(self):
        self.grid.visit(0.5, 0.5, 2.0, 10.0)
        self.grid.trace_ray((0.5, 0.5, 2.0), (1.5, 0.5, 2.0), 11.0)
        report = self.grid.report(13.0)
        self.assertEqual(report["visited_cells"], 1)
        self.assertEqual(report["visible_cells"], 2)
        self.assertEqual(report["seconds_since_new_cell"], 2.0)

    def test_quaternion_rotation(self):
        result = coverage.rotate((0, 0, 0, 1), (1, 2, 3))
        self.assertEqual(result, (1, 2, 3))


class CoverageGrid3DTests(unittest.TestCase):
    def setUp(self):
        self.grid = coverage.CoverageGrid3D({
            "kind": "diagnostic_only_3d_reachable_centres_26_connected",
            "resolution_m": 1.0,
            "origin_xyz_m": [0.0, 0.0, 0.0],
            "reachable_cells": [[0, 0, 0], [1, 0, 0], [1, 0, 1]],
        })

    def test_distinguishes_stacked_heights(self):
        self.grid.visit(0.5, 0.5, 0.5, 1.0)
        self.grid.trace_ray((0.5, 0.5, 0.5), (1.5, 0.5, 0.5), 2.0)
        report = self.grid.report(3.0)
        self.assertEqual(report["visible_cells"], 2)
        self.assertEqual(report["visited_cells"], 1)
        self.assertEqual(report["unseen_cells"], 1)
        self.assertEqual(report["largest_unseen_components"][0]["xyz_range_m"][2],
                         [1.5, 1.5])
        self.grid.trace_ray((1.5, 0.5, 0.5), (1.5, 0.5, 1.5), 4.0)
        self.assertTrue(self.grid.report(5.0)["complete"])

    def test_rejects_2d_reference(self):
        with self.assertRaises(ValueError):
            coverage.CoverageGrid3D({"kind": "diagnostic_only_coarse_xy_reachability"})


if __name__ == "__main__":
    unittest.main()
