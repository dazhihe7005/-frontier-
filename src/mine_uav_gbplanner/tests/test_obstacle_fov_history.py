"""Offline sensor-aperture audit geometry; no Gazebo truth enters control."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "tools/audit_obstacle_fov_history.py"
SPEC = importlib.util.spec_from_file_location("audit_obstacle_fov_history", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ObstacleFovHistoryTests(unittest.TestCase):
    def test_world_to_body_rotation(self):
        q = (0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4))
        body = MODULE.rotate_inverse((1.0, 0.0, 0.0), q)
        self.assertAlmostEqual(body[0], 0.0, places=6)
        self.assertAlmostEqual(body[1], -1.0, places=6)
        world = MODULE.rotate_forward(body, q)
        self.assertAlmostEqual(world[0], 1.0, places=6)
        self.assertAlmostEqual(world[1], 0.0, places=6)

    def test_side_down_nearest_point_outside_both_apertures(self):
        result = MODULE.aperture((0.24777, -0.72127, -0.71003))
        self.assertFalse(result["mid360_fov"])
        self.assertFalse(result["sim_down_fov"])
        self.assertAlmostEqual(result["mid360_sensor_elevation_deg"],
                               -42.23, places=2)

    def test_overhead_nearest_point_inside_mid360_aperture(self):
        result = MODULE.aperture((0.14986, -0.41938, 0.89143))
        self.assertTrue(result["mid360_fov"])
        self.assertFalse(result["sim_down_fov"])

    def test_history_uses_only_armed_exploration_samples(self):
        rows = [
            {"sim_time": "0", "x": "0", "y": "0", "z": "0",
             "qx": "0", "qy": "0", "qz": "0", "qw": "1",
             "armed": "0", "offboard": "0", "exploration_started": "0"},
            {"sim_time": "1", "x": "0", "y": "0", "z": "0",
             "qx": "0", "qy": "0", "qz": "0", "qw": "1",
             "armed": "1", "offboard": "1", "exploration_started": "1"},
        ]
        result = MODULE.audit(rows, (3.0, 0.0, 0.0))
        self.assertEqual(result["counts"]["samples"], 1)
        self.assertEqual(result["counts"]["mid360_fov_samples"], 1)


if __name__ == "__main__":
    unittest.main()
