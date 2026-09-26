"""Visual velocity transport must fail closed on jumps and rotate correctly."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "vision_velocity_geometry.py"
SPEC = importlib.util.spec_from_file_location("vision_velocity_geometry", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class VisionVelocityGeometryTests(unittest.TestCase):
    def test_yaw_rotates_world_velocity_into_body(self):
        half = math.sqrt(0.5)
        body = MODULE.world_to_body((0.0, 1.0, 0.25), (0.0, 0.0, half, half))
        self.assertAlmostEqual(body[0], 1.0)
        self.assertAlmostEqual(body[1], 0.0)
        self.assertAlmostEqual(body[2], 0.25)

    def test_three_distinct_frames_required(self):
        window = MODULE.VelocityWindow()
        self.assertIsNone(window.add(1.0, (0.0, 0.0, 0.0)))
        self.assertIsNone(window.add(1.1, (0.1, 0.0, 0.0)))
        velocity = window.add(1.2, (0.2, 0.0, 0.0))
        self.assertAlmostEqual(velocity[0], 1.0)
        self.assertEqual(velocity[2], 0.0)

    def test_discontinuity_and_nonmonotonic_stamp_reset(self):
        window = MODULE.VelocityWindow()
        for stamp, x in ((1.0, 0.0), (1.1, 0.1), (1.2, 0.2)):
            window.add(stamp, (x, 0.0, 0.0))
        self.assertIsNone(window.add(1.3, (5.0, 0.0, 0.0)))
        self.assertIsNone(window.add(1.2, (5.0, 0.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
