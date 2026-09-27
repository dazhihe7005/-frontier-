"""No ROS master needed: timestamp alignment for the read-only ray audit."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diagnostic_ray_snapshot.py"
spec = importlib.util.spec_from_file_location("diagnostic_ray_snapshot", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RaySnapshotPoseAlignmentTest(unittest.TestCase):
    def test_interpolates_scan_acquisition_pose(self):
        poses = [
            (10.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            (10.1, (1.0, 0.0, 0.0),
             (0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4))),
        ]
        result = module.interpolate_pose(poses, 10.05, 0.15)
        self.assertIsNotNone(result)
        position, quaternion, bracket_width, pose_gap = result
        self.assertAlmostEqual(position[0], 0.5)
        self.assertAlmostEqual(bracket_width, 0.1)
        self.assertAlmostEqual(pose_gap, 0.05)
        self.assertAlmostEqual(2*math.atan2(quaternion[2], quaternion[3]),
                               math.pi/4)

    def test_rejects_extrapolation_and_stale_brackets(self):
        poses = [(10.0, (0, 0, 0), (0, 0, 0, 1)),
                 (10.5, (0, 0, 0), (0, 0, 0, 1))]
        self.assertIsNone(module.interpolate_pose(poses, 9.9, 0.15))
        self.assertIsNone(module.interpolate_pose(poses, 10.6, 0.15))
        self.assertIsNone(module.interpolate_pose(poses, 10.25, 0.15))

    def test_quaternion_sign_equivalence(self):
        poses = [(10.0, (0, 0, 0), (0, 0, 0, 1)),
                 (10.1, (0, 0, 0), (0, 0, 0, -1))]
        result = module.interpolate_pose(poses, 10.05, 0.15)
        self.assertAlmostEqual(abs(result[1][3]), 1.0)


if __name__ == "__main__":
    unittest.main()
