"""Read-only exact voxel-ray audit geometry; no ROS master required."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "fastlio_voxblox_adapter.py"
spec = importlib.util.spec_from_file_location("fastlio_voxblox_adapter", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VoxelInputAuditTest(unittest.TestCase):
    def test_segment_crosses_voxel_before_endpoint(self):
        hit = module.segment_intersects_voxel
        self.assertTrue(hit((0, 0, 0), (3, 0, 0), (2, 0, 0), 0.1))
        self.assertFalse(hit((0, 0, 0), (1, 0, 0), (2, 0, 0), 0.1))

    def test_parallel_ray_outside_voxel(self):
        self.assertFalse(module.segment_intersects_voxel(
            (0, 0.2, 0), (3, 0.2, 0), (2, 0, 0), 0.1))


if __name__ == "__main__":
    unittest.main()
