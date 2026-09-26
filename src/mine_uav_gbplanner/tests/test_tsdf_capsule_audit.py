"""Offline capsule reconstruction geometry regressions."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "tools" / "audit_tsdf_capsule.py"
SPEC = importlib.util.spec_from_file_location("audit_tsdf_capsule", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TsdfCapsuleAuditTests(unittest.TestCase):
    def test_segment_distance_clamps_to_endpoints(self):
        distance = MODULE.capsule_distance
        self.assertAlmostEqual(distance((0.5, 1.0, 0.0),
                                        (0, 0, 0), (1, 0, 0)), 1.0)
        self.assertAlmostEqual(distance((2.0, 0.0, 0.0),
                                        (0, 0, 0), (1, 0, 0)), 1.0)

    def test_known_occupied_is_separate_from_unknown(self):
        row = {"x": "0.1", "y": "0.1", "z": "0.1",
               "tsdf_distance_m": "0.01", "sim_time": "5.0",
               "frame_id": "camera_init", "ros_run_id": "test"}
        report = MODULE.analyze([row], (0, 0, 0), (0.2, 0, 0),
                                0.2, 0.2, 0.05)
        self.assertGreater(report["counts"]["known_occupied"], 0)
        self.assertGreater(report["counts"]["unknown_in_csv"], 0)
        self.assertFalse(report["planner_decision_reproduced_exactly"])

    def test_outside_snapshot_is_not_called_unknown(self):
        row = {"x": "0.1", "y": "0.1", "z": "0.1",
               "tsdf_distance_m": "0.2", "sim_time": "5.0",
               "frame_id": "camera_init", "ros_run_id": "test"}
        report = MODULE.analyze([row], (0, 0, 0), (0.2, 0, 0),
                                0.2, 0.2, 0.05,
                                (0.0, 0.2, 0.0, 0.2, 0.0, 0.2))
        self.assertGreater(report["counts"]["outside_snapshot"], 0)
        self.assertEqual(report["counts"]["unknown_in_csv"], 0)


if __name__ == "__main__":
    unittest.main()
