"""Offline capsule reconstruction geometry regressions."""

import importlib.util
import math
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

    def test_tilted_mid360_fov_changes_with_yaw(self):
        visible = MODULE.mid360_visible_from_body
        body = (0.0, 0.0, 1.16)
        low_behind = (-2.0, 0.0, 0.5)
        self.assertFalse(visible(low_behind, body, 0.0))
        self.assertTrue(visible(low_behind, body, math.pi))
        low_oblique = (1.3, 0.7, 0.3)
        self.assertFalse(any(visible(low_oblique, body,
                                     2.0*math.pi*i/72.0)
                             for i in range(72)))

    def test_observed_near_wall_surface_is_blind_for_full_yaw_sweep(self):
        # Independent mesh nearest point from the 120.184 s facility near
        # miss in run b3db8548. The actual Mid-360S -7..+52 deg aperture
        # cannot reveal it merely by yawing the aircraft in place.
        body = (-4.48287, -1.36765, 1.43406)
        surface = (-4.43517, -2.21845, 0.80644)
        self.assertFalse(any(MODULE.mid360_visible_from_body(
            surface, body, 2.0*math.pi*i/720.0)
            for i in range(720)))


if __name__ == "__main__":
    unittest.main()
