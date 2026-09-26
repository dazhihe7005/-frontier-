"""The offline velocity comparison must use new vision frames only."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "tools" / "analyze_vertical_estimator.py"
SPEC = importlib.util.spec_from_file_location("analyze_vertical_estimator", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def row(t, truth, vision, px4_vz, vision_stamp=None):
    return {"ros_run_id": "one", "armed": "1", "offboard": "1",
            "exploration_started": "1", "sim_time": str(t),
            "z": str(truth), "fastlio_vision_z": str(vision),
            "fastlio_vision_stamp": str(t if vision_stamp is None else vision_stamp),
            "px4_local_z": "1.0", "px4_vz": str(px4_vz)}


class VerticalEstimatorAuditTests(unittest.TestCase):
    def test_sign_conflict_and_duplicate_frame(self):
        rows = [row(1.0, 1.0, 0.5, -0.2),
                row(1.02, 1.01, 0.5, -0.2, vision_stamp=1.0),
                row(1.1, 1.1, 0.6, -0.2),
                row(1.2, 1.2, 0.7, -0.2)]
        result = MODULE.analyze_rows(rows)
        self.assertEqual(result["vision_frames"], 3)
        self.assertEqual(result["velocity_intervals"], 2)
        self.assertEqual(result["px4_vz_opposite_truth_intervals"], 2)
        self.assertEqual(result["vision_vz_opposite_truth_intervals"], 0)

    def test_no_control_data_does_not_fake_valid_frames(self):
        rows = [row(1.0, 1.0, 0.5, -0.2),
                {**row(1.1, 1.1, 0.6, -0.2), "armed": "0"}]
        with self.assertRaises(ValueError):
            MODULE.analyze_rows(rows)


if __name__ == "__main__":
    unittest.main()
