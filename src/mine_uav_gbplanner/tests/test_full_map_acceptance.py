"""The offline full-map precheck must fail closed on missing evidence."""

import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / "tools/check_full_map_acceptance.py"
SPEC = importlib.util.spec_from_file_location("check_full_map_acceptance", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.truth_path = Path(self.temporary.name) / "truth.csv"
        self.truth_path.write_text(
            "sim_time,x,y,z,armed,offboard,exploration_started,ros_run_id\n"
            "0.0,0,0,0,0,0,0,trial-one\n"
            "1.0,0,0,2,1,1,1,trial-one\n"
            "2.0,5,0,2,1,1,1,trial-one\n"
            "3.0,0,0,0,0,0,1,trial-one\n", encoding="utf-8")
        self.truth = MODULE.read_truth(self.truth_path)
        self.chain = {
            "passed": True, "pass_scope": "full_chain_and_sampled_sensor_checks_only",
            "checks": {"observation_complete": True},
            "ros_run_id": "trial-one",
            "observation_start_s": 0.5, "observation_end_s": 3.0,
        }
        self.coverage = {
            "kind": "diagnostic_only_lidar_visibility_proxy",
            "reference_cells": 100, "visible_cells": 100,
            "unseen_cells": 0, "complete": True,
            "three_d": {
                "kind": "diagnostic_only_3d_lidar_visibility_proxy",
                "reference_cells": 200, "visible_cells": 200,
                "unseen_cells": 0, "complete": True,
            },
            "reference_3d_mesh_sha256": {name: "sha" for name in
                                         ("ground", "infrastructure", "rock", "roof")},
            "first_truth_sim_time_s": 0.0, "last_truth_sim_time_s": 3.0,
            "last_accepted_scan_sim_time_s": 2.0,
            "ros_run_id": "trial-one",
        }
        self.mesh = {
            "kind": "diagnostic_only_truth_to_collision_mesh_distance",
            "trajectory_file": str(self.truth_path),
            "ros_run_id": "trial-one",
            "all_truth_rows": 4, "armed_offboard_exploration_samples": 2,
            "mesh_sha256": {name: "sha" for name in
                            ("ground", "infrastructure", "rock", "roof")},
            "safety_radius_m": 1.0, "sampled_min_clearance_m": 1.3,
            "sampled_below_radius_count": 0,
            "samples_with_no_surface_within_cutoff": 0,
            "max_sample_gap_s": 0.1,
            "piecewise_linear_clearance_lower_bound_m": 1.2,
        }

    def evaluate(self):
        return MODULE.evaluate(self.chain, self.coverage, self.mesh,
                               self.truth, self.truth_path)

    def test_complete_artifacts_pass_only_automatic_precheck(self):
        result = self.evaluate()
        self.assertTrue(result["automatic_precheck_passed"])
        self.assertFalse(result["continuous_3d_safety_or_coverage_proven"])

    def test_sampled_sensor_pass_cannot_hide_mesh_violation(self):
        self.mesh["sampled_min_clearance_m"] = 0.9957
        self.mesh["sampled_below_radius_count"] = 1
        self.assertFalse(self.evaluate()["checks"]["sampled_mesh_radius_with_margin"])

    def test_cross_run_mesh_fails(self):
        self.mesh["trajectory_file"] = "/tmp/different_flight.csv"
        self.assertFalse(self.evaluate()["checks"]["mesh_matches_whole_truth_record"])

    def test_cross_run_id_fails(self):
        self.coverage["ros_run_id"] = "different-run"
        self.assertFalse(self.evaluate()["checks"]["same_ros_launch_run"])

    def test_partial_chain_or_coverage_fails(self):
        self.chain["observation_end_s"] = 1.5
        self.coverage["visible_cells"] = 99
        result = self.evaluate()
        self.assertFalse(result["checks"]["chain_observed_whole_exploration"])
        self.assertFalse(result["checks"]["coverage_reference_all_visible"])

    def test_missing_or_partial_3d_coverage_fails(self):
        self.coverage.pop("three_d")
        self.assertFalse(self.evaluate()["checks"]["coverage_3d_reference_all_visible"])
        self.coverage["three_d"] = {
            "kind": "diagnostic_only_3d_lidar_visibility_proxy",
            "reference_cells": 200, "visible_cells": 199,
            "unseen_cells": 1, "complete": False,
        }
        self.assertFalse(self.evaluate()["checks"]["coverage_3d_reference_all_visible"])

    def test_reference_mesh_mismatch_fails(self):
        self.coverage["reference_3d_mesh_sha256"]["rock"] = "other"
        self.assertFalse(self.evaluate()["checks"]["coverage_3d_mesh_matches_flight"])

    def test_no_return_fails(self):
        self.truth_path.write_text(
            "sim_time,x,y,z,armed,offboard,exploration_started,ros_run_id\n"
            "0.0,0,0,0,0,0,0,trial-one\n"
            "1.0,0,0,2,1,1,1,trial-one\n"
            "2.0,5,0,2,1,1,1,trial-one\n"
            "3.0,5,0,2,1,1,1,trial-one\n", encoding="utf-8")
        self.truth = MODULE.read_truth(self.truth_path)
        self.assertFalse(self.evaluate()["checks"]["returned_and_disarmed_near_home"])


if __name__ == "__main__":
    unittest.main()
