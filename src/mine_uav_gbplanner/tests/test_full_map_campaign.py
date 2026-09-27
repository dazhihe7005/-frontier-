"""Repeated full-map acceptance must reject partial or incomparable trials."""

import json
import sys
from pathlib import Path
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from check_full_map_campaign import evaluate_campaign, inspect_run


class CampaignTests(unittest.TestCase):
    def setUp(self):
        mesh = {name: "same-sha" for name in
                ("ground", "infrastructure", "rock", "roof")}
        self.runs = [{
            "ros_run_id": "cold-run-{}".format(index),
            "checks": {
                "single_flight_precheck_passed": True,
                "cold_start_unarmed": True,
                "within_exploration_time_budget": True,
                "effective_speed_target_met": True,
            },
            "mesh_sha256": dict(mesh),
            "reference_cells_2d": 3089,
            "reference_cells_3d": 8460,
        } for index in (1, 2)]

    def evaluate(self):
        return evaluate_campaign(self.runs, 900.0, 0.8)

    def test_two_independent_complete_runs_pass_only_precheck(self):
        result = self.evaluate()
        self.assertTrue(result["campaign_precheck_passed"])
        self.assertFalse(result["continuous_3d_safety_or_coverage_proven"])

    def test_one_run_is_insufficient(self):
        self.runs.pop()
        result = self.evaluate()
        self.assertFalse(result["checks"]["at_least_two_recorded_runs"])
        self.assertFalse(result["campaign_precheck_passed"])

    def test_reused_run_id_is_not_repeatability(self):
        self.runs[1]["ros_run_id"] = self.runs[0]["ros_run_id"]
        self.assertFalse(self.evaluate()["checks"]["independent_ros_runs"])

    def test_map_change_invalidates_campaign(self):
        self.runs[1]["mesh_sha256"]["rock"] = "different-sha"
        self.assertFalse(self.evaluate()["checks"]["same_collision_mesh"])

    def test_reference_denominator_change_invalidates_campaign(self):
        self.runs[1]["reference_cells_3d"] = 8000
        self.assertFalse(self.evaluate()["checks"]["same_coverage_denominator"])

    def test_single_slow_or_unsafe_run_invalidates_campaign(self):
        self.runs[1]["checks"]["effective_speed_target_met"] = False
        self.assertFalse(self.evaluate()["checks"]["every_flight_passed"])
        self.runs[1]["checks"]["effective_speed_target_met"] = True
        self.runs[0]["checks"]["single_flight_precheck_passed"] = False
        self.assertFalse(self.evaluate()["checks"]["every_flight_passed"])

    def test_nonfinite_thresholds_rejected(self):
        with self.assertRaises(ValueError):
            evaluate_campaign(self.runs, float("inf"), 0.8)
        with self.assertRaises(ValueError):
            evaluate_campaign(self.runs, 900.0, float("nan"))

    def test_effective_speed_includes_hovering_and_time_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            truth = root / "truth.csv"
            truth.write_text(
                "sim_time,x,y,z,armed,offboard,exploration_started,ros_run_id\n"
                "0,0,0,0,0,0,0,cold-run-1\n"
                "1,0,0,2,1,1,1,cold-run-1\n"
                "1.1,0.05,0,2,1,1,1,cold-run-1\n"
                "1.2,0,0,2,1,1,1,cold-run-1\n"
                "3,0,0,0,0,0,1,cold-run-1\n", encoding="utf-8")
            reports = []
            for name in ("chain", "coverage", "mesh"):
                path = root / (name+".json")
                path.write_text(json.dumps({}), encoding="utf-8")
                reports.append(path)
            run = inspect_run((*reports, truth), 0.19, 0.6)
            self.assertAlmostEqual(run["effective_speed_mps"], 0.5)
            self.assertFalse(run["checks"]["within_exploration_time_budget"])
            self.assertFalse(run["checks"]["effective_speed_target_met"])


if __name__ == "__main__":
    unittest.main()
