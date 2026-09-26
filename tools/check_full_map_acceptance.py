#!/usr/bin/env python3
"""Fail-closed, offline precheck of one GBPlanner2 simulation flight.

Only read-only diagnostic artifacts are consumed. Passing this precheck is
*not* a proof of continuous safety or complete map exploration: 2-D and 3-D
coverage references are finite-resolution visibility proxies, and trajectory
samples are discrete. It never supplies Gazebo truth to flight software.
"""

import argparse
import csv
import json
import math
from pathlib import Path


def read_truth(path):
    first = last = first_exploration = last_exploration = None
    rows = exploration_rows = 0
    run_ids = set()
    with path.open("r", encoding="utf-8", newline="") as source:
        for row in csv.DictReader(source):
            rows += 1
            run_ids.add(row.get("ros_run_id", ""))
            stamp = float(row["sim_time"])
            position = tuple(float(row[key]) for key in ("x", "y", "z"))
            if not math.isfinite(stamp) or not all(map(math.isfinite, position)):
                raise ValueError("non-finite truth trajectory sample")
            sample = {"time": stamp, "position": position,
                      "armed": row["armed"] == "1",
                      "offboard": row["offboard"] == "1",
                      "exploration": row["exploration_started"] == "1"}
            if last is not None and stamp <= last["time"]:
                raise ValueError("truth trajectory sim_time is not increasing")
            if first is None:
                first = sample
            last = sample
            if sample["armed"] and sample["offboard"] and sample["exploration"]:
                exploration_rows += 1
                if first_exploration is None:
                    first_exploration = sample
                last_exploration = sample
    if not rows:
        raise ValueError("empty truth trajectory")
    return {"rows": rows, "exploration_rows": exploration_rows,
            "ros_run_id": next(iter(run_ids)) if len(run_ids) == 1 else None,
            "first": first, "last": last,
            "first_exploration": first_exploration,
            "last_exploration": last_exploration}


def number(data, key):
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def evaluate(chain, coverage, mesh, truth, truth_path):
    """Return conservative checks; missing or cross-run evidence fails."""
    first = truth["first"]
    last = truth["last"]
    flight_start = truth["first_exploration"]
    flight_end = truth["last_exploration"]
    start = number(chain, "observation_start_s")
    end = number(chain, "observation_end_s")
    coverage_end = number(coverage, "last_truth_sim_time_s")
    scan_end = number(coverage, "last_accepted_scan_sim_time_s")
    coverage_3d = coverage.get("three_d", {})
    if not isinstance(coverage_3d, dict):
        coverage_3d = {}
    mesh_path = mesh.get("trajectory_file")
    same_truth = isinstance(mesh_path, str) and Path(mesh_path).resolve() == truth_path.resolve()
    run_id = truth.get("ros_run_id")
    home_xy = math.dist(first["position"][:2], last["position"][:2])
    home_z = abs(first["position"][2]-last["position"][2])
    checks = {
        "same_ros_launch_run": isinstance(run_id, str) and bool(run_id) and
            chain.get("ros_run_id") == run_id and
            coverage.get("ros_run_id") == run_id and
            mesh.get("ros_run_id") == run_id,
        "chain_checks_passed": chain.get("passed") is True and
            chain.get("pass_scope") == "full_chain_and_sampled_sensor_checks_only" and
            chain.get("checks", {}).get("observation_complete") is True,
        "chain_observed_whole_exploration": flight_start is not None and
            start is not None and end is not None and
            start <= flight_start["time"]+0.2 and
            end >= flight_end["time"]-0.2,
        "coverage_reference_all_visible": coverage.get("kind") ==
            "diagnostic_only_lidar_visibility_proxy" and
            isinstance(coverage.get("reference_cells"), int) and
            coverage["reference_cells"] > 0 and
            coverage.get("visible_cells") == coverage["reference_cells"] and
            coverage.get("unseen_cells") == 0 and coverage.get("complete") is True,
        "coverage_3d_reference_all_visible": coverage_3d.get("kind") ==
            "diagnostic_only_3d_lidar_visibility_proxy" and
            isinstance(coverage_3d.get("reference_cells"), int) and
            coverage_3d["reference_cells"] > 0 and
            coverage_3d.get("visible_cells") == coverage_3d["reference_cells"] and
            coverage_3d.get("unseen_cells") == 0 and
            coverage_3d.get("complete") is True,
        "coverage_3d_mesh_matches_flight":
            coverage.get("reference_3d_mesh_sha256") ==
            mesh.get("mesh_sha256") and
            set(coverage.get("reference_3d_mesh_sha256") or {}) ==
            {"ground", "infrastructure", "rock", "roof"},
        "coverage_observed_whole_flight": flight_start is not None and
            coverage_end is not None and scan_end is not None and
            number(coverage, "first_truth_sim_time_s") is not None and
            coverage["first_truth_sim_time_s"] <= first["time"]+0.2 and
            coverage_end >= flight_end["time"]-0.2 and
            scan_end >= flight_end["time"]-2.0,
        "mesh_matches_whole_truth_record": mesh.get("kind") ==
            "diagnostic_only_truth_to_collision_mesh_distance" and
            same_truth and mesh.get("all_truth_rows") == truth["rows"] and
            mesh.get("armed_offboard_exploration_samples") == truth["exploration_rows"] and
            truth["exploration_rows"] > 0 and
            set(mesh.get("mesh_sha256", {})) ==
            {"ground", "infrastructure", "rock", "roof"},
        "sampled_mesh_radius_with_margin": number(mesh, "safety_radius_m") == 1.0 and
            number(mesh, "sampled_min_clearance_m") is not None and
            mesh["sampled_min_clearance_m"] >= 1.15 and
            mesh.get("sampled_below_radius_count") == 0 and
            mesh.get("samples_with_no_surface_within_cutoff") == 0 and
            number(mesh, "max_sample_gap_s") is not None and
            mesh["max_sample_gap_s"] <= 0.2 and
            number(mesh, "piecewise_linear_clearance_lower_bound_m") is not None and
            mesh["piecewise_linear_clearance_lower_bound_m"] >= 1.0,
        "returned_and_disarmed_near_home": flight_start is not None and
            last["time"]-flight_end["time"] >= 1.0 and
            not last["armed"] and home_xy <= 2.0 and home_z <= 0.5,
    }
    return {
        "kind": "diagnostic_only_full_map_automatic_precheck",
        "ros_run_id": run_id,
        "automatic_precheck_passed": all(checks.values()),
        "continuous_3d_safety_or_coverage_proven": False,
        "checks": checks,
        "metrics": {
            "reference_visible_cells": coverage.get("visible_cells"),
            "reference_total_cells": coverage.get("reference_cells"),
            "reference_3d_visible_cells": coverage_3d.get("visible_cells"),
            "reference_3d_total_cells": coverage_3d.get("reference_cells"),
            "sampled_min_mesh_clearance_m": mesh.get("sampled_min_clearance_m"),
            "mesh_samples_below_1m": mesh.get("sampled_below_radius_count"),
            "truth_exploration_samples": truth["exploration_rows"],
            "home_xy_error_m": round(home_xy, 3),
            "home_z_error_m": round(home_z, 3),
            "mean_moving_speed_mps": chain.get("mean_moving_speed_mps"),
            "blocked_events": chain.get("blocked_events"),
            "recovery_events": chain.get("recovery_events"),
        },
        "limitations": [
            "2-D and 3-D coverage references are finite-resolution lidar-visibility proxies, not a proof of all continuous reachable free space.",
            "Discrete mesh samples and linear interpolation do not prove continuous flight clearance.",
            "Simulated extra downward sensing is not confirmed real hardware.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chain", required=True, type=Path)
    parser.add_argument("--coverage", required=True, type=Path)
    parser.add_argument("--mesh", required=True, type=Path)
    parser.add_argument("--truth", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    chain = json.loads(args.chain.read_text(encoding="utf-8"))
    coverage = json.loads(args.coverage.read_text(encoding="utf-8"))
    mesh = json.loads(args.mesh.read_text(encoding="utf-8"))
    result = evaluate(chain, coverage, mesh, read_truth(args.truth), args.truth)
    formatted = json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2)
    print(formatted)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(formatted + "\n", encoding="utf-8")
    return 0 if result["automatic_precheck_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
