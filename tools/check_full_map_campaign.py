#!/usr/bin/env python3
"""Fail-closed offline precheck for repeated full-map GBPlanner2 flights.

Each --run supplies chain, coverage, mesh and truth artifacts from one ROS
launch. This remains a finite-resolution precheck, not a proof of continuous
flight safety, real-hardware fidelity, or all continuously reachable space.
"""

import argparse
import json
import math
from pathlib import Path

from check_full_map_acceptance import evaluate, number, read_truth


MESH_NAMES = {"ground", "infrastructure", "rock", "roof"}


def inspect_run(paths, max_exploration_s, min_effective_speed_mps):
    chain_path, coverage_path, mesh_path, truth_path = map(Path, paths)
    chain = json.loads(chain_path.read_text(encoding="utf-8"))
    coverage = json.loads(coverage_path.read_text(encoding="utf-8"))
    mesh = json.loads(mesh_path.read_text(encoding="utf-8"))
    truth = read_truth(truth_path)
    precheck = evaluate(chain, coverage, mesh, truth, truth_path)
    duration = number(truth, "flight_duration_s")
    effective_speed = number(truth, "effective_flight_speed_mps")
    checks = {
        "single_flight_precheck_passed": precheck["automatic_precheck_passed"],
        "cold_start_unarmed": not truth["first"]["armed"],
        "within_exploration_time_budget": duration is not None and
            duration > 0 and duration <= max_exploration_s,
        "effective_speed_target_met": effective_speed is not None and
            effective_speed >= min_effective_speed_mps,
    }
    return {
        "ros_run_id": truth["ros_run_id"],
        "checks": checks,
        "duration_s": duration,
        "distance_m": round(truth["flight_distance_m"], 3),
        "effective_speed_mps": effective_speed,
        "moving_speed_mps": number(chain, "mean_moving_speed_mps"),
        "mesh_sha256": mesh.get("mesh_sha256"),
        "reference_cells_2d": coverage.get("reference_cells"),
        "reference_cells_3d": coverage.get("three_d", {}).get(
            "reference_cells") if isinstance(coverage.get("three_d"), dict)
            else None,
        "precheck": precheck,
    }


def evaluate_campaign(runs, max_exploration_s, min_effective_speed_mps):
    if not math.isfinite(max_exploration_s) or max_exploration_s <= 0:
        raise ValueError("max_exploration_s must be positive and finite")
    if (not math.isfinite(min_effective_speed_mps) or
            min_effective_speed_mps <= 0):
        raise ValueError("min_effective_speed_mps must be positive and finite")
    ids = [run["ros_run_id"] for run in runs]
    meshes = [run["mesh_sha256"] for run in runs]
    reference_counts = [(run["reference_cells_2d"],
                         run["reference_cells_3d"]) for run in runs]
    checks = {
        "at_least_two_recorded_runs": len(runs) >= 2,
        "independent_ros_runs": len(ids) >= 2 and all(
            isinstance(run_id, str) and run_id for run_id in ids) and
            len(set(ids)) == len(ids),
        "same_collision_mesh": len(meshes) >= 2 and all(
            isinstance(mesh, dict) and set(mesh) == MESH_NAMES and
            all(isinstance(value, str) and value for value in mesh.values())
            for mesh in meshes) and
            all(mesh == meshes[0] for mesh in meshes[1:]),
        "same_coverage_denominator": len(reference_counts) >= 2 and all(
            isinstance(two_d, int) and two_d > 0 and
            isinstance(three_d, int) and three_d > 0
            for two_d, three_d in reference_counts) and
            all(count == reference_counts[0]
                for count in reference_counts[1:]),
        "every_flight_passed": len(runs) >= 2 and all(
            all(run["checks"].values()) for run in runs),
    }
    return {
        "kind": "diagnostic_only_repeated_full_map_automatic_precheck",
        "campaign_precheck_passed": all(checks.values()),
        "continuous_3d_safety_or_coverage_proven": False,
        "max_exploration_s": max_exploration_s,
        "min_effective_speed_mps": min_effective_speed_mps,
        "checks": checks,
        "runs": runs,
        "limitations": [
            "Finite-resolution coverage and sampled trajectory audits do not prove continuous safety or completeness.",
            "Performance thresholds are explicit acceptance inputs, not inferred from aircraft capability.",
            "Simulated downward sensing still lacks confirmed real-hardware correspondence.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", nargs=4, action="append", required=True,
                        metavar=("CHAIN", "COVERAGE", "MESH", "TRUTH"))
    parser.add_argument("--max-exploration-s", type=float, required=True)
    parser.add_argument("--min-effective-speed-mps", type=float, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        runs = [inspect_run(paths, args.max_exploration_s,
                            args.min_effective_speed_mps)
                for paths in args.run]
        result = evaluate_campaign(runs, args.max_exploration_s,
                                   args.min_effective_speed_mps)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    rendered = json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered+"\n", encoding="utf-8")
    return 0 if result["campaign_precheck_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
