#!/usr/bin/env python3
"""Offline check of projected GBPlanner points against true collision meshes.

The projected points depend on a short-lived odometry/truth pose pairing and
are approximate. This is diagnostic evidence, never a source of flight data.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_baixiangshan_reference import DEFAULT_MODEL, MESH_NAMES, load_obj_bvh


def parse_args():
    argv = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--planned", required=True, type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--x-min", type=float, default=-math.inf)
    parser.add_argument("--x-max", type=float, default=math.inf)
    parser.add_argument("--y-min", type=float, default=-math.inf)
    parser.add_argument("--y-max", type=float, default=math.inf)
    parser.add_argument("--time-min", type=float, default=-math.inf)
    parser.add_argument("--time-max", type=float, default=math.inf)
    args = parser.parse_args(argv)
    if args.safety_radius <= 0 or args.x_min > args.x_max or \
            args.y_min > args.y_max or args.time_min > args.time_max:
        parser.error("invalid safety radius or region")
    return args


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024*1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_bound(value):
    return value if math.isfinite(value) else None


def main():
    args = parse_args()
    meshes = {name: args.model / "meshes" / ("baixianshan_"+name+".obj")
              for name in MESH_NAMES}
    bvhs = {name: load_obj_bvh(path) for name, path in meshes.items()}
    rows = 0
    selected = 0
    below_radius = 0
    minimum = math.inf
    worst = None
    per_mesh_min = {name: math.inf for name in MESH_NAMES}
    per_plan = {}
    run_ids = set()
    with args.planned.open("r", encoding="utf-8", newline="") as source:
        for row in csv.DictReader(source):
            rows += 1
            run_ids.add(row.get("ros_run_id", ""))
            try:
                stamp = float(row["sim_time"])
                xyz = tuple(float(row[key]) for key in ("x", "y", "z"))
                if not all(math.isfinite(value) for value in (stamp, *xyz)):
                    continue
                if not (args.x_min <= xyz[0] <= args.x_max and
                        args.y_min <= xyz[1] <= args.y_max and
                        args.time_min <= stamp <= args.time_max):
                    continue
                plan_index = int(row.get("trajectory_index") or row["header_seq"])
            except (KeyError, TypeError, ValueError):
                continue
            selected += 1
            center = Vector(xyz)
            distances = {}
            for name, bvh in bvhs.items():
                _, _, _, distance = bvh.find_nearest(center, 10.0)
                distances[name] = distance if distance is not None else math.inf
                per_mesh_min[name] = min(per_mesh_min[name], distances[name])
            clearance = min(distances.values())
            if clearance < args.safety_radius:
                below_radius += 1
            plan = per_plan.setdefault(plan_index, {
                "header_seq": row["header_seq"], "points": 0,
                "min_clearance_m": math.inf,
                "first_point_error_m": float(row["first_point_error_m"]),
            })
            plan["points"] += 1
            plan["min_clearance_m"] = min(plan["min_clearance_m"], clearance)
            if clearance < minimum:
                minimum = clearance
                worst = {
                    "sim_time": stamp, "trajectory_index": plan_index,
                    "header_seq": row["header_seq"],
                    "point_index": int(row["point_index"]),
                    "point_time_s": float(row.get("point_time_s") or 0.0),
                    "projected_xyz_m": xyz,
                    "nearest_mesh": min(distances, key=distances.get),
                    "clearance_m": round(clearance, 5),
                    "first_point_error_m": plan["first_point_error_m"],
                    "truth_age_s": float(row["truth_age_s"]),
                    "odom_age_s": float(row["odom_age_s"]),
                }
    report = {
        "kind": "diagnostic_only_approx_projected_plan_to_collision_mesh",
        "planned_file": str(args.planned),
        "ros_run_id": next(iter(run_ids)) if len(run_ids) == 1 else None,
        "model": str(args.model),
        "mesh_sha256": {name: sha256(path) for name, path in meshes.items()},
        "safety_radius_m": args.safety_radius,
        "region": {"x_m": [finite_bound(args.x_min), finite_bound(args.x_max)],
                   "y_m": [finite_bound(args.y_min), finite_bound(args.y_max)],
                   "sim_time_s": [finite_bound(args.time_min),
                                  finite_bound(args.time_max)]},
        "all_rows": rows,
        "selected_points": selected,
        "selected_trajectories": len(per_plan),
        "projected_points_below_radius": below_radius,
        "min_projected_clearance_m": (
            round(minimum, 5) if math.isfinite(minimum) else None),
        "per_mesh_min_m": {name: round(value, 5) if math.isfinite(value)
                           else None for name, value in per_mesh_min.items()},
        "worst_projected_point": worst,
        "worst_trajectories": [
            {"trajectory_index": index, "header_seq": plan["header_seq"],
             "points": plan["points"],
             "min_clearance_m": round(plan["min_clearance_m"], 5),
             "first_point_error_m": plan["first_point_error_m"]}
            for index, plan in sorted(per_plan.items(),
                                      key=lambda item: item[1]["min_clearance_m"])[:10]],
        "executed_or_controller_safe_proven": False,
    }
    formatted = json.dumps(report, indent=2, allow_nan=False)
    print(formatted, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(formatted+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
