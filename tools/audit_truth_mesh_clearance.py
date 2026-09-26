#!/usr/bin/env python3
"""Offline nearest-surface audit for a read-only Gazebo truth trajectory.

Run with Blender's Python. This never publishes ROS data or flight commands.
The result checks sampled body-center distance against the actual collision
meshes; it cannot prove unsampled continuous motion or localization safety.
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
from build_baixiangshan_reference import MESH_NAMES, load_obj_bvh


def parse_args():
    argv = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--model", type=Path, default=Path(
        "/home/nuc/frontier-upload/models/baixianshan_tunnel"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--nearest-cutoff", type=float, default=10.0)
    args = parser.parse_args(argv)
    if args.safety_radius <= 0 or args.nearest_cutoff <= args.safety_radius:
        parser.error("nearest cutoff must exceed positive safety radius")
    return args


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    args = parse_args()
    meshes = {name: args.model / "meshes" / ("baixianshan_" + name + ".obj")
              for name in MESH_NAMES}
    bvhs = {name: load_obj_bvh(path) for name, path in meshes.items()}
    selected = []
    all_rows = 0
    with args.trajectory.open("r", encoding="utf-8", newline="") as source:
        for row in csv.DictReader(source):
            all_rows += 1
            if not all(row.get(field) == "1" for field in
                       ("armed", "offboard", "exploration_started")):
                continue
            values = tuple(float(row[key]) for key in
                           ("sim_time", "x", "y", "z"))
            if all(math.isfinite(value) for value in values):
                selected.append(values)
    if not selected:
        raise RuntimeError("no armed OFFBOARD exploration samples in trajectory")

    minimum = math.inf
    worst = None
    per_mesh_min = {name: math.inf for name in MESH_NAMES}
    below_radius = 0
    no_surface = 0
    long_gaps = 0
    maximum_gap = 0.0
    maximum_step = 0.0
    piecewise_linear_lower_bound = math.inf
    prior = None
    for stamp, x, y, z in selected:
        center = Vector((x, y, z))
        clearance_by_mesh = {}
        for name, bvh in bvhs.items():
            _, _, _, distance = bvh.find_nearest(center, args.nearest_cutoff)
            clearance_by_mesh[name] = distance if distance is not None else math.inf
            per_mesh_min[name] = min(per_mesh_min[name], clearance_by_mesh[name])
        clearance = min(clearance_by_mesh.values())
        if not math.isfinite(clearance):
            no_surface += 1
        if clearance < args.safety_radius:
            below_radius += 1
        if clearance < minimum:
            minimum = clearance
            worst = {"sim_time": stamp, "xyz_m": [x, y, z],
                     "nearest_mesh": min(clearance_by_mesh,
                                         key=clearance_by_mesh.get),
                     "clearance_m": round(clearance, 5)}
        if prior is not None:
            gap = stamp-prior[0]
            step = math.dist((x, y, z), prior[1])
            maximum_gap = max(maximum_gap, gap)
            maximum_step = max(maximum_step, step)
            if gap > 0.2:
                long_gaps += 1
            # Bound only the straight segment between two sampled positions.
            # It is not a proof about unobserved curvature between samples.
            piecewise_linear_lower_bound = min(
                piecewise_linear_lower_bound,
                min(prior[2], clearance)-0.5*step)
        prior = (stamp, (x, y, z), clearance)

    report = {
        "kind": "diagnostic_only_truth_to_collision_mesh_distance",
        "trajectory_file": str(args.trajectory),
        "model": str(args.model),
        "mesh_sha256": {name: file_sha256(path) for name, path in meshes.items()},
        "safety_radius_m": args.safety_radius,
        "all_truth_rows": all_rows,
        "armed_offboard_exploration_samples": len(selected),
        "sampled_min_clearance_m": round(minimum, 5),
        "sampled_below_radius_count": below_radius,
        "per_mesh_min_clearance_m": {
            key: round(value, 5) if math.isfinite(value) else None
            for key, value in per_mesh_min.items()},
        "worst_sample": worst,
        "samples_with_no_surface_within_cutoff": no_surface,
        "max_sample_gap_s": round(maximum_gap, 5),
        "gaps_over_0p2s": long_gaps,
        "max_position_step_m": round(maximum_step, 5),
        "piecewise_linear_clearance_lower_bound_m":
            round(piecewise_linear_lower_bound, 5),
        "sampled_radius_pass": below_radius == 0,
        "continuous_flight_proven": False,
    }
    formatted = json.dumps(report, sort_keys=True, indent=2)
    print(formatted, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(formatted + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
