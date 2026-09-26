#!/usr/bin/env python3
"""Compare one diagnostic Gazebo MID360 scan with the collision mesh rays.

Run with Blender Python. A mismatch is evidence to investigate; Gazebo and
Blender ray engines, pose timing, and mesh sidedness may differ. No flight
data is published or changed.
"""

import argparse
import csv
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
    parser.add_argument("--snapshot", required=True, type=Path)
    parser.add_argument("--model", default=DEFAULT_MODEL, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--target", nargs=3, type=float)
    parser.add_argument("--target-radius", type=float, default=1.0)
    parser.add_argument("--range-tolerance", type=float, default=0.5)
    args = parser.parse_args(argv)
    if args.target_radius <= 0 or args.range_tolerance <= 0:
        parser.error("positive target radius and range tolerance required")
    return args


def main():
    args = parse_args()
    bvhs = {name: load_obj_bvh(args.model / "meshes" /
                              ("baixianshan_"+name+".obj"))
            for name in MESH_NAMES}
    total = misses = spurious = expected_hits = 0
    run_ids = set()
    per_mesh_miss = {name: 0 for name in MESH_NAMES}
    target_rays = target_detected = 0
    target_examples = []
    worst_misses = []
    sim_time = truth_age = None
    target = Vector(args.target) if args.target else None
    with args.snapshot.open("r", encoding="utf-8", newline="") as source:
        for row in csv.DictReader(source):
            run_ids.add(row.get("ros_run_id", ""))
            origin = Vector(tuple(float(row[key]) for key in
                                  ("sensor_x", "sensor_y", "sensor_z")))
            direction = Vector(tuple(float(row[key]) for key in
                                     ("direction_x", "direction_y", "direction_z")))
            measured = float(row["measured_range_m"])
            if not all(math.isfinite(value) for value in
                       (*origin, *direction, measured)):
                continue
            direction.normalize()
            total += 1
            sim_time = float(row["sim_time"])
            truth_age = float(row["truth_age_s"])
            best_distance = math.inf
            best_name = None
            best_point = None
            for name, bvh in bvhs.items():
                point, _, _, distance = bvh.ray_cast(origin, direction, 30.0)
                if distance is not None and distance < best_distance:
                    best_distance = distance
                    best_name = name
                    best_point = point
            if best_name is None:
                continue
            expected_hits += 1
            difference = measured-best_distance
            if difference > args.range_tolerance:
                misses += 1
                per_mesh_miss[best_name] += 1
                example = {"ray_index": int(row["ray_index"]),
                           "expected_mesh": best_name,
                           "expected_range_m": round(best_distance, 3),
                           "gazebo_range_m": round(measured, 3),
                           "miss_distance_m": round(difference, 3),
                           "expected_hit_xyz_m": [round(v, 3) for v in best_point]}
                worst_misses.append(example)
                worst_misses.sort(key=lambda item: item["miss_distance_m"],
                                  reverse=True)
                del worst_misses[10:]
            elif difference < -args.range_tolerance:
                spurious += 1
            if target is not None and (best_point-target).length <= args.target_radius:
                target_rays += 1
                if abs(difference) <= args.range_tolerance:
                    target_detected += 1
                if len(target_examples) < 10:
                    target_examples.append({
                        "ray_index": int(row["ray_index"]),
                        "expected_mesh": best_name,
                        "expected_range_m": round(best_distance, 3),
                        "gazebo_range_m": round(measured, 3),
                        "expected_hit_xyz_m": [round(v, 3) for v in best_point],
                    })
    report = {
        "kind": "diagnostic_only_gazebo_vs_mesh_ray_returns",
        "snapshot_file": str(args.snapshot),
        "ros_run_id": next(iter(run_ids)) if len(run_ids) == 1 else None,
        "sim_time_s": sim_time,
        "truth_pose_age_s": truth_age,
        "rays": total,
        "rays_with_expected_mesh_hit": expected_hits,
        "gazebo_missed_nearer_mesh_rays": misses,
        "gazebo_shorter_than_mesh_rays": spurious,
        "misses_by_nearest_mesh": per_mesh_miss,
        "worst_misses": worst_misses,
        "target_xyz_m": args.target,
        "target_radius_m": args.target_radius if target else None,
        "expected_rays_near_target": target_rays if target else None,
        "matching_gazebo_rays_near_target": target_detected if target else None,
        "target_examples": target_examples,
        "controls_or_map_affected": False,
    }
    formatted = json.dumps(report, indent=2, allow_nan=False)
    print(formatted, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(formatted+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
