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
    parser.add_argument("--freespace-cutoff", type=float, default=29.5,
                        help="raw-range threshold used by fastlio_voxblox_adapter")
    parser.add_argument("--freespace-stride", type=int, default=4,
                        help="no-return stride used by fastlio_voxblox_adapter")
    args = parser.parse_args(argv)
    if (args.target_radius <= 0 or args.range_tolerance <= 0 or
            args.freespace_cutoff <= 0 or args.freespace_stride <= 0):
        parser.error("positive target radius, tolerance, cutoff and stride required")
    return args


def main():
    args = parse_args()
    bvhs = {name: load_obj_bvh(args.model / "meshes" /
                              ("baixianshan_"+name+".obj"))
            for name in MESH_NAMES}
    total = misses = spurious = expected_hits = 0
    run_ids = set()
    per_mesh_miss = {name: 0 for name in MESH_NAMES}
    incidence_counts = {name: {"front_hit": 0, "front_miss": 0,
                               "back_hit": 0, "back_miss": 0}
                        for name in MESH_NAMES}
    target_rays = target_detected = 0
    target_examples = []
    worst_misses = []
    no_return_rays = selected_free_rays = 0
    no_return_mesh_conflicts = selected_free_mesh_conflicts = 0
    selected_free_conflicts_by_mesh = {name: 0 for name in MESH_NAMES}
    selected_free_conflict_examples = []
    sim_time = truth_age = capture_delay = pose_bracket = None
    target = Vector(args.target) if args.target else None
    def nearest_range(ray_origin, ray_direction):
        distances = [bvh.ray_cast(ray_origin, ray_direction, 30.0)[3]
                     for bvh in bvhs.values()]
        return min((d for d in distances if d is not None),
                   default=None)
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
            ray_index = int(row["ray_index"])
            no_return = measured >= args.freespace_cutoff
            selected_free = no_return and ray_index % args.freespace_stride == 0
            no_return_rays += int(no_return)
            selected_free_rays += int(selected_free)
            sim_time = float(row["sim_time"])
            truth_age = float(row["truth_age_s"])
            if row.get("capture_sim_time"):
                capture_delay = float(row["capture_sim_time"])-sim_time
            if row.get("truth_pose_bracket_s"):
                pose_bracket = float(row["truth_pose_bracket_s"])
            best_distance = math.inf
            best_name = None
            best_point = None
            best_normal = None
            for name, bvh in bvhs.items():
                point, normal, _, distance = bvh.ray_cast(
                    origin, direction, 30.0)
                if distance is not None and distance < best_distance:
                    best_distance = distance
                    best_name = name
                    best_point = point
                    best_normal = normal
            if best_name is None:
                continue
            expected_hits += 1
            difference = measured-best_distance
            incidence = "front" if best_normal.dot(direction) < 0 else "back"
            incidence_counts[best_name][incidence + (
                "_miss" if difference > args.range_tolerance else "_hit")] += 1
            if (no_return and
                    best_distance + args.range_tolerance <
                    args.freespace_cutoff):
                no_return_mesh_conflicts += 1
                if selected_free:
                    selected_free_mesh_conflicts += 1
                    selected_free_conflicts_by_mesh[best_name] += 1
                    if len(selected_free_conflict_examples) < 20:
                        selected_free_conflict_examples.append({
                            "ray_index": ray_index,
                            "expected_mesh": best_name,
                            "expected_range_m": round(best_distance, 3),
                            "gazebo_range_m": round(measured, 3),
                            "expected_hit_xyz_m": [round(v, 3)
                                                   for v in best_point],
                            "face_normal_dot_ray": round(
                                best_normal.dot(direction), 4),
                        })
            if difference > args.range_tolerance:
                misses += 1
                per_mesh_miss[best_name] += 1
                example = {"ray_index": int(row["ray_index"]),
                           "expected_mesh": best_name,
                           "expected_range_m": round(best_distance, 3),
                           "gazebo_range_m": round(measured, 3),
                           "miss_distance_m": round(difference, 3),
                           "face_normal_dot_ray": round(
                               best_normal.dot(direction), 4),
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
                    # ModelStates is callback-stamped, not simulator-stamped.
                    # Check whether a few centimetres of pose displacement
                    # could flip the expected near-surface return.
                    pose_sensitivity = {}
                    for shift_m in (0.01, 0.03, 0.05):
                        ranges = []
                        for axis in range(3):
                            for sign in (-1, 1):
                                offset = Vector((0.0, 0.0, 0.0))
                                offset[axis] = sign*shift_m
                                ranges.append(nearest_range(
                                    origin+offset, direction))
                        pose_sensitivity[str(shift_m)] = {
                            "min_expected_range_m": round(min(ranges), 3)
                            if all(r is not None for r in ranges) else None,
                            "max_expected_range_m": round(max(ranges), 3)
                            if all(r is not None for r in ranges) else None,
                        }
                    target_examples.append({
                        "ray_index": int(row["ray_index"]),
                        "expected_mesh": best_name,
                        "expected_range_m": round(best_distance, 3),
                        "gazebo_range_m": round(measured, 3),
                        "face_normal_dot_ray": round(
                            best_normal.dot(direction), 4),
                        "axis_pose_shift_sensitivity_m": pose_sensitivity,
                        "expected_hit_xyz_m": [round(v, 3) for v in best_point],
                    })
    report = {
        "kind": "diagnostic_only_gazebo_vs_mesh_ray_returns",
        "snapshot_file": str(args.snapshot),
        "ros_run_id": next(iter(run_ids)) if len(run_ids) == 1 else None,
        "sim_time_s": sim_time,
        "pose_time_alignment_method": (
            "interpolated_at_scan_stamp" if pose_bracket is not None
            else "legacy_latest_pose_at_capture"),
        "truth_pose_age_s": truth_age if pose_bracket is None else None,
        "truth_pose_max_interpolation_gap_s": (
            truth_age if pose_bracket is not None else None),
        "scan_capture_delay_s": capture_delay,
        "truth_pose_bracket_s": pose_bracket,
        "rays": total,
        "rays_with_expected_mesh_hit": expected_hits,
        "gazebo_missed_nearer_mesh_rays": misses,
        "gazebo_shorter_than_mesh_rays": spurious,
        "misses_by_nearest_mesh": per_mesh_miss,
        "incidence_counts_by_nearest_mesh": incidence_counts,
        "worst_misses": worst_misses,
        "adapter_freespace_cutoff_m": args.freespace_cutoff,
        "adapter_freespace_stride": args.freespace_stride,
        "gazebo_no_return_rays": no_return_rays,
        "adapter_selected_freespace_rays": selected_free_rays,
        "no_return_rays_with_nearer_mesh_hit": no_return_mesh_conflicts,
        "selected_freespace_rays_with_nearer_mesh_hit":
            selected_free_mesh_conflicts,
        "selected_freespace_conflicts_by_mesh":
            selected_free_conflicts_by_mesh,
        "selected_freespace_conflict_examples":
            selected_free_conflict_examples,
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
