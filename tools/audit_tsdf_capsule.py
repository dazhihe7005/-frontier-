#!/usr/bin/env python3
"""Read-only reconstruction of the GBPlanner sphere/capsule TSDF decision.

The CSV is one timestamped snapshot, not necessarily the planner's exact map
at its decision time. Missing rows are unknown only inside the recorded CSV
bounds. This tool never publishes navigation or control data.
"""

import argparse
import csv
import json
import math
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True, type=Path)
    parser.add_argument("--start", nargs=3, required=True, type=float)
    parser.add_argument("--end", nargs=3, required=True, type=float)
    parser.add_argument("--voxel-size", type=float, default=0.2)
    parser.add_argument("--radius", type=float, default=1.0)
    parser.add_argument("--occupied-distance", type=float, default=0.05)
    parser.add_argument("--snapshot-bounds", nargs=6, type=float,
                        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--mid360-yaw-sweep", action="store_true",
                        help="diagnose FOV-only visibility of unknown cells; ignores occlusion")
    args = parser.parse_args()
    if (args.voxel_size <= 0 or args.radius <= 0 or
            args.occupied_distance < 0 or
            not all(math.isfinite(v) for v in (*args.start, *args.end))):
        parser.error("invalid geometry")
    return args


def capsule_distance(point, start, end):
    delta = tuple(end[i]-start[i] for i in range(3))
    length_sq = sum(v*v for v in delta)
    t = max(0.0, min(1.0,
        sum((point[i]-start[i])*delta[i] for i in range(3))/length_sq
    )) if length_sq > 1e-12 else 0.0
    return math.dist(point, tuple(start[i]+t*delta[i] for i in range(3)))


def mid360_visible_from_body(point, body, yaw):
    """FOV-only check for the mounted MID360S; does not claim a clear ray."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    dx, dy, dz = (point[i]-body[i] for i in range(3))
    body_x = cos_yaw*dx + sin_yaw*dy - 0.1315
    body_y = -sin_yaw*dx + cos_yaw*dy
    body_z = dz - 0.223
    pitch = math.radians(25.0)
    sensor_x = math.cos(pitch)*body_x - math.sin(pitch)*body_z
    sensor_z = math.sin(pitch)*body_x + math.cos(pitch)*body_z
    ray_range = math.sqrt(sensor_x*sensor_x + body_y*body_y + sensor_z*sensor_z)
    elevation = math.degrees(math.atan2(
        sensor_z, math.hypot(sensor_x, body_y)))
    return 0.2 <= ray_range <= 30.0 and -7.0 <= elevation <= 52.0


def analyze(rows, start, end, voxel_size, radius, occupied_distance,
            snapshot_bounds=None, mid360_yaw_sweep=False):
    observed = {}
    times = set()
    frames = set()
    run_ids = set()
    for row in rows:
        xyz = tuple(float(row[key]) for key in ("x", "y", "z"))
        distance = float(row["tsdf_distance_m"])
        if not all(math.isfinite(v) for v in (*xyz, distance)):
            continue
        index = tuple(math.floor(v/voxel_size) for v in xyz)
        observed[index] = distance
        times.add(row.get("sim_time", ""))
        frames.add(row.get("frame_id", ""))
        run_ids.add(row.get("ros_run_id", ""))
    expanded = radius+math.sqrt(3.0)*voxel_size/2.0
    lower = tuple(min(start[i], end[i])-expanded for i in range(3))
    upper = tuple(max(start[i], end[i])+expanded for i in range(3))
    first = tuple(math.floor(v/voxel_size) for v in lower)
    last = tuple(math.floor(v/voxel_size) for v in upper)
    counts = {"known_free": 0, "known_occupied": 0,
              "unknown_in_csv": 0, "outside_snapshot": 0}
    examples = {}
    unknown_by_z_index = {}
    unknown_examples = []
    outside_fov_examples = []
    fov_counts = {"visible_at_yaw_zero": 0, "visible_during_yaw_sweep": 0,
                  "outside_yaw_sweep_fov": 0,
                  "visible_at_opposite_yaws": 0,
                  "visible_at_four_cardinal_yaws": 0}
    minimum_occupied_center_distance = math.inf
    for ix in range(first[0], last[0]+1):
        for iy in range(first[1], last[1]+1):
            for iz in range(first[2], last[2]+1):
                index = (ix, iy, iz)
                center = tuple((v+0.5)*voxel_size for v in index)
                center_distance = capsule_distance(center, start, end)
                if center_distance > expanded:
                    continue
                tsdf = observed.get(index)
                if tsdf is None:
                    if snapshot_bounds and not all(
                            snapshot_bounds[2*i] <= center[i] <= snapshot_bounds[2*i+1]
                            for i in range(3)):
                        kind = "outside_snapshot"
                    else:
                        kind = "unknown_in_csv"
                elif tsdf <= occupied_distance+1e-6:
                    kind = "known_occupied"
                    minimum_occupied_center_distance = min(
                        minimum_occupied_center_distance, center_distance)
                else:
                    kind = "known_free"
                counts[kind] += 1
                if kind == "unknown_in_csv":
                    unknown_by_z_index[str(iz)] = (
                        unknown_by_z_index.get(str(iz), 0) + 1)
                    if len(unknown_examples) < 20:
                        unknown_examples.append({
                            "center_m": center,
                            "center_to_capsule_axis_m": round(center_distance, 5),
                        })
                    if mid360_yaw_sweep:
                        if mid360_visible_from_body(center, start, 0.0):
                            fov_counts["visible_at_yaw_zero"] += 1
                        if any(mid360_visible_from_body(
                                center, start, math.pi*i)
                                for i in range(2)):
                            fov_counts["visible_at_opposite_yaws"] += 1
                        if any(mid360_visible_from_body(
                                center, start, math.pi*i/2.0)
                                for i in range(4)):
                            fov_counts["visible_at_four_cardinal_yaws"] += 1
                        if any(mid360_visible_from_body(
                                center, start, 2.0*math.pi*i/72.0)
                                for i in range(72)):
                            fov_counts["visible_during_yaw_sweep"] += 1
                        else:
                            fov_counts["outside_yaw_sweep_fov"] += 1
                            if len(outside_fov_examples) < 20:
                                outside_fov_examples.append({
                                    "center_m": center,
                                    "center_to_capsule_axis_m": round(
                                        center_distance, 5),
                                })
                if kind not in examples:
                    examples[kind] = {
                        "index": index, "center_m": center,
                        "center_to_capsule_axis_m": round(center_distance, 5),
                        "tsdf_distance_m": tsdf,
                    }
    return {
        "kind": "diagnostic_only_tsdf_snapshot_capsule_reconstruction",
        "start_m": start, "end_m": end, "radius_m": radius,
        "voxel_size_m": voxel_size,
        "expanded_center_test_radius_m": round(expanded, 5),
        "occupancy_tsdf_threshold_m": occupied_distance,
        "counts": counts, "first_examples_in_cpp_scan_order": examples,
        "unknown_by_z_index": unknown_by_z_index,
        "unknown_examples": unknown_examples,
        "mid360_fov_only_unknown_counts": (
            fov_counts if mid360_yaw_sweep else None),
        "mid360_outside_fov_examples": (
            outside_fov_examples if mid360_yaw_sweep else None),
        "min_known_occupied_center_to_axis_m": (
            round(minimum_occupied_center_distance, 5)
            if math.isfinite(minimum_occupied_center_distance) else None),
        "snapshot_sim_times": sorted(times),
        "snapshot_bounds_m": snapshot_bounds,
        "snapshot_frames": sorted(frames),
        "ros_run_ids": sorted(run_ids),
        "planner_decision_reproduced_exactly": False,
    }


def main():
    args = parse_args()
    with args.snapshot.open("r", encoding="utf-8", newline="") as source:
        result = analyze(list(csv.DictReader(source)), tuple(args.start),
                         tuple(args.end), args.voxel_size, args.radius,
                         args.occupied_distance, args.snapshot_bounds,
                         args.mid360_yaw_sweep)
    result["snapshot_file"] = str(args.snapshot)
    rendered = json.dumps(result, indent=2, sort_keys=True, allow_nan=False)
    print(rendered, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
