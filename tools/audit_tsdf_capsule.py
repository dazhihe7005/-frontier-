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


def analyze(rows, start, end, voxel_size, radius, occupied_distance,
            snapshot_bounds=None):
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
                         args.occupied_distance, args.snapshot_bounds)
    result["snapshot_file"] = str(args.snapshot)
    rendered = json.dumps(result, indent=2, sort_keys=True, allow_nan=False)
    print(rendered, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
