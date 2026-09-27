#!/usr/bin/env python3
"""Read-only mesh/line-of-sight classification of unpublished TSDF cells.

Run with Blender Python. Planner-to-world alignment comes from a near-time
diagnostic trajectory anchor; it is approximate and never enters control.
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


def arguments():
    argv = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--capsule", required=True, type=Path)
    parser.add_argument("--projected-trajectory", required=True, type=Path)
    parser.add_argument("--ray-snapshot", required=True, type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path)
    return parser.parse_args(argv)


def main():
    args = arguments()
    capsule = json.loads(args.capsule.read_text(encoding="utf-8"))
    stamp = float(capsule["snapshot_sim_times"][0])
    run_id = capsule["ros_run_ids"][0]
    with args.projected_trajectory.open(newline="", encoding="utf-8") as source:
        anchors = list(csv.DictReader(source))
    anchor = min(anchors, key=lambda r: abs(float(r["sim_time"])-stamp))
    if anchor["ros_run_id"] != run_id:
        raise RuntimeError("capsule and trajectory run IDs differ")
    with args.ray_snapshot.open(newline="", encoding="utf-8") as source:
        rays = list(csv.DictReader(source))
    ray = rays[0]
    if ray["ros_run_id"] != run_id:
        raise RuntimeError("capsule and ray snapshot run IDs differ")
    origin = Vector(tuple(float(ray[key]) for key in
                          ("sensor_x", "sensor_y", "sensor_z")))
    scan_rays = [(Vector(tuple(float(row[key]) for key in
                          ("direction_x", "direction_y", "direction_z"))),
                  float(row["measured_range_m"])) for row in rays]
    truth_anchor = tuple(float(anchor["truth_anchor_"+axis])
                         for axis in "xyz")
    odom_anchor = tuple(float(anchor["odom_anchor_"+axis])
                        for axis in "xyz")
    yaw_delta = float(anchor["yaw_delta_rad"])
    cosine, sine = math.cos(yaw_delta), math.sin(yaw_delta)
    bvhs = {name: load_obj_bvh(args.model / "meshes" /
                              ("baixianshan_"+name+".obj"))
            for name in MESH_NAMES}
    half_diagonal = math.sqrt(3.0)*float(capsule["voxel_size_m"])/2.0
    results = []
    for cell in capsule["unpublished_examples"]:
        x, y, z = cell["center_m"]
        dx, dy = x-odom_anchor[0], y-odom_anchor[1]
        point = Vector((truth_anchor[0]+cosine*dx-sine*dy,
                        truth_anchor[1]+sine*dx+cosine*dy,
                        truth_anchor[2]+z-odom_anchor[2]))
        closest_name, closest_distance = None, math.inf
        first_ray_name, first_ray_distance = None, math.inf
        direction = point-origin
        range_to_cell = direction.length
        direction.normalize()
        scan_rays_near_cell = scan_rays_reaching_cell = 0
        for scan_direction, measured_range in scan_rays:
            along = (point-origin).dot(scan_direction)
            if along <= 0.0:
                continue
            lateral = ((point-origin)-along*scan_direction).length
            if lateral > half_diagonal:
                continue
            scan_rays_near_cell += 1
            if measured_range >= along-half_diagonal:
                scan_rays_reaching_cell += 1
        for name, bvh in bvhs.items():
            _, _, _, nearest = bvh.find_nearest(point, 5.0)
            if nearest is not None and nearest < closest_distance:
                closest_name, closest_distance = name, nearest
            _, _, _, hit_distance = bvh.ray_cast(origin, direction,
                                                  range_to_cell)
            if hit_distance is not None and hit_distance < first_ray_distance:
                first_ray_name, first_ray_distance = name, hit_distance
        results.append({
            "planner_center_m": cell["center_m"],
            "projected_world_center_m": [round(v, 4) for v in point],
            "capsule_axis_distance_m": cell["center_to_capsule_axis_m"],
            "nearest_mesh": closest_name,
            "nearest_mesh_distance_m": round(closest_distance, 4),
            "voxel_cube_may_intersect_mesh": closest_distance <= half_diagonal,
            "first_mesh_on_sensor_ray": first_ray_name,
            "first_mesh_range_m": (round(first_ray_distance, 4)
                                   if math.isfinite(first_ray_distance) else None),
            "sensor_to_cell_range_m": round(range_to_cell, 4),
            "line_of_sight_occluded_before_voxel": (
                first_ray_distance < range_to_cell-half_diagonal),
            "scan_rays_within_voxel_half_diagonal": scan_rays_near_cell,
            "scan_rays_reaching_voxel": scan_rays_reaching_cell,
        })
    report = {
        "kind": "diagnostic_only_unknown_voxel_mesh_projection",
        "ros_run_id": run_id,
        "tsdf_snapshot_sim_time_s": stamp,
        "trajectory_anchor_sim_time_s": float(anchor["sim_time"]),
        "trajectory_anchor_offset_s": round(float(anchor["sim_time"])-stamp, 4),
        "ray_snapshot_sim_time_s": float(ray["sim_time"]),
        "voxel_half_diagonal_m": round(half_diagonal, 5),
        "unpublished_examples_examined": len(results),
        "unpublished_total_in_capsule": capsule["counts"]["not_in_visualization"],
        "limited_to_first_20_examples": (
            capsule["counts"]["not_in_visualization"] > len(results)),
        "voxel_cubes_near_mesh": sum(r["voxel_cube_may_intersect_mesh"]
                                      for r in results),
        "sensor_rays_occluded_before_voxel": sum(
            r["line_of_sight_occluded_before_voxel"] for r in results),
        "cells_with_raw_scan_ray_reaching_voxel": sum(
            r["scan_rays_reaching_voxel"] > 0 for r in results),
        "cells": results,
        "alignment_is_approximate": True,
        "one_scan_ray_passage_proves_tsdf_update": False,
        "controls_or_map_affected": False,
    }
    output = json.dumps(report, indent=2, allow_nan=False)
    print(output)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
