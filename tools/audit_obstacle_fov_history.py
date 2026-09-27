#!/usr/bin/env python3
"""Read-only aperture history for one world-space obstacle point.

This uses Gazebo truth only offline. An in-aperture result is a possibility,
not proof of a real lidar return: the ray may be occluded, fall between scan
samples, be filtered, or have an invalid timestamp. An out-of-aperture result
does rule out a direct return from that sensor at that sampled body pose.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys


def rotate_inverse(vector, quaternion):
    """Rotate world vector into body FLU with quaternion (x,y,z,w)."""
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-9:
        raise ValueError("invalid body quaternion")
    ux, uy, uz, scalar = -qx/norm, -qy/norm, -qz/norm, qw/norm
    vx, vy, vz = vector
    tx = 2.0*(uy*vz-uz*vy)
    ty = 2.0*(uz*vx-ux*vz)
    tz = 2.0*(ux*vy-uy*vx)
    return (vx+scalar*tx+uy*tz-uz*ty,
            vy+scalar*ty+uz*tx-ux*tz,
            vz+scalar*tz+ux*ty-uy*tx)


def rotate_forward(vector, quaternion):
    qx, qy, qz, qw = quaternion
    return rotate_inverse(vector, (-qx, -qy, -qz, qw))


def aperture(body_vector):
    """Return continuous MID360S and simulated downward-fan FOV tests."""
    x, y, z = body_vector
    mid_x, mid_y, mid_z = x-0.1315, y, z-0.223
    pitch = math.radians(25.0)
    mid_sensor_x = math.cos(pitch)*mid_x-math.sin(pitch)*mid_z
    mid_sensor_z = math.sin(pitch)*mid_x+math.cos(pitch)*mid_z
    mid_range = math.sqrt(mid_sensor_x**2+mid_y**2+mid_sensor_z**2)
    mid_elevation = math.degrees(math.atan2(
        mid_sensor_z, math.hypot(mid_sensor_x, mid_y)))
    mid_visible = 0.20 <= mid_range <= 30.0 and -7.0 <= mid_elevation <= 52.0

    # The SDF's local pose composes with the 25-degree MID360 link pose to
    # place this simulation-only sensor 0.35 m below the body, facing -Z.
    down_x, down_y, down_z = -(z+0.35), y, x
    down_range = math.sqrt(down_x**2+down_y**2+down_z**2)
    down_azimuth = math.degrees(math.atan2(down_y, down_x))
    down_elevation = math.degrees(math.atan2(
        down_z, math.hypot(down_x, down_y)))
    down_visible = (0.20 <= down_range <= 30.0 and down_x > 0.0
                    and abs(down_azimuth) <= 45.0
                    and abs(down_elevation) <= 45.0)
    return {
        "mid360_fov": mid_visible,
        "mid360_sensor_elevation_deg": mid_elevation,
        "mid360_range_m": mid_range,
        "sim_down_fov": down_visible,
        "sim_down_azimuth_deg": down_azimuth,
        "sim_down_elevation_deg": down_elevation,
        "sim_down_range_m": down_range,
    }


def audit(rows, target, time_min=-math.inf, time_max=math.inf, bvhs=None):
    counts = {"samples": 0, "mid360_fov_samples": 0,
              "sim_down_fov_samples": 0, "either_fov_samples": 0}
    first = {"mid360_fov": None, "sim_down_fov": None,
             "either_fov": None}
    last = dict(first)
    closest = None
    run_ids = set()
    los_count = 0
    first_los = None
    last_los = None
    for row in rows:
        try:
            stamp = float(row["sim_time"])
            if not time_min <= stamp <= time_max or any(
                    row.get(key) != "1" for key in
                    ("armed", "offboard", "exploration_started")):
                continue
            position = tuple(float(row[key]) for key in ("x", "y", "z"))
            quaternion = tuple(float(row[key]) for key in
                               ("qx", "qy", "qz", "qw"))
            if not all(math.isfinite(v) for v in
                       (stamp, *position, *quaternion)):
                continue
            world_delta = tuple(target[i]-position[i] for i in range(3))
            body_delta = rotate_inverse(world_delta, quaternion)
        except (KeyError, ValueError):
            continue
        run_ids.add(row.get("ros_run_id", ""))
        result = aperture(body_delta)
        if result["mid360_fov"] and bvhs is not None:
            from mathutils import Vector
            sensor_offset = rotate_forward((0.1315, 0.0, 0.223), quaternion)
            origin = Vector(tuple(position[i]+sensor_offset[i]
                                  for i in range(3)))
            direction = Vector(target)-origin
            target_distance = direction.length
            direction.normalize()
            nearest_hit = min((distance for bvh in bvhs.values()
                               for _, _, _, distance in
                               (bvh.ray_cast(origin, direction,
                                             target_distance),)
                               if distance is not None), default=math.inf)
            result["mid360_mesh_los"] = (
                nearest_hit >= target_distance-0.05)
            if result["mid360_mesh_los"]:
                los_count += 1
                if first_los is None:
                    first_los = stamp
                last_los = stamp
        counts["samples"] += 1
        for sensor in ("mid360_fov", "sim_down_fov"):
            if result[sensor]:
                counts[sensor+"_samples"] += 1
                if first[sensor] is None:
                    first[sensor] = stamp
                last[sensor] = stamp
        if result["mid360_fov"] or result["sim_down_fov"]:
            counts["either_fov_samples"] += 1
            if first["either_fov"] is None:
                first["either_fov"] = stamp
            last["either_fov"] = stamp
        distance = math.dist(position, target)
        if closest is None or distance < closest["body_to_target_m"]:
            closest = {"sim_time_s": stamp,
                       "body_to_target_m": distance,
                       "body_delta_m": body_delta,
                       **result}
    if not counts["samples"]:
        raise ValueError("no eligible flight samples")
    return {
        "kind": "diagnostic_only_continuous_aperture_history",
        "target_world_m": target,
        "time_window_s": [time_min if math.isfinite(time_min) else None,
                          time_max if math.isfinite(time_max) else None],
        "ros_run_ids": sorted(run_ids),
        "counts": counts,
        "first_in_fov_sim_time_s": first,
        "last_in_fov_sim_time_s": last,
        "closest_sample": closest,
        "mid360_mesh_los_fov_samples": los_count if bvhs is not None else None,
        "first_mid360_mesh_los_sim_time_s": first_los,
        "last_mid360_mesh_los_sim_time_s": last_los,
        "mesh_los_tolerance_m": 0.05 if bvhs is not None else None,
        "does_not_test_actual_scan_rays": True,
        "mesh_los_checked": bvhs is not None,
        "gazebo_truth_used_for_control": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--target", nargs=3, type=float, required=True)
    parser.add_argument("--time-min", type=float, default=-math.inf)
    parser.add_argument("--time-max", type=float, default=math.inf)
    parser.add_argument("--mesh-los", action="store_true",
                        help="Blender-only ray-cast occlusion test")
    parser.add_argument("--model", type=Path, default=Path(
        "/home/nuc/frontier-upload/models/baixianshan_tunnel"))
    parser.add_argument("--output", type=Path)
    arguments = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else None
    args = parser.parse_args(arguments)
    if args.time_min > args.time_max or not all(
            math.isfinite(v) for v in args.target):
        parser.error("invalid target or time window")
    bvhs = None
    if args.mesh_los:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from build_baixiangshan_reference import MESH_NAMES, load_obj_bvh
        bvhs = {name: load_obj_bvh(args.model / "meshes" /
                                 ("baixianshan_"+name+".obj"))
                for name in MESH_NAMES}
    with args.trajectory.open(newline="", encoding="utf-8") as source:
        result = audit(csv.DictReader(source), tuple(args.target),
                       args.time_min, args.time_max, bvhs)
    result["trajectory_file"] = str(args.trajectory)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
