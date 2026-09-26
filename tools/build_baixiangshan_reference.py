#!/usr/bin/env python3
"""Build an offline reachability reference from the 1:1 Gazebo collision OBJ.

Run with Blender's bundled Python.  This reference is diagnostic-only: never
publish it to ROS or use it as a planner input.  A 1 m XY grid is a coarse
coverage denominator, not a proof that every continuous 3-D route is safe.
"""

import argparse
from collections import deque
import gzip
import json
import math
from pathlib import Path
import sys
import time

from mathutils import Vector
from mathutils.bvhtree import BVHTree


MESH_NAMES = ("ground", "roof", "rock", "infrastructure")
DEFAULT_MODEL = Path("/home/nuc/frontier-upload/models/baixianshan_tunnel")


def load_obj_bvh(path):
    vertices = []
    triangles = []
    with path.open("r", encoding="utf-8", errors="replace") as source:
        for line in source:
            if line.startswith("v "):
                coordinates = line.split()
                vertices.append(Vector(tuple(map(float, coordinates[1:4]))))
            elif line.startswith("f "):
                indices = [int(token.split("/", 1)[0])-1
                           for token in line.split()[1:]]
                for index in range(1, len(indices)-1):
                    triangles.append((indices[0], indices[index],
                                      indices[index+1]))
    if not vertices or not triangles:
        raise ValueError("empty collision mesh: {}".format(path))
    print("Loaded {}: {} vertices, {} triangles".format(
        path.name, len(vertices), len(triangles)), flush=True)
    return BVHTree.FromPolygons(vertices, triangles, all_triangles=True)


def first_hit(bvh, origin, direction, limit):
    hit, _, _, distance = bvh.ray_cast(origin, direction, limit)
    return (hit, distance) if hit is not None else (None, None)


def min_surface_distance(bvhs, point, cutoff):
    best = math.inf
    for bvh in bvhs.values():
        _, _, _, distance = bvh.find_nearest(point, cutoff)
        if distance is not None:
            best = min(best, distance)
    return best


def build_reference(args):
    started = time.monotonic()
    bvhs = {
        name: load_obj_bvh(args.model / "meshes" /
                           ("baixianshan_" + name + ".obj"))
        for name in MESH_NAMES
    }
    nx = math.ceil((args.x_max-args.x_min)/args.resolution)
    ny = math.ceil((args.y_max-args.y_min)/args.resolution)
    valid = {}
    multiple_floors = 0
    ray_down = Vector((0.0, 0.0, -1.0))
    ray_up = Vector((0.0, 0.0, 1.0))
    cutoff = args.safety_radius + args.reserve
    for ix in range(nx):
        x = args.x_min + (ix+0.5)*args.resolution
        for iy in range(ny):
            y = args.y_min + (iy+0.5)*args.resolution
            floor, _ = first_hit(
                bvhs["ground"], Vector((x, y, args.ray_top)),
                ray_down, args.ray_top-args.ray_bottom)
            if floor is None:
                continue
            second_floor, _ = first_hit(
                bvhs["ground"], Vector((x, y, floor.z-0.03)),
                ray_down, floor.z-args.ray_bottom)
            if second_floor is not None:
                multiple_floors += 1
            roof, _ = first_hit(
                bvhs["roof"], Vector((x, y, floor.z+0.03)),
                ray_up, args.ray_top-floor.z)
            if roof is None or roof.z-floor.z < 2.0*cutoff:
                continue
            center_z = min(args.max_flight_z,
                           (floor.z+roof.z)/2.0)
            if center_z-floor.z < cutoff or roof.z-center_z < cutoff:
                continue
            center = Vector((x, y, center_z))
            clearance = min_surface_distance(bvhs, center, 10.0)
            if clearance < cutoff:
                continue
            valid[(ix, iy)] = (center_z, clearance)
        if ix % 20 == 0:
            print("Sampled {}/{} X columns; valid cells {}".format(
                ix+1, nx, len(valid)), flush=True)

    spawn = min(valid, key=lambda key: math.hypot(
        args.x_min+(key[0]+0.5)*args.resolution-args.spawn_x,
        args.y_min+(key[1]+0.5)*args.resolution-args.spawn_y))
    spawn_distance = math.hypot(
        args.x_min+(spawn[0]+0.5)*args.resolution-args.spawn_x,
        args.y_min+(spawn[1]+0.5)*args.resolution-args.spawn_y)
    if spawn_distance > args.max_spawn_distance:
        raise RuntimeError("no valid cell within {:.1f} m of spawn; closest {:.2f} m".format(
            args.max_spawn_distance, spawn_distance))
    reachable = {spawn}
    queue = deque([spawn])
    while queue:
        ix, iy = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            neighbor = (ix+dx, iy+dy)
            if neighbor not in valid or neighbor in reachable:
                continue
            if dx and dy and ((ix+dx, iy) not in valid or
                              (ix, iy+dy) not in valid):
                continue
            horizontal = args.resolution*math.hypot(dx, dy)
            if abs(valid[neighbor][0]-valid[(ix, iy)][0]) > \
                    args.max_grade*horizontal:
                continue
            reachable.add(neighbor)
            queue.append(neighbor)
    reference = {
        "kind": "diagnostic_only_coarse_xy_reachability",
        "model": str(args.model),
        "resolution_m": args.resolution,
        "origin_xy_m": [args.x_min, args.y_min],
        "size_xy_cells": [nx, ny],
        "safety_radius_m": args.safety_radius,
        "reserve_m": args.reserve,
        "max_grade_m_per_m": args.max_grade,
        "spawn_cell": list(spawn),
        "spawn_cell_distance_m": round(spawn_distance, 3),
        "valid_cell_count": len(valid),
        "reachable_cell_count": len(reachable),
        "multiple_floor_hit_cells": multiple_floors,
        "reachable_cells": [[ix, iy, round(valid[(ix, iy)][0], 3)]
                            for ix, iy in sorted(reachable)],
        "reachable_center_clearance_m": [
            [ix, iy, round(valid[(ix, iy)][1], 3)]
            for ix, iy in sorted(reachable)],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(args.output, "wt", encoding="utf-8") as target:
        json.dump(reference, target, separators=(",", ":"))
    print("Wrote {}: {} reachable / {} valid cells, {} multiple-floor "
          "cells, {:.1f}s".format(args.output, len(reachable), len(valid),
                                   multiple_floors,
                                   time.monotonic()-started), flush=True)


def parse_args():
    argv = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=Path(
        "/home/nuc/gbplanner2_isolated_ws/runtime/map_reference/"
        "baixianshan_reachable_1m.json.gz"))
    parser.add_argument("--resolution", type=float, default=1.0)
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--reserve", type=float, default=0.05)
    parser.add_argument("--max-grade", type=float, default=0.60)
    parser.add_argument("--max-flight-z", type=float, default=10.0)
    parser.add_argument("--x-min", type=float, default=-132.0)
    parser.add_argument("--x-max", type=float, default=146.0)
    parser.add_argument("--y-min", type=float, default=-14.0)
    parser.add_argument("--y-max", type=float, default=55.0)
    parser.add_argument("--ray-top", type=float, default=20.0)
    parser.add_argument("--ray-bottom", type=float, default=-12.0)
    parser.add_argument("--spawn-x", type=float, default=-7.0)
    parser.add_argument("--spawn-y", type=float, default=0.2)
    parser.add_argument("--max-spawn-distance", type=float, default=3.0)
    args = parser.parse_args(argv)
    if args.resolution <= 0 or args.safety_radius <= 0:
        parser.error("resolution and safety radius must be positive")
    return args


if __name__ == "__main__":
    build_reference(parse_args())
