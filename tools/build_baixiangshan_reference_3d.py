#!/usr/bin/env python3
"""Offline, collision-mesh-only 3-D reachable-centre reference.

This is a diagnostic denominator, never a planner input. A grid cannot prove
continuous completeness, so the output records its resolution and checks each
accepted graph edge continuously using the 1-Lipschitz surface distance bound.
"""

import argparse
from collections import deque
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_baixiangshan_reference import DEFAULT_MODEL, MESH_NAMES, load_obj_bvh


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def vertical_hits(bvh, x, y, high, low):
    """All distinct intersections of one downwards vertical ray."""
    heights = []
    cursor = high
    direction = Vector((0.0, 0.0, -1.0))
    while cursor > low:
        point, _, _, distance = bvh.ray_cast(
            Vector((x, y, cursor)), direction, cursor - low)
        if point is None:
            break
        height = float(point.z)
        if not heights or abs(height - heights[-1]) > 0.015:
            heights.append(height)
        cursor = height - 0.02
    return heights


def between_floor_and_roof(z, floors, roofs, clearance):
    below = next((floor for floor in floors if floor < z - clearance), None)
    above = next((roof for roof in reversed(roofs) if roof > z + clearance), None)
    if below is None or above is None or below >= above:
        return False
    # A second floor or roof inside the apparent interval indicates a stacked
    # or non-manifold region; do not assert free space through that surface.
    if any(below + 0.015 < surface < above - 0.015
           for surface in floors):
        return False
    if any(below + 0.015 < surface < above - 0.015
           for surface in roofs):
        return False
    return True


def surface_clearance(bvhs, point, cutoff):
    # BVH nearest-with-cutoff reports no distance for farther surfaces. The
    # cutoff is then a proven *lower bound*, not infinity; using infinity here
    # would make the edge Lipschitz certificate unsound.
    best = cutoff
    for bvh in bvhs.values():
        _, _, _, distance = bvh.find_nearest(point, cutoff)
        if distance is not None:
            best = min(best, float(distance))
    return best


def edge_safe(bvhs, start, end, start_clearance, end_clearance,
              required_clearance, depth=0):
    """Certify a straight edge using sampled exact BVH distance and a bound.

    Distance to any closed surface is 1-Lipschitz. If both endpoints are at
    least required + half the segment length away, the full segment passes.
    Otherwise bisect until certified or an unsafe point is found. Unresolved
    edges fail closed after 10 splits (under 1 mm for a 0.5 m grid edge).
    """
    length = (end - start).length
    if min(start_clearance, end_clearance) - 0.5 * length >= required_clearance:
        return True
    middle = (start + end) * 0.5
    middle_clearance = surface_clearance(
        bvhs, middle, required_clearance + 0.5 * length)
    if middle_clearance < required_clearance or depth >= 10:
        return False
    return (edge_safe(bvhs, start, middle, start_clearance,
                      middle_clearance, required_clearance, depth + 1) and
            edge_safe(bvhs, middle, end, middle_clearance,
                      end_clearance, required_clearance, depth + 1))


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=Path(
        "/home/nuc/gbplanner2_isolated_ws/runtime/map_reference/"
        "baixianshan_reachable_3d_0p5m.json.gz"))
    parser.add_argument("--resolution", type=float, default=0.5)
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--reserve", type=float, default=0.05)
    parser.add_argument("--x-min", type=float, default=-132.0)
    parser.add_argument("--x-max", type=float, default=146.0)
    parser.add_argument("--y-min", type=float, default=-14.0)
    parser.add_argument("--y-max", type=float, default=55.0)
    parser.add_argument("--z-min", type=float, default=-12.0)
    parser.add_argument("--z-max", type=float, default=20.0)
    parser.add_argument("--spawn-x", type=float, default=-7.0)
    parser.add_argument("--spawn-y", type=float, default=0.2)
    parser.add_argument("--spawn-z", type=float, default=1.8)
    parser.add_argument("--max-spawn-distance", type=float, default=2.0)
    args = parser.parse_args(argv)
    if (args.resolution <= 0 or args.safety_radius <= 0 or args.reserve < 0):
        parser.error("invalid resolution, safety radius, or reserve")
    for axis in "xyz":
        if getattr(args, axis + "_min") >= getattr(args, axis + "_max"):
            parser.error("invalid {} bounds".format(axis))
    return args


def build(args):
    started = time.monotonic()
    paths = {name: args.model / "meshes" /
             ("baixianshan_" + name + ".obj") for name in MESH_NAMES}
    bvhs = {name: load_obj_bvh(path) for name, path in paths.items()}
    origin = Vector((args.x_min, args.y_min, args.z_min))
    n = tuple(math.ceil((getattr(args, axis + "_max") -
                         getattr(args, axis + "_min")) / args.resolution)
              for axis in "xyz")
    required = args.safety_radius + args.reserve
    valid = {}
    stacked_columns = 0
    for ix in range(n[0]):
        x = origin.x + (ix + 0.5) * args.resolution
        for iy in range(n[1]):
            y = origin.y + (iy + 0.5) * args.resolution
            floors = vertical_hits(bvhs["ground"], x, y,
                                   args.z_max, args.z_min)
            if not floors:
                continue
            roofs = vertical_hits(bvhs["roof"], x, y,
                                  args.z_max, args.z_min)
            if not roofs:
                continue
            if len(floors) > 1 or len(roofs) > 1:
                stacked_columns += 1
            for iz in range(n[2]):
                z = origin.z + (iz + 0.5) * args.resolution
                if not between_floor_and_roof(z, floors, roofs, required):
                    continue
                clearance = surface_clearance(
                    bvhs, Vector((x, y, z)),
                    required + 0.5 * math.sqrt(3.0) * args.resolution + 1e-5)
                if clearance >= required:
                    valid[(ix, iy, iz)] = clearance
        if ix % 20 == 0:
            print("3-D sampled {}/{} x-columns; {} valid cells".format(
                ix + 1, n[0], len(valid)), flush=True)
    if not valid:
        raise RuntimeError("no valid 3-D cells")

    def center(key):
        return origin + Vector(tuple((index + 0.5) * args.resolution
                                     for index in key))

    spawn_position = Vector((args.spawn_x, args.spawn_y, args.spawn_z))
    spawn = min(valid, key=lambda key: (center(key) - spawn_position).length)
    spawn_distance = (center(spawn) - spawn_position).length
    if spawn_distance > args.max_spawn_distance:
        raise RuntimeError("closest safe spawn cell is {:.3f} m away".format(
            spawn_distance))

    offsets = [(dx, dy, dz) for dx in (-1, 0, 1)
               for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               if (dx, dy, dz) != (0, 0, 0)]
    reachable = {spawn}
    queue = deque([spawn])
    checked_edges = accepted_edges = 0
    while queue:
        current = queue.popleft()
        start = center(current)
        for offset in offsets:
            neighbor = tuple(current[i] + offset[i] for i in range(3))
            if neighbor not in valid or neighbor in reachable:
                continue
            checked_edges += 1
            if not edge_safe(bvhs, start, center(neighbor), valid[current],
                             valid[neighbor], required):
                continue
            accepted_edges += 1
            reachable.add(neighbor)
            queue.append(neighbor)

    result = {
        "kind": "diagnostic_only_3d_reachable_centres_26_connected",
        "model": str(args.model),
        "mesh_sha256": {name: sha256(path) for name, path in paths.items()},
        "resolution_m": args.resolution,
        "origin_xyz_m": list(origin),
        "size_xyz_cells": list(n),
        "safety_radius_m": args.safety_radius,
        "reserve_m": args.reserve,
        "spawn_cell": list(spawn),
        "spawn_distance_m": round(spawn_distance, 4),
        "valid_cell_count": len(valid),
        "reachable_cell_count": len(reachable),
        "stacked_floor_or_roof_columns": stacked_columns,
        "checked_candidate_edges": checked_edges,
        "accepted_graph_edges": accepted_edges,
        "reachable_cells": [list(key) for key in sorted(reachable)],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(args.output, "wt", encoding="utf-8") as target:
        json.dump(result, target, separators=(",", ":"))
    print("Wrote {}: {} / {} valid, {} checked edges, {} accepted; {:.1f}s".format(
        args.output, len(reachable), len(valid), checked_edges,
        accepted_edges, time.monotonic() - started), flush=True)


if __name__ == "__main__":
    build(parse_args())
