#!/usr/bin/env python3
"""Read-only 3-D clearance scan across a tunnel slice of the Gazebo mesh.

Run with Blender Python. This diagnostic must never feed a flight controller.
"""

import argparse
import json
from pathlib import Path
import sys

from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_baixiangshan_reference import (DEFAULT_MODEL, MESH_NAMES,
                                          first_hit, load_obj_bvh,
                                          min_surface_distance)


def parse_args():
    argv = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--x", type=float, nargs="+", required=True)
    parser.add_argument("--y-min", type=float, default=-4.0)
    parser.add_argument("--y-max", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=0.1)
    parser.add_argument("--radius", type=float, default=1.0)
    parser.add_argument("--probe", nargs=3, type=float, action="append",
                        default=[], metavar=("X", "Y", "Z"))
    args = parser.parse_args(argv)
    if args.step <= 0 or args.radius <= 0 or args.y_max <= args.y_min:
        parser.error("invalid slice bounds, step, or safety radius")
    return args


def main():
    args = parse_args()
    bvhs = {name: load_obj_bvh(args.model / "meshes" /
                              ("baixianshan_"+name+".obj"))
            for name in MESH_NAMES}
    results = []
    probes = []
    for coordinates in args.probe:
        center = Vector(coordinates)
        clearance_by_mesh = {}
        for name, bvh in bvhs.items():
            _, _, _, distance = bvh.find_nearest(center, 10.0)
            clearance_by_mesh[name] = (round(distance, 4)
                                       if distance is not None else None)
        probes.append({"xyz_m": coordinates,
                       "mesh_clearance_m": clearance_by_mesh})
    for x in args.x:
        feasible = []
        best = None
        for yi in range(round((args.y_max-args.y_min)/args.step)+1):
            y = args.y_min+yi*args.step
            floor, _ = first_hit(bvhs["ground"], Vector((x, y, 20.0)),
                                 Vector((0, 0, -1)), 32.0)
            if floor is None:
                continue
            roof, _ = first_hit(bvhs["roof"],
                                Vector((x, y, floor.z+0.03)),
                                Vector((0, 0, 1)), 20.0-floor.z)
            if roof is None:
                continue
            lower = floor.z+args.radius
            upper = roof.z-args.radius
            for zi in range(round((upper-lower)/args.step)+1):
                z = lower+zi*args.step
                if z > upper+1e-9:
                    continue
                clearance = min_surface_distance(
                    bvhs, Vector((x, y, z)), 10.0)
                if clearance >= args.radius:
                    item = {"x": round(x, 3), "y": round(y, 3),
                            "z": round(z, 3),
                            "clearance_m": round(clearance, 4),
                            "floor_z_m": round(floor.z, 3),
                            "roof_z_m": round(roof.z, 3)}
                    feasible.append(item)
                    if best is None or clearance > best["clearance_m"]:
                        best = item
        results.append({"x_m": x, "safe_sample_count": len(feasible),
                        "best_center": best,
                        "safe_y_range_m": [min(i["y"] for i in feasible),
                                           max(i["y"] for i in feasible)]
                        if feasible else None,
                        "safe_z_range_m": [min(i["z"] for i in feasible),
                                           max(i["z"] for i in feasible)]
                        if feasible else None})
    print(json.dumps({"diagnostic_only": True,
                      "mesh_model": str(args.model),
                      "safety_radius_m": args.radius,
                      "grid_step_m": args.step,
                      "slices": results,
                      "probes": probes}, indent=2), flush=True)


if __name__ == "__main__":
    main()
