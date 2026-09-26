#!/usr/bin/env python3
"""Offline certificate for an explicitly surveyed simulation launch corridor.

This reads collision meshes only. It never supplies Gazebo truth or a prior to
the running vehicle. A real deployment needs an independent field survey.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_baixiangshan_reference import DEFAULT_MODEL, MESH_NAMES, load_obj_bvh


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--start", nargs=3, type=float,
                        default=[-9.8, 0.15, 1.8])
    parser.add_argument("--end", nargs=3, type=float,
                        default=[-6.8, 0.15, 1.8])
    parser.add_argument("--sample-step", type=float, default=0.05)
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--position-error", type=float, default=0.10)
    parser.add_argument("--reserve", type=float, default=0.05)
    args = parser.parse_args(argv)
    if args.sample_step <= 0 or args.safety_radius <= 0 or \
            args.position_error < 0 or args.reserve < 0:
        parser.error("invalid corridor safety parameters")
    return args


def main():
    args = parse_args()
    paths = {name: args.model / "meshes" /
             ("baixianshan_" + name + ".obj") for name in MESH_NAMES}
    bvhs = {name: load_obj_bvh(path) for name, path in paths.items()}
    start = Vector(args.start)
    end = Vector(args.end)
    length = (end - start).length
    intervals = max(1, math.ceil(length / args.sample_step))
    minimum = math.inf
    worst = None
    for index in range(intervals + 1):
        point = start.lerp(end, index / intervals)
        for name, bvh in bvhs.items():
            _, _, _, distance = bvh.find_nearest(point, 10.0)
            if distance is not None and distance < minimum:
                minimum = float(distance)
                worst = {"mesh": name, "xyz_m": [round(v, 5) for v in point],
                         "sample_index": index}
    # Surface distance is 1-Lipschitz. Every point between two samples lies
    # within half a sample spacing of one endpoint. The position-error term
    # covers a separate radial perturbation of the centreline.
    lower_bound = minimum - 0.5 * length / intervals - args.position_error
    required = args.safety_radius + args.reserve
    result = {
        "kind": "diagnostic_only_surveyed_launch_corridor_mesh_check",
        "model": str(args.model),
        "mesh_sha256": {name: sha256(path) for name, path in paths.items()},
        "start_xyz_m": args.start,
        "end_xyz_m": args.end,
        "centreline_length_m": round(length, 5),
        "sample_count": intervals + 1,
        "max_sample_step_m": round(length / intervals, 5),
        "position_error_m": args.position_error,
        "safety_radius_m": args.safety_radius,
        "reserve_m": args.reserve,
        "sampled_min_clearance_m": round(minimum, 5),
        "continuous_clearance_lower_bound_m": round(lower_bound, 5),
        "worst_sample": worst,
        "sim_mesh_precheck_passed": lower_bound >= required,
        "real_site_survey_proven": False,
    }
    formatted = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(formatted)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(formatted + "\n", encoding="utf-8")
    return 0 if result["sim_mesh_precheck_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
