#!/usr/bin/env python3
"""Audit sampled SUPER commands against the PX4-local height fence.

This is a diagnostic, not a trajectory-safety proof: samples can miss an
overshoot between publication times, and rosbag receipt order is not a full
record of the planner's internal state.
"""

import argparse
import json
import math

import rosbag


COMMAND_TOPIC = "/planning/pos_cmd"
ALIGNMENT_TOPIC = "/mine_uav/task1/fastlio_to_px4_alignment"


def audit(bag_path, min_height, max_height, alignment_z=None, tolerance=1e-6):
    if not math.isfinite(min_height) or not math.isfinite(max_height):
        raise ValueError("height limits must be finite")
    if min_height >= max_height:
        raise ValueError("min_height must be below max_height")
    if alignment_z is not None and not math.isfinite(alignment_z):
        raise ValueError("alignment_z must be finite")

    active_alignment = alignment_z
    alignment_source = "explicit" if alignment_z is not None else None
    result = {
        "bag": bag_path,
        "px4_local_bounds_m": [min_height, max_height],
        "alignment_source": alignment_source,
        "alignment_updates": 0,
        "active_commands": 0,
        "unpaired_commands": 0,
        "wrong_frame_commands": 0,
        "nonfinite_commands": 0,
        "below_min_commands": 0,
        "above_max_commands": 0,
        "max_px4_local_z_m": None,
        "min_px4_local_z_m": None,
        "first_violation": None,
    }
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, stamp in bag.read_messages(
            topics=[ALIGNMENT_TOPIC, COMMAND_TOPIC]
        ):
            if topic == ALIGNMENT_TOPIC:
                z = msg.transform.translation.z
                if not math.isfinite(z):
                    active_alignment = None
                    continue
                active_alignment = z
                result["alignment_source"] = "bag"
                result["alignment_updates"] += 1
                continue
            if msg.trajectory_flag not in (1, 2):
                continue
            result["active_commands"] += 1
            if msg.header.frame_id != "camera_init":
                result["wrong_frame_commands"] += 1
                continue
            if not math.isfinite(msg.position.z):
                result["nonfinite_commands"] += 1
                continue
            if active_alignment is None:
                result["unpaired_commands"] += 1
                continue
            px4_z = msg.position.z + active_alignment
            old_min = result["min_px4_local_z_m"]
            old_max = result["max_px4_local_z_m"]
            result["min_px4_local_z_m"] = px4_z if old_min is None else min(old_min, px4_z)
            result["max_px4_local_z_m"] = px4_z if old_max is None else max(old_max, px4_z)
            if px4_z < min_height - tolerance or px4_z > max_height + tolerance:
                if px4_z < min_height - tolerance:
                    result["below_min_commands"] += 1
                else:
                    result["above_max_commands"] += 1
                if result["first_violation"] is None:
                    result["first_violation"] = {
                        "bag_time_s": stamp.to_sec(),
                        "planner_camera_init_z_m": msg.position.z,
                        "alignment_z_m": active_alignment,
                        "px4_local_z_m": px4_z,
                    }
    result["pass"] = (
        result["active_commands"] > 0
        and result["alignment_source"] is not None
        and result["unpaired_commands"] == 0
        and result["wrong_frame_commands"] == 0
        and result["nonfinite_commands"] == 0
        and result["below_min_commands"] == 0
        and result["above_max_commands"] == 0
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Task-one rosbag")
    parser.add_argument("--min-height", type=float, default=-2.0)
    parser.add_argument("--max-height", type=float, default=1.8)
    parser.add_argument(
        "--alignment-z", type=float,
        help="Only for bags without alignment topic; explicit camera_init to PX4-local z offset",
    )
    args = parser.parse_args()
    result = audit(args.bag, args.min_height, args.max_height, args.alignment_z)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
