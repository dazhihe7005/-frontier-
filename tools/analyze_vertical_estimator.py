#!/usr/bin/env python3
"""Offline comparison of visual, PX4, and Gazebo vertical motion.

This reads only the diagnostic truth CSV. It does not publish ROS data, tune
PX4, or provide flight-control inputs. Vision-frame derivatives are used
because this FAST-LIO2 build leaves the Odometry twist fields unset.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def _number(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def analyze_rows(rows, minimum_speed=0.05):
    frames = []
    prior_stamp = None
    run_ids = set()
    for row in rows:
        run_ids.add(row.get("ros_run_id", ""))
        if any(row.get(key) != "1" for key in
               ("armed", "offboard", "exploration_started")):
            continue
        stamp = _number(row, "fastlio_vision_stamp")
        sim_time = _number(row, "sim_time")
        values = tuple(_number(row, key) for key in
                       ("z", "fastlio_vision_z", "px4_local_z", "px4_vz"))
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in values):
            continue
        if abs(sim_time-stamp) > 0.20 or stamp == prior_stamp:
            continue
        if prior_stamp is not None and stamp < prior_stamp:
            frames.clear()  # Simulation clock reset: never splice the runs.
        prior_stamp = stamp
        frames.append((stamp, *values))
    samples = []
    for first, second in zip(frames, frames[1:]):
        dt = second[0]-first[0]
        if not 0.05 <= dt <= 0.25:
            continue
        truth_vz = (second[1]-first[1])/dt
        vision_vz = (second[2]-first[2])/dt
        px4_vz = 0.5*(first[4]+second[4])
        samples.append((dt, truth_vz, vision_vz, px4_vz))
    if not samples:
        raise ValueError("no synchronized visual frames in armed OFFBOARD exploration")
    contradictory_px4 = [s for s in samples if
                         abs(s[1]) >= minimum_speed and
                         abs(s[3]) >= minimum_speed and s[1]*s[3] < 0.0]
    contradictory_vision = [s for s in samples if
                            abs(s[1]) >= minimum_speed and
                            abs(s[2]) >= minimum_speed and s[1]*s[2] < 0.0]
    duration = sum(s[0] for s in samples)
    px4_offsets = [frame[1]-frame[3] for frame in frames]
    vision_offsets = [frame[1]-frame[2] for frame in frames]
    return {
        "kind": "diagnostic_only_vertical_estimator_comparison",
        "ros_run_id": next(iter(run_ids)) if len(run_ids) == 1 else None,
        "vision_frames": len(frames),
        "velocity_intervals": len(samples),
        "compared_duration_s": round(duration, 3),
        "minimum_motion_speed_mps": minimum_speed,
        "px4_vz_opposite_truth_intervals": len(contradictory_px4),
        "px4_vz_opposite_truth_duration_s": round(
            sum(s[0] for s in contradictory_px4), 3),
        "vision_vz_opposite_truth_intervals": len(contradictory_vision),
        "vision_vz_opposite_truth_duration_s": round(
            sum(s[0] for s in contradictory_vision), 3),
        "px4_vz_vs_truth_mae_mps": round(statistics.mean(
            abs(s[3]-s[1]) for s in samples), 4),
        "vision_vz_vs_truth_mae_mps": round(statistics.mean(
            abs(s[2]-s[1]) for s in samples), 4),
        "px4_height_offset_range_m": [round(min(px4_offsets), 4),
                                       round(max(px4_offsets), 4)],
        "vision_height_offset_range_m": [round(min(vision_offsets), 4),
                                          round(max(vision_offsets), 4)],
        "notes": [
            "Gazebo truth is read only; this report is not a control input.",
            "Frame-to-frame velocity is unfiltered and may amplify pose noise.",
            "Pose offsets include different initialization origins.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--minimum-speed", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.minimum_speed <= 0:
        parser.error("minimum speed must be positive")
    with args.trajectory.open("r", encoding="utf-8", newline="") as source:
        result = analyze_rows(csv.DictReader(source), args.minimum_speed)
    result["trajectory_file"] = str(args.trajectory)
    formatted = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(formatted)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(formatted+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
