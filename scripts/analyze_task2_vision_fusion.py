#!/usr/bin/env python3

"""Audit PX4 vision-height SITL evidence; Gazebo truth is not a real sensor."""

import argparse
import json
import sys

import rosbag
from pyulog import ULog


def analyze(bag_path, ulog_path):
    injection_s = None
    completion_s = None
    vision_after = 0
    estimator_after = 0
    invalid_z_after = 0
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[
            "/mine_uav/sitl/px4_z_failure_status",
            "/mine_uav/shaft/status",
            "/mavros/vision_pose/pose_cov",
            "/mavros/estimator_status",
        ]):
            t = stamp.to_sec()
            if topic == "/mine_uav/sitl/px4_z_failure_status":
                if msg.data.startswith("INJECTED_BARO_GPS_OFF") and injection_s is None:
                    injection_s = t
            elif topic == "/mine_uav/shaft/status":
                if msg.data == "COMPLETE" and completion_s is None:
                    completion_s = t
            elif injection_s is not None and (completion_s is None or t <= completion_s):
                if topic == "/mavros/vision_pose/pose_cov":
                    vision_after += 1
                elif topic == "/mavros/estimator_status":
                    estimator_after += 1
                    if not msg.pos_vert_abs_status_flag:
                        invalid_z_after += 1
    ulog = ULog(ulog_path, message_name_filter_list=["estimator_status_flags"])
    flags = ulog.get_dataset("estimator_status_flags").data
    return {
        "baro_gps_off_accepted_s": injection_s,
        "task_complete_s": completion_s,
        "vision_messages_after_injection": vision_after,
        "estimator_messages_after_injection": estimator_after,
        "invalid_px4_z_after_injection": invalid_z_after,
        "ulog_vision_height_fused_samples": int(sum(flags["cs_ev_hgt"])),
        "ulog_vision_position_fused_samples": int(sum(flags["cs_ev_pos"])),
        "ulog_baro_height_fused_samples": int(sum(flags["cs_baro_hgt"])),
        "ulog_samples": len(flags["cs_ev_hgt"]),
        "ulog_last_vision_height_fused": bool(flags["cs_ev_hgt"][-1]),
        "ulog_last_baro_height_fused": bool(flags["cs_baro_hgt"][-1]),
        "ulog_last_gps_fused": bool(flags["cs_gps"][-1]),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("ulog")
    args = parser.parse_args()
    result = analyze(args.bag, args.ulog)
    print(json.dumps(result, indent=2))
    failures = []
    if result["baro_gps_off_accepted_s"] is None:
        failures.append("no accepted baro/GPS failure injection")
    if result["task_complete_s"] is None or (
            result["baro_gps_off_accepted_s"] is not None and
            result["task_complete_s"] <= result["baro_gps_off_accepted_s"]):
        failures.append("task did not complete after injection")
    if result["vision_messages_after_injection"] < 100 or \
            result["estimator_messages_after_injection"] < 20:
        failures.append("insufficient post-injection vision/estimator observation")
    if result["invalid_px4_z_after_injection"]:
        failures.append("PX4 Z invalid after injection")
    if not result["ulog_last_vision_height_fused"] or \
            result["ulog_last_baro_height_fused"] or \
            result["ulog_last_gps_fused"]:
        failures.append("PX4 ULog does not show vision-only height at end")
    if failures:
        print("FAIL: " + "; ".join(failures), file=sys.stderr)
        sys.exit(1)
