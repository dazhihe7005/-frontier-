#!/usr/bin/env python3
"""Compare SUPER position commands and PX4 positions with maze baffle geometry.

Only axis-aligned baffle collision boxes are considered. Distances are from
vehicle center to the closest rectangle in the horizontal plane, not certified
blade clearance. SITL starts at world x=-8 m; local x is world x+8 m.
"""

import argparse
import bisect
import json
import math
import xml.etree.ElementTree as ET

import rosbag


def baffles_from_world(path, spawn_x):
    root = ET.parse(path).getroot()
    boxes = {}
    for model in root.findall(".//world/model"):
        name = model.get("name", "")
        if not name.startswith("baffle_"):
            continue
        pose = [float(value) for value in model.findtext("pose").split()]
        size = [float(value) for value in model.findtext("link/collision/geometry/box/size").split()]
        if any(abs(angle) > 1e-9 for angle in pose[3:]):
            raise ValueError("rotated baffle not supported: " + name)
        boxes[name] = (pose[0] - spawn_x - size[0] / 2,
                       pose[0] - spawn_x + size[0] / 2,
                       pose[1] - size[1] / 2,
                       pose[1] + size[1] / 2)
    if not boxes:
        raise ValueError("no baffle collision boxes in " + path)
    return boxes


def horizontal_distance(x, y, box):
    xmin, xmax, ymin, ymax = box
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


def nearest(samples, stamp):
    stamps = [sample[0] for sample in samples]
    index = bisect.bisect_left(stamps, stamp)
    choices = [samples[i] for i in (index - 1, index) if 0 <= i < len(samples)]
    return min(choices, key=lambda sample: abs(sample[0] - stamp))


def minimum_record(samples, box):
    stamp, x, y = min(samples, key=lambda sample: horizontal_distance(sample[1], sample[2], box))
    return {"time_s": stamp, "local_x": x, "local_y": y,
            "center_distance_m": horizontal_distance(x, y, box)}


def analyze(bag_path, boxes, vehicle_radius):
    planned = []
    actual = []
    with rosbag.Bag(bag_path) as bag:
        for topic, message, receipt in bag.read_messages(
                topics=["/planning/pos_cmd", "/mavros/local_position/pose"]):
            stamp = message.header.stamp.to_sec() or receipt.to_sec()
            if topic == "/planning/pos_cmd":
                point = message.position
                planned.append((stamp, point.x, point.y))
            else:
                point = message.pose.position
                actual.append((stamp, point.x, point.y))
    if not planned or not actual:
        raise ValueError("bag requires both /planning/pos_cmd and /mavros/local_position/pose")
    planned.sort()
    actual.sort()
    # Ignore preflight and post-task PX4 poses; compare the same active interval.
    actual = [sample for sample in actual if planned[0][0] <= sample[0] <= planned[-1][0]]
    if not actual:
        raise ValueError("no PX4 pose overlaps SUPER command interval")
    results = {}
    for name, box in boxes.items():
        plan_min = minimum_record(planned, box)
        actual_min = minimum_record(actual, box)
        command_near_actual = nearest(planned, actual_min["time_s"])
        command_age = abs(command_near_actual[0] - actual_min["time_s"])
        results[name] = {
            "planned_minimum": plan_min,
            "actual_minimum": actual_min,
            "planned_surface_margin_m": plan_min["center_distance_m"] - vehicle_radius,
            "actual_surface_margin_m": actual_min["center_distance_m"] - vehicle_radius,
            "command_at_actual_minimum": {
                "time_s": command_near_actual[0],
                "local_x": command_near_actual[1],
                "local_y": command_near_actual[2],
                "center_distance_m": horizontal_distance(command_near_actual[1], command_near_actual[2], box),
                "command_age_s": command_age,
                "tracking_xy_error_m": math.hypot(
                    actual_min["local_x"] - command_near_actual[1],
                    actual_min["local_y"] - command_near_actual[2]),
            },
        }
    return {"vehicle_radius_m": vehicle_radius, "planned_samples": len(planned),
            "actual_samples": len(actual), "baffles": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("world")
    parser.add_argument("--spawn-x", type=float, default=-8.0)
    parser.add_argument("--vehicle-radius", type=float, default=0.4)
    args = parser.parse_args()
    print(json.dumps(analyze(args.bag, baffles_from_world(args.world, args.spawn_x),
                             args.vehicle_radius), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
