#!/usr/bin/env python3
"""Measure SUPER front-end guide-path segments against maze collision boxes."""

import argparse
import json
import math

import rosbag

from analyze_maze_clearance import baffles_from_world, horizontal_distance


TOPIC = "/fsm_node/visualization/frontend_path"


def point_segment_distance_xy(x, y, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_squared = dx * dx + dy * dy
    fraction = ((x - a[0]) * dx + (y - a[1]) * dy) / length_squared if length_squared else 0.0
    fraction = max(0.0, min(1.0, fraction))
    return math.hypot(x - a[0] - fraction * dx,
                      y - a[1] - fraction * dy)


def segment_box_distance_xy(a, b, box):
    """Exact 2-D distance between a line segment and an axis-aligned box."""
    xmin, xmax, ymin, ymax = box
    dx, dy = b[0] - a[0], b[1] - a[1]
    lower, upper = 0.0, 1.0
    intersects = True
    for slope, offset in ((-dx, a[0] - xmin), (dx, xmax - a[0]),
                          (-dy, a[1] - ymin), (dy, ymax - a[1])):
        if abs(slope) < 1e-12:
            if offset < 0.0:
                intersects = False
                break
            continue
        fraction = offset / slope
        if slope < 0.0:
            lower = max(lower, fraction)
        else:
            upper = min(upper, fraction)
        if lower > upper:
            intersects = False
            break
    if intersects:
        return 0.0
    distances = [horizontal_distance(*a, box), horizontal_distance(*b, box)]
    for x in (xmin, xmax):
        for y in (ymin, ymax):
            distances.append(point_segment_distance_xy(x, y, a, b))
    return min(distances)


def nearest_guide_path(bag_path, time_s):
    best = None
    with rosbag.Bag(bag_path) as bag:
        for _, message, receipt in bag.read_messages(topics=[TOPIC]):
            lines = [marker for marker in message.markers
                     if marker.ns == "guide_path_line" and len(marker.points) == 2]
            if not lines:
                continue
            difference = abs(receipt.to_sec() - time_s)
            if best is None or difference < best[0]:
                best = (difference, receipt.to_sec(), lines)
            if best is not None and receipt.to_sec() > time_s + 1.0:
                break
    if best is None:
        raise ValueError("bag has no non-empty SUPER guide-path marker")
    return best[1], best[2]


def analyze(bag_path, time_s, world_path, spawn_x):
    stamp, lines = nearest_guide_path(bag_path, time_s)
    boxes = baffles_from_world(world_path, spawn_x)
    result = {"path_stamp_s": stamp, "segment_count": len(lines),
              "baffles": {}}
    for name, box in boxes.items():
        records = []
        for index, marker in enumerate(lines):
            a, b = marker.points
            start, end = (a.x, a.y), (b.x, b.y)
            records.append((segment_box_distance_xy(start, end, box),
                            index, start, end))
        distance, index, start, end = min(records)
        result["baffles"][name] = {
            "minimum_center_distance_m": distance,
            "segment_index": index,
            "segment_start_xy": start,
            "segment_end_xy": end,
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("world")
    parser.add_argument("--time", type=float, required=True)
    parser.add_argument("--spawn-x", type=float, default=-8.0)
    args = parser.parse_args()
    print(json.dumps(analyze(args.bag, args.time, args.world, args.spawn_x),
                     indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
