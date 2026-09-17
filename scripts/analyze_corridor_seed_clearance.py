#!/usr/bin/env python3
"""Measure raw and inflated ROG-map points near a failed SUPER seed line."""

import argparse
import json
import math

import rosbag
from sensor_msgs import point_cloud2


TOPICS = ("/fsm_node/rog_map/occ", "/fsm_node/rog_map/inf_occ")


def point_segment_distance(point, start, end):
    direction = [end[i] - start[i] for i in range(3)]
    relative = [point[i] - start[i] for i in range(3)]
    length_squared = sum(value * value for value in direction)
    fraction = (sum(relative[i] * direction[i] for i in range(3)) /
                length_squared if length_squared else 0.0)
    fraction = max(0.0, min(1.0, fraction))
    return math.sqrt(sum((relative[i] - fraction * direction[i]) ** 2
                         for i in range(3)))


def closest_clouds(bag_path, time_s):
    selected = {}
    with rosbag.Bag(bag_path) as bag:
        for topic, message, _ in bag.read_messages(topics=TOPICS):
            difference = abs(message.header.stamp.to_sec() - time_s)
            if topic not in selected or difference < selected[topic][0]:
                selected[topic] = (difference, message)
    if set(selected) != set(TOPICS):
        raise ValueError("bag must contain both raw and inflated occupancy clouds")
    return selected


def analyze(bag_path, time_s, start, end):
    output = {}
    for topic, (_, cloud) in closest_clouds(bag_path, time_s).items():
        nearest = None
        count = 0
        for point in point_cloud2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=True):
            count += 1
            distance = point_segment_distance(point, start, end)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, point)
        output[topic] = {
            "cloud_stamp_s": cloud.header.stamp.to_sec(),
            "frame_id": cloud.header.frame_id,
            "point_count": count,
            "nearest_distance_m": nearest[0] if nearest else None,
            "nearest_point": list(nearest[1]) if nearest else None,
        }
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--time", type=float, required=True)
    parser.add_argument("--start", type=float, nargs=3, required=True)
    parser.add_argument("--end", type=float, nargs=3, required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.bag, args.time, args.start, args.end),
                     indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
