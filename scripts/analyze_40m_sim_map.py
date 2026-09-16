#!/usr/bin/env python3

"""Measure the saved 40 x 40 x 30 m SITL cloud against the known room walls.

This is an offline ground-truth check for this Gazebo world, not a runtime
completion rule or a substitute for real-world map quality inspection.
"""

import argparse
import bisect
import itertools
import json
import math
import statistics

import rosbag
import rospy
from sensor_msgs import point_cloud2


TOPIC = "/mine_uav/sitl/global_cloud"


def coverage(points, axis, surface, along_axis, along_start, along_end,
             z_start=-16.15, z_end=13.85, bin_size=2.0, tolerance=0.8):
    along_bins = int(math.ceil((along_end - along_start) / bin_size))
    height_bins = int(math.ceil((z_end - z_start) / bin_size))
    cells = set()
    count = 0
    z_values = []
    for point in points:
        if abs(point[axis] - surface) > tolerance:
            continue
        along = point[along_axis]
        z = point[2]
        if not (along_start <= along < along_end and z_start <= z < z_end):
            continue
        along_bin = min(along_bins - 1, int((along - along_start) / bin_size))
        height_bin = min(height_bins - 1, int((z - z_start) / bin_size))
        cells.add((along_bin, height_bin))
        count += 1
        z_values.append(z)
    along_seen = {cell[0] for cell in cells}
    height_seen = {cell[1] for cell in cells}
    return {
        "point_count": count,
        "z_min": min(z_values) if z_values else None,
        "z_max": max(z_values) if z_values else None,
        "surface_cell_fraction": len(cells) / float(along_bins * height_bins),
        "along_bins_covered": len(along_seen),
        "along_bins_total": along_bins,
        "height_bins_covered": len(height_seen),
        "height_bins_total": height_bins,
        "height_bins_missing": [
            index for index in range(height_bins) if index not in height_seen
        ],
        "longest_along_gap_m": bin_size * max(
            (len(list(group)) for present, group in itertools.groupby(
                index in along_seen for index in range(along_bins)
            ) if not present), default=0
        ),
    }


def horizontal_coverage(points, height, origin_x, origin_y,
                        bin_size=2.0, tolerance=0.8):
    cells = set()
    count = 0
    for x, y, z in points:
        if abs(z - height) > tolerance or not (-origin_x <= x < 40.0 - origin_x):
            continue
        if not (-20.0 - origin_y <= y < 20.0 - origin_y):
            continue
        cells.add((int((x + origin_x) / bin_size),
                   int((y + origin_y + 20.0) / bin_size)))
        count += 1
    return {"point_count": count, "surface_cell_fraction": len(cells) / 400.0,
            "occupied_cells": len(cells), "total_cells": 400}


def _local_origin_from_bag(bag):
    end = bag.get_end_time()
    start = rospy.Time.from_sec(max(0.0, end - 5.0))
    links = []
    odometry = []
    for topic, message, stamp in bag.read_messages(
        topics=["/gazebo/link_states", "/Odometry"], start_time=start
    ):
        if topic == "/gazebo/link_states":
            try:
                index = message.name.index("iris::iris::base_link")
            except ValueError:
                continue
            links.append((stamp.to_sec(), message.pose[index].position))
        else:
            odometry.append((stamp.to_sec(), message.pose.pose.position))
    if not links or not odometry:
        raise RuntimeError("bag lacks /gazebo/link_states and /Odometry; "
                           "supply --origin-x/--origin-y/--origin-z")
    times = [stamp for stamp, _position in links]
    offsets = []
    for stamp, position in odometry:
        index = bisect.bisect_left(times, stamp)
        candidates = [value for value in (index - 1, index)
                      if 0 <= value < len(times)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda value: abs(times[value] - stamp))
        if abs(times[nearest] - stamp) > 0.02:
            continue
        world = links[nearest][1]
        offsets.append((world.x - position.x, world.y - position.y,
                        world.z - position.z))
    if len(offsets) < 3:
        raise RuntimeError("insufficient synchronized Gazebo/PX4 frame samples")
    return tuple(statistics.median(sample[i] for sample in offsets)
                 for i in range(3))


def analyze(bag_path, map_capacity=400000, origin=None):
    with rosbag.Bag(bag_path) as bag:
        if origin is None:
            origin = _local_origin_from_bag(bag)
        end = bag.get_end_time()
        first_full_map_stamp = None
        last = None
        for _topic, message, stamp in bag.read_messages(topics=[TOPIC]):
            if message.width >= map_capacity and first_full_map_stamp is None:
                first_full_map_stamp = stamp.to_sec()
        for _topic, message, stamp in bag.read_messages(
            topics=[TOPIC], start_time=rospy.Time.from_sec(max(0.0, end - 3.0))
        ):
            last = (message, stamp.to_sec())
    if last is None:
        raise RuntimeError("no final global cloud in the last 3 s of the bag")
    message, stamp = last
    points = list(point_cloud2.read_points(message, field_names=("x", "y", "z"),
                                           skip_nans=True))
    origin_x, origin_y, origin_z = origin
    # World to camera_init is measured from synchronous Gazebo/PX4 poses.
    # It is NOT identical to the SDF spawn pose after vehicle settling.
    walls = {
        "left": coverage(points, 1, 19.8 - origin_y, 0,
                         -origin_x, 40.0 - origin_x,
                         -origin_z, 30.0 - origin_z),
        "right": coverage(points, 1, -19.8 - origin_y, 0,
                          -origin_x, 40.0 - origin_x,
                          -origin_z, 30.0 - origin_z),
        "far": coverage(points, 0, 39.8 - origin_x, 1,
                        -20.0 - origin_y, 20.0 - origin_y,
                        -origin_z, 30.0 - origin_z),
    }
    horizontal = {
        "floor": horizontal_coverage(points, 0.2 - origin_z,
                                     origin_x, origin_y),
        "ceiling": horizontal_coverage(points, 29.8 - origin_z,
                                       origin_x, origin_y),
    }
    checks = {
        name: result["surface_cell_fraction"] >= 0.70
        and result["longest_along_gap_m"] <= 4.0
        and result["height_bins_covered"] >= 13
        for name, result in walls.items()
    }
    checks["map_not_capacity_clipped"] = first_full_map_stamp is None
    for name, result in horizontal.items():
        checks[name] = result["surface_cell_fraction"] >= 0.70
    return {
        "bag": bag_path,
        "map_stamp": stamp,
        "map_frame": message.header.frame_id,
        "local_origin_in_world": origin,
        "total_points": len(points),
        "map_capacity": map_capacity,
        "first_capacity_stamp": first_full_map_stamp,
        "walls": walls,
        "horizontal_surfaces": horizontal,
        "checks": checks,
        "verdict": "PASS" if all(checks.values()) else "FAIL",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--json-output")
    parser.add_argument("--map-capacity", type=int, default=400000)
    parser.add_argument("--origin-x", type=float)
    parser.add_argument("--origin-y", type=float)
    parser.add_argument("--origin-z", type=float)
    args = parser.parse_args()
    origin_values = (args.origin_x, args.origin_y, args.origin_z)
    if any(value is not None for value in origin_values) and not all(
        value is not None for value in origin_values
    ):
        parser.error("--origin-x, --origin-y and --origin-z must be supplied together")
    origin = tuple(origin_values) if all(value is not None for value in origin_values) else None
    result = analyze(args.bag, args.map_capacity, origin)
    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as output:
            json.dump(result, output, indent=2)
            output.write("\n")
    print("TASK1_40M_SIM_MAP: %s (%d points)" % (
        result["verdict"], result["total_points"]
    ))
    for name, wall in result["walls"].items():
        print("%s: %d points, 3D cells %.3f, along %d/%d, height %d/%d, "
              "z %.1f..%.1f, missing heights %s, max gap %.1f m" % (
                  name, wall["point_count"], wall["surface_cell_fraction"],
                  wall["along_bins_covered"], wall["along_bins_total"],
                  wall["height_bins_covered"], wall["height_bins_total"],
                  wall["z_min"], wall["z_max"],
                  wall["height_bins_missing"],
                  wall["longest_along_gap_m"],
              ))
    for name, surface in result["horizontal_surfaces"].items():
        print("%s: %d points, 2D cells %.3f" % (
            name, surface["point_count"], surface["surface_cell_fraction"]
        ))
    raise SystemExit(0 if result["verdict"] == "PASS" else 1)


if __name__ == "__main__":
    main()
