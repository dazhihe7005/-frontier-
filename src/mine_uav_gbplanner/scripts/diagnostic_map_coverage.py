#!/usr/bin/env python3
"""Read-only, Gazebo-truth-based coverage audit for the 1:1 tunnel.

The reference and truth pose are used *only* here, never published to the
planner, estimator, PX4 or flight controls.  Ray coverage is a visibility
proxy, not proof that Voxblox stored every free voxel or that every 3-D path
is traversable.
"""

import gzip
import json
import math
import os
from collections import deque


def rotate(q, p):
    x, y, z, w = q
    px, py, pz = p
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz,
            pz+w*tz+x*ty-y*tx)


class CoverageGrid:
    def __init__(self, reference, height_tolerance=0.75):
        self.resolution = float(reference["resolution_m"])
        self.origin_x, self.origin_y = map(float, reference["origin_xy_m"])
        self.cells = {(int(ix), int(iy)): float(z)
                      for ix, iy, z in reference["reachable_cells"]}
        self.height_tolerance = float(height_tolerance)
        self.seen = set()
        self.visited = set()
        self.first_seen_at = None
        self.last_new_at = None

    def key(self, x, y):
        return (math.floor((x-self.origin_x)/self.resolution),
                math.floor((y-self.origin_y)/self.resolution))

    def _mark(self, target, x, y, z, stamp):
        key = self.key(x, y)
        reference_z = self.cells.get(key)
        if reference_z is None or abs(z-reference_z) > self.height_tolerance:
            return False
        if key not in target:
            target.add(key)
            if target is self.seen:
                if self.first_seen_at is None:
                    self.first_seen_at = stamp
                self.last_new_at = stamp
            return True
        return False

    def visit(self, x, y, z, stamp):
        self._mark(self.visited, x, y, z, stamp)
        self._mark(self.seen, x, y, z, stamp)

    def trace_ray(self, origin, endpoint, stamp, step=0.5):
        dx, dy, dz = (endpoint[i]-origin[i] for i in range(3))
        distance = math.sqrt(dx*dx+dy*dy+dz*dz)
        if not math.isfinite(distance) or distance < 0.01:
            return
        steps = max(1, math.ceil(distance/step))
        for index in range(steps+1):
            fraction = index/steps
            self._mark(self.seen, origin[0]+fraction*dx,
                       origin[1]+fraction*dy,
                       origin[2]+fraction*dz, stamp)

    def report(self, now):
        total = len(self.cells)
        missing = set(self.cells)-self.seen
        x_values = [self.origin_x+(key[0]+0.5)*self.resolution
                    for key in missing]
        components = []
        remaining = set(missing)
        while remaining:
            seed = remaining.pop()
            connected = [seed]
            queue = deque([seed])
            while queue:
                ix, iy = queue.popleft()
                for neighbor in ((ix+1, iy), (ix-1, iy),
                                 (ix, iy+1), (ix, iy-1)):
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        connected.append(neighbor)
                        queue.append(neighbor)
            xs = [self.origin_x+(key[0]+0.5)*self.resolution
                  for key in connected]
            ys = [self.origin_y+(key[1]+0.5)*self.resolution
                  for key in connected]
            components.append({
                "cells": len(connected),
                "area_m2": round(len(connected)*self.resolution**2, 2),
                "x_range_m": [round(min(xs), 2), round(max(xs), 2)],
                "y_range_m": [round(min(ys), 2), round(max(ys), 2)],
            })
        components.sort(key=lambda component: component["cells"],
                        reverse=True)
        return {
            "kind": "diagnostic_only_lidar_visibility_proxy",
            "reference_cells": total,
            "visible_cells": len(self.seen),
            "visibility_fraction": round(len(self.seen)/total, 5),
            "visited_cells": len(self.visited),
            "visited_fraction": round(len(self.visited)/total, 5),
            "unseen_cells": len(missing),
            "unseen_x_range_m": [round(min(x_values), 2),
                                  round(max(x_values), 2)] if missing else None,
            "unseen_component_count": len(components),
            "largest_unseen_components": components[:5],
            "seconds_since_new_cell": round(now-self.last_new_at, 1)
            if self.last_new_at is not None else None,
            "complete": len(missing) == 0,
        }


def main():
    import rospy
    from gazebo_msgs.msg import ModelStates
    from sensor_msgs.msg import PointCloud

    rospy.init_node("diagnostic_map_coverage")
    reference_file = rospy.get_param(
        "~reference_file",
        "/home/nuc/gbplanner2_isolated_ws/runtime/map_reference/"
        "baixianshan_reachable_0p5m.json.gz")
    output_file = rospy.get_param(
        "~output_file",
        "/home/nuc/gbplanner2_isolated_ws/runtime/map_reference/"
        "latest_coverage.json")
    with gzip.open(reference_file, "rt", encoding="utf-8") as source:
        reference = json.load(source)
    grid = CoverageGrid(reference, float(rospy.get_param(
        "~height_tolerance", 0.75)))
    pose = None
    pose_stamp = None
    pitch = float(rospy.get_param("~sensor_pitch", 0.436332313))
    sensor_q = (0.0, math.sin(pitch/2), 0.0, math.cos(pitch/2))
    sensor_offset = tuple(rospy.get_param(
        "~sensor_offset", [0.1315, 0.0, 0.223]))
    max_pose_age = float(rospy.get_param("~max_pose_age", 0.20))
    ray_stride = max(1, int(rospy.get_param("~ray_stride", 8)))
    cloud_stride = max(1, int(rospy.get_param("~cloud_stride", 2)))
    scan_count = 0
    accepted_scans = 0
    rejected_stale_scans = 0
    flight_x = []
    all_x_min = math.inf
    all_x_max = -math.inf
    all_y_min = math.inf
    all_y_max = -math.inf
    last_travel_pose = None
    last_travel_stamp = None
    travel_distance = 0.0

    def truth_cb(message):
        nonlocal pose, pose_stamp
        nonlocal all_x_min, all_x_max, all_y_min, all_y_max
        nonlocal last_travel_pose, last_travel_stamp, travel_distance
        try:
            index = message.name.index("iris")
        except ValueError:
            return
        current = message.pose[index]
        p = current.position
        q = current.orientation
        body_q = (q.x, q.y, q.z, q.w)
        pose = ((p.x, p.y, p.z), body_q)
        pose_stamp = rospy.Time.now().to_sec()
        all_x_min = min(all_x_min, p.x)
        all_x_max = max(all_x_max, p.x)
        all_y_min = min(all_y_min, p.y)
        all_y_max = max(all_y_max, p.y)
        if last_travel_stamp is None or pose_stamp-last_travel_stamp >= 0.1:
            if last_travel_pose is not None:
                travel_distance += math.sqrt(sum(
                    (a-b)**2 for a, b in zip((p.x, p.y, p.z),
                                              last_travel_pose)))
            last_travel_pose = (p.x, p.y, p.z)
            last_travel_stamp = pose_stamp
        grid.visit(p.x, p.y, p.z, pose_stamp)
        flight_x.append(p.x)
        if len(flight_x) > 1000:
            del flight_x[:500]

    def cloud_cb(message):
        nonlocal scan_count, accepted_scans, rejected_stale_scans
        scan_count += 1
        if scan_count % cloud_stride or pose is None:
            return
        # ModelStates has no ROS header.  Use receipt time only to reject a
        # stale truth sample; this is an audit, not a scan deskewer.
        now = rospy.Time.now().to_sec()
        if abs(now-pose_stamp) > max_pose_age:
            rejected_stale_scans += 1
            return
        body_p, body_q = pose
        offset = rotate(body_q, sensor_offset)
        sensor_p = tuple(body_p[i]+offset[i] for i in range(3))
        for index in range(0, len(message.points), ray_stride):
            raw = message.points[index]
            if not all(math.isfinite(v) for v in (raw.x, raw.y, raw.z)):
                continue
            body_ray = rotate(sensor_q, (raw.x, raw.y, raw.z))
            world_ray = rotate(body_q, body_ray)
            endpoint = tuple(sensor_p[i]+world_ray[i] for i in range(3))
            grid.trace_ray(sensor_p, endpoint, now)
        accepted_scans += 1

    def report_cb(_event):
        report = grid.report(rospy.Time.now().to_sec())
        report.update({
            "accepted_scans": accepted_scans,
            "rejected_stale_scans": rejected_stale_scans,
            "recent_truth_x_range_m": [round(min(flight_x), 2),
                                        round(max(flight_x), 2)]
            if flight_x else None,
            "whole_run_truth_x_range_m": [round(all_x_min, 2),
                                          round(all_x_max, 2)]
            if math.isfinite(all_x_min) else None,
            "whole_run_truth_y_range_m": [round(all_y_min, 2),
                                          round(all_y_max, 2)]
            if math.isfinite(all_y_min) else None,
            "truth_travel_distance_m": round(travel_distance, 2),
            "reference_file": reference_file,
        })
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        temporary = output_file + ".tmp"
        with open(temporary, "w", encoding="utf-8") as target:
            json.dump(report, target, sort_keys=True, indent=2)
        os.replace(temporary, output_file)
        rospy.loginfo("COVERAGE_AUDIT=%s", json.dumps(report, sort_keys=True))

    rospy.Subscriber("/gazebo/model_states", ModelStates, truth_cb,
                     queue_size=10)
    rospy.Subscriber("/mine_uav/sitl/mid360/points", PointCloud, cloud_cb,
                     queue_size=2, buff_size=8*1024*1024)
    rospy.Timer(rospy.Duration(float(rospy.get_param(
        "~report_interval", 15.0))), report_cb)
    rospy.loginfo("Diagnostic coverage audit: %d reachable cells, %.2f m grid",
                  len(grid.cells), grid.resolution)
    rospy.spin()


if __name__ == "__main__":
    main()
