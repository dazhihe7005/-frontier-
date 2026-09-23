#!/usr/bin/env python3
"""Recover GBPlanner from a persistent empty-decision loop.

The upstream global planner can have no retained frontier at a dead end even
though unexplored branches exist behind the vehicle.  In that case repeatedly
calling the local planner cannot change either the map or its root state.  This
node pauses PCI, retraces one previously executed (therefore collision-checked)
local trajectory, and resumes automatic exploration from the earlier root.

It deliberately does not use Gazebo ground truth and it never invents a direct
line through mapped space.  Recovery is limited to the reverse of an accepted
GBPlanner command, with a constant yaw to avoid a sharp 180-degree attitude
command.
"""

import copy
import math
import threading
from collections import deque

import rospy
from geometry_msgs.msg import Transform
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import (MultiDOFJointTrajectory,
                                 MultiDOFJointTrajectoryPoint)


RECOVERY_JOINT = "gbplanner_stall_recovery_backtrack"


def _distance(left, right):
    return math.sqrt(
        (left.x - right.x) ** 2 +
        (left.y - right.y) ** 2 +
        (left.z - right.z) ** 2)


class StallRecovery:
    def __init__(self):
        self._empty_limit = max(4, int(rospy.get_param("~empty_limit", 15)))
        self._speed = min(0.6, max(
            0.15, float(rospy.get_param("~backtrack_speed", 0.45))))
        self._max_distance = min(8.0, max(
            2.0, float(rospy.get_param("~max_backtrack_distance", 6.0))))
        self._endpoint_tolerance = max(
            0.3, float(rospy.get_param("~endpoint_tolerance", 1.5)))
        self._resume_margin = max(
            0.5, float(rospy.get_param("~resume_margin", 1.0)))
        self._lock = threading.Lock()
        self._history = deque(maxlen=200)
        self._last_key = None
        self._odom = None
        self._vision_healthy = False
        self._state = State()
        self._empty_count = 0
        self._recovering = False
        self._recovery_seq = 0

        self._trajectory_pub = rospy.Publisher(
            "/gbplanner/command/trajectory", MultiDOFJointTrajectory,
            queue_size=2)
        self._stop = rospy.ServiceProxy(
            "/planner_control_interface/std_srvs/stop", Trigger)
        self._resume = rospy.ServiceProxy(
            "/planner_control_interface/std_srvs/automatic_planning", Trigger)
        rospy.Subscriber("/gbplanner/command/trajectory",
                         MultiDOFJointTrajectory, self._trajectory_cb,
                         queue_size=10)
        rospy.Subscriber("/gbplanner_status", Bool, self._status_cb,
                         queue_size=50)
        rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                         self._odom_cb, queue_size=10)
        rospy.Subscriber("/mine_uav/gbplanner/vision_healthy", Bool,
                         self._vision_cb, queue_size=5)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=5)

    def _odom_cb(self, message):
        with self._lock:
            self._odom = message

    def _vision_cb(self, message):
        with self._lock:
            self._vision_healthy = bool(message.data)

    def _state_cb(self, message):
        with self._lock:
            self._state = message

    def _trajectory_cb(self, message):
        if RECOVERY_JOINT in message.joint_names or len(message.points) < 2:
            return
        transforms = [point.transforms[0] for point in message.points
                      if point.transforms]
        if len(transforms) < 2:
            return
        start = transforms[0].translation
        end = transforms[-1].translation
        if _distance(start, end) < 0.25:
            return
        key = (message.header.seq, round(end.x, 3), round(end.y, 3),
               round(end.z, 3))
        with self._lock:
            if key == self._last_key:
                return
            self._last_key = key
            self._history.append(copy.deepcopy(message))
            self._empty_count = 0

    def _status_cb(self, message):
        start_worker = False
        with self._lock:
            if message.data:
                self._empty_count = 0
                return
            if self._recovering:
                return
            self._empty_count += 1
            if self._empty_count >= self._empty_limit:
                state_ready = (self._vision_healthy and self._state.connected and
                               self._state.armed and self._state.mode == "OFFBOARD")
                if self._odom is not None and state_ready and self._history:
                    self._recovering = True
                    start_worker = True
        if start_worker:
            threading.Thread(target=self._recover, daemon=True).start()

    def _select_path(self, current):
        """Pop the newest path whose endpoint is close to the vehicle."""
        with self._lock:
            while self._history:
                candidate = self._history.pop()
                transforms = [p.transforms[0] for p in candidate.points
                              if p.transforms]
                if transforms and _distance(
                        current, transforms[-1].translation) <= \
                        self._endpoint_tolerance:
                    return candidate
        return None

    def _make_reverse(self, source, odom):
        output = MultiDOFJointTrajectory()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = source.header.frame_id
        self._recovery_seq += 1
        output.header.seq = 1000000 + self._recovery_seq
        output.joint_names = [RECOVERY_JOINT]

        current = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        elapsed = 0.0
        travelled = 0.0

        def append_point(position):
            point = MultiDOFJointTrajectoryPoint()
            transform = Transform()
            transform.translation.x = position.x
            transform.translation.y = position.y
            transform.translation.z = position.z
            transform.rotation = copy.deepcopy(orientation)
            point.transforms = [transform]
            point.time_from_start = rospy.Duration(elapsed)
            output.points.append(point)

        append_point(current)
        previous = copy.deepcopy(current)
        transforms = [point.transforms[0] for point in source.points
                      if point.transforms]
        for transform in reversed(transforms):
            segment = _distance(previous, transform.translation)
            if segment < 0.05:
                continue
            if travelled + segment > self._max_distance:
                ratio = (self._max_distance - travelled) / segment
                target = copy.deepcopy(previous)
                target.x += ratio * (transform.translation.x - previous.x)
                target.y += ratio * (transform.translation.y - previous.y)
                target.z += ratio * (transform.translation.z - previous.z)
                segment = _distance(previous, target)
            else:
                target = transform.translation
            elapsed += max(0.05, segment / self._speed)
            append_point(target)
            travelled += segment
            previous = copy.deepcopy(target)
            if travelled >= self._max_distance - 1.0e-6:
                break
        return output, travelled, elapsed

    def _recover(self):
        try:
            with self._lock:
                odom = copy.deepcopy(self._odom)
                failures = self._empty_count
            source = self._select_path(odom.pose.pose.position)
            if source is None:
                rospy.logerr(
                    "Stall recovery has no executed path ending near the vehicle")
                return
            trajectory, distance, duration = self._make_reverse(source, odom)
            if len(trajectory.points) < 2 or distance < 0.5:
                rospy.logerr("Stall recovery path is too short")
                return
            rospy.wait_for_service(
                "/planner_control_interface/std_srvs/stop", timeout=2.0)
            stopped = self._stop()
            if not stopped.success:
                rospy.logerr("PCI refused stall-recovery pause")
                return
            rospy.sleep(0.25)
            self._trajectory_pub.publish(trajectory)
            rospy.logwarn(
                "GBPlanner stall x%d: backtracking %.2fm over %.2fs on "
                "previously executed safe path", failures, distance, duration)
            rospy.sleep(duration + self._resume_margin)
            rospy.wait_for_service(
                "/planner_control_interface/std_srvs/automatic_planning",
                timeout=2.0)
            resumed = self._resume()
            if resumed.success:
                rospy.logwarn("GBPlanner automatic exploration resumed after backtrack")
            else:
                rospy.logerr("PCI refused automatic exploration resume")
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("GBPlanner stall recovery failed: %s", error)
        finally:
            with self._lock:
                self._empty_count = 0
                self._recovering = False


if __name__ == "__main__":
    rospy.init_node("gbplanner_stall_recovery")
    StallRecovery()
    rospy.spin()
