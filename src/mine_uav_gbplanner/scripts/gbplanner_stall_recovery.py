#!/usr/bin/env python3
"""Recover GBPlanner from a persistent empty-decision loop.

The upstream global planner can have no retained frontier at a dead end even
though unexplored branches exist behind the vehicle.  In that case repeatedly
calling the local planner cannot change either the map or its root state.  This
node pauses PCI, retraces dense *observed odometry* rather than an unexecuted
tail of a commanded trajectory, and resumes automatic exploration.

It deliberately does not use Gazebo ground truth. The reverse path still
requires the live obstacle guard because the environment or pose may change.
"""

import copy
import math
from pathlib import Path
import sys
import threading
from collections import deque

import rospy
from geometry_msgs.msg import Transform
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import (MultiDOFJointTrajectory,
                                 MultiDOFJointTrajectoryPoint)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from recovery_path_geometry import distance, reverse_observed_path


RECOVERY_JOINT = "gbplanner_stall_recovery_backtrack"


class StallRecovery:
    def __init__(self):
        self._empty_limit = max(4, int(rospy.get_param("~empty_limit", 15)))
        self._speed = min(0.6, max(
            0.15, float(rospy.get_param("~backtrack_speed", 0.45))))
        self._max_distance = min(8.0, max(
            2.0, float(rospy.get_param("~max_backtrack_distance", 6.0))))
        self._arrival_tolerance = max(
            0.15, float(rospy.get_param("~arrival_tolerance", 0.35)))
        self._arrival_stable_duration = max(
            0.25, float(rospy.get_param(
                "~arrival_stable_duration", 0.75)))
        self._resume_margin = max(
            0.5, float(rospy.get_param("~resume_margin", 1.0)))
        self._lock = threading.Lock()
        self._history = deque(maxlen=2000)
        self._odom = None
        self._vision_healthy = False
        self._mission_started = False
        self._state = State()
        self._empty_count = 0
        self._recovering = False
        self._recovery_seq = 0
        self._backtrack_published_at = rospy.Time(0)
        self._backtrack_blocked_since = rospy.Time(0)

        self._trajectory_pub = rospy.Publisher(
            "/gbplanner/command/trajectory", MultiDOFJointTrajectory,
            queue_size=2)
        self._active_pub = rospy.Publisher(
            "/mine_uav/gbplanner/recovery_active", Bool, queue_size=1,
            latch=True)
        self._active_pub.publish(Bool(data=False))
        self._stop = rospy.ServiceProxy(
            "/planner_control_interface/std_srvs/stop", Trigger)
        self._resume = rospy.ServiceProxy(
            "/planner_control_interface/std_srvs/automatic_planning", Trigger)
        rospy.Subscriber("/gbplanner_status", Bool, self._status_cb,
                         queue_size=50)
        rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                         self._odom_cb, queue_size=10)
        rospy.Subscriber("/mine_uav/gbplanner/vision_healthy", Bool,
                         self._vision_cb, queue_size=5)
        rospy.Subscriber("/mine_uav/gbplanner/execution_blocked", Bool,
                         self._blocked_cb, queue_size=2)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=5)
        rospy.Subscriber("/mine_uav/gbplanner/mission_status", String,
                         self._mission_cb, queue_size=2)

    def _odom_cb(self, message):
        with self._lock:
            self._odom = message
            if not (self._mission_started and self._vision_healthy and
                    self._state.armed and
                    self._state.mode == "OFFBOARD"):
                return
            position = message.pose.pose.position
            xyz = (position.x, position.y, position.z)
            if not all(math.isfinite(value) for value in xyz):
                self._history.clear()
                return
            if self._history:
                gap = distance(xyz, self._history[-1])
                if gap > 0.30:
                    # Never splice across estimator resets or stale samples.
                    self._history.clear()
                elif gap < 0.05:
                    return
            self._history.append(xyz)

    def _vision_cb(self, message):
        with self._lock:
            self._vision_healthy = bool(message.data)
            if not self._vision_healthy:
                self._history.clear()

    def _state_cb(self, message):
        with self._lock:
            self._state = message
            if not (message.connected and message.armed and
                    message.mode == "OFFBOARD"):
                self._history.clear()

    def _mission_cb(self, message):
        started = message.data == "AUTOMATIC_EXPLORATION_STARTED"
        with self._lock:
            if started != self._mission_started:
                self._history.clear()
            self._mission_started = started

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
                state_ready = (self._mission_started and self._vision_healthy and
                               self._state.connected and
                               self._state.armed and self._state.mode == "OFFBOARD")
                if self._odom is not None and state_ready and self._history:
                    self._recovering = True
                    start_worker = True
        if start_worker:
            threading.Thread(target=self._recover, daemon=True).start()

    def _blocked_cb(self, message):
        start_worker = False
        with self._lock:
            if self._recovering:
                # The path was collision-checked when first flown, not at
                # recovery time. Once the live guard blocks the new reverse
                # trajectory, do not wait tens of seconds at a wall.
                if not self._backtrack_published_at.is_zero():
                    if message.data:
                        if self._backtrack_blocked_since.is_zero():
                            self._backtrack_blocked_since = rospy.Time.now()
                    else:
                        self._backtrack_blocked_since = rospy.Time(0)
                return
            if message.data:
                ready = (self._mission_started and self._odom is not None and
                         self._vision_healthy and
                         self._state.connected and
                         self._state.armed and self._state.mode == "OFFBOARD")
                if ready:
                    self._recovering = True
                    start_worker = True
        if start_worker:
            rospy.logwarn("Live MID360 guard blocked execution; replanning")
            threading.Thread(target=self._recover, daemon=True).start()

    def _restart_without_motion(self, reason):
        rospy.wait_for_service(
            "/planner_control_interface/std_srvs/stop", timeout=2.0)
        stopped = self._stop()
        if not stopped.success:
            rospy.logerr("PCI refused blocked-path pause")
            return
        rospy.sleep(0.5)
        rospy.wait_for_service(
            "/planner_control_interface/std_srvs/automatic_planning",
            timeout=2.0)
        resumed = self._resume()
        if resumed.success:
            rospy.logwarn("GBPlanner replanned from actual pose: %s", reason)
        else:
            rospy.logerr("PCI refused blocked-path replan")

    def _make_reverse(self, positions, odom):
        output = MultiDOFJointTrajectory()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = odom.header.frame_id
        self._recovery_seq += 1
        output.header.seq = 1000000 + self._recovery_seq
        output.joint_names = [RECOVERY_JOINT]

        orientation = odom.pose.pose.orientation
        elapsed = 0.0
        travelled = 0.0

        def append_point(position):
            point = MultiDOFJointTrajectoryPoint()
            transform = Transform()
            transform.translation.x = position[0]
            transform.translation.y = position[1]
            transform.translation.z = position[2]
            transform.rotation = copy.deepcopy(orientation)
            point.transforms = [transform]
            point.time_from_start = rospy.Duration(elapsed)
            output.points.append(point)

        append_point(positions[0])
        previous = positions[0]
        for target in positions[1:]:
            segment = distance(previous, target)
            elapsed += max(0.05, segment / self._speed)
            append_point(target)
            travelled += segment
            previous = target
        return output, travelled, elapsed

    def _recover(self):
        try:
            with self._lock:
                odom = copy.deepcopy(self._odom)
                failures = self._empty_count
                history = list(self._history)
            if odom is None:
                self._restart_without_motion("no odometry for backtrack")
                return
            current = odom.pose.pose.position
            positions, _ = reverse_observed_path(
                (current.x, current.y, current.z), history,
                self._max_distance)
            trajectory, backtrack_distance, duration = self._make_reverse(
                positions, odom)
            if len(trajectory.points) < 2 or backtrack_distance < 0.5:
                self._restart_without_motion("no continuous observed backtrack")
                return
            rospy.wait_for_service(
                "/planner_control_interface/std_srvs/stop", timeout=2.0)
            stopped = self._stop()
            if not stopped.success:
                rospy.logerr("PCI refused stall-recovery pause")
                return
            rospy.sleep(0.25)
            self._active_pub.publish(Bool(data=True))
            rospy.sleep(0.10)
            with self._lock:
                self._backtrack_published_at = rospy.Time.now()
                self._backtrack_blocked_since = rospy.Time(0)
            self._trajectory_pub.publish(trajectory)
            rospy.logwarn(
                "GBPlanner stall x%d: backtracking %.2fm over %.2fs on "
                "recorded odometry path with live guard", failures,
                backtrack_distance, duration)
            target = trajectory.points[-1].transforms[0].translation
            # The live 1 m guard may deliberately pause a reverse trajectory
            # while it performs a vertical escape.  Two nominal trajectory
            # durations proved too short in that valid case, so retain a
            # bounded but generous completion window.  If it still expires,
            # resume planning from the measured pose instead of leaving PCI
            # paused indefinitely.
            deadline = rospy.Time.now() + rospy.Duration(
                max(duration*3.0, duration+20.0))
            arrived_since = rospy.Time(0)
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                with self._lock:
                    current_odom = copy.deepcopy(self._odom)
                    healthy = (self._vision_healthy and self._state.connected and
                               self._state.armed and self._state.mode == "OFFBOARD")
                    backtrack_blocked_since = self._backtrack_blocked_since
                if (not backtrack_blocked_since.is_zero() and
                        (rospy.Time.now()-backtrack_blocked_since).to_sec() >= 1.0):
                    rospy.logwarn("Live guard blocked the reverse path; "
                                  "aborting backtrack and replanning")
                    self._restart_without_motion("reverse path blocked")
                    return
                if current_odom is None or not healthy:
                    arrived_since = rospy.Time(0)
                else:
                    current_position = current_odom.pose.pose.position
                    error = distance(
                        (current_position.x, current_position.y,
                         current_position.z),
                        (target.x, target.y, target.z))
                    if error <= self._arrival_tolerance:
                        if arrived_since.is_zero():
                            arrived_since = rospy.Time.now()
                        elif (rospy.Time.now()-arrived_since).to_sec() >= \
                                self._arrival_stable_duration:
                            break
                    else:
                        arrived_since = rospy.Time(0)
                rate.sleep()
            else:
                if rospy.is_shutdown():
                    return
                rospy.logwarn(
                    "Stall recovery did not reach its endpoint; replanning "
                    "from the actual pose")
                self._restart_without_motion(
                    "backtrack endpoint timeout")
                return
            rospy.sleep(self._resume_margin)
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
            self._active_pub.publish(Bool(data=False))
            with self._lock:
                self._empty_count = 0
                self._recovering = False
                self._backtrack_published_at = rospy.Time(0)
                self._backtrack_blocked_since = rospy.Time(0)
                self._history.clear()
                if (self._odom is not None and self._mission_started and
                        self._vision_healthy and self._state.connected and
                        self._state.armed and self._state.mode == "OFFBOARD"):
                    p = self._odom.pose.pose.position
                    xyz = (p.x, p.y, p.z)
                    if all(math.isfinite(value) for value in xyz):
                        self._history.append(xyz)


if __name__ == "__main__":
    rospy.init_node("gbplanner_stall_recovery")
    StallRecovery()
    rospy.spin()
