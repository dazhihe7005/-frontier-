#!/usr/bin/env python3
"""Start GBPlanner automatic exploration after the SITL takeoff handoff."""

import threading

import rospy
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MissionTrigger:
    def __init__(self):
        self.delay = max(1.0, float(rospy.get_param("~map_warmup", 4.0)))
        self.retry = max(1.0, float(rospy.get_param("~retry_interval", 3.0)))
        self.channel = max(0, int(rospy.get_param("~rc_channel_index", 6)))
        self.threshold = int(rospy.get_param("~rc_threshold", 1700))
        self._lock = threading.Lock()
        self._enabled_since = rospy.Time(0)
        self._last_call = rospy.Time(0)
        self._vision = False
        self._state = State()
        self._odom_time = rospy.Time(0)
        self._started = False
        self._status_pub = rospy.Publisher(
            "/mine_uav/gbplanner/mission_status", String, queue_size=1,
            latch=True)
        rospy.Subscriber("/mine_uav/sitl/rc/in", RCIn, self._rc_cb, queue_size=2)
        rospy.Subscriber("/mine_uav/gbplanner/vision_healthy", Bool,
                         self._vision_cb, queue_size=2)
        rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                         self._odom_cb, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        self._client = rospy.ServiceProxy(
            "/planner_control_interface/std_srvs/automatic_planning", Trigger)
        rospy.Timer(rospy.Duration(0.5), self._timer)
        self._publish("WAIT_TAKEOFF")

    def _rc_cb(self, message):
        high = len(message.channels) > self.channel and \
            message.channels[self.channel] >= self.threshold
        with self._lock:
            if high and self._enabled_since.is_zero():
                self._enabled_since = rospy.Time.now()
            elif not high:
                self._enabled_since = rospy.Time(0)

    def _vision_cb(self, message):
        with self._lock:
            self._vision = message.data

    def _odom_cb(self, _message):
        with self._lock:
            self._odom_time = rospy.Time.now()

    def _state_cb(self, message):
        with self._lock:
            self._state = message

    def _timer(self, _event):
        now = rospy.Time.now()
        with self._lock:
            enabled_since = self._enabled_since
            vision = self._vision
            state = self._state
            odom_time = self._odom_time
            started = self._started
        if started:
            return
        if enabled_since.is_zero():
            self._publish("WAIT_TAKEOFF")
            return
        if not vision or odom_time.is_zero() or (now-odom_time).to_sec() > 0.5:
            self._publish("WAIT_FASTLIO")
            return
        if not state.connected or not state.armed or state.mode != "OFFBOARD":
            self._publish("WAIT_PX4_OFFBOARD")
            return
        if (now-enabled_since).to_sec() < self.delay:
            self._publish("MAP_WARMUP_AFTER_TAKEOFF")
            return
        if not self._last_call.is_zero() and (now-self._last_call).to_sec() < self.retry:
            return
        self._last_call = now
        try:
            response = self._client()
            if response.success:
                with self._lock:
                    self._started = True
                self._publish("AUTOMATIC_EXPLORATION_STARTED")
                rospy.loginfo("GBPlanner2 automatic exploration started: %s",
                              response.message)
            else:
                self._publish("PLANNER_REJECTED:" + response.message)
        except rospy.ServiceException as error:
            self._publish("PLANNER_SERVICE_ERROR")
            rospy.logwarn_throttle(3.0, "GBPlanner trigger failed: %s", error)

    def _publish(self, text):
        self._status_pub.publish(String(data=text))


if __name__ == "__main__":
    rospy.init_node("gbplanner_mission_trigger")
    MissionTrigger()
    rospy.spin()
