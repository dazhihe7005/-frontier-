#!/usr/bin/env python3

"""Opt-in PX4 SITL baro+GPS OFF injection while shaft Task 2 descends.

This is a destructive *simulation fault* and must never connect to a real FCU.
It only runs with simulated time and the launch's exact localhost UDP URL.
"""

import os
from urllib.parse import urlparse

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, ParamGet, ParamPull, ParamSet
from mavros_msgs.msg import ParamValue
from std_msgs.msg import Float64, String


class SitlPx4ZFailureInjector:
    FCU_URL = "udp://:14540@localhost:14557"

    def __init__(self):
        self.trigger_depth = float(rospy.get_param("~trigger_depth", 6.0))
        if self.trigger_depth <= 0:
            raise ValueError("trigger_depth must be positive")
        self.state = State()
        self.depth = None
        self.shaft_status = "IDLE"
        self.prepared = False
        self.injected = False
        self.terminal = False
        self.last_setup_attempt = rospy.Time(0)
        self.status_pub = rospy.Publisher(
            "/mine_uav/sitl/px4_z_failure_status", String,
            queue_size=1, latch=True)
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=5)
        rospy.Subscriber("/mine_uav/shaft/relative_depth_m", Float64,
                         self.on_depth, queue_size=5)
        rospy.Subscriber("/mine_uav/shaft/status", String,
                         self.on_shaft_status, queue_size=5)
        self.pull = rospy.ServiceProxy("/mavros/param/pull", ParamPull)
        self.get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
        self.set = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self.command = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        rospy.Timer(rospy.Duration(0.2), self.tick)
        self.publish("WAIT_SITL")

    def publish(self, value):
        self.status_pub.publish(String(data=value))

    def on_state(self, message):
        self.state = message

    def on_depth(self, message):
        self.depth = message.data

    def on_shaft_status(self, message):
        self.shaft_status = message.data

    def sitl_only(self):
        try:
            master_port = urlparse(os.environ.get("ROS_MASTER_URI", "")).port
        except ValueError:
            return False
        return (master_port == 11319 and
                rospy.get_param("/use_sim_time", False) and
                rospy.get_param("/mavros/fcu_url", "") == self.FCU_URL)

    def setup(self):
        try:
            pulled = self.pull(force_pull=True)
            if not pulled.success:
                return
            current = self.get(param_id="SYS_FAILURE_EN")
            if not current.success:
                return
            if current.value.integer != 0 and not self.set_permission(0):
                self.publish("STALE_FAILURE_PERMISSION_RESET_FAILED")
                return
            self.prepared = True
            self.publish("SITL_FAILURE_INJECTION_PREPARED")
            rospy.logwarn("PX4 SITL failure injection prepared; permission remains OFF")
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "SITL failure setup retry: %s", error)

    def set_permission(self, value):
        response = self.set(
            param_id="SYS_FAILURE_EN",
            value=ParamValue(integer=value, real=0.0))
        confirmed = self.get(param_id="SYS_FAILURE_EN")
        return (response.success and confirmed.success and
                confirmed.value.integer == value)

    def inject(self):
        self.terminal = True  # Never issue two sets of failure commands.
        permission_attempted = False
        try:
            permission_attempted = True
            if not self.set_permission(1):
                self.publish("FAILURE_PERMISSION_ENABLE_FAILED")
                return
            self.publish("SITL_FAILURE_INJECTION_ENABLED")
            for unit, label in ((3, "BARO"), (4, "GPS")):
                response = self.command(
                    broadcast=False, command=420, confirmation=0,
                    param1=float(unit), param2=1.0, param3=0.0,
                    param4=0.0, param5=0.0, param6=0.0, param7=0.0)
                if not response.success or response.result != 0:
                    self.publish("FAILURE_COMMAND_REJECTED:" + label)
                    return
            self.injected = True
            self.publish("INJECTED_BARO_GPS_OFF")
            rospy.logwarn("PX4 SITL baro and GPS OFF injected at %.2f m",
                          self.depth)
        except rospy.ServiceException as error:
            self.publish("INJECTION_OR_PARAM_RESET_SERVICE_ERROR")
            rospy.logerr("PX4 SITL failure injection service error: %s", error)
        finally:
            # Permission is enabled only in this short section. Reset even
            # when a failure command is rejected or its service raises.
            if permission_attempted:
                try:
                    disabled = self.set_permission(0)
                except rospy.ServiceException:
                    disabled = False
                if disabled:
                    suffix = ("INJECTED_BARO_GPS_OFF_PERMISSION_DISABLED"
                              if self.injected else "FAILURE_ABORTED_PERMISSION_DISABLED")
                    self.publish(suffix)
                else:
                    self.publish("FAILURE_PARAM_RESET_FAILED")
                    rospy.logerr("SITL SYS_FAILURE_EN reset failed; reset it manually")

    def tick(self, _event):
        if self.terminal or self.injected:
            return
        if not self.sitl_only():
            rospy.logerr_throttle(2.0, "PX4 failure injector requires local UDP SITL")
            return
        if not self.state.connected:
            return
        now = rospy.Time.now()
        if not self.prepared:
            if self.state.armed:
                self.publish("MISSED_DISARMED_SETUP")
                self.terminal = True
                return
            if self.last_setup_attempt.is_zero() or \
                    (now - self.last_setup_attempt).to_sec() >= 1.0:
                self.last_setup_attempt = now
                self.setup()
            return
        if (self.state.armed and self.state.mode == "OFFBOARD" and
                self.shaft_status == "DESCENDING" and self.depth is not None and
                self.depth >= self.trigger_depth):
            self.inject()


if __name__ == "__main__":
    rospy.init_node("sitl_shaft_px4_z_failure_injector")
    SitlPx4ZFailureInjector()
    rospy.spin()
