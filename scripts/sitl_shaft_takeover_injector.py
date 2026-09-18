#!/usr/bin/env python3

"""SITL-only external mode change at a chosen shaft depth.

`SetMode.mode_sent` only means the command was sent. Confirmation requires a
subsequent /mavros/state update showing the requested mode.
"""

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from std_msgs.msg import Float64, String


class SitlShaftTakeoverInjector:
    def __init__(self):
        self.trigger_depth = float(rospy.get_param("~trigger_depth", 6.0))
        self.target_mode = rospy.get_param("~target_mode", "AUTO.LOITER")
        self.confirm_timeout = float(rospy.get_param("~confirm_timeout", 3.0))
        if self.trigger_depth < 0 or self.confirm_timeout <= 0:
            raise ValueError("trigger_depth and confirm_timeout must be positive")
        self.depth = None
        self.shaft_state = "IDLE"
        self.px4_mode = ""
        self.request_time = None
        self.terminal = False
        self.status_pub = rospy.Publisher(
            "/mine_uav/sitl/shaft_takeover_status", String,
            queue_size=1, latch=True,
        )
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.Subscriber("/mine_uav/shaft/relative_depth_m", Float64,
                         self.on_depth, queue_size=5)
        rospy.Subscriber("/mine_uav/shaft/status", String,
                         self.on_shaft_state, queue_size=5)
        rospy.Subscriber("/mavros/state", State,
                         self.on_px4_state, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self.tick)
        self.publish("WAIT_DEPTH")
        rospy.logwarn("SITL-only shaft takeover injector; never connect to a real FCU")

    def publish(self, status):
        self.status_pub.publish(String(data=status))

    def on_depth(self, message):
        self.depth = message.data

    def on_shaft_state(self, message):
        self.shaft_state = message.data

    def on_px4_state(self, message):
        self.px4_mode = message.mode
        if self.request_time is not None and not self.terminal and \
                message.mode == self.target_mode:
            self.terminal = True
            self.publish("CONFIRMED:" + self.target_mode)
            rospy.loginfo("SITL external mode change confirmed: %s", self.target_mode)

    def tick(self, _event):
        if not rospy.get_param("/use_sim_time", False):
            rospy.logerr_throttle(2.0, "SITL takeover requires /use_sim_time")
            return
        if rospy.get_param("/mavros/fcu_url", "") != \
                "udp://:14540@localhost:14557":
            rospy.logerr_throttle(2.0, "SITL takeover requires the local PX4 UDP FCU URL")
            return
        if self.terminal:
            return
        now = rospy.Time.now()
        if self.request_time is not None:
            if (now - self.request_time).to_sec() >= self.confirm_timeout:
                self.terminal = True
                self.publish("NOT_CONFIRMED:" + self.target_mode)
                rospy.logerr("PX4 did not enter requested mode %s", self.target_mode)
            return
        if (self.depth is None or self.depth < self.trigger_depth or
                self.shaft_state != "DESCENDING" or self.px4_mode != "OFFBOARD"):
            return
        self.request_time = now
        self.publish("SENT_WAIT_CONFIRM:" + self.target_mode)
        try:
            response = self.mode_client(custom_mode=self.target_mode)
        except rospy.ServiceException as error:
            self.terminal = True
            self.publish("SERVICE_ERROR")
            rospy.logerr("SITL mode request failed: %s", error)
            return
        if not response.mode_sent:
            self.terminal = True
            self.publish("NOT_SENT:" + self.target_mode)
            return
        rospy.logwarn("SITL mode command sent at depth %.2f m; awaiting PX4 state",
                      self.depth)


if __name__ == "__main__":
    rospy.init_node("sitl_shaft_takeover_injector")
    SitlShaftTakeoverInjector()
    rospy.spin()
