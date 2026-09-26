#!/usr/bin/env python3
"""Inject one impossible FAST-LIO odometry sample in isolated SITL only.

Diagnostic fault injection. Never launch against a real MAVROS/PX4 master.
The genuine lidar stream continues; the vision bridge should fail closed and
the executor should latch a stationary target without using Gazebo truth.
"""

import copy
import os

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class Injector:
    def __init__(self):
        self.state = State()
        self.healthy = False
        self.odometry = None
        self.local_pose = None
        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=2)
        rospy.Subscriber("/mine_uav/gbplanner/vision_healthy", Bool,
                         self.health_cb, queue_size=2)
        rospy.Subscriber("/Odometry", Odometry, self.odom_cb, queue_size=2)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self.local_pose_cb, queue_size=2)
        self.publisher = rospy.Publisher("/Odometry", Odometry, queue_size=1)

    def state_cb(self, message):
        self.state = message

    def health_cb(self, message):
        self.healthy = bool(message.data)

    def odom_cb(self, message):
        self.odometry = message

    def local_pose_cb(self, message):
        self.local_pose = message

    def run(self):
        if os.environ.get("ROS_MASTER_URI") != "http://127.0.0.1:11331":
            raise RuntimeError("fault injection requires the isolated SITL ROS master")
        if rospy.get_param("/use_sim_time", False) is not True:
            raise RuntimeError("fault injection requires Gazebo simulation time")
        if rospy.get_param("/mine_uav/gbplanner/allow_real_fcu", False):
            raise RuntimeError("fault injection refused on real FCU")
        delta = float(rospy.get_param("~jump_m", 5.0))
        min_altitude = float(rospy.get_param("~min_local_altitude_m", 0.8))
        if delta < 3.0 or delta > 10.0:
            raise ValueError("jump_m must be in [3, 10] m")
        deadline = rospy.Time.now() + rospy.Duration(120.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if (self.state.connected and self.state.armed and
                    self.state.mode == "OFFBOARD" and self.healthy and
                    self.odometry is not None and
                    self.local_pose is not None and
                    self.local_pose.pose.position.z >= min_altitude and
                    self.publisher.get_num_connections() > 0):
                fault = copy.deepcopy(self.odometry)
                fault.header.stamp = rospy.Time.now()
                fault.pose.pose.position.x += delta
                self.publisher.publish(fault)
                rospy.logwarn("Injected one %.1f m odometry jump in isolated SITL",
                              delta)
                return
            rate.sleep()
        raise RuntimeError("SITL was not healthy and armed before timeout")


if __name__ == "__main__":
    rospy.init_node("inject_fastlio_jump_once", anonymous=True)
    Injector().run()
