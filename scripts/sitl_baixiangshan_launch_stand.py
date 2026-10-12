#!/usr/bin/env python3
"""Simulation-only physical launch stand for Bai Xiang Shan SITL.

While PX4 is disarmed, apply a support wrench that balances gravity and
lightly damps pose error at the configured launch point. This preserves a
physical ~1 g IMU specific force, unlike teleporting the model. Release
permanently as soon as PX4 arms. Gazebo state never enters flight control,
localization, Frontier or SUPER.
"""
import math
import threading
import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench
from mavros_msgs.msg import State

def rpy_from_q(q):
    sinr = 2.0 * (q.w*q.x + q.y*q.z)
    cosr = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w*q.y - q.z*q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class LaunchStand:
    def __init__(self):
        self.link = rospy.get_param("~link", "iris_xy_vision::iris::base_link")
        self.x0 = float(rospy.get_param("~x", -7.0))
        self.y0 = float(rospy.get_param("~y", 0.5))
        self.z0 = float(rospy.get_param("~z", 1.1162))
        self.yaw0 = float(rospy.get_param("~yaw", 0.0))
        self.mass = float(rospy.get_param("~vehicle_mass", 6.0))
        self.kp_xy = float(rospy.get_param("~kp_xy", 18.0))
        self.kd_xy = float(rospy.get_param("~kd_xy", 14.0))
        self.kp_z = float(rospy.get_param("~kp_z", 30.0))
        self.kd_z = float(rospy.get_param("~kd_z", 20.0))
        self.kp_att = float(rospy.get_param("~kp_att", 4.0))
        self.kd_att = float(rospy.get_param("~kd_att", 1.5))
        self.released = False
        self.release_start = rospy.Time(0)
        self.release_ramp_duration = max(0.5, float(rospy.get_param("~release_ramp_duration", 3.0)))
        self.pose = None
        self.twist = None
        self.lock = threading.Lock()
        self.apply = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=5)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.on_links, queue_size=5)
        rospy.Timer(rospy.Duration(0.02), self.tick)
        rospy.logwarn("Bai Xiang Shan physical SITL launch stand active; simulation only")

    def on_state(self, msg):
        if msg.armed:
            with self.lock:
                if not self.released and self.release_start.is_zero():
                    self.release_start = rospy.Time.now()
                    rospy.logwarn("Bai Xiang Shan launch stand beginning %.2f s release ramp", self.release_ramp_duration)

    def on_links(self, msg):
        try:
            i = msg.name.index(self.link)
        except ValueError:
            return
        with self.lock:
            self.pose = msg.pose[i]
            self.twist = msg.twist[i]

    def tick(self, _event):
        with self.lock:
            if self.released or self.pose is None or self.twist is None:
                return
            p, v = self.pose, self.twist
            release_start = self.release_start
        support_scale = 1.0
        if not release_start.is_zero():
            elapsed = max(0.0, (rospy.Time.now() - release_start).to_sec())
            support_scale = max(0.0, 1.0 - elapsed / self.release_ramp_duration)
            if support_scale <= 0.0:
                with self.lock:
                    self.released = True
                rospy.logwarn("Bai Xiang Shan launch stand release ramp complete")
                return
        roll, pitch, yaw = rpy_from_q(p.orientation)
        w = Wrench()
        w.force.x = support_scale * (self.kp_xy * (self.x0 - p.position.x) - self.kd_xy * v.linear.x)
        w.force.y = support_scale * (self.kp_xy * (self.y0 - p.position.y) - self.kd_xy * v.linear.y)
        w.force.z = support_scale * (self.mass * 9.81 + self.kp_z * (self.z0 - p.position.z) - self.kd_z * v.linear.z)
        w.torque.x = support_scale * (-self.kp_att * roll - self.kd_att * v.angular.x)
        w.torque.y = support_scale * (-self.kp_att * pitch - self.kd_att * v.angular.y)
        w.torque.z = support_scale * (-self.kp_att * wrap(yaw - self.yaw0) - self.kd_att * v.angular.z)
        try:
            self.apply(
                body_name=self.link,
                reference_frame="world",
                reference_point=Point(),
                wrench=w,
                start_time=rospy.Time(0),
                duration=rospy.Duration(0.04),
            )
        except rospy.ServiceException:
            pass

if __name__ == "__main__":
    rospy.init_node("sitl_baixiangshan_launch_stand")
    LaunchStand()
    rospy.spin()
