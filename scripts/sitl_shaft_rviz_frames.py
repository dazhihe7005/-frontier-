#!/usr/bin/env python3
"""Visualization-only TF for the Task-2 Gazebo model.

The simulated MID360-like ray cloud uses mid360_link while the SITL path uses
camera_init. This node bridges those frames from simulated PX4 odometry; it
does not publish flight commands or provide localization to PX4.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("sitl_shaft_rviz_frames")
    if not rospy.get_param("/use_sim_time", False):
        rospy.logfatal("RViz frame helper is only allowed in simulation")
        return

    dynamic_tf = tf2_ros.TransformBroadcaster()
    static_tf = tf2_ros.StaticTransformBroadcaster()
    mount = TransformStamped()
    mount.header.stamp = rospy.Time.now()
    mount.header.frame_id = "base_link"
    mount.child_frame_id = "mid360_link"
    mount.transform.translation.z = 0.14  # iris_mid360.sdf fixed-joint offset
    mount.transform.rotation.w = 1.0
    static_tf.sendTransform(mount)

    def on_odom(odom):
        if odom.header.frame_id != "camera_init":
            rospy.logwarn_throttle(5.0, "RViz TF ignored odom frame %s", odom.header.frame_id)
            return
        transform = TransformStamped()
        transform.header = odom.header
        transform.header.frame_id = "camera_init"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        dynamic_tf.sendTransform(transform)

    rospy.Subscriber("/Odometry", Odometry, on_odom, queue_size=20)
    rospy.spin()


if __name__ == "__main__":
    main()
