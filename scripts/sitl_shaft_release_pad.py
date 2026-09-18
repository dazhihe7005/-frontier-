#!/usr/bin/env python3

"""Delete only the named temporary Gazebo entrance pad after SITL takeoff."""

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel
from mavros_msgs.msg import State
from std_msgs.msg import Bool


class SitlShaftReleasePad:
    def __init__(self):
        self.vehicle_model = rospy.get_param("~vehicle_model", "iris")
        self.enabled = False
        self.armed = False
        self.drone_z = None
        self.released = False
        self.pad_name = "shaft_launch_pad"
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.Subscriber("/mine_uav/mission/shaft_enable", Bool, self.on_enable)
        rospy.Subscriber("/mavros/state", State, self.on_state)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_models)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def on_enable(self, message):
        self.enabled = message.data

    def on_state(self, message):
        self.armed = message.armed

    def on_models(self, message):
        try:
            self.drone_z = message.pose[message.name.index(self.vehicle_model)].position.z
        except ValueError:
            self.drone_z = None

    def tick(self, _event):
        if self.released or not self.enabled:
            return
        if not rospy.get_param("/use_sim_time", False):
            rospy.logerr_throttle(2.0, "SITL pad release requires /use_sim_time")
            return
        if not self.armed or self.drone_z is None or self.drone_z < 1.0:
            return
        try:
            response = self.delete_model(self.pad_name)
            if response.success:
                self.released = True
                rospy.logwarn("SITL shaft entrance pad released at drone z=%.2f",
                              self.drone_z)
            else:
                rospy.logerr("SITL shaft pad deletion rejected: %s", response.status_message)
                rospy.signal_shutdown("SITL pad release failed")
        except rospy.ServiceException as error:
            rospy.logerr("SITL shaft pad deletion failed: %s", error)
            rospy.signal_shutdown("SITL pad release failed")


if __name__ == "__main__":
    rospy.init_node("sitl_shaft_release_pad")
    SitlShaftReleasePad()
    rospy.spin()
