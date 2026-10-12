#!/usr/bin/env python3
"""Forward valid task-one SITL ground ranges to MAVROS.

Gazebo's ray plugin reports its maximum range as a finite Range message when
there is no return. PX4 otherwise learns this as a 30 m height bias before
takeoff and rejects the first actual ground return as a sensor fault.
"""

import math

import rospy
from sensor_msgs.msg import Range


def valid_ground_range(message, max_valid_range):
    """A maximum-range Gazebo ray is no return, not a height measurement."""
    return (
        math.isfinite(message.range)
        and message.min_range <= message.range
        and message.range < min(message.max_range - 0.05, max_valid_range)
    )


class DownwardRangeFilter:
    def __init__(self):
        self.max_valid_range = rospy.get_param("~max_valid_range", 5.0)
        source = rospy.get_param(
            "~input_topic", "/mine_uav/sitl/shaft_downward_range"
        )
        output = rospy.get_param(
            "~output_topic", "/mavros/distance_sensor/sonar_1_sub"
        )
        self.publisher = rospy.Publisher(output, Range, queue_size=10)
        self.subscriber = rospy.Subscriber(
            source, Range, self.on_range, queue_size=10
        )

    def on_range(self, message):
        if valid_ground_range(message, self.max_valid_range):
            self.publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("task1_sitl_downward_range_filter")
    DownwardRangeFilter()
    rospy.spin()
