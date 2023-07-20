#!/usr/bin/env python
# ros1_clock.py
"""ROS2 Clock compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

try:
    import rospy
except:
    pass

from .ros1_time import Time


######################
# Clock class
######################

class Clock(object):
    """Clock class to mimic ROS2 `Clock`.

    Reference:
    https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/clock.py
    """

    def now(self):
        return Time(seconds = rospy.get_time())
