#!/usr/bin/env python
# ros1_qos.py
"""ROS2 QoSProfile compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

from enum import Enum


######################
# QoSDurabilityPolicy
######################

class DurabilityPolicy(Enum):
    """Enum to mimic ROS2 `DurabilityPolicy`.

    Reference:
    https://docs.ros2.org/foxy/api/rclpy/api/qos.html#rclpy.qos.DurabilityPolicy
    """

    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


######################
# QoSReliablityPolicy
######################

class ReliabilityPolicy(Enum):
    """Enum to mimic ROS2 `ReliabilityPolicy`.

    Reference:
    https://docs.ros2.org/foxy/api/rclpy/api/qos.html#rclpy.qos.ReliabilityPolicy
    """

    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2


######################
# QoSProfile class
######################

class QoSProfile(object):
    """QoSProfile class to mimic ROS2 `QoSProfile`.

    Reference:
    https://docs.ros2.org/foxy/api/rclpy/api/qos.html
    https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
    """

    def __init__(self, depth,
            durability = DurabilityPolicy.SYSTEM_DEFAULT,
            reliability = ReliabilityPolicy.SYSTEM_DEFAULT,
        **kwargs):
        """Initialize the class.

        Arguments:
        depth -- number of messages to hold (queue size), int
        durability -- how to treat messages for new subscribers, DurabilityPolicy
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/foxy/api/rclpy/api/qos.html#rclpy.qos.QoSProfile
        """
        self.depth = depth
        self.durability = durability
        self.reliability = reliability
