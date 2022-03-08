#!/usr/bin/env python
# ros1_node.py
"""ROS2 Node compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

import rospy


######################
# Node class
######################

class Node(object):
    """Node class to mimic ROS2 `Node`.

    Reference:
    https://docs.ros2.org/latest/api/rclpy/api/node.html
    """

    def __init__(self, name, **kwargs):
        """Initialize the class and ROS node.

        Arguments:
        name -- name of the ROS node, str
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node
        """
        rospy.init_node(name = name)


    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        """Create a publisher.

        Arguments:
        msg_type -- class of the ROS message
        topic -- name of the topic to publish onto, str
        qos_profile -- number of messages to be kept in a queue, int
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_publisher
        """
        return rospy.Publisher(name = topic, data_class = msg_type, queue_size = qos_profile)


    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        """Create a subscriber.

        Arguments:
        msg_type -- class of the used ROS message
        topic -- name of the topic to subscribe to, str
        callback -- function to be called upon receiving a message, Callable[msg_type]
        qos_profile -- number of messages to be kept in a queue, int
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_subscription
        """
        return rospy.Subscriber(name = topic, data_class = msg_type, callback = callback, queue_size = qos_profile)


    def create_rate(self, frequency, **kwargs):
        """Create a rate object for sleeping in a loop.

        Arguments:
        frequency -- frequency to determine sleeping, float
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_rate
        """
        return rospy.Rate(hz = int(frequency))


    def create_timer(self, timer_period_sec, callback, **kwargs):
        """Create a timer object to execute function periodically.

        Arguments:
        timer_period_sec -- period of the timer, secs, int
        callback -- function to be called, Callable
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_timer

        Note:
        Callback in ROS1 takes an argument 'rospy.TimerEvent', however not in ROS2.
        """
        return rospy.Timer(period = rospy.Duration(timer_period_sec), callback = callback)
