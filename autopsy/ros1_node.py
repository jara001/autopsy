#!/usr/bin/env python
# ros1_node.py
"""ROS2 Node compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

try:
    import rospy
except:
    pass

from .ros1_qos import DurabilityPolicy
from .ros1_clock import Clock
from .ros1_logger import Logger


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


    def get_name(self):
        """Obtain the name of the node.

        Reference:
        http://docs.ros.org/en/kinetic/api/rospy/html/rospy-module.html#get_name
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.get_name
        """
        return rospy.get_name()


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
        return rospy.Publisher(name = topic, data_class = msg_type, latch = qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL, queue_size = qos_profile.depth)


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
        return rospy.Subscriber(name = topic, data_class = msg_type, callback = callback, queue_size = qos_profile.depth)


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


    def create_service(self, srv_type, srv_name, callback, **kwargs):
        """Create a service object.

        Arguments:
        srv_type -- class of the used ROS service message
        srv_name -- name of the service
        callback -- function to be called upon receiving a service request, Callable[srv_type/ServiceRequest]
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_service
        """
        return rospy.Service(name = srv_name, service_class = srv_type, handler = callback)


    def get_clock(self):
        """Get clock used by the node."""
        return Clock()


    def get_logger(self):
        """Mimic a logger object."""
        return Logger()
