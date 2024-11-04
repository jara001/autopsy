#!/usr/bin/env python
# ros1_node.py
"""ROS2 Node compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

import warnings

try:
    import rospy
except:
    pass

from .ros1_qos import *
from .ros1_time import Time
from .ros1_clock import Clock
from .ros1_logger import Logger
from .ros1_parameter import Parameter


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
        **kwargs -- other arguments passed to the rospy.init_node()

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node
        """
        rospy.init_node(name = name, **kwargs)

        # Part of workaround for Time.now()
        self.Time = Time


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
        return rospy.Publisher(name = topic, data_class = msg_type, tcp_nodelay = isinstance(qos_profile, QoSProfile) and qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT, latch = isinstance(qos_profile, QoSProfile) and qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL, queue_size = qos_profile.depth if isinstance(qos_profile, QoSProfile) else qos_profile)


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
        return rospy.Subscriber(name = topic, data_class = msg_type, callback = callback, queue_size = qos_profile.depth if isinstance(qos_profile, QoSProfile) else qos_profile, tcp_nodelay = isinstance(qos_profile, QoSProfile) and qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT)


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


    def create_client(self, srv_type, srv_name, **kwargs):
        """Create a service client.

        Arguments:
        srv_type -- class of the used ROS service message
        srv_name -- name of the service
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_client
        """
        return rospy.ServiceProxy(name = srv_name, service_class = srv_type)


    def declare_parameter(self, name, value = None, **kwargs):
        """Declare and initialize a ROS parameter.

        Arguments:
        name -- name of the parameter, including the namespace
        value -- value of the parameter
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.declare_parameter
        """
        if not rospy.has_param(name):
            rospy.set_param(name, value)

        return Parameter(name, value = value)


    def get_clock(self):
        """Get clock used by the node."""
        return Clock()


    def get_logger(self):
        """Mimic a logger object."""
        return Logger()


    def get_parameter(self, name):
        """Get a ROS parameter by its name.

        Arguments:
        name -- name of the parameter, including the namespace

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.declare_parameter
        """
        warnings.warn(
            "get_parameter(): autopsy does not fully support ROS2 "
            "parameters, for true compatibility use `autopsy.reconfigure` "
            "instead",
            RuntimeWarning
        )

        return Parameter(name, value = rospy.get_param(name))
