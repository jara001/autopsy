#!/usr/bin/env python
# node.py
"""Universal node class with ROS 1 / ROS 2 support.

Universal node (or uninode) serves as a compatibility layer for a ROS node
to be executable from both ROS versions. Basically, for ROS2 we use mostly
the original 'rclpy.node.Node', whereas for ROS1 we try to match its func-
tions onto this class.

This module contains class Node.

Example:
```
from autopsy.node import Node
from std_msgs.msg import Int32

class MyNode(Node):

    def __init__(self, name):
        super(Node, self).__init__(name)

        self._subscriber = self.Subscriber("/topic", Int32, self.topic_callback)


    def topic_callback(self, msg):
        print ("Received:", msg.data)
```
"""
######################
# Imports & Globals
######################

# Figure out ROS version
try:
    import rospy

    from .ros1_node import Node as NodeI
except:
    try:
        from rclpy.node import Node as NodeI
    except:
        print ("No ROS package detected.")


######################
# Universal Node
######################

class Node(NodeI):
    """Universal Node that supports both ROS1 and ROS2."""

    def __init__(self, name):
        """Initialize the class and ROS node.

        Arguments:
        name -- name of the ROS node
        """
        super(Node, self).__init__(name)


    def Publisher(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None):
        """Create a publisher. (ROS1 version)

        Arguments (only those that are used):
        name -- name of the topic to publish to, str
        data_class -- class of the ROS message
        queue_size -- number of messages to be kept in queue, int

        Reference:
        http://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Publisher-class.html
        """
        return super(Node, self).create_publisher(msg_type = data_class, topic = name, qos_profile = queue_size)


    def Subscriber(self, name, data_class, callback=None, callback_args=None, queue_size=None, buff_size=65536, tcp_nodelay=False):
        """Create a subscriber. (ROS1 version)

        Arguments (only those that are used):
        name -- name of the topic to subscribe to, str
        data_class -- class of the ROS message
        callback -- function to be called upon receiving a message, Callable[msg_type]
        queue_size -- number of messages to be kept in queue, int

        Reference:
        http://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html
        """
        return super(Node, self).create_subscription(msg_type = data_class, topic = name, callback = callback, qos_profile = queue_size)


    def Rate(self, hz, reset=False):
        """Create a rate object for sleeping in a loop. (ROS1 version)

        Arguments (only those that are used):
        hz -- frequency to determine sleeping, int

        Reference:
        http://docs.ros.org/en/kinetic/api/rospy/html/rospy.timer.Rate-class.html
        """
        return super(Node, self).create_rate(frequency = hz)


    def Timer(self, period, callback, oneshot=False, reset=False):
        """Create a timer object to execute function periodically.

        Arguments (only those that are used):
        period -- period of the timer, secs, int
        callback -- function to be called, Callable

        Reference:
        http://docs.ros.org/en/kinetic/api/rospy/html/rospy.timer.Timer-class.html
        """
        return super(Node, self).create_timer(timer_period_sec = period, callback = callback)
