#!/usr/bin/env python
# node.py
"""Universal node class with ROS 1 / ROS 2 support.

Universal node (or uninode) serves as a compatibility layer for a ROS node
to be executable from both ROS versions. Basically, for ROS2 we use mostly
the original 'rclpy.node.Node', whereas for ROS1 we try to match its func-
tions onto this class.

This module contains class Node.

Relations:
-------------------------------------------------------------------------------------
|           ROS 1           |          uninode          |           ROS 2           |
|-----------------------------------------------------------------------------------|
| rospy.init_node           |                     self.__init__                     |
| rospy.get_name            |                     self.get_name                     |
| rospy.Publisher           | self.Publisher+           | self.create_publisher     |
| rospy.Subscriber          | self.Subscriber+          | self.create_subscription  |
| rospy.Rate                | self.Rate+                | self.create_rate          |
| rospy.Timer               | self.Timer+               | self.create_timer         |
| rospy.Service             | self.Service+             | self.create_service       |
-------------------------------------------------------------------------------------

Note: Lines with '+' denote that the same function as for ROS2 can be used for uninode.

Differences:
- ROS1
    - Subscriber in ROS1 can have None 'queue_size', setting it to the infinite value.
      However, this is not supported in ROS2. Therefore, in here, we set the default
      value to 10, as a default value in ROS2:
      https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-profiles
      This is not done for Publisher, as ROS1 writes a warning when passing None.
- ROS2
    - Timer in ROS2 does not take any arguments (in contrast to the 'rospy.TimerEvent
      in ROS1). Therefore, the function has to be created as:
      `def callback(self, *args, **kwargs)`


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
    from .ros1_qos import *

    ROS_VERSION = 1
except:
    try:
        from rclpy.node import Node as NodeI
        from rclpy.qos import *

        from .ros1_node import Node as NodeR1

        ROS_VERSION = 2
    except:
        print ("No ROS package detected.")
        ROS_VERSION = 0


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
        return super(Node, self).create_publisher(msg_type = data_class, topic = name, qos_profile = QoSProfile(depth = queue_size, durability = DurabilityPolicy.TRANSIENT_LOCAL if latch else DurabilityPolicy.VOLATILE))


    def Subscriber(self, name, data_class, callback=None, callback_args=None, queue_size=10, buff_size=65536, tcp_nodelay=False):
        """Create a subscriber. (ROS1 version)

        Arguments (only those that are used):
        name -- name of the topic to subscribe to, str
        data_class -- class of the ROS message
        callback -- function to be called upon receiving a message, Callable[msg_type]
        queue_size -- number of messages to be kept in queue, int

        Reference:
        http://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html
        """
        return super(Node, self).create_subscription(msg_type = data_class, topic = name, callback = callback, qos_profile = QoSProfile(depth = queue_size))


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


    def Service(self, name, service_class, handler, buff_size=65536, error_handler=None):
        """Create a service object.

        Arguments (only those that are used):
        name -- name of the service, str
        service_class -- class of the ROS service message
        handler -- function to be called upon receiving service request, Callable[service_class/ServiceRequest]

        Reference:
        http://docs.ros.org/en/kinetic/api/rospy/html/rospy.impl.tcpros_service.Service-class.html
        """
        return super(Node, self).create_service(srv_type = service_class, srv_name = name, callback = handler)


    def __getattr__(self, name):
        """Try to find undefined function in the current ROS environment.

        Arguments:
        name -- name of the attribute / method
        """

        if ROS_VERSION == 1:
            if hasattr(rospy, name):
                print ("[WARNING] Unsupported attribute 'rospy.%s'." % name)

            return getattr(rospy, name)
        else:
            # Raise AttributeError
            return super(Node, self).__getattribute__(name)


    def __getattribute__(self, name):
        """Get attribute and warn if not supported in uninode.

        Arguments:
        name -- name of the attribute / method
        """

        if ROS_VERSION == 2:
            if not hasattr(NodeR1, name):
                print ("[WARNING] Unsupported attribute 'self.%s'." % name)

        return super(Node, self).__getattribute__(name)
