#!/usr/bin/env python
# core.py
"""Universal core class with ROS 1 / ROS 2 support.

Universal core (or unicore) serves as a compatibility layer for running
a ROS node that is executable from both ROS versions.
"""
######################
# Imports & Globals
######################

# Figure out ROS version
try:
    import rospy

    ROS_VERSION = 1

    import autopsy.ros1_duration as duration
except:
    try:
        import rclpy

        ROS_VERSION = 2

        import rclpy.duration as duration
    except:
        print ("No ROS package detected.")
        ROS_VERSION = 0


######################
# Decorators
######################

def _ros_version_only(ros_version, f):
    """Decorator for disabling functions that are ROS version dependent.

    Using an optional argument a substitute function may be set.
    This function will be called instead of the original one,
    when ROS version is not available.
    """
    global ROS_VERSION

    func = None

    def substitute_or_block(*args, **kwargs):
        # Substitute
        if ( len(args) == 1 ) and ( callable(args[0]) ):

            def substitute(*args, **kwargs):
                return f ( *args, **kwargs )

            return substitute
        # Block

    def let_pass(*args, **kwargs):
        # Let pass function that is decorated without params
        if ( len(args) == 0 ) or ( not ( callable(args[0]) ) ):
            return f(*args, **kwargs)
        # Ignore parameters of a decorator and run original function instead
        else:
            #nonlocal func
            func = args[0]

            def pass_it(*args, **kwargs):
                return func(*args, **kwargs)

            return pass_it

    return let_pass if ROS_VERSION == ros_version else substitute_or_block


def ros1_only(f):
    """Decorator for enabling functions only with ROS1."""
    return _ros_version_only(1, f)


def ros2_only(f):
    """Decorator for enabling functions only with ROS2."""
    return _ros_version_only(2, f)


######################
# Universal Core functions
######################

class Core(object):
    """Universal core class that supports both ROS 1 and ROS 2."""

    duration = duration

    @ros2_only
    def init(self, args = None, context = None):
        """Initialize ROS communications for a given context.

        Arguments:
        args -- command line arguments, list
        context -- context to initialize, if None default is used

        Reference:
        https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.init
        """
        rclpy.init(args = args, context = context)


    def spin(self, node = None, executor = None):
        """Execute work and block until the node is shutdown.

        Arguments:
        node -- instance of Node to be checked for work
        executor -- executor to use, if None global is used

        Reference:
        https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.spin
        """
        if ROS_VERSION == 1:
            rospy.spin()
        elif ROS_VERSION == 2:
            rclpy.spin(node = node, executor = executor)


    @ros2_only(spin)
    def spin_once(self, node, executor = None, timeout_sec = None):
        """Execute one item of work or wait until timeout expires.

        Arguments:
        node -- instance of Node to be checked for work
        executor -- executor to use, if None global is used
        timeout_sec -- seconds to wait, if None or <0 block forever

        Reference:
        https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.spin_once

        Note:
        ROS1 (rospy) does not support this. Something similar can be
        achieved using `rospy.wait_for_message()`, but it is not the
        same. So we translate this into ordinary `rospy.spin()`.
        """
        rclpy.spin_once(node = node, executor = executor, timeout_sec = timeout_sec)


    @ros2_only(spin)
    def spin_until_future_complete(self, node, future, executor = None, timeout_sec = None):
        """Execute work until the future is complete.

        Arguments:
        node -- instance of Node to be checked for work
        future -- future object to wait on
        executor -- executor to use, if None global is used
        timeout_sec -- seconds to wait, if None or <0 block forever

        Reference:
        https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.spin_until_future_complete

        Note:
        ROS1 (rospy) does not support this, so we translate
        it into ordinary `rospy.spin()`.
        """
        rclpy.spin_until_future_complete(node = node, future = future, executor = executor, timeout_sec = timeout_sec)


    @ros2_only
    def shutdown(self, context = None):
        """Shutdown a previously initialized context and global executor.

        Arguments:
        context -- context to invalidate, if None default is used

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html#rclpy.shutdown
        """
        rclpy.shutdown(context = context)


# Workaround for 'unbound method' error in Python 2.
Core = Core()
