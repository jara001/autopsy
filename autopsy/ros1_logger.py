#!/usr/bin/env python
# ros1_logger.py
"""ROS2 Logger compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

try:
    import rospy
except:
    pass


######################
# Logger class
######################

class Logger(object):
    """Logger class to mimic ROS2 `Logger`.

    Reference:
    https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/logger.py
    """

    def debug(self, message, **kwargs):
        """Log a message with severity 'DEBUG'."""
        return rospy.logdebug(message, **kwargs)


    def info(self, message, **kwargs):
        """Log a message with severity 'INFO'."""
        return rospy.loginfo(message, **kwargs)


    def warning(self, message, **kwargs):
        """Log a message with severity 'WARN'."""
        return rospy.logwarn(message, **kwargs)


    def error(self, message, **kwargs):
        """Log a message with severity 'ERROR'."""
        return rospy.logerr(message, **kwargs)


    def fatal(self, message, **kwargs):
        """Log a message with severity 'FATAL'."""
        return rospy.logfatal(message, **kwargs)
