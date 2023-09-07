#!/usr/bin/env python
# ros1_time.py
"""ROS2 Time compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

from std_msgs.msg import Time as Time_msg
from autopsy.core import ROS_VERSION


CONVERSION_CONSTANT = 10 ** 9


######################
# Time class
######################

class Time(object):
    """Time class to mimic ROS2 `Time`.

    Reference:
    https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/time.py
    """

    def __init__(self, seconds = 0, nanoseconds = 0, **kwargs):
        """Initialize the class.

        Arguments:
        seconds -- number of seconds, float
        nanoseconds -- number of nanoseconds, int
        **kwargs -- other, currently unsupported arguments

        Reference:
        https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/time.py
        """
        self.total_nanoseconds = int(seconds * CONVERSION_CONSTANT) + int(nanoseconds)
        self.clock_type = None


    @property
    def nanoseconds(self):
        return self.total_nanoseconds


    def seconds_nanoseconds(self):
        """Return the Time as two separate values, secs and nsecs.

        Returns:
        (secs, nsecs) -- 2-tuple of ints
        """
        return (self.total_nanoseconds // CONVERSION_CONSTANT, self.total_nanoseconds % CONVERSION_CONSTANT)


    def to_msg(self):
        """Convert the object into ROS message.

        VERY IMPORTANT Note:
        This function is a trick. With ROS2 it behaves as expected,
        however in ROS1 it does not return the message, but the
        inner structure instead. This was done in order to be
        able to use this for Header stamps.
        In ROS2 the Header contains message type 'Time', however
        in ROS1 stamp is contained in its own data type 'time'.
        """
        msg = Time_msg()
        msg.data.secs, msg.data.nsecs = self.seconds_nanoseconds()
        return msg if ROS_VERSION == 2 else msg.data


    @classmethod
    def from_msg(cls, msg, **kwargs):
        """Obtain Time from a message."""
        return cls(seconds = msg.data.secs, nanoseconds = msg.data.nsecs)


    ## Operators ##
    # Source: https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/time.py
    def __repr__(self):
        return 'Time(nanoseconds=%s, clock_type=%s)' % (
            self.nanoseconds, self.clock_type)

    def __add__(self, other):
        return NotImplemented

    def __sub__(self, other):
        return NotImplemented

    def __eq__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds == other.nanoseconds
        # Raise instead of returning NotImplemented to prevent comparison with invalid types,
        # e.g. ints.
        # Otherwise `Time(nanoseconds=5) == 5` will return False instead of raising, and this
        # could lead to hard-to-find bugs.
        raise TypeError("Can't compare time with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds < other.nanoseconds
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds <= other.nanoseconds
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds > other.nanoseconds
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds >= other.nanoseconds
        return NotImplemented
