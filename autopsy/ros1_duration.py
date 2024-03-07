#!/usr/bin/env python
# duration.py
"""ROS2 Duration compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

from rospy import Duration as DurationR1

CONVERSION_CONSTANT = 10 ** 9


######################
# Duration class
######################

class Duration(object):
    """Duration class to mimic ROS2 `Duration`.

    Reference:
    https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/duration.py
    """

    def __init__(self, seconds=0, nanoseconds=0):
        """Initialize the class.

        Arguments:
        seconds -- number of seconds, float
        nanoseconds -- number of nanoseconds, int

        Reference:
        https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/duration.py
        """
        self.total_nanoseconds = int(seconds * CONVERSION_CONSTANT)
        self.total_nanoseconds += int(nanoseconds)

        if self.total_nanoseconds >= 2**63 or self.total_nanoseconds < -2**63:
            # pybind11 would raise TypeError, but we want OverflowError
            # We keep it here for some ROS version consistency.
            raise OverflowError(
                'Total nanoseconds value is too large to store in C duration.'
            )

    @property
    def nanoseconds(self):
        return self.total_nanoseconds


    def to_msg(self):
        """Convert the object into ROS message."""
        return DurationR1(
            secs = self.nanoseconds // CONVERSION_CONSTANT,
            nsecs = self.nanoseconds % CONVERSION_CONSTANT
        )


    @classmethod
    def from_msg(cls, msg, **kwargs):
        """Obtain Duration from a message.

        TODO: Not sure how this one translates to ROS 1.
        """
        return NotImplemented


    def get_c_duration(self):
        return NotImplemented


    ## Operators ##
    # Source: https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/duration.py
    def __repr__(self):
        return 'Duration(nanoseconds=%d)' % self.nanoseconds

    def __str__(self):
        if self == Infinite:
            return 'Infinite'
        return '%d nanoseconds' % self.nanoseconds

    def __eq__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds == other.nanoseconds
        # Raise instead of returning NotImplemented to prevent comparison with invalid types,
        # e.g. ints.
        # Otherwise `Duration(nanoseconds=5) == 5` will return False instead of raising, and this
        # could lead to hard-to-find bugs.
        raise TypeError("Can't compare duration with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds < other.nanoseconds
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds <= other.nanoseconds
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds > other.nanoseconds
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds >= other.nanoseconds
        return NotImplemented


Infinite = Duration(seconds = -1)
