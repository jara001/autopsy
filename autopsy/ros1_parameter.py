#!/usr/bin/env python
# ros1_parameter.py
"""ROS2 Parameter compatible implementation for ROS1.
"""
######################
# Imports & Globals
######################

from enum import Enum


######################
# Parameter class
######################

class Parameter(object):
    """Parameter class to mimic ROS2 `Parameter`.

    Reference:
    https://docs.ros2.org/latest/api/rclpy/api/parameters.html
    https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/parameter.py
    """

    class Type(Enum):
        """Enum to mimic ROS2 `Parameter.Type`.

        Reference:
        hhttps://docs.ros2.org/latest/api/rclpy/api/parameters.html#rclpy.parameter.Parameter.Type
        """

        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4


    class ParameterValue(object):
        """ParameterValue class to mimic ROS2 `rcl_interfaces.ParameterValue`.

        Reference:
        https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/msg/ParameterValue.msg
        """

        def __init__(self, value):
            """Initialize object.

            Arguments:
            value -- value of the parameter

            Note: This is not the same as in ROS2! Instead we do
                  a lazy conversion here.
            """
            self._value = value

        #
        # Do lazy type conversion.

        @property
        def bool_value(self) -> bool:
            """Get the parameter value as bool."""
            return bool(self._value)


        @property
        def integer_value(self) -> int:
            """Get the parameter value as integer."""
            return int(self._value)


        @property
        def double_value(self) -> float:
            """Get the parameter value as float."""
            return float(self._value)


        @property
        def string_value(self) -> str:
            """Get the parameter value as string."""
            return str(self._value)


    def __init__(self, name, type_ = None, value = None):
        """Initialize the class.

        Arguments:
        name -- name of the parameter
        type_ -- type of the parameter, Parameter.Type, optional
        value -- value of the parameter, optional

        Reference:
        https://docs.ros2.org/latest/api/rclpy/api/parameters.html#rclpy.parameter.Parameter
        """
        self._type_ = type_
        self._name = name
        self._value = value


    def get_parameter_value(self) -> ParameterValue:
        """Obtain the parameter value as a separate object."""
        return Parameter.ParameterValue(self._value)
