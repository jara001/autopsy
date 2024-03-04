#!/usr/bin/env python
# qos.py
"""Wrapper for QoS classes for ROS1.

This serves as a way to import QoSProfile and related classes without specific
ROS version.
"""
######################
# Imports & Globals
######################

from autopsy.core import ROS_VERSION

if ROS_VERSION == 1:
    from .ros1_qos import *

else: # ROS_VERSION == 2
    from rclpy.qos import *
