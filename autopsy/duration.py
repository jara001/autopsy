#!/usr/bin/env python
# duration.py
"""Wrapper for Duration class for ROS1.

This serves as a way to import Duration class without specific ROS version.
"""
######################
# Imports & Globals
######################

from autopsy.core import ROS_VERSION

if ROS_VERSION == 1:
    from autopsy.ros1_duration import *

else: # ROS_VERSION == 2
    from rclpy.duration import *
