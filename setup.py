#!/usr/bin/env python
# -*- coding: utf-8 -*-
# setup.py
"""Install script for this package."""

import os
from setuptools import setup, find_packages

try:
    from _version import Version

    # Utility function to read the README file.
    # Used for the long_description.  It's nice, because now 1) we have a top level
    # README file and 2) it's easier to type in the README file than to put a raw
    # string in below ...
    #def read(fname):
    #    return open(os.path.join(os.path.dirname(__file__), fname)).read()

    VERSION = str(Version(os.popen("git describe --tags --dirty --always").read()[1:-1]))

    if os.path.exists("VERSION"):
        STORED = open("VERSION", "r").read()

        if ".dev" in VERSION:
            _len = len(VERSION[:VERSION.index(".dev")+4])
        else:
            _len = len(VERSION)

        if STORED[:_len] == VERSION[:_len] and ".dev" in VERSION:
            # Obtain dev number
            VERSION = VERSION + str(int(STORED[_len:]) + 1)
        else:
            if ".dev" in VERSION:
                VERSION = VERSION + "0"
    else:
        if ".dev" in VERSION:
            VERSION = VERSION + "0"


    with open("VERSION", "w") as file:
        file.write(VERSION)


    # Also store the version to be seen from the code
    with open("./autopsy/version.py", "w") as file:
        file.write("__version__ = '%s'" % VERSION)
except:
    # When not building the wheel use the package version
    VERSION=str(os.popen("grep '<version>' package.xml | grep -o '[0-9\.]*'").read()[:-1])

    with open("./autopsy/version.py", "w") as file:
        file.write("__version__ = '%s'" % VERSION)


setup(
    name = "autopsy",
    version = VERSION,
    author = "Jaroslav Klapálek",
    author_email = "klapajar@fel.cvut.cz",
    description = ("A set of Python utils for F1Tenth project."),
    license = "GPLv3",
    keywords = "f1tenth autonomous car ros robot",
    #url = "http://packages.python.org/an_example_pypi_project",
    packages=find_packages(),
    #long_description=read('README'),
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Programming Language :: Python",
        "Topic :: Scientific/Engineering",
    ],
    extras_require={
        "ros1": "rospy",
        "ros2": "rclpy",
    },
)
