# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
### Fixed
- `uninode`: Default value of `queue_size` for Subscriber is set to 10.

## 0.4.0 - 2022-03-08
### Added
- `uninode` utility to create a compatibility layer for ROS1-ROS2 nodes.
- `release.sh` to automatically create new release (from `ng_trajectory`).
- `Makefile` for fast working with the package (from `ng_trajectory`).
- Optional dependency on `rclpy`.
- `package.xml`, `CMakeLists.txt`, `resource/` and `setup.cfg` to be able to build the package inside the ROS workspace.
- License file.

### Changed
- `rospy` dependency is optional.

## 0.3.0 - 2021-06-29
### Added
- Support for enumerated values in `reconfigure`.

## 0.2.0 - 2021-06-28
### Changed
- Completely rewritten `reconfigure` utility. Now it is much simpler to use, but it is not directly backwards compatible.

## 0.1.0 - 2021-04-08
### Added
- `reconfigure` utility to expand and enhance usability of dynamic reconfigure in Python ROS nodes.
