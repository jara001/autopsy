# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
### Added
- `reconfigure`:
  - Property `callback` that runs associated function on parameter change (dynamic change only).
  - Function `reconfigure()` can be called with an optional parameter to set the namespace of the `ParameterServer`.
  - Operators for the parameters (at least most of them).
  - Function `reconfigure()` can be passed node object to support (hopefully) ROS2.
- `uninode`:
  - `QoSProfile` compatible implementation for ROS1.
  - Support for latched publishers.
  - ROS services.
  - Function `get_name()` for ROS1 version of the module.
- Documentation for the package.

### Changed
- `reconfigure`:
  - Parameters are now kept in the same order as registered.

### Fixed
- `reconfigure`:
  - Internal class variables are properly initialized so the data are not shared between instances.
  - Bool parameter is now correctly registered as boolean instead of integer.

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
