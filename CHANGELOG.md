# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
### Added
- `unicore`:
  - New compatibility layer to cover node spinning and initialization.
  - Object `Core`:
    - Functions: `spin()` and `init()`.
  - Decorators: `@ros1_only` and `@ros2_only` with an optional argument to use different function instead.

### Changed
- `uninode`:
  - `ROS_VERSION` is determined inside `unicore`.

## 0.8.1 - 2023-08-08
### Fixed
- `uninode`:
  - AttributeError caused by not importing Time object for ROS1.

## 0.8.0 - 2023-07-20
### Added
- `reconfigure`:
  - Namespace is now used for recognizing whether the parameter being updated belongs to the current ParameterList. (This is required for ROS2.)
- `uninode`:
  - Functions for obtaining current ROS time: `get_time()`, `Time.now()`, and `get_clock().now()`.
  - Logging support using `log*` and `get_logger().*` functions.

### Changed
- `reconfigure`:
  - [BREAKING CHANGE] ROS1: Node name is automatically prepended to the namespace.

## 0.7.0 - 2023-04-27
### Added
- `reconfigure`:
  - ROS2 is now supported.

### Removed
- `uninode`:
  - Node no longer produces warnings when using unimplmented functions.

## 0.6.0 - 2022-08-24
### Added
- `reconfigure`:
  - Parameters can be now used within conditions.
  - Implementation of `__contains__` to support `if ... in P`.
  - `update()` now takes optional argument `only_existing` (def. `False`) to only update existing parameters.
  - (ROS1 only) Parameters and their values are exposed to the ROS Parameter Server.
  - `link(ConstrainedP, ConstrainedP)` to link two parameters together. First cannot be larger then the second one.

## 0.5.1 - 2022-03-23
### Added
- `reconfigure`:
  - `update()` now supports `list(tuple(str, any))` for ordered addition of multiple parameters.

### Updated
- Readme now contains details about `update()`.

## 0.5.0 - 2022-03-22
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
  - `ROS_VERSION` variable.
  - Functions not implemented are called in the current ROS version with warning.
- Documentation for the package.

### Changed
- `reconfigure`:
  - Parameters are now kept in the same order as registered.

### Fixed
- `reconfigure`:
  - Internal class variables are properly initialized so the data are not shared between instances.
  - Bool parameter is now correctly registered as boolean instead of integer.

## 0.4.1 - 2022-03-17
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
