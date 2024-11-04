# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
### Added
- `duration`:
    - Wrapper for importing Duration class in the same way as `rclpy`.
- `helpers`:
    - `Publisher`, `Subscriber` and `Timer` decorators for ROS1 nodes.
    - `Execute` function to create node instance and spin it.
- `qos`:
  - Enum for `HistoryPolicy`.
- `uninode`:
    - Support for ServiceProxy/create_client.

### Deprecated
- `uninode`:
    - `Core.duration` is deprecated. Use `autopsy.duration` instead.

### Fixed
- `reconfigure`:
    - Remove `**{}` to make code Py2 compatible.

## 0.10.7 - 2024-04-22
### Added
- `uninode`:
    - `Node.__init__()` now allows to pass **kwargs that are forwarded to the base class initializer.
    - ROS1: Node class support for `**kwargs`.

## 0.10.6 - 2024-04-22
### Changed
- `unicode`:
    - Raise an `ImportError` exception when no suitable ROS package is found.

## 0.10.5 - 2024-04-19
### Added
- `reconfigure`
    - In ROS2, FloatingPointRange and IntegerRange are used to store the variable limits and show the slider in GUI.

## 0.10.4 - 2024-04-09
### Added
- `reconfigure`
    - `ParameterServer.getValue()` to obtain the parameter value.

## 0.10.3 - 2024-04-09
### Added
- Before building the project, all uncommitted changes are stashed.

### Fixed
- `uninode`
    - `Node.Time` should now be properly used everywhere.

## 0.10.2 - 2024-03-11
### Added
- `time`:
    - Keyword-only argument to `@duration` to report summary only every `interval`.
    - A warning is generated when the argument is not used.
    - `interval` can be set to `None` in order to disable automatic statistics.
    - Argument `filename` that stores every measurement into the file.

### Changed
- `time`:
    - Timer summary now also contains minimum value.

## 0.10.1 - 2024-03-07
### Fixed
- `duration`:
  - Remove `*` from arguments to support Py2.

## 0.10.0 - 2024-03-04
### Added
- `qos`:
  - Wrapper for importing the QoS settings in the same way as `rclpy`.
- `time`:
  - New module for measuring duration of code snippets. Port from `rosmeasure` ROS package.
- `uninode`
  - Module `duration` with class `Duration` that is used for timestamps in the messages.

### Fixed
- `uninode`
  - `Time()` now works properly in ROS2.

## 0.9.5 - 2023-10-17
### Fixed
- `uninode`
  - `Time.now()` is now properly set in Python 3.

## 0.9.4 - 2023-10-11
### Added
- `uninode`:
  - Function `wait_for_message()` with ROS2 port from rolling.

## 0.9.3 - 2023-09-25
### Fixed
- `uninode`
  - Add missing import of `rospy`.

## 0.9.2 - 2023-09-08
### Fixed
- `reconfigure`
  - `ROS_VERSION` and `rospy` is now properly imported from `unicore`.

## 0.9.1 - 2023-09-07
### Changed
- `uninode`:
  - `Time.to_msg()` now returns data type 'time' in ROS1 instead of the message.

### Fixed
- `uninode`:
  - ROS2 functions properly handle integers instead of QoSProfile.
  - `Time.to_msg()` handles data properly.
  - `get_time()` returns float instead of long.

## 0.9.0 - 2023-09-05
### Added
- `unicore`:
  - New compatibility layer to cover node spinning and initialization.
  - Object `Core`:
    - Functions: `spin()`, `spin_once()`, `spin_until_future_complete()`, `init()`, `shutdown()`.
  - Decorators: `@ros1_only` and `@ros2_only` with an optional argument to use different function instead.
- `uninode`:
  - Parameter `tcp_nodelay` is now supported for both Publisher and Subscriber. It translates to `BEST_EFFORT` and vice versa.
  - QoS ReliabilityPolicy is in ROS1.

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
