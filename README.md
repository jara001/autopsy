# Autopsy
_Python utilities for the F1tenth project._

Currently, this package contains following modules:
- [reconfigure](#reconfigure-module)
- [uninode](#uninode-module)



## Reconfigure module

This module is used to make dynamic reconfiguration for our nodes much easier.

- [Background](#background)
- [Usage](#usage)
  - [Import](#import)
  - [Parameter definition](#parameter-definition)
      - [Enumerated values](#enumerated-values)
      - [Callbacks](#callbacks)
  - [Reconfiguration](#reconfiguration-itself)
- [Compatibility](#compatibility)
  - [Until 0.4.0](#until-040)
  - [ROS 2](#ros-2)
- [Example](#full-example)


### Background

Dynamic reconfiguration is a great tool of ROS, using which you can adjust parameters of nodes real-time.

In ROS this feature is created by package [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure). In case you would like to learn how to create a dynamic reconfiguration support, see their [tutorial](http://wiki.ros.org/dynamic_reconfigure/Tutorials).

Unfortunately, usage of this feature is really troublesome. Especially, when combined with Python, since:

- It is compiled.
- You need to know the variables and their limit before running the node.
- The syntax is really hard.

Because of this, there is an enhanced version called [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure) that allows to modify the parameters using the dynamic reconfigure framework without having to write configuration files. It is written in C++ and used, e.g., in RealSense package.

To make the whole process easier, we have created our own Py script for dynamic reconfiguration support.

The GUI for this remains the same, i.e., [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure).


### Usage

#### Import

```python
from autopsy.reconfigure import ParameterServer

# Create an instance of the ParameterServer
P = ParameterServer()
```

#### Parameter definition

Parameters are 'properties' of `ParameterServer`.

We currently support:
- [x] integers
- [x] floats
- [x] strings
- [x] booleans
- [x] enums

Upon assigning a value to parameter, an internal object is created.
- Mandatory
    - `name` -- name of the parameter, set to the name of the parameter
    - `default` -- default value of the parameter, set to the assigned value
- Optional
    - `description` -- short comment for the parameter, str
    - `value` -- current value of the parameter, same as type
    - `min` -- minimum value for the parameter, int/float only
    - `max` -- maximum value for the parameter, int/float only
    - `callback` -- function to be called upon receiving parameter change, callable

**Accessing the value** can be done using `.value`. However, the operators are overloaded, meaning that you can freely use `P.something + 5` without `.value`.

_Note: The `.value` is used only when you want to store it somewhere else._

_Note: Features `is`, `not` and `bool()` are not currently available here._

Example:
```python
# 1) Create an int named 'something' with value 5:
P.something = 5

# 2) Add some constraints/etc. for 'something':
P.something.description = "This is just a number"
P.something.min = 2
P.something.max = 6
# Note: Storing different value than min<=X<=max raises an exception.

# 3) Change the value in 'something':
P.something = 4

# 4) Create parameters of other types:
P.floatparam = 4.5
P.juststring = "I am a string."
P.boolean = True

# 5) Set limits right during creation:
P.anotherparam = {'default': 5.2, 'min': 2, 'max': 6, 'description': 'I am just a float.'}

# 6) Convert dynamic_reconfiguration cfg
# Instead of:
# gen.add("Speed_max", double_t, 0, "Level-1 speed", 0.158483034, 0, 0.3)
# do this:
P.smax = ["Speed_max", float, 0, "Level-1 speed", 0.158483034, 0, 0.3]
```

Parameters may be also defined at once using function `P.update(PARAMETERS)`. There are two ways to use it:

- Using dictionary (parameters are ordered randomly):
    ```python
    PARAMETERS = {
        "something": 4,
        "anotherparam": {'default': 5.2, 'min': 2, 'max': 6, 'description': 'I am just a float.'},
    }
    ```
- Using list of tuples (parameters are ordered):
    ```python
    PARAMETERS = [
        ("something", 4),
        ("anotherparam", {'default': 5.2, 'min': 2, 'max': 6, 'description': 'I am just a float.'}),
    ]
    ```

##### Enumerated values

Example:
```python
# 7) Enumerated parameters
from autopsy.reconfigure import ParameterEnum

class standardEnum(Enum):
    Value1 = 5
    Value2 = 10

class extendedEnum(ParameterEnum):
    Value1 = 9.5, "First value"
    Value2 = 9.68, "Second value"

inlineEnum = ParameterEnum("NOT_USED", {"Value1": (8, "with help"), "Value2": 9})

P.enumeratedParam = standardEnum
P.enumeratedParam2 = extendedEnum
P.enumeratedParam3 = inlineEnum

# Value assignment
P.enumeratedParam = standardEnum.Value1
```

##### Callbacks

Callback is a function that is called upon receiving a changed value of the parameter **from the reconfigure service**.

This function has to:
- Have at least one argument, that contains the new value of the parameter.
- Return value that is correct, i.e., that is sent back to the reconfigure server.

Example:
```python
P = ParameterServer()
P.number = 5

def limit_value(new_value):
    "Limits the value to be lower then 10."

    if new_value <= 10:
        value = new_value
    else:
        value = P.number.value

    return value

P.number.callback = limit_value

P.reconfigure()

# limit_value is run on every change of 'P.number'.
```
_Note: Callback function receives new value of the parameter, and is required to return the true new value. It is then filled inside the parameter and announced to the reconfigure GUI._


#### Reconfiguration itself

```python
# 1) Update parameters (with values in the ROS Parameter Server)
# This is a "mass update", just dict of values
if rospy.has_param("~"):
    P.update(rospy.get_param("~"))

# 2) Current values of Parameters
print (P)
print (P.something)

# 3) Allow reconfiguration
# Call this after 'init_node'. It will create the callbacks and so on.
P.reconfigure()
```
_Note: Passing a string to `P.reconfigure()` changes the namespace of the ParameterServer in ROS._


### Compatibility

In order to make `autopsy.reconfiguration` optional, you can use following construct:

```python
try:
    from autopsy.reconfigure import ParameterServer
    AUTOPSY_AVAILABLE = True

except:
    class ParameterServer(dict):
        """dot.notation access to dictionary attributes

        Source:
        https://stackoverflow.com/questions/2352181/how-to-use-a-dot-to-access-members-of-dictionary
        """
        __getattr__ = dict.get
        __setattr__ = dict.__setitem__
        __delattr__ = dict.__delitem__

    AUTOPSY_AVAILABLE = False
```

This is effective for `>0.4.0`. Just do not use `.value` at all.

#### Until 0.4.0

To make your code runnable on `<= 0.4.0` stick to these rules:
- `P.reconfigure()` has no argument.
- Everytime you access the parameter value use `.value`.

#### ROS 2

Current implementation is not compatible with ROS2. Some parts are usable, however the reconfiguration part is not perfect. At least, the definition of the parameters and accessing their value seems working. Pull requests are welcome!

Since it requires `dynamic_reconfigure` messages, you can download them [here](https://github.com/jara001/dynamic_reconfigure).

If you want to test the reconfiguration, you have to pass the Node instance inside `node` argument:

```python
P.reconfigure(node = node)
```


### Full example

```python
#!/usr/bin/env python
# reconfiguration_example.py

import rospy
from autopsy.reconfigure import ParameterServer

P = ParameterServer()

P.value = 5

rospy.init_node("reconfiguration_example")

rate = rospy.Rate(1)

if rospy.has_param("~"):
    P.update(rospy.get_param("~"))

print ("Current values of the parameters:")
print (P)

P.reconfigure()

while not rospy.is_shutdown():
    print ("Current value:", P.value)
    rate.sleep()
```



## Uninode module

Universal node class with ROS 1 / ROS 2 support.

Universal node (or uninode) serves as a compatibility layer for a ROS node to be executable from both ROS versions. Basically, for ROS2 we use mostly the original 'rclpy.node.Node', whereas for ROS1 we try to match its functions onto this class.

- [Relations](#relations)
- [Differences](#differences)
- [Example](#full-example-1)


### Relations

|           ROS 1           |          uninode          |           ROS 2           |
| ------------------------- | ------------------------- | ------------------------- |
| [rospy.init_node](http://docs.ros.org/en/kinetic/api/rospy/html/rospy-module.html#init_node)                          | [self.\_\_init\_\_](autopsy/node.py#L76)          | [self.\_\_init\_\_](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node)                                 |
| [rospy.get_name](http://docs.ros.org/en/kinetic/api/rospy/html/rospy-module.html#get_name)                            | self.get_name                                     | [self.get_name](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.get_name)                            |
| [rospy.Publisher](http://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Publisher-class.html)                    | [self.Publisher](autopsy/node.py#L85)+            | [self.create_publisher](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_publisher)            |
| [rospy.Subscriber](http://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html)                  | [self.Subscriber](autopsy/node.py#L99)+           | [self.create_subscription](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_subscription)      |
| [rospy.Rate](http://docs.ros.org/en/kinetic/api/rospy/html/rospy.timer.Rate-class.html)                               | [self.Rate](autopsy/node.py#L114)+                | [self.create_rate](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_rate)                      |
| [rospy.Timer](http://docs.ros.org/en/kinetic/api/rospy/html/rospy.timer.Timer-class.html)                             | [self.Timer](autopsy/node.py#L126)+               | [self.create_timer](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_timer)                    |
| [rospy.Service](http://docs.ros.org/en/kinetic/api/rospy/html/rospy.impl.tcpros_service.Service-class.html)           | [self.Service](autopsy/node.py#L139)+             | [self.create_service](https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_service)                |


Note: Lines with '+' denote that the same function as for ROS2 can be used for uninode.


### Differences

- ROS1
    - Subscriber in ROS1 can have None 'queue_size', setting it to the infinite value.
      However, this is not supported in ROS2. Therefore, in here, we set the default
      value to 10, as a default value in ROS2:
      https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-profiles
      This is not done for Publisher, as ROS1 writes a warning when passing None.
- ROS2
    - Timer in ROS2 does not take any arguments (in contrast to the 'rospy.TimerEvent
      in ROS1). Therefore, the function has to be created as:
      `def callback(self, *args, **kwargs)`
- Common
    - Services are handled slightly differently in both ROS versions. At first, service
      message is compiled into two in ROS1. In ROS2 there is only one message type.
      In addition, service callback has only one argument in ROS1, whereas there are
      two arguments in ROS2 (second is the response).
      To make it compatible with both versions stick to this:
      ```python
      if autopsy.node.ROS_VERSION == 1:
          from package.srv import ResponseMessage

      def callback(self, msg, response = None):
          ...
          if response is None:
              return ResponseMessage(...)
          else:
              response.data = ...
              return response
      ```


### Full example

Example:
```python
from autopsy.node import Node
from std_msgs.msg import Int32

class MyNode(Node):

    def __init__(self, name):
        super(Node, self).__init__(name)

        self._subscriber = self.Subscriber("/topic", Int32, self.topic_callback)


    def topic_callback(self, msg):
        print ("Received:", msg.data)
```
