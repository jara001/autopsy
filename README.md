# Autopsy
_Python utilities for the F1tenth project._

Currently, this package contains following modules:
- [reconfigure](#reconfigure-module)
- [uninode](#uninode-module)



## Reconfigure module

This module is used to make dynamic reconfiguration for our nodes much easier.


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

**The value of the parameter is received by `.value`.**

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

Callback is a function that is called upon receiving a changed value of the parameter.

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
Note: Callback function receives new value of the parameter,
and is required to return the true new value. It is then filled
inside the parameter and announced to the reconfigure GUI.


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

This module contains class Node.

Relations:
-------------------------------------------------------------------------------------
|           ROS 1           |          uninode          |           ROS 2           |
|-----------------------------------------------------------------------------------|
| rospy.init_node           |                     self.__init__                     |
| rospy.Publisher           | self.Publisher+           | self.create_publisher     |
| rospy.Subscriber          | self.Subscriber+          | self.create_subscription  |
| rospy.Rate                | self.Rate+                | self.create_rate          |
| rospy.Timer               | self.Timer+               | self.create_timer         |
-------------------------------------------------------------------------------------

Note: Lines with '+' denote that the same function as for ROS2 can be used for uninode.

Differences:
- ROS2
    - Timer in ROS2 does not take any arguments (in contrast to the 'rospy.TimerEvent
      in ROS1). Therefore, the function has to be created as:
      `def callback(self, *args, **kwargs)`


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
