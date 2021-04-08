#!/usr/bin/env python
# reconfigure.py
"""Extension for dynamic reconfigure.

This module contains two classes: Parameter and ParameterHandler.

_Parameter_ is an object that represents a parameter in the application. It contains
several attributes, see Parameter.__init__().

_ParameterHandler_ is an object that you register your Parameter into (using register
function). After all parameters are registered, call init_reconfigure() which creates
all necessary stuff that is required in order to support dynamic reconfiguration (and
rqt_reconfigure).


Example:
```
PARAMETERS = ParameterHandler()

PARAMETERS.register(
    Parameter(
        name = "testing_boolean",
        default = "False",
        description = "Just a testing boolean.",
    )
)

if rospy.has_param("~"):
    for param, value in rospy.get_param("~").items():
        PARAMETERS.set(param, value)

print ("Current values of the parameters:")
print (str(PARAMETERS))

PARAMETERS.init_reconfigure()
```
"""
######################
# Imports & Globals
######################

# ROS Python client library
import rospy


# Message types
from dynamic_reconfigure.msg import ConfigDescription, Config, Group, ParamDescription, BoolParameter, IntParameter, StrParameter, DoubleParameter, GroupState
from dynamic_reconfigure.srv import Reconfigure, ReconfigureResponse


# https://stackoverflow.com/questions/2440692/formatting-floats-without-trailing-zeros
formatNumber = lambda x: ("%f" % x).rstrip("0").rstrip(".")


######################
# Parameter
######################

class Parameter(object):
    """Object that represents a parameter."""

    def __init__(self, name, default, type = None, level = 0, description = "", min = None, max = None, value = None, **overflown):
        """Initialize a parameter object.

        Arguments:
        name -- name of the parameter, str
        default -- default value of the parameter, bool/int/str/float
        type -- type of the parameter, optional, type
        level -- identification number for this parameter, int
        description -- short comment for this parameter, str
        min -- minimum value of the parameter, optional, (same as default)
        max -- maximum value of the parameter, optional, (same as default)
        value -- current value of the parameter, optional, (same as default)
        **overflown kwargs

        Note: min, max are not applicable for str and bool types.
        """

        # Mandatory arguments
        self.name = name
        self.default = default

        # Optional arguments
        self.type = type if type is not None else default.__class__
        self.typestr = self.type.__name__ if self.type != float else "double"
        self.level = level
        self.description = description
        self.min = min
        self.max = max
        self.value = value if value is not None else default


    def __str__(self):
        """Formats the parameter into a string.

        Returns:
        str
        """

        if isinstance(self.type, str):
            _value = self.value
        elif isinstance(self.type, bool):
            _value = str(self.value)
        else:
            _value = "%s" % formatNumber(self.value)

        return "%s: %s" % (self.name, self.value)


    def valid(self):
        """Checks whether this parameter is valid.

        Returns:
        valid -- True when valid, otherwise False, bool
        """

        # Is the type bool/int/str/float?
        if self.type not in [bool, int, str, float]:
            return False

        # Is value of a correct type?
        if not isinstance(self.value, self.type):
            return False

        return True


######################
# ParameterHandler
######################

class ParameterHandler(object):
    """Object that stores parameters, updates their values, and provides an interface for dynamic reconfiguration."""

    # Internal storage of parameters
    _parameters = {}
    _description = ConfigDescription()
    _update = Config()
    _pub_description = None
    _pub_update = None
    _service = None


    def __init__(self):
        """Initialize a Parameter handler."""

        for field in ["location", "bool", "int", "str", "double"]:
            self._parameters[field] = {}


    def init_reconfigure(self):
        """Initializes the topics and services for dynamic reconfiguration."""

        self._pub_description = rospy.Publisher("%s/parameter_descriptions" % rospy.get_name(),
                                                ConfigDescription, queue_size = 1, latch = True)
        self._pub_update = rospy.Publisher("%s/parameter_updates" % rospy.get_name(),
                                                Config, queue_size = 1, latch = True)
        self._service = rospy.Service("%s/set_parameters" % rospy.get_name(), Reconfigure, self.reconfigure)

        self._redescribe()
        self.describe()

        self._reupdate()
        self.update()


    def describe(self):
        """Sends a message with parameters' description."""

        if self._pub_description is None:
            return

        self._pub_description.publish(self._description)


    def update(self):
        """Sends a message with parameters' update."""

        if self._pub_update is None:
            return

        self._pub_update.publish(self._update)


    def get(self, name, default = None):
        """Returns a value of parameter, or default if not found.

        Arguments:
        name -- name of the parameter, str
        default -- value to be returned when parameter not found, optional, Any

        Returns:
        value / default
        """
        return self._parameters[self._parameters[name]][name].value if name in self._parameters else default


    def _get(self, ptype, field, condition):
        """Returns names of the parameters of a given type along with selected values.

        Arguments:
        ptype -- parameter type of the parameters, str
        field -- field to be returned, lambda function
        condition -- when the result is True, parameter is added, lambda function

        Returns:
        plist -- list of parameters, 2-list(str,Any) list
        """
        return [ [param.name, field(param)] for param in self._parameters[ptype].values() if condition(param) ]


    def _get_bools(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("bool", field, condition)


    def _get_ints(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("int", field, condition)


    def _get_strs(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("str", field, condition)


    def _get_doubles(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("double", field, condition)


    def _get_config(self, field = lambda x: x.value, condition = lambda _: True):
        """Builds a Config message from all parameters.

        Arguments:
        field -- field to be used in the Config, lambda function

        Returns:
        Config -- message for the reconfiguration, Config
        """
        return Config(
            bools = [ BoolParameter(*param) for param in self._get_bools(field, condition) ],
            ints = [ IntParameter(*param) for param in self._get_ints(field, condition) ],
            strs = [ StrParameter(*param) for param in self._get_strs(field, condition) ],
            doubles = [ DoubleParameter(*param) for param in self._get_doubles(field, condition) ],
        )


    def _redescribe(self):
        """Creates a description of the parameters."""

        self._description = ConfigDescription(
            groups = [
                Group(
                    name = "Default",
                    type = "",
                    parameters = [
                        ParamDescription(
                            name = self._parameters[ptype][pname].name,
                            type = self._parameters[ptype][pname].typestr,
                            level = self._parameters[ptype][pname].level,
                            description = self._parameters[ptype][pname].description,
                        ) for pname, ptype in self._parameters["location"].items()
                    ],
                ),
            ],
            max = self._get_config(lambda x: x.max),
            min = self._get_config(lambda x: x.min),
            dflt = self._get_config(lambda x: x.default),
        )


    def _reupdate(self):
        """Creates an update description of the parameters."""

        _config = self._get_config(condition = lambda x: x.value != x.default)
        _config.groups = [
            GroupState(
                name = "Default",
                state = True,
                id = 0,
                parent = 0,
            )
        ]

        self._update = _config


    def reconfigure(self, data):
        """Service callback for dynamic reconfiguration.

        Arguments:
        data -- reconfiguration request, Reconfigure

        Returns:
        rdata -- current values of the parameters, ReconfigureResponse
        """

        _updated = []

        # Update parameters
        for param in data.config.bools + data.config.ints + data.config.strs + data.config.doubles:
            self.set(param.name, param.value)
            _updated.append(param.name)


        if len(_updated) > 0:
            self._reupdate()
            self.update()

        return ReconfigureResponse(
            self._get_config(condition = lambda x: x in _updated)
        )


    def register(self, parameter):
        """Registers a parameter in the handler.

        Arguments:
        parameter -- Parameter class

        Returns:
        success -- True when the parameter was registered successfully, otherwise False, bool
        """

        # Check that we do not know this parameter
        if parameter.name in self._parameters:
            return False

        # Check that the parameter is valid
        if not parameter.valid():
            return False

        # Add it into proper dict
        self._parameters[parameter.typestr][parameter.name] = parameter
        self._parameters["location"][parameter.name] = parameter.typestr


        return True


    def set(self, name, value):
        """Sets a value of the parameter.

        Arguments:
        name -- name of the parameter, str
        value -- value of the parameter, bool/int/str/float

        Returns:
        success -- True when parameter was found and set successfully, otherwise False, bool
        """

        if name in self._parameters["location"]:
            self._parameters[self._parameters["location"][name]][name].value = value
            return True

        return False


    def __str__(self):
        """Formats the handler into a string.

        Returns:
        str
        """

        return "\n".join(
            [
                "\t%s" % str(self._parameters[ptype][pname]) for pname, ptype in self._parameters["location"].items()
            ]
        )

