#!/usr/bin/env python
# reconfigure.py
"""Extension for dynamic reconfigure.

This module contains multiple classes:
  - Parameter,
    - ConstrainedP,
      - IntP,
      - DoubleP,
    - BoolP,
    - StrP,
  - ParameterReconfigure,
    - ParameterServer,
  - ParameterEnum.

However, from the user side, only the last two, ParameterServer and ParameterEnum are used.

_Parameter_ is an object that represents a parameter in the application, and it serves
as a base for other types. Mandatory attributes are: name, default (value) and type.
Its derivatives are documented separately.

_ParameterReconfigure_ is an object that you register your parameters into. Calling
reconfigure() creates all necessary stuff that is required in order to support dynamic
reconfiguration (and rqt_reconfigure).

_ParameterServer_ is a derivative of ParameterReconfigure that is used in the userspace
only. In comparison to the previous ParameterHandler, it is using properties for accessing
the parameters which reduces the number of changes that is required to perform on the code.

_ParameterEnum_ is an extension of standard Enum class. Instead of taking just one value,
ParameterEnum supports also second value containing the description of each parameter.
However, Enum can be still used.


Example:
```
P = ParameterServer()

P.number = 5 # Creates a parameter 'number' of 'int' type with value '5'.
P.number.description = "Just a number." # Adds a docstring to the parameter.

P.floating = {'default': 5.2, 'min': 2, 'max': 6, 'description': 'float'}
# Creates a parameter 'floating' that is a float within 2 <= floating <= 6.

if rospy.has_param("~"):
    P.update(rospy.get_param("~"))

print ("Current values of the parameters:")
print (P)

P.reconfigure()
```

Enumeration example:
```
P = ParameterServer()

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

Callback example:
```
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
"""
######################
# Imports & Globals
######################

# ROS Python client library (inside)
import autopsy.node


# Overloading the operators
import operator


# Enumerated parameters support
from enum import Enum, EnumMeta


# Message types
from dynamic_reconfigure.msg import ConfigDescription, Config, Group, ParamDescription, BoolParameter, IntParameter, StrParameter, DoubleParameter, GroupState
from dynamic_reconfigure.srv import Reconfigure

if autopsy.node.ROS_VERSION == 1:
    # ROS1 compiles the messages into two, in contrast to ROS2.
    from dynamic_reconfigure.srv import ReconfigureResponse


# https://stackoverflow.com/questions/2440692/formatting-floats-without-trailing-zeros
formatNumber = lambda x: ("%f" % x).rstrip("0").rstrip(".")


######################
# ParameterEnum
######################

class ParameterEnum(Enum):
    """Object that contains both Enum and its description.

    Source:
    https://stackoverflow.com/questions/50473951/how-can-i-attach-documentation-to-members-of-a-python-enum
    """

    def __new__(cls, value, doc = ""):
        self = object.__new__(cls)
        self._value_ = value
        self.__doc__ = doc
        return self


######################
# Parameter
######################

class Parameter(object):
    """Object that represents a parameter."""

    def __init__(self, name, default, type, level = 0, description = "", value = None, enum = None, callback = None, *args, **kwargs):
        """Initialize a parameter object.

        Arguments:
        name -- name of the parameter, str
        default -- default value of the parameter, bool/int/str/float
        type -- type of the parameter, type
        level -- identification number for this parameter, int
        description -- short comment for this parameter, str
        value -- current value of the parameter, optional, (same as default)
        enum -- enumerate object for interpreting the values, optional
        callback -- function to be called upon receiving dynamic reconfiguration, optional, Callable
        *overflown args
        **overflown kwargs
        """

        # Sanity check
        if not isinstance(default, type) or ( value is not None and not isinstance(value, type) ):
            raise TypeError("Type '%s' does not match given values '%s / %s'." % (type.__name__, default, value))

        # TODO: Do properties dynamically using property().
        # But beware of https://stackoverflow.com/questions/1325673/how-to-add-property-to-a-class-dynamically

        # Mandatory arguments
        self.name = name
        self.default = default
        self.type = type

        # Optional arguments
        #self.type = type if type is not None else default.__class__
        self.typestr = self.type.__name__ if self.type != float else "double"
        self.level = level
        self.description = description
        self.value = value if value is not None else default

        # Enumeration
        self.__enum = enum

        # Callbacks
        self.callback = callback


    @property
    def name(self):
        """Name of the parameter."""
        return self.__name

    @name.setter
    def name(self, new_value):
        self.__name = new_value


    @property
    def default(self):
        """Default value of the parameter."""
        return self.__default

    @default.setter
    def default(self, new_value):
        self.__default = new_value


    @property
    def type(self):
        """Type of the parameter."""
        return self.__type

    @type.setter
    def type(self, new_value):
        self.__type = new_value


    @property
    def typestr(self):
        """Type of the parameter as string (not same since float != double)."""
        return self.__typestr

    @typestr.setter
    def typestr(self, new_value):
        self.__typestr = new_value


    @property
    def level(self):
        """Level the parameter belongs to."""
        return self.__level

    @level.setter
    def level(self, new_value):
        self.__level = new_value


    @property
    def description(self):
        """Description of the parameter shown in GUI."""
        return self.__description

    @description.setter
    def description(self, new_value):
        self.__description = new_value


    @property
    def value(self):
        """Current value of the parameter."""
        return self.__value

    @value.setter
    def value(self, new_value):
        if hasattr(self, "enum") and self.enum is not None:
            if isinstance(new_value, Enum):
                new_value = new_value.value

            if new_value not in [ v.value for v in self.enum.__members__.values() ]:
                raise ValueError("Value '%s' is not a valid enumerated element." % (new_value))

        if not isinstance(new_value, self.type):
            raise ValueError("Value '%s' is not of a type '%s'." % (new_value, self.type.__name__))

        self.__value = new_value


    @property
    def enum(self):
        """Current enum object of the parameter."""
        return self.__enum


    @property
    def repr_enum(self):
        """Current enum in the dynamic reconfigure friendly format."""

        if self.enum is None:
            return ""

        _d = {"enum": [], "enum_description": self.description if self.enum.__doc__ is None else self.enum.__doc__}

        for _e in list(self.enum):
            _d["enum"].append(
                {
                    "name": _e.name,
                    "type": self.typestr,
                    "description": "" if _e.__doc__ is None else _e.__doc__,
                    "value": _e.value,
                }
            )

        return repr(_d)


    @property
    def callback(self):
        """Return the callback function."""
        return self.__callback


    @callback.setter
    def callback(self, new_value):
        if not callable(new_value) and new_value is not None:
            raise ValueError("Value '%s' is not a callable." % (new_value))

        self.__callback = new_value


    def __str__(self):
        """Formats the parameter into a string.

        Returns:
        str
        """
        return "%s: %s" % (self.name, self.value)


    # Overloading operators
    # Note: It would be nice to get around this by returning a value
    # when calling P.parameter instead of the object. But currently, I have no
    # idea how to do this.
    # Other note: Currently, 'is', 'not' and 'bool()' are not supported.
    @staticmethod
    def __operator__(first, second = None, operator = None):
        if second is None:
            return operator(
                first.value if isinstance(first, Parameter) else first
            )
        elif hasattr(first, "enum") and isinstance(second, Enum):
            return operator(
                first.enum(first.value),
                second
            )
        elif isinstance(first, Enum) and hasattr(second, "enum"):
            return operator(
                first,
                second.enum(second.value)
            )
        else:
            return operator(
                first.value if isinstance(first, Parameter) else first,
                second.value if isinstance(second, Parameter) else second
            )

    __operators = ["abs", "add", "and", "div", "floordiv", "lshift", "mod", "mul", "or", "pow", "rshift", "sub", "truediv", "xor"]

    __comparators = ["lt", "le", "eq", "ne", "ge", "gt"]

    __uoperators = ["abs", "index", "invert", "neg", "pos"]

    __functions = ["int", "float", "complex", "round", "trunc", "floor", "ceil"]

    for op in __operators:
        vars()["__%s__" % op] = lambda first, second, op = "__%s__" % op: Parameter.__operator__(first, second, operator.__dict__[op])
        vars()["__r%s__" % op] = lambda first, second, op = "__%s__" % op: Parameter.__operator__(second, first, operator.__dict__[op])

        # Could be also done using:
        #def _(first, second, op = _operator):
        #    return Parameter.__operator__(first, second, operator.__dict__[op])
        #
        #def __(first, second, op = _operator):
        #    return Parameter.__operator__(second, first, operator.__dict__[op])
        #
        #vars()[_operator] = _
        #vars()["__r%s__" % op] = __

    for op in __comparators:
        vars()["__%s__" % op] = lambda first, second, op = "__%s__" % op: Parameter.__operator__(first, second, operator.__dict__[op])

    for op in __uoperators:
        vars()["__%s__" % op] = lambda first, second = None, op = "__%s__" % op: Parameter.__operator__(first, second, operator.__dict__[op])

    for op in __functions:
        vars()["__%s__" % op] = lambda first, op = "__%s__" % op: getattr(first.value, op)()


class ConstrainedP(Parameter):
    """Parameter that is constrained by min and max values."""

    # Constrain the value even further by looking at a linked variable.
    _link = None


    def __init__(self, name, default, type, min = -2147483647, max = 2147483647, **kwargs):
        """Initialize a constrained parameter object.

        Arguments:
        name -- name of the parameter, str
        default -- default value of the parameter, bool/int/str/float
        type -- type of the parameter, type
        min -- minimum value of the parameter (included), optional, (same as default)
        max -- maximum value of the parameter (included), optional, (same as default)
        **overflown kwargs
        """

        if min > max:
            raise ValueError("Lower bound is larger than upper bound.")

        self.type = type

        self.min = type(min)
        self.max = type(max)

        super(ConstrainedP, self).__init__(name, default, type, **kwargs)


    @property
    def min(self):
        """Minimum value of the parameter (included)."""
        return self.__min

    @min.setter
    def min(self, new_value):
        if not isinstance(new_value, self.type):
            raise ValueError("Value '%s' is not of a type '%s'." % (new_value, self.type.__name__))

        self.__min = new_value


    @property
    def max(self):
        """Maximum value of the parameter (included)."""
        return self.__max

    @max.setter
    def max(self, new_value):
        if not isinstance(new_value, self.type):
            raise ValueError("Value '%s' is not of a type '%s'." % (new_value, self.type.__name__))

        self.__max = new_value


    @Parameter.value.setter
    def value(self, new_value):
        if hasattr(self, "enum") and self.enum is not None:
            if isinstance(new_value, Enum):
                new_value = new_value.value

            if new_value not in [ v.value for v in self.enum.__members__.values() ]:
                raise ValueError("Value '%s' is not a valid enumerated element." % (new_value))

        if not isinstance(new_value, self.type):
            raise ValueError("Value '%s' is not of a type '%s'." % (new_value, self.type.__name__))

        if not self.min <= new_value <= self.max:
            raise ValueError("Value '%s' is not in %s <= value <= %s range." % (formatNumber(new_value), formatNumber(self.min), formatNumber(self.max)))

        if self._link is not None:
            new_value = self._link(new_value)

        Parameter.value.fset(self, new_value)


    def __str__(self):
        """Formats the parameter into a string.

        Returns:
        str
        """
        return "%s: %s" % (self.name, formatNumber(self.value))


class IntP(ConstrainedP):
    """Parameter of a type integer."""

    def __init__(self, name, default, *args, **kwargs):
        super(IntP, self).__init__(name, default, int, *args, **kwargs)


    def __nonzero__(self):
        """Used for evaluating conditions (Py2)."""
        return self.value != 0

    def __bool__(self):
        """Used for evaluating conditions (Py3)."""
        return self.value != 0


class DoubleP(ConstrainedP):
    """Parameter of a type float/double."""

    def __init__(self, name, default, *args, **kwargs):
        super(DoubleP, self).__init__(name, default, float, *args, **kwargs)


    def __nonzero__(self):
        """Used for evaluating conditions (Py2)."""
        return self.value != 0.0

    def __bool__(self):
        """Used for evaluating conditions (Py3)."""
        return self.value != 0.0


class BoolP(Parameter):
    """Parameter of a type bool."""

    def __init__(self, name, default, *args, **kwargs):
        self.min = None
        self.max = None

        super(BoolP, self).__init__(name, default, bool, *args, **kwargs)


    def __nonzero__(self):
        """Used for evaluating conditions (Py2)."""
        return self.value

    def __bool__(self):
        """Used for evaluating conditions (Py3)."""
        return self.value


class StrP(Parameter):
    """Parameter of a type string."""

    def __init__(self, name, default, *args, **kwargs):
        self.min = None
        self.max = None

        super(StrP, self).__init__(name, default, str, *args, **kwargs)


    def __len__(self):
        """Size of the string. Also used for evaluating conditions."""
        return len(self.value)


######################
# ParameterDict
######################

class ParameterDict(dict):
    """Extension on ordinary dict that maintains the order of elements."""

    # Auxiliary storage
    _order = None


    def __init__(self):
        """Initialize the object."""
        super(ParameterDict, self).__init__()

        self._order = []


    def __setitem__(self, name, value):
        """Overload the original setitem to keep order."""

        super(ParameterDict, self).__setitem__(name, value)

        if name not in self._order:
            self._order.append(name)


    def __delitem__(self, name):
        """Overload item deletion to remove it from the order."""

        super(ParameterDict, self).__delitem__(name)

        self._order.remove(name)


    def items(self):
        """Return items in a given order."""
        return sorted(
            super(ParameterDict, self).items(),
            key = lambda i: self._order.index(i[0])
        )


    def values(self):
        """Return values in a given order."""
        return [ value for _, value in self.items() ]


######################
# ParameterReconfigure
######################

class ParameterReconfigure(object):
    """Object that provides an interface for dynamic reconfiguration."""

    # Internal storage of parameters
    _parameters = None
    _description = None
    _update = None
    _pub_description = None
    _pub_update = None
    _service = None
    _node = None
    _expose_parameters = False

    def __init__(self):
        """Initialize variables of the object.

        This is a Python feature, as setting them before makes the variables
        shared between all instances.

        References:
        https://stackoverflow.com/questions/13389325/why-do-two-class-instances-appear-to-be-sharing-the-same-data
        https://stackoverflow.com/questions/1132941/least-astonishment-and-the-mutable-default-argument
        """
        self._parameters = ParameterDict()
        self._description = ConfigDescription()
        self._update = Config()
        pass


    def reconfigure(self, namespace = None, node = None):

        if node is None:
            self._node = autopsy.node.rospy
        else:
            self._node = node

        if namespace is None:
            namespace = self._node.get_name()

        self._pub_description = self._node.Publisher("%s/parameter_descriptions" % namespace,
                                                ConfigDescription, queue_size = 1, latch = True)
        self._pub_update = self._node.Publisher("%s/parameter_updates" % namespace,
                                                Config, queue_size = 1, latch = True)
        self._service = self._node.Service("%s/set_parameters" % namespace, Reconfigure, self._reconfigureCallback)

        # Expose parameters to the ROS Parameter Server (ROS1 only)
        if hasattr(self._node, "set_param"):
            self._expose_parameters = True
            for _name, _param in self._parameters.items():
                self._node.set_param("~%s" % _name, _param.value)

        self._redescribe()
        self._describePub()

        self._reupdate()
        self._updatePub()


    def _describePub(self):
        """Sends a message with parameters' description."""

        if self._pub_description is None:
            return

        self._pub_description.publish(self._description)


    def _updatePub(self):
        """Sends a message with parameters' update."""

        if self._pub_update is None:
            return

        self._pub_update.publish(self._update)


    def _get(self, ptype, field, condition):
        """Returns names of the parameters of a given type along with selected values.

        Arguments:
        ptype -- parameter type of the parameters, str
        field -- field to be returned, lambda function
        condition -- when the result is True, parameter is added, lambda function

        Returns:
        plist -- list of parameters, 2-list(str,Any) list
        """
        return [ [param.name, field(param)] for param in self._parameters.values() if condition(param) ]


    def _get_bools(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("bool", field, lambda x: condition(x) and x.type == bool)


    def _get_ints(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("int", field, lambda x: condition(x) and x.type == int)


    def _get_strs(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("str", field, lambda x: condition(x) and x.type == str)


    def _get_doubles(self, field = lambda x: x.value, condition = lambda _: True):
        return self._get("double", field, lambda x: condition(x) and x.type == float)


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
                            name = _param.name,
                            type = _param.typestr,
                            level = _param.level,
                            description = _param.description,
                            edit_method = "" if _param.enum is None else _param.repr_enum
                        ) for _name, _param in self._parameters.items()
                    ],
                ),
            ],
            max = self._get_config(lambda x: x.max),
            min = self._get_config(lambda x: x.min),
            dflt = self._get_config(lambda x: x.default),
        )


    def _reupdate(self):
        """Creates an update description of the parameters."""

        # Note: Condition was removed because of the linked variables, as when
        # the value should be moved to its default state, nothing would happen.
        _config = self._get_config()#condition = lambda x: x.value != x.default)
        _config.groups = [
            GroupState(
                name = "Default",
                state = True,
                id = 0,
                parent = 0,
            )
        ]

        self._update = _config


    def _reconfigureCallback(self, data, response = None):
        """Service callback for dynamic reconfiguration.

        Arguments:
        data -- reconfiguration request, Reconfigure

        Returns:
        rdata -- current values of the parameters, ReconfigureResponse
        """

        _updated = []

        # Update parameters
        for param in data.config.bools + data.config.ints + data.config.strs + data.config.doubles:
            if self._parameters[param.name].callback is None:
                self._parameters[param.name].value = param.value
            else:
                self._parameters[param.name].value = self._parameters[param.name].callback(param.value)

            # Expose the update to the ROS Parameter Server (ROS1 only)
            if self._expose_parameters:
                self._node.set_param("~%s" % param.name, param.value)

            _updated.append(param.name)


        if len(_updated) > 0:
            self._reupdate()
            self._updatePub()


        if response is None:
            return ReconfigureResponse(
                self._get_config(condition = lambda x: x in _updated)
            )
        else:
            response.config = self._get_config(condition = lambda x: x in _updated)
            return response


    def __str__(self):
        """Formats the handler into a string.

        Returns:
        str
        """

        return "\n".join(
            [
                "\t%s" % str(param) for param in self._parameters.values()
            ]
        )


######################
# ParameterServer
######################

class ParameterServer(ParameterReconfigure):
    """Object that stores parameters, updates their values, and provides an interface for dynamic reconfiguration."""


    def __init__(self):
        """Initialize the object by calling the super init."""
        super(ParameterServer, self).__init__()


    def __hasattr__(self, name):
        pass


    def __getattr__(self, name):
        super(ParameterServer, self).__setattr__(name, None)
        return name


    def __getattribute__(self, name):
        """Function overload to obtain parameters as properties."""

        #if name == "_parameters":
        if name[0] == "_":
            return super(ParameterServer, self).__getattribute__(name)

        if name in self._parameters:
            return self._parameters[name]

        return super(ParameterServer, self).__getattribute__(name)


    def __setattr__(self, name, value):
        """Function overload to set parameters as properties."""

        if name[0] == "_":
            super(ParameterServer, self).__setattr__(name, value)

        elif name in self._parameters:
            if isinstance(value, dict):
                for _prop, _val in value.items():
                    self._parameters[name]._prop = _val
            else:
                self._parameters[name].value = value

        else:
            if isinstance(value, dict) and "default" in value :
                kwargs = value
                value = value.get("default")
                del kwargs["default"]
            elif isinstance(value, list):
                # Dynamic Reconfigure support (instead of .add() pass a list)
                _type = value[1]
                kwargs = dict(
                    zip(
                        ["level", "description", "default", "min", "max"],
                        value[2:] # Omit name and type
                    )
                )
                value = _type(kwargs.get("default"))
                del kwargs["default"]
            elif isinstance(value, EnumMeta):
                # Enum support
                kwargs = {"enum": value}
                value = list(value)[0].value
            else:
                kwargs = dict()

            if isinstance(value, bool):
                self._parameters[name] = BoolP(name, value, **kwargs)
            elif isinstance(value, int):
                self._parameters[name] = IntP(name, value, **kwargs)
            elif isinstance(value, float):
                self._parameters[name] = DoubleP(name, value, **kwargs)
            elif isinstance(value, str):
                self._parameters[name] = StrP(name, value, **kwargs)
            else:
                raise TypeError("Unable to create a parameter of type '%s'." % type(value))


    def __contains__(self, name):
        """Check whether a parameter name exists. Used for 'if name in P'.

        Arguments:
        name -- name of the parameter, str
        """
        return name in self._parameters


    def link(self, param1, param2):
        """Links two constrained parameters together so one cannot be more then the other.

        Arguments:
        name1 -- name of the first ConstrainedP parameter, str
        name2 -- name of the second ConstrainedP parameter, str
        """

        for param in (param1, param2):
            if not isinstance(param, ConstrainedP):
                raise TypeError("Parameter '%s' is of a type '%s'." % (param.name, type(param)))

        param1._link = lambda x: min(x, param2.value)
        param2._link = lambda x: max(x, param1.value)


    def update(self, parameters, only_existing = False):
        """Updates the parameters according to the passed dictionary.

        Arguments:
        parameters -- new values of the parameters, dict(str, any) or list(tuple(str, any))
        only_existing -- when True only update values, do not add new parameters, bool, default False
        """

        if isinstance(parameters, dict):
            for param, value in parameters.items():
                if not only_existing or param in self:
                    self.__setattr__(param, value)
        elif isinstance(parameters, list):
            for param, value in parameters:
                if not only_existing or param in self:
                    self.__setattr__(param, value)
        else:
            raise NotImplementedError("ParameterServer.update() is not supported for type '%s'." % type(parameters))
