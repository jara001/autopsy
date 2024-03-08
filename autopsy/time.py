#!/usr/bin/env python
# time.py
"""Utilities for measuring duration of code blocks.

Originally a part of `rosmeasure` package.

1) Classes
The autopsy.time utility provides following classes:
- TimeMeasurer

## TimeMeasurer ##

TimeMeasurer measures time spend in the selected section of the
code. It is created as:

    tm = TimeMeasurer(
        name = "Measurer",  # Name of the Measurer
        units = "s"         # Time units used for the measuring
    )

Upon creation, these functions are used:
 - start() -- Starts the measurement.
 - end() -- Ends the measurement, storing the values inside.
 - summary() -- Prints out the statistics for the Measurer.

Another way of using this measurer is as follows:

    with TimeMeasurer(name, units) as _ :
        ...


2) Decorators
The autopsy.time utility also provides decorators to be used
instead of the classes.

## @duration ##

Duration decorator is basically the same as TimeMeasurer. It
is used as follows:

    @duration(name, units)
    def function():
        pass

The decorator supplies following section of the code:

    TM = TimeMeasurer(name, units)
    TM.start()
    output = function()
    TM.end()
    TM.summary()
    return output

However, in contrast to TimeMeasurer, a keyword-only argument 'interval'
can be passed to the decorator to report the summary periodically, and not
on every function call.

Since >0.10.1 not passing 'interval' generates a warning on start-up.
"""
######################
# Imports & Globals
######################

# Py2: Allow import of module with the same name
from __future__ import absolute_import

# ReportTimer
from threading import _Timer

# Timing
import time

# Exceptions
import traceback


# Parameters
# TODO: Support disable from code.
DISABLED = False


######################
# conddisable decorator
######################

def conddisable():
    """Disable function when DISABLED."""
    global DISABLED

    def block(f):
        return lambda *x, **y: None

    def let_pass(f):
        return f

    return block if DISABLED else let_pass


######################
# ReportTimer class
######################

class ReportTimer(_Timer):
    """A thread for reporting the measurer.

    Source:
    https://stackoverflow.com/questions/12435211/threading-timer-repeat-function-every-n-seconds
    """

    def run(self):
        """Run a thread function every 'self.interval'."""
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


######################
# Measurer class
######################

class Measurer(object):
    """Abstract class for measuring time."""

    UNITS = ["", "s", "ms", "us", "ns"]

    def __init__(self, name = "", unit = "", **kwargs):
        """Initialize the Measurer class.

        Arguments
        ---------
        name: str = ""
            name of the Measurer
        unit: str = ""
            units used by the measurer

        Raises
        ------
        ValueError
            when unit is not in UNITS
        """
        if unit not in self.UNITS:
            raise ValueError("unknown unit '%s'" % unit)

        self._name = name if name != "" else "Measurer"
        self._unit = unit if unit != "" else "s"
        self._start = 0
        self._count = 0
        self._sum = 0
        self._min = []  # min([], X) returns the number everytime
        self._max = 0
        self._last = 0


    def unitExp(self, unit):
        """Convert unit to power.

        Arguments
        ---------
        unit: str
            name of the unit

        Returns
        -------
        power: int
            power of unit

        Raises
        ------
        ValueError
            when unit not in UNITS
        """
        if unit not in self.UNITS:
            raise ValueError("unknown unit '%s'" % unit)

        if unit == "s":
            return 0
        elif unit == "ms":
            return -3
        elif unit == "us":
            return -6
        elif unit == "ns":
            return -9
        else:
            return 0


    def convertUnits(self, value, unit_from, unit_to):
        """Convert measured value to different unit.

        Arguments
        ---------
        value: float
            value to convert
        unit_from: str
            current unit of the value
        unit_to: str
            unit to convert to

        Returns
        -------
        new_value: float
            converted value

        Notes
        -----
        When the units are same the original value is returned.
        """
        f = self.unitExp(unit_from)
        t = self.unitExp(unit_to)

        if f == t:
            return value
        else:
            return value * pow(10, f - t)


    @conddisable()
    def updateStatistics(self):
        """Update internal statistics of the Measurer."""
        self._count += 1
        self._sum += self._last
        self._min = min(self._min, self._last)
        self._max = max(self._max, self._last)


    @conddisable()
    def summary(self):
        """Print out the summary of the measurer."""
        if self._count == 0:
            print("%s: EMPTY" % self._name)
        else:
            print("%s: cur=%.4f%s min=%.4f%s avg=%.4f%s max=%.4f%s" % (
                  self._name, self._last, self._unit,
                  self._min, self._unit,
                  self._sum / self._count, self._unit,
                  self._max, self._unit
                  ))


######################
# TimeMeasurer class
######################

class TimeMeasurer(Measurer):
    """Class for TimeMeasurer, that records time spent in selected section."""

    def __init__(self, *args, **kwargs):
        """Initialize the TimeMeasurer class."""
        Measurer.__init__(self, *args, **kwargs)


    @conddisable()
    def start(self):
        """Start a measurement."""
        self._start = time.time()


    @conddisable()
    def end(self):
        """End a measurement."""
        self._last = self.convertUnits(
            time.time() - self._start,
            "s",
            self._unit
        )

        self.updateStatistics()


    @conddisable()
    def __enter__(self):
        """Start the measurement using 'with' statement."""
        # TODO: Reuse TimeMeasurers.
        self.start()
        return self


    @conddisable()
    def __exit__(self, exc_type, exc_value, tb):
        """End the measurement when 'with' statement concludes.

        Source:
        https://stackoverflow.com/questions/22417323/how-do-enter-and-exit-work-in-python-decorator-classes
        """
        if exc_type is not None:
            traceback.print_exception(exc_type, exc_value, tb)
            # return False # uncomment to pass exception through

        self.end()
        self.summary()
        return True


######################
# Decorator measurement
######################

def duration(*args, **kwargs):
    """Measure duration of a function using TimeMeasurer."""
    global DISABLED

    TM = TimeMeasurer(*args, **kwargs)

    if "interval" in kwargs:
        report = ReportTimer(kwargs.get("interval"), TM.summary)
        report.daemon = True
        report.start()
    else:
        print (
            "Warning: @duration is used without kwarg 'interval', "
            "therefore it will report summary on every call."
        )

    def wrapper(function, *args, **kwags):
        def duration_measurer(*args, **kwargs):
            TM.start()
            output = function(*args, **kwargs)
            TM.end()

            TM.summary()
            return output
        return duration_measurer

    def wrapper_with_timer(function, *args, **kwags):
        def duration_measurer(*args, **kwargs):
            TM.start()
            output = function(*args, **kwargs)
            TM.end()

            return output
        return duration_measurer

    def let_pass(function, *args, **kwargs):
        return lambda *x, **y: function(*x, **y)

    return let_pass if DISABLED else (
        wrapper if "interval" not in kwargs else wrapper_with_timer
    )
