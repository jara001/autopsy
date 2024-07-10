#!/usr/bin/env python
# helpers.py
"""Set of helper classes and decorators for ROS nodes.

Currently implemented:
 - Publisher and Subscriber (ROS1) decorators for class Nodes.
"""
######################
# Imports & Globals
######################

from autopsy.core import Core


######################
# Decorators
######################

class Publisher(object):
    """ROS1 style Publisher decorator."""

    def __init__(self, *args, **kwargs):
        """Initialize the decorator."""
        super(Publisher, self).__init__()
        self.__args = args
        self.__kwargs = kwargs


    def __call__(self, func, *args, **kwargs):
        """Execute the Publisher."""
        # This is the actual function that runs when the decorated function
        # is called.
        def publish(cls, *args, **kwargs):
            pubs = getattr(cls, "_pub_%s" % func.__name__, [])

            if len(pubs) < 1:
                raise ValueError(
                    "Trying to publish a message using '%s()' however the "
                    "underlying publisher '%s' has not been found."
                    % (func.__name__, "_pub_%s" % func.__name__)
                )

            msg = func(cls, *args, **kwargs)

            # Only one message is returned; send it to every publisher.
            if not isinstance(msg, list):
                for pub in pubs:
                    pub.publish(msg)

            # The returned structure is a list.
            # But raise an exception if the number of returned messages
            # does not correspond to the number of the publishers.
            # Note: List with one message is allowed everytime.
            elif len(msg) != len(pubs) and len(msg) > 1:
                raise ValueError(
                    "Unable to publish messages using '%s()' as the number "
                    "or returned messages is '%d' while number of registered "
                    "publishers is '%d'."
                    % (func.__name__, len(msg), len(pubs))
                )

            else:
                for i, pub in enumerate(pubs):
                    # Skip the publisher is the msg is None.
                    if msg[i % len(msg)] is not None:
                        pub.publish(
                            msg[i % len(msg)]
                        )

        # Initialize decorated function, prepare it for being a publisher.
        if not getattr(func, "_is_publisher", False):
            publish._is_publisher = True
            publish._publisher_args = [self.__args]
            publish._publisher_kwargs = [self.__kwargs]
            return publish
        else:
            # However, if this is not the first publisher attached to this
            # function, (meaning that the func is actually publish() above),
            # just append everything there as it is able to properly handle
            # all publishers.
            func._publisher_args.append(self.__args)
            func._publisher_kwargs.append(self.__kwargs)
            return func


class Subscriber(object):
    """Subscriber decorator."""

    def __init__(self, *args, **kwargs):
        """Initialize the decorator."""
        super(Subscriber, self).__init__()
        self.__args = args
        self.__kwargs = kwargs


    def __call__(self, func, *args, **kwargs):
        """Execute the Subscriber."""
        if not getattr(func, "_is_callback", False):
            func._is_callback = True
            func._subscriber_args = [self.__args]
            func._subscriber_kwargs = [self.__kwargs]
        else:
            func._subscriber_args.append(self.__args)
            func._subscriber_kwargs.append(self.__kwargs)

        # Return the augmented function.
        return func


class Timer(object):
    """Timer decorator."""

    def __init__(self, *args, **kwargs):
        """Initialize the decorator."""
        super(Timer, self).__init__()
        self.__args = args
        self.__kwargs = kwargs


    def __call__(self, func, *args, **kwargs):
        """Execute the Timer."""
        if not getattr(func, "_is_timer", False):
            func._is_timer = True
            func._timer_args = [self.__args]
            func._timer_kwargs = [self.__kwargs]
        else:
            func._timer_args.append(self.__args)
            func._timer_kwargs.append(self.__kwargs)

        # Return the augmented function.
        return func


######################
# Functions
######################

def Execute(node, name = None):
    """Execute a node class if the script is run directly.

    Note: This is provided to avoid repeating code in every script.

    Arguments:
    node -- node to execute, Node class
    name -- name of the node, str, optional
    """
    # Initialize the unicore
    Core.init()

    # Create a node instance and spin it
    if name is None:
        n = node()
    else:
        n = node(name)

    Core.spin(n)

    # Shutdown the unicore on node exit
    Core.shutdown()