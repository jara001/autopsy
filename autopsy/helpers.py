#!/usr/bin/env python
# helpers.py
"""Set of helper classes and decorators for ROS nodes.

Currently implemented:
 - Publisher and Subscriber (ROS1) decorators for class Nodes.
"""
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
            print(cls, args, kwargs)

            pub = getattr(cls, "_pub_%s" % func.__name__, None)

            if pub is None:
                raise ValueError(
                    "Trying to publish a message using '%s()' however the "
                    "underlying publisher '%s' has not been found."
                    % (func.__name__, "_pub_%s" % func.__name__)
                )

            pub.publish(
                func(cls, *args, **kwargs)
            )

        publish._is_publisher = True
        publish._publisher_args = self.__args
        publish._publisher_kwargs = self.__kwargs

        # Initialize decorated function, prepare it for being a publisher.
        return publish


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
