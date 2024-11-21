#!/usr/bin/env python3.6
# __init__.py
"""Initialize script for 'autopsy' package."""

# Version of the package
try:
    from .version import __version__  # noqa: F401
except ImportError:
    # In some situations the version file is not created. However, it is
    # not required for running the package, so we just skip it.
    pass
