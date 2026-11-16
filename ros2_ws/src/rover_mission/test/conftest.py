"""Make the ROS-free modules importable without a colcon install."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.normpath(os.path.join(_HERE, ".."))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)
