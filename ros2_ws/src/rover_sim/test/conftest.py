"""Make the ROS-free modules importable without a colcon install."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(_HERE, ".."), os.path.join(_HERE, "..", "..", "rover_core"),
          os.path.join(_HERE, "..", "..", "rover_hardware")):
    p = os.path.normpath(p)
    if p not in sys.path:
        sys.path.insert(0, p)
