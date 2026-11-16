"""Make the ROS-free modules importable without a colcon install
(``python -m pytest ros2_ws/src/rover_hardware/test``)."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(_HERE, ".."), os.path.join(_HERE, "..", "..", "rover_core")):
    p = os.path.normpath(p)
    if p not in sys.path:
        sys.path.insert(0, p)
