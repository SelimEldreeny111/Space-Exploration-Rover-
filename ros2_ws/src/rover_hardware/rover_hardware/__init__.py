"""MISSION X rover hardware bridges (SSP serial <-> ROS 2).

ROS-free modules (importable and unit-tested without rclpy):
``conversions``, ``serial_link``, ``ground_station_core``.
ROS nodes: ``mobility_bridge``, ``payload_bridge``, ``arm_bridge``,
``ground_station_bridge`` (helpers in ``ros_common``).
"""
__version__ = "2.0.0"
