"""ROS 2 helpers shared by the bridges and the simulator node (needs rclpy).

Message builders take SI values (see :mod:`rover_hardware.conversions`) so
the real bridges and ``rover_sim`` publish identical messages.
"""
from __future__ import annotations

import array
import math
from typing import Callable, List, Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Quaternion, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import UInt8MultiArray

from rover_core.protocol import Addr
from rover_core.ssp import Frame

from .conversions import diag3, quaternion_from_rpy
from .serial_link import SerialLink, parse_frames

#: commands, odometry, status: reliable (compatible with every subscriber)
RELIABLE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
#: state topics late joiners must receive (/rover/mode)
LATCHED_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
#: high-rate sensor streams (/scan): best effort
SENSOR_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)


# ----------------------------------------------------------------- builders
def make_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    x, y, z, w = quaternion_from_rpy(roll, pitch, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


def make_odometry(stamp, frame_id: str, child_frame_id: str, x: float, y: float, theta: float,
                  v: float, w: float, pose_cov: Sequence[float],
                  twist_cov: Sequence[float]) -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation = make_quaternion(0.0, 0.0, theta)
    msg.pose.covariance = [float(c) for c in pose_cov]
    msg.twist.twist.linear.x = float(v)
    msg.twist.twist.angular.z = float(w)
    msg.twist.covariance = [float(c) for c in twist_cov]
    return msg


def make_planar_tf(stamp, frame_id: str, child_frame_id: str, x: float, y: float,
                   theta: float) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.rotation = make_quaternion(0.0, 0.0, theta)
    return t


def make_range(stamp, frame_id: str, value: float, fov_rad: float, min_range: float,
               max_range: float) -> Range:
    msg = Range()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.radiation_type = Range.ULTRASOUND
    msg.field_of_view = float(fov_rad)
    msg.min_range = float(min_range)
    msg.max_range = float(max_range)
    msg.range = float(value)
    return msg


def make_imu(stamp, frame_id: str, roll: float, pitch: float, ax: float, ay: float, az: float,
             gx: float, gy: float, gz: float, rp_var: float, yaw_var: float, gyro_var: float,
             accel_var: float) -> Imu:
    """Imu with roll/pitch orientation (yaw unknown -> large yaw variance)."""
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.orientation = make_quaternion(roll, pitch, 0.0)
    msg.orientation_covariance = diag3([rp_var, rp_var, yaw_var])
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
        float(gx), float(gy), float(gz))
    msg.angular_velocity_covariance = diag3([gyro_var] * 3)
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
        float(ax), float(ay), float(az))
    msg.linear_acceleration_covariance = diag3([accel_var] * 3)
    return msg


def make_joint_state(stamp, names: Sequence[str], positions: Sequence[float]) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = list(names)
    msg.position = [float(p) for p in positions]
    return msg


def bytes_to_msg(raw: bytes) -> UInt8MultiArray:
    msg = UInt8MultiArray()
    msg.data = array.array("B", raw)
    return msg


def msg_to_bytes(msg: UInt8MultiArray) -> bytes:
    return bytes(msg.data)


def subscribe_cmd_vel(node: Node, topic: str, stamped: bool,
                      callback: Callable[[float, float], None]):
    """Subscribe to Twist or TwistStamped (ICD section 10, ``use_stamped``)."""
    if stamped:
        return node.create_subscription(
            TwistStamped, topic, lambda m: callback(m.twist.linear.x, m.twist.angular.z),
            RELIABLE_QOS)
    return node.create_subscription(
        Twist, topic, lambda m: callback(m.linear.x, m.angular.z), RELIABLE_QOS)


# ----------------------------------------------------------------- MCU bridge base
class McuBridgeNode(Node):
    """Common part of the mobility / payload / arm bridges: serial link,
    frame polling and the /ssp/to_<name>, /ssp/from_<name> routing topics."""

    def __init__(self, node_name: str, default_port: str, ssp_name: str, mcu_addr: int) -> None:
        super().__init__(node_name)
        self.mcu_addr = int(mcu_addr)
        self.ssp_name = ssp_name
        port = self.declare_parameter("port", default_port).value
        baud = self.declare_parameter("baud", 115200).value
        transparency = self.declare_parameter("transparency", False).value
        ibt = self.declare_parameter("inter_byte_timeout_s", 0.02).value
        poll_hz = self.declare_parameter("poll_rate_hz", 200.0).value
        self.link = SerialLink(port, baud, timeout_s=ibt, transparency=transparency).start()
        self.routed_pub = self.create_publisher(UInt8MultiArray, f"ssp/from_{ssp_name}",
                                                RELIABLE_QOS)
        self.create_subscription(UInt8MultiArray, f"ssp/to_{ssp_name}", self._on_routed,
                                 RELIABLE_QOS)
        self.create_timer(1.0 / max(10.0, float(poll_hz)), self._poll)
        self.create_timer(5.0, self._watch_link)
        self._was_connected: Optional[bool] = None
        self.get_logger().info(f"{node_name}: SSP link {port} @ {baud} baud (MCU 0x{mcu_addr:02X})")

    # frames from the MCU
    def _poll(self) -> None:
        for fr in self.link.poll():
            if fr.dest == Addr.GROUND:               # answer to a routed GS request
                self.routed_pub.publish(bytes_to_msg(fr.encode()))
            elif fr.dest in (Addr.ROVER, Addr.BROADCAST):
                try:
                    self.handle_frame(fr)
                except ValueError as exc:            # malformed payload
                    self.get_logger().warn(f"bad payload {fr!r}: {exc}")

    def handle_frame(self, fr: Frame) -> None:      # overridden
        pass

    # frames from the ground station (via ground_station_bridge)
    def _on_routed(self, msg: UInt8MultiArray) -> None:
        for fr in parse_frames(msg_to_bytes(msg)):
            if not self.link.send(fr):
                self.get_logger().warn(f"routed frame dropped (link down): {fr!r}")

    def send(self, ptype: int, payload: bytes = b"") -> bool:
        return self.link.send(Frame(self.mcu_addr, Addr.ROVER, int(ptype), bytes(payload)))

    def _watch_link(self) -> None:
        c = self.link.connected
        if c != self._was_connected:
            if c:
                self.get_logger().info(f"{self.link.port} connected")
            else:
                self.get_logger().warn(f"{self.link.port} not connected: {self.link.last_error}")
            self._was_connected = c
        s = self.link.decoder.stats
        if s.crc_errors or s.framing_errors:
            self.get_logger().debug(f"link stats {self.link.stats()}")

    def destroy_node(self):
        self.link.stop()
        return super().destroy_node()


def run_node(node_factory: Callable[[], Node], args: Optional[List[str]] = None) -> None:
    """Standard rclpy main loop with clean shutdown on Ctrl-C."""
    rclpy.init(args=args)
    node = None
    try:
        node = node_factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def finite_or_nan(x: Optional[float]) -> float:
    return float(x) if x is not None and math.isfinite(x) else float("nan")
