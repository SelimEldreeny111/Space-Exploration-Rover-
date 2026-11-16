"""ground_station_bridge: HC-12 radio link to the ground station (ICD v2.0, link L1).

Serial 9600 8N1, own address 0x01 (ROVER / Brain). Command semantics are in
:class:`rover_hardware.ground_station_core.GroundStationCore` (ROS-free, unit
tested); this node provides the ROS side effects:

* owner of the operating mode: publishes ``/rover/mode`` (std_msgs/UInt8,
  transient-local, re-published every ``mode_publish_period_s``) and serves
  ``/rover/set_mode`` (rover_interfaces/SetMode);
* GET_TLM -> TlmV2 from the latest ``/environment``, ``/imu/data``,
  ``/rover/status``, ``/ultrasonic/*`` and the pose from TF map -> base_link
  (fallback ``/odometry/filtered``);
* CAP_IMAGE / GET_IMAGE from ``/camera/image_raw`` (cv_bridge + OpenCV when
  installed, numpy fallback, placeholder pattern without a camera);
* START_SCAN / STOP_SCAN -> mode AUTONOMOUS / IDLE (the explorer follows the
  mode); STOP_SCAN also cancels every NavigateToPose goal and publishes zero
  ``/cmd_vel``;
* COLLECT_OBJECT -> ``/arm/command`` pose sequence; ESTOP / CLEAR_ESTOP ->
  ``/rover/estop``; DRIVE -> timed ``/cmd_vel`` in TELEOP mode;
* frames addressed to 0x02 / 0x03 / 0x04 are routed over ``/ssp/to_<mcu>``;
  MCU replies come back on ``/ssp/from_<mcu>`` and are sent to the radio.
"""
from __future__ import annotations

import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, Range
from std_msgs.msg import Bool, UInt8, UInt8MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from rover_core.protocol import Addr, Fault, Mode
from rover_core.ssp import Frame
from rover_interfaces.msg import ArmCommand, EnvironmentTelemetry, RoverStatus
from rover_interfaces.srv import SetMode

from .conversions import (LEGACY_IMAGE_SIZE, ULTRASONIC_NAMES, TelemetrySnapshot, gray_from_raw,
                          placeholder_image, resize_nearest, rpy_from_quaternion)
from .ground_station_core import ROUTED, GroundStationCore
from .ros_common import LATCHED_QOS, RELIABLE_QOS, bytes_to_msg, msg_to_bytes, run_node
from .serial_link import SerialLink, parse_frames

try:  # optional: better image conversion / JPEG for v2 GET_IMAGE
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None
try:
    from cv_bridge import CvBridge  # type: ignore
except ImportError:  # pragma: no cover
    CvBridge = None


class GroundStationBridge(Node):
    def __init__(self) -> None:
        super().__init__("ground_station_bridge")
        dp = self.declare_parameter
        port = dp("port", "/dev/rover_radio").value
        self.baud = int(dp("baud", 9600).value)
        self.own_addr = int(dp("own_address", int(Addr.ROVER)).value)
        transparency = bool(dp("transparency", False).value)
        ibt = float(dp("inter_byte_timeout_s", 0.05).value)
        drive_v = float(dp("drive_max_speed", 0.4).value)
        drive_w = float(dp("drive_max_yaw_rate", 0.8).value)
        drive_hz = float(dp("drive_rate_hz", 10.0).value)
        self.map_frame = dp("map_frame", "map").value
        self.base_frame = dp("base_frame", "base_link").value
        self.chunk_gap_s = float(dp("image_chunk_gap_s", 0.02).value)
        self.v2_size = (int(dp("v2_image_width", 160).value), int(dp("v2_image_height", 120).value))
        poses = list(dp("collect_poses", [2, 4, 3, 5, 6, 7, 4, 1]).value)
        holds = list(dp("collect_hold_s", [2.0, 1.0, 2.5, 1.5, 2.5, 2.5, 1.0, 2.0]).value)
        self.collect_steps: List[Tuple[int, float]] = list(zip(poses, holds))
        mode_period = float(dp("mode_publish_period_s", 1.0).value)
        self.mode = int(dp("initial_mode", int(Mode.IDLE)).value)
        if self.mode not in {int(m) for m in Mode}:
            self.mode = int(Mode.IDLE)
        nav_action = dp("navigate_action", "navigate_to_pose").value
        image_topic = dp("image_topic", "camera/image_raw").value

        # ---------------------------------------------------------- outputs
        self.mode_pub = self.create_publisher(UInt8, "rover/mode", LATCHED_QOS)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", RELIABLE_QOS)
        self.arm_pub = self.create_publisher(ArmCommand, "arm/command", RELIABLE_QOS)
        self.estop_pub = self.create_publisher(Bool, "rover/estop", RELIABLE_QOS)
        self.route_pubs = {addr: self.create_publisher(UInt8MultiArray, f"ssp/to_{name}",
                                                       RELIABLE_QOS)
                           for addr, name in ROUTED.items()}
        self.cancel_client = self.create_client(CancelGoal, f"{nav_action}/_action/cancel_goal")
        self.create_service(SetMode, "rover/set_mode", self._srv_set_mode)

        # ---------------------------------------------------------- inputs
        self._env: Optional[EnvironmentTelemetry] = None
        self._imu: Optional[Imu] = None
        self._odom: Optional[Odometry] = None
        self._image: Optional[Image] = None
        self._mobility_faults = 0
        self._battery_v = float("nan")
        self._ranges = [float("inf")] * len(ULTRASONIC_NAMES)
        for name in ROUTED.values():
            self.create_subscription(UInt8MultiArray, f"ssp/from_{name}", self._on_from_mcu,
                                     RELIABLE_QOS)
        self.create_subscription(EnvironmentTelemetry, "environment",
                                 lambda m: setattr(self, "_env", m), RELIABLE_QOS)
        self.create_subscription(Imu, "imu/data", lambda m: setattr(self, "_imu", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "odometry/filtered",
                                 lambda m: setattr(self, "_odom", m), RELIABLE_QOS)
        self.create_subscription(RoverStatus, "rover/status", self._on_status, RELIABLE_QOS)
        self.create_subscription(Bool, "rover/estop", self._on_estop_topic, RELIABLE_QOS)
        for i, name in enumerate(ULTRASONIC_NAMES):
            self.create_subscription(Range, f"ultrasonic/{name}",
                                     lambda m, i=i: self._ranges.__setitem__(i, m.range),
                                     qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, lambda m: setattr(self, "_image", m),
                                 qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------- state
        self._estop_requested = False
        self._drive_cmd = (0.0, 0.0)
        self._drive_until: Optional[float] = None
        self._zero_burst = 0
        self._collect_idx: Optional[int] = None
        self._collect_next_t = 0.0
        self._captured_gray: Optional[np.ndarray] = None
        self._captured_full: Optional[np.ndarray] = None
        self._placeholder_count = 0
        self._cv_bridge = CvBridge() if CvBridge is not None else None
        self._tx_lock = threading.Lock()
        self._start = time.monotonic()

        self.link = SerialLink(port, self.baud, timeout_s=ibt, transparency=transparency,
                               own_addr=None).start()
        self.core = GroundStationCore(self, self.own_addr, drive_v, drive_w)
        self.create_timer(0.01, self._poll)
        self.create_timer(1.0 / max(1.0, drive_hz), self._drive_tick)
        self.create_timer(0.1, self._collect_tick)
        self.create_timer(max(0.1, mode_period), self._publish_mode)
        self._publish_mode()
        self.get_logger().info(
            f"ground station link {port} @ {self.baud} baud, address 0x{self.own_addr:02X}; "
            f"cv_bridge={'yes' if self._cv_bridge else 'no'} opencv={'yes' if cv2 else 'no'}")

    # ================================================================ radio I/O
    def _poll(self) -> None:
        for fr in self.link.poll():
            try:
                responses = self.core.handle(fr)
            except Exception as exc:  # noqa: BLE001 - never kill the link on one frame
                self.get_logger().error(f"error handling {fr!r}: {exc}")
                continue
            if len(responses) > 3:          # image transfer: paced, in a worker thread
                threading.Thread(target=self._send_paced, args=(responses,), daemon=True).start()
            else:
                for r in responses:
                    self.link.send(r)

    def _send_paced(self, frames: List[Frame]) -> None:
        with self._tx_lock:                  # one transfer at a time
            for fr in frames:
                if not rclpy.ok() or not self.link.send(fr):
                    return
                # wait for the UART to drain (10 bits/byte) so the OS buffer never fills
                time.sleep((len(fr.data) + 8) * 10.0 / self.baud + self.chunk_gap_s)

    def _on_from_mcu(self, msg: UInt8MultiArray) -> None:
        for fr in parse_frames(msg_to_bytes(msg)):
            self.link.send(fr)

    # ================================================================ ports (see GroundStationCore)
    def get_mode(self) -> int:
        return self.mode

    def set_mode(self, mode: int) -> None:
        mode = int(mode)
        if mode == self.mode:
            return
        prev, self.mode = self.mode, mode
        self.get_logger().info(f"mode {Mode(prev).name} -> {Mode(mode).name}")
        if prev == Mode.TELEOP:
            self._drive_until = None
        if mode == Mode.IDLE:
            self._zero_burst = max(self._zero_burst, 5)
        self._publish_mode()

    def estop_latched(self) -> bool:
        return self._estop_requested or bool(self._mobility_faults & Fault.ESTOP)

    def estop(self, engage: bool) -> None:
        self._apply_estop(engage)
        self.estop_pub.publish(Bool(data=bool(engage)))

    def _apply_estop(self, engage: bool) -> None:
        self._estop_requested = bool(engage)
        if engage:
            self.stop_navigation()
            self._abort_collect("e-stop")
            self.set_mode(Mode.IDLE)
        else:  # the next HEARTBEAT re-asserts the bit if the hardware switch is still pressed
            self._mobility_faults &= ~int(Fault.ESTOP)

    def _on_estop_topic(self, msg: Bool) -> None:
        # e-stop engaged or cleared by another node or from the ROS command line: keep this
        # bridge in step. Our own messages are ignored because the state already matches.
        if bool(msg.data) != self._estop_requested:
            self._apply_estop(bool(msg.data))

    def drive(self, v: float, w: float, duration_s: float) -> None:
        self._drive_cmd = (v, w)
        self._drive_until = time.monotonic() + max(0.0, duration_s)

    def stop_navigation(self) -> None:
        if self.cancel_client.service_is_ready():
            # zero goal id + zero stamp = cancel all goals (action_msgs/CancelGoal)
            self.cancel_client.call_async(CancelGoal.Request())
        else:
            self.get_logger().warn("NavigateToPose cancel service not available")
        self._drive_until = None
        self._zero_burst = max(self._zero_burst, 5)

    def start_collect(self) -> bool:
        if self._collect_idx is not None or not self.collect_steps:
            return False
        if self.mode == Mode.AUTONOMOUS:
            self.stop_navigation()
        if self.mode != Mode.IDLE:
            self.set_mode(Mode.IDLE)                 # never move while the arm is down
        self._collect_idx = 0
        self._collect_next_t = time.monotonic()
        return True

    def capture_image(self) -> bool:
        msg = self._image
        if msg is not None:
            try:
                gray, full = self._convert_image(msg)
                w, h = LEGACY_IMAGE_SIZE
                if cv2 is not None:
                    self._captured_gray = cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA)
                else:
                    self._captured_gray = resize_nearest(gray, w, h)
                self._captured_full = full
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"image conversion failed: {exc}")
        self._captured_gray = placeholder_image(*LEGACY_IMAGE_SIZE, counter=self._placeholder_count)
        self._captured_full = None
        self._placeholder_count += 1
        return False

    def image_gray(self) -> np.ndarray:
        if self._captured_gray is None:
            self.capture_image()
        return self._captured_gray

    def image_v2(self, quality: int) -> Tuple[bytes, int, int, int]:
        gray = self.image_gray()
        q = int(quality) if 1 <= int(quality) <= 100 else 50
        if cv2 is not None:
            src = self._captured_full if self._captured_full is not None else gray
            img = cv2.resize(src, self.v2_size, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if ok:
                return buf.tobytes(), self.v2_size[0], self.v2_size[1], 1
        return gray.tobytes(), gray.shape[1], gray.shape[0], 0

    def telemetry(self) -> TelemetrySnapshot:
        s = TelemetrySnapshot(mode=self.mode,
                              fault_flags=self._mobility_faults | (Fault.ESTOP if self._estop_requested else 0),
                              battery_v=self._battery_v,
                              uptime_s=time.monotonic() - self._start,
                              ranges=list(self._ranges))
        if self._env is not None:
            e = self._env
            s.temperature_c, s.humidity_pct, s.pressure_pa = e.temperature_c, e.humidity_pct, e.pressure_pa
            s.mq2_raw, s.mq4_raw, s.soil_raw, s.rain_raw = e.mq2_raw, e.mq4_raw, e.soil_raw, e.rain_raw
            s.metal_detected = e.metal_detected
        if self._imu is not None:
            q = self._imu.orientation
            s.roll, s.pitch, _ = rpy_from_quaternion(q.x, q.y, q.z, q.w)
            a, g = self._imu.linear_acceleration, self._imu.angular_velocity
            s.ax, s.ay, s.az, s.gx, s.gy, s.gz = a.x, a.y, a.z, g.x, g.y, g.z
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            q = t.transform.rotation
            s.x, s.y = t.transform.translation.x, t.transform.translation.y
            s.heading = rpy_from_quaternion(q.x, q.y, q.z, q.w)[2]
        except TransformException:
            if self._odom is not None:
                p = self._odom.pose.pose
                s.x, s.y = p.position.x, p.position.y
                s.heading = rpy_from_quaternion(p.orientation.x, p.orientation.y,
                                                p.orientation.z, p.orientation.w)[2]
        return s

    def route(self, dest: int, raw: bytes) -> bool:
        pub = self.route_pubs.get(int(dest))
        if pub is None or pub.get_subscription_count() == 0:
            return False                      # bridge not running -> NACK(HW_FAULT)
        pub.publish(bytes_to_msg(raw))
        return True

    # ================================================================ helpers
    def _convert_image(self, msg: Image) -> Tuple[np.ndarray, np.ndarray]:
        if self._cv_bridge is not None and cv2 is not None:
            full = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            return cv2.cvtColor(full, cv2.COLOR_BGR2GRAY), full
        gray = gray_from_raw(bytes(msg.data), msg.width, msg.height, msg.encoding, msg.step)
        return gray, gray

    def _on_status(self, msg: RoverStatus) -> None:
        if msg.source == Addr.MOBILITY:
            was_latched = bool(self._mobility_faults & Fault.ESTOP)
            self._mobility_faults = int(msg.fault_flags)
            self._battery_v = float(msg.battery_v)
            if (self._mobility_faults & Fault.ESTOP) and not was_latched \
                    and not self._estop_requested:
                # Latched on the Mega itself (hardware switch, or an ESTOP that did not come
                # through this bridge): act as for a GS E-STOP, so that clearing it can never
                # restart exploration or a sample sequence by itself.
                self.get_logger().warn("mobility MCU reports e-stop latched (hardware switch?)")
                self.estop(True)

    def _publish_mode(self) -> None:
        self.mode_pub.publish(UInt8(data=int(self.mode)))

    def _srv_set_mode(self, req, resp):
        m = int(req.mode)
        if m not in {int(x) for x in Mode}:
            resp.success, resp.message = False, f"invalid mode {m}"
        elif m in (Mode.AUTONOMOUS, Mode.TELEOP) and self.estop_latched():
            resp.success, resp.message = False, "e-stop latched"
        else:
            if self.mode == Mode.AUTONOMOUS and m != Mode.AUTONOMOUS:
                self.stop_navigation()
            self.set_mode(m)
            resp.success, resp.message = True, f"mode {Mode(m).name}"
        resp.mode = self.mode
        return resp

    def _drive_tick(self) -> None:
        if self._drive_until is not None:
            if time.monotonic() < self._drive_until and self.mode == Mode.TELEOP:
                v, w = self._drive_cmd
                t = Twist()
                t.linear.x, t.angular.z = float(v), float(w)
                self.cmd_pub.publish(t)
                return
            self._drive_until = None
            self._zero_burst = max(self._zero_burst, 3)
        if self._zero_burst > 0:
            self.cmd_pub.publish(Twist())
            self._zero_burst -= 1

    def _abort_collect(self, why: str) -> None:
        if self._collect_idx is not None:
            self._collect_idx = None
            self.get_logger().warn(f"sample collection sequence aborted ({why})")

    def _collect_tick(self) -> None:
        if self._collect_idx is None:
            return
        if self.estop_latched():      # also the hardware switch, seen in the mobility HEARTBEAT
            self._abort_collect("e-stop latched")
            return
        if time.monotonic() < self._collect_next_t:
            return
        if self._collect_idx >= len(self.collect_steps):
            self._collect_idx = None
            self.get_logger().info("sample collection sequence finished")
            return
        pose_id, hold = self.collect_steps[self._collect_idx]
        cmd = ArmCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.use_pose = True
        cmd.pose_id = int(pose_id)
        self.arm_pub.publish(cmd)
        self._collect_idx += 1
        self._collect_next_t = time.monotonic() + float(hold)

    def destroy_node(self):
        self.link.stop()
        return super().destroy_node()


def main(args=None) -> None:
    run_node(GroundStationBridge, args)


if __name__ == "__main__":
    main()
