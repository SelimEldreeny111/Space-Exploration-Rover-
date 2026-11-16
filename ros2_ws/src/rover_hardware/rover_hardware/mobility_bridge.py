"""mobility_bridge: ROS 2 <-> Arduino Mega 2560 (SSP address 0x02, ICD v2.0).

* ``/cmd_vel`` (Twist, or TwistStamped with ``use_stamped_cmd_vel``) ->
  SET_VELOCITY, limited with ``rover_core.kinematics.limit_twist`` and
  refreshed at ``cmd_rate_hz`` (>= 10 Hz). A stale command (older than
  ``cmd_vel_timeout_s``) is replaced by zero; the MCU also stops on its own
  if no SET_VELOCITY arrives within ``mcu_timeout_ms`` (default 500 ms).
* WHEEL_ODOM -> ``/wheel/odom`` (nav_msgs/Odometry via
  ``rover_core.kinematics.WheelOdometry``), ``/wheel/joint_states`` and,
  optionally, TF odom -> base_footprint (``publish_tf``, default false: the
  robot_localization EKF owns that transform).
* RANGES -> ``/ultrasonic/<name>`` (sensor_msgs/Range, frame us_<name>_link).
* HEARTBEAT -> ``/rover/status`` (rover_interfaces/RoverStatus).
* E-stop: ``/rover/estop`` (std_msgs/Bool, true = engage) and the services
  ``~/estop`` / ``~/clear_estop`` (std_srvs/Trigger) -> ESTOP / CLEAR_ESTOP.
"""
from __future__ import annotations

import math
import time

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Range
from tf2_ros import TransformBroadcaster

from rover_core.kinematics import WheelOdometry
from rover_core.protocol import PT, Addr, Fault, Heartbeat, Ranges, WheelOdom
from rover_interfaces.msg import RoverStatus

from .conversions import (ULTRASONIC_NAMES, WHEEL_JOINTS, odom_covariances, ranges_to_meters,
                          rover_params, twist_to_set_velocity, wheel_angles)
from .ros_common import (RELIABLE_QOS, McuBridgeNode, make_joint_state, make_odometry,
                         make_planar_tf, make_range, run_node, subscribe_cmd_vel)


class MobilityBridge(McuBridgeNode):
    def __init__(self) -> None:
        super().__init__("mobility_bridge", "/dev/rover_mobility", "mobility", Addr.MOBILITY)
        dp = self.declare_parameter
        self.params = rover_params(
            wheel_radius=dp("wheel_radius", 0.100).value,
            track=dp("track", 0.58).value,
            wheelbase=dp("wheelbase", 0.82).value,
            skid_chi=dp("skid_chi", 1.5).value,
            counts_per_wheel_rev=dp("counts_per_wheel_rev", 840).value,
            max_speed=dp("max_speed", 0.45).value,
            max_yaw_rate=dp("max_yaw_rate", 1.0).value,
        )
        self.cmd_rate_hz = max(10.0, float(dp("cmd_rate_hz", 10.0).value))
        self.mcu_timeout_ms = int(dp("mcu_timeout_ms", 500).value)
        self.cmd_vel_timeout_s = float(dp("cmd_vel_timeout_s", 0.5).value)
        stamped = bool(dp("use_stamped_cmd_vel", False).value)
        self.publish_tf = bool(dp("publish_tf", False).value)
        self.odom_frame = dp("odom_frame", "odom").value
        self.base_frame = dp("base_frame", "base_footprint").value
        self.publish_joint_states = bool(dp("publish_joint_states", True).value)
        self.no_echo_as_max = bool(dp("no_echo_as_max_range", False).value)
        self.us_fov = math.radians(float(dp("ultrasonic_fov_deg", 15.0).value))
        self.us_min = float(dp("ultrasonic_min_range", 0.02).value)
        self.us_max = float(dp("ultrasonic_max_range", 4.0).value)

        self.odo = WheelOdometry(self.params)
        self.odom_pub = self.create_publisher(Odometry, "wheel/odom", RELIABLE_QOS)
        self.js_pub = self.create_publisher(JointState, "wheel/joint_states", RELIABLE_QOS)
        self.status_pub = self.create_publisher(RoverStatus, "rover/status", RELIABLE_QOS)
        self.range_pubs = [self.create_publisher(Range, f"ultrasonic/{n}", RELIABLE_QOS)
                           for n in ULTRASONIC_NAMES]
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.cmd = (0.0, 0.0)
        self.last_cmd_t = -1e9
        self.estop_latched = False
        self.mcu_faults = 0
        subscribe_cmd_vel(self, "cmd_vel", stamped, self._on_cmd_vel)
        self.create_subscription(Bool, "rover/estop", lambda m: self._set_estop(m.data),
                                 RELIABLE_QOS)
        self.create_service(Trigger, "~/estop", self._srv_estop)
        self.create_service(Trigger, "~/clear_estop", self._srv_clear)
        self.create_timer(1.0 / self.cmd_rate_hz, self._send_cmd)

    # ------------------------------------------------------------ commands
    def _on_cmd_vel(self, v: float, w: float) -> None:
        self.cmd = (v, w)
        self.last_cmd_t = time.monotonic()

    def _send_cmd(self) -> None:
        v, w = self.cmd
        if self.estop_latched or time.monotonic() - self.last_cmd_t > self.cmd_vel_timeout_s:
            v = w = 0.0
        self.send(PT.SET_VELOCITY,
                  twist_to_set_velocity(v, w, self.params, self.mcu_timeout_ms).pack())

    def _set_estop(self, engage: bool) -> None:
        engage = bool(engage)
        self.estop_latched = engage
        if engage:
            self.cmd = (0.0, 0.0)
            self.send(PT.SET_VELOCITY, twist_to_set_velocity(0.0, 0.0, self.params,
                                                             self.mcu_timeout_ms).pack())
            self.send(PT.ESTOP)
            self.get_logger().warn("E-STOP engaged")
        else:
            self.send(PT.CLEAR_ESTOP)
            self.get_logger().info("E-stop cleared")

    def _srv_estop(self, _req, resp):
        self._set_estop(True)
        resp.success, resp.message = True, "ESTOP sent"
        return resp

    def _srv_clear(self, _req, resp):
        self._set_estop(False)
        resp.success, resp.message = True, "CLEAR_ESTOP sent"
        return resp

    # ------------------------------------------------------------ telemetry
    def handle_frame(self, fr) -> None:
        if fr.type == PT.WHEEL_ODOM:
            self._on_wheel_odom(WheelOdom.unpack(fr.data))
        elif fr.type == PT.RANGES:
            self._on_ranges(Ranges.unpack(fr.data))
        elif fr.type == PT.HEARTBEAT:
            self._on_heartbeat(fr.src, Heartbeat.unpack(fr.data))
        elif fr.type in (PT.ACK, PT.NACK):
            self.get_logger().debug(f"MCU reply {fr!r}")

    def _on_wheel_odom(self, wo: WheelOdom) -> None:
        pose = self.odo.update(wo.ticks, wo.t_ms / 1000.0)
        stamp = self.get_clock().now().to_msg()
        pose_cov, twist_cov = odom_covariances(self.odo.v, self.odo.w)
        self.odom_pub.publish(make_odometry(stamp, self.odom_frame, self.base_frame, pose.x, pose.y,
                                            pose.theta, self.odo.v, self.odo.w, pose_cov,
                                            twist_cov))
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                make_planar_tf(stamp, self.odom_frame, self.base_frame, pose.x, pose.y, pose.theta))
        if self.publish_joint_states:
            self.js_pub.publish(make_joint_state(
                stamp, WHEEL_JOINTS, wheel_angles(wo.ticks, self.params.counts_per_wheel_rev)))

    def _on_ranges(self, r: Ranges) -> None:
        stamp = self.get_clock().now().to_msg()
        values = ranges_to_meters(r.range_mm, self.us_min, self.us_max, self.no_echo_as_max)
        for name, pub, value in zip(ULTRASONIC_NAMES, self.range_pubs, values):
            pub.publish(make_range(stamp, f"us_{name}_link", value, self.us_fov, self.us_min,
                                   self.us_max))

    def _on_heartbeat(self, src: int, hb: Heartbeat) -> None:
        self.mcu_faults = hb.fault_flags
        msg = RoverStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = int(src)
        msg.mode = hb.mode
        msg.fault_flags = hb.fault_flags
        msg.battery_v = float("nan")              # not carried by HEARTBEAT (ICD v2.0)
        msg.uptime_s = hb.uptime_ms / 1000.0
        self.status_pub.publish(msg)
        if hb.fault_flags & Fault.ESTOP and not self.estop_latched:
            self.get_logger().warn("MCU reports a latched e-stop (hardware switch?)")


def main(args=None) -> None:
    run_node(MobilityBridge, args)


if __name__ == "__main__":
    main()
