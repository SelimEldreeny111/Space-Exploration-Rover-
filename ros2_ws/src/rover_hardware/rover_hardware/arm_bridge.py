"""arm_bridge: ROS 2 <-> Arduino Uno + PCA9685 arm MCU (SSP address 0x04, ICD v2.0).

* ``/arm/command`` (rover_interfaces/ArmCommand) -> ARM_SET_POSE (use_pose)
  or ARM_SET_JOINTS (angles clamped to the ICD section 9.5 limits).
* ARM_STATE (5 Hz and on change) -> ``/arm/joint_states`` (sensor_msgs/JointState,
  servo angles in rad) and ``/arm/moving`` (std_msgs/Bool).
* ACK / NACK from the MCU are logged; HEARTBEAT -> ``/rover/status``.
"""
from __future__ import annotations

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from rover_core.protocol import PT, Addr, ArmState, Heartbeat, Nack
from rover_interfaces.msg import ArmCommand, RoverStatus

from .conversions import ARM_JOINT_NAMES, arm_command_payload, arm_state_to_radians
from .ros_common import RELIABLE_QOS, McuBridgeNode, make_joint_state, run_node


class ArmBridge(McuBridgeNode):
    def __init__(self) -> None:
        super().__init__("arm_bridge", "/dev/rover_arm", "arm", Addr.ARM)
        self.js_pub = self.create_publisher(JointState, "arm/joint_states", RELIABLE_QOS)
        self.moving_pub = self.create_publisher(Bool, "arm/moving", RELIABLE_QOS)
        self.status_pub = self.create_publisher(RoverStatus, "rover/status", RELIABLE_QOS)
        self.create_subscription(ArmCommand, "arm/command", self._on_command, RELIABLE_QOS)

    def _on_command(self, msg: ArmCommand) -> None:
        try:
            ptype, data = arm_command_payload(msg.use_pose, msg.pose_id, list(msg.joint_deg),
                                              msg.duration_ms)
        except ValueError as exc:
            self.get_logger().error(f"rejected arm command: {exc}")
            return
        if not self.send(ptype, data):
            self.get_logger().warn("arm link down, command dropped")

    def handle_frame(self, fr) -> None:
        if fr.type == PT.ARM_STATE:
            st = ArmState.unpack(fr.data)
            self.js_pub.publish(make_joint_state(self.get_clock().now().to_msg(), ARM_JOINT_NAMES,
                                                 arm_state_to_radians(st)))
            self.moving_pub.publish(Bool(data=bool(st.moving)))
        elif fr.type == PT.ACK:
            self.get_logger().debug(f"arm ACK {fr.data.hex()}")
        elif fr.type == PT.NACK:
            n = Nack.unpack(fr.data)
            self.get_logger().warn(f"arm NACK type=0x{n.rejected_type:02X} reason={n.reason}")
        elif fr.type == PT.HEARTBEAT:
            hb = Heartbeat.unpack(fr.data)
            msg = RoverStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.source, msg.mode, msg.fault_flags = int(fr.src), hb.mode, hb.fault_flags
            msg.battery_v = float("nan")
            msg.uptime_s = hb.uptime_ms / 1000.0
            self.status_pub.publish(msg)


def main(args=None) -> None:
    run_node(ArmBridge, args)


if __name__ == "__main__":
    main()
