"""rover_sim_node: lightweight 2.5-D simulator of the MISSION X rover.

Wraps ``rover_core.sim.RoverSim`` and replaces both the MCUs and the LiDAR,
publishing exactly the topics of the real stack, so robot_localization,
slam_toolbox and Nav2 run unchanged:

  /scan (LaserScan, laser_link, 360 beams, best effort)   /wheel/odom
  /imu/data   /ultrasonic/<name> (x6)   /wheel/joint_states
  /ground_truth/odom (frame ``world``)   /sim/collisions   /clock

Subscribes /cmd_vel (limited like mobility_bridge, 0.5 s watchdog) and
/rover/estop. Also broadcasts the static TF world -> map (= start pose, the
origin of a SLAM map) and, optionally, odom -> base_footprint.

The node is the clock source: it always runs on wall time itself (its own
``use_sim_time`` is forced to false) and stamps every message with the
simulated time when ``publish_clock`` is true.
"""
from __future__ import annotations

import math
import os
from typing import Optional

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan, Range
from std_msgs.msg import Bool, UInt32
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from rover_core.kinematics import WheelOdometry, limit_twist
from rover_core.sim import RoverSim, World
from rover_hardware.conversions import (ULTRASONIC_NAMES, WHEEL_JOINTS, normalise_range,
                                        odom_covariances, rover_params, wheel_angles)
from rover_hardware.ros_common import (RELIABLE_QOS, SENSOR_QOS, make_imu, make_joint_state,
                                       make_odometry, make_planar_tf, make_range, run_node,
                                       subscribe_cmd_vel)

from .sim_core import (CollisionCounter, RateSchedule, body_twist_from_poses, clean_scan,
                       float32_array, laser_scan_fields, resolve_world_path, seconds_to_stamp)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None


class RoverSimNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_sim")
        # the simulator IS the clock: never wait for /clock ourselves
        if self.get_parameter("use_sim_time").value:
            self.get_logger().warn("rover_sim must run on wall time; forcing use_sim_time:=false")
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, False)])
        dp = self.declare_parameter
        world_param = dp("world", "").value
        seed = int(dp("seed", 0).value)
        noise = bool(dp("noise", True).value)
        self.rate_hz = float(dp("rate_hz", 50.0).value)
        rtf = float(dp("real_time_factor", 1.0).value)
        self.publish_clock = bool(dp("publish_clock", True).value)
        rates = {k: float(dp(f"{k}_rate_hz", v).value) for k, v in
                 (("scan", 5.0), ("odom", 20.0), ("imu", 25.0), ("ultrasonic", 10.0),
                  ("ground_truth", 20.0))}
        self.laser_frame = dp("laser_frame", "laser_link").value
        self.imu_frame = dp("imu_frame", "imu_link").value
        self.odom_frame = dp("odom_frame", "odom").value
        self.base_frame = dp("base_frame", "base_footprint").value
        self.world_frame = dp("world_frame", "world").value
        self.map_frame = dp("map_frame", "map").value
        publish_world_tf = bool(dp("publish_world_tf", True).value)
        self.publish_odom_tf = bool(dp("publish_odom_tf", False).value)
        self.cmd_timeout = float(dp("cmd_vel_timeout_s", 0.5).value)
        stamped = bool(dp("use_stamped_cmd_vel", False).value)
        self.no_echo_as_max = bool(dp("no_echo_as_max_range", False).value)
        self.rp_var = float(dp("orientation_variance", 3.0e-4).value)
        self.yaw_var = float(dp("yaw_variance", 1.0e3).value)
        self.gyro_var = float(dp("angular_velocity_variance", 4.0e-4).value)
        self.accel_var = float(dp("linear_acceleration_variance", 1.0e-2).value)
        # odometry model of the *software* (the simulated rover slips differently)
        self.params = rover_params(skid_chi=float(dp("skid_chi", 1.5).value))

        worlds_dir = None
        if get_package_share_directory is not None:
            try:
                worlds_dir = os.path.join(get_package_share_directory("rover_sim"), "worlds")
            except Exception:  # noqa: BLE001 - package not installed (tests)
                worlds_dir = None
        path = resolve_world_path(world_param, worlds_dir)
        world = World.from_yaml(path) if path else World.default_lab()
        self.sim = RoverSim(world, params=self.params, seed=seed, noise=noise)
        self.odo = WheelOdometry(self.params)
        self.start_pose = tuple(self.sim.true_pose)
        self.get_logger().info(
            f"world '{path or 'default_lab'}', start pose {tuple(round(v, 3) for v in self.start_pose)}, "
            f"{self.rate_hz:.0f} Hz physics, real-time factor {rtf}")

        # ---------------------------------------------------------- publishers
        self.clock_pub = self.create_publisher(Clock, "/clock", 10) if self.publish_clock else None
        self.scan_pub = self.create_publisher(LaserScan, "scan", SENSOR_QOS)
        self.odom_pub = self.create_publisher(Odometry, "wheel/odom", RELIABLE_QOS)
        self.imu_pub = self.create_publisher(Imu, "imu/data", RELIABLE_QOS)
        self.js_pub = self.create_publisher(JointState, "wheel/joint_states", RELIABLE_QOS)
        self.gt_pub = self.create_publisher(Odometry, "ground_truth/odom", RELIABLE_QOS)
        self.coll_pub = self.create_publisher(UInt32, "sim/collisions", RELIABLE_QOS)
        self.range_pubs = [self.create_publisher(Range, f"ultrasonic/{n}", RELIABLE_QOS)
                           for n in ULTRASONIC_NAMES]
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None
        if publish_world_tf:
            self.static_tf = StaticTransformBroadcaster(self)
            x, y, th = self.start_pose
            self.static_tf.sendTransform(make_planar_tf(self._stamp(), self.world_frame,
                                                        self.map_frame, x, y, th))

        # ---------------------------------------------------------- inputs
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1e9
        self.estop = False
        subscribe_cmd_vel(self, "cmd_vel", stamped, self._on_cmd_vel)
        self.create_subscription(Bool, "rover/estop", self._on_estop, RELIABLE_QOS)

        self.sched = {k: RateSchedule(v) for k, v in rates.items()}
        self.coll_sched = RateSchedule(1.0)
        self.scan_meta: Optional[dict] = None
        self.scan_rate = rates["scan"]
        self.gt_prev = (self.sim.t, self.start_pose)
        # RoverSim.collided latches after the first contact: count events from
        # its per-step counter RoverSim.collisions instead
        self.collision_counter = CollisionCounter(gap_s=1.0)
        self.dt = 1.0 / self.rate_hz
        self.create_timer(self.dt / max(1e-3, rtf), self._tick)

    # ------------------------------------------------------------ helpers
    def _stamp(self) -> TimeMsg:
        if self.publish_clock:
            sec, nsec = seconds_to_stamp(self.sim.t)
            return TimeMsg(sec=sec, nanosec=nsec)
        return self.get_clock().now().to_msg()

    def _on_cmd_vel(self, v: float, w: float) -> None:
        self.cmd = (v, w)
        self.cmd_t = self.sim.t

    def _on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    # ------------------------------------------------------------ main loop
    def _tick(self) -> None:
        v, w = self.cmd
        if self.estop or self.sim.t - self.cmd_t > self.cmd_timeout:   # MCU watchdog
            v = w = 0.0
        self.sim.set_cmd(*limit_twist(v, w, self.params))
        self.sim.step(self.dt)
        if self.collision_counter.update(self.sim.t, self.sim.collisions):
            self.get_logger().warn(
                f"collision #{self.collision_counter.events} at "
                f"{tuple(round(c, 2) for c in self.sim.true_pose)}")

        t = self.sim.t
        stamp = self._stamp()
        if self.clock_pub is not None:
            self.clock_pub.publish(Clock(clock=stamp))
        if self.sched["odom"].due(t):
            self._publish_odom(stamp)
        if self.sched["imu"].due(t):
            self._publish_imu(stamp)
        if self.sched["ultrasonic"].due(t):
            self._publish_ultrasonic(stamp)
        if self.sched["scan"].due(t):
            self._publish_scan(stamp)
        if self.sched["ground_truth"].due(t):
            self._publish_ground_truth(stamp)
        if self.coll_sched.due(t):
            self.coll_pub.publish(UInt32(data=self.collision_counter.events))

    def _publish_odom(self, stamp) -> None:
        ticks = self.sim.wheel_ticks()
        pose = self.odo.update(ticks, self.sim.t)
        pose_cov, twist_cov = odom_covariances(self.odo.v, self.odo.w)
        self.odom_pub.publish(make_odometry(stamp, self.odom_frame, self.base_frame, pose.x, pose.y,
                                            pose.theta, self.odo.v, self.odo.w, pose_cov, twist_cov))
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(make_planar_tf(stamp, self.odom_frame, self.base_frame,
                                                             pose.x, pose.y, pose.theta))
        self.js_pub.publish(make_joint_state(stamp, WHEEL_JOINTS,
                                             wheel_angles(ticks, self.params.counts_per_wheel_rev)))

    def _publish_imu(self, stamp) -> None:
        m = self.sim.imu()
        # flat world: roll = pitch = 0 (the MCU would report its complementary filter)
        self.imu_pub.publish(make_imu(stamp, self.imu_frame, 0.0, 0.0, m["ax"], m["ay"], m["az"],
                                      m["gx"], m["gy"], m["gz"], self.rp_var, self.yaw_var,
                                      self.gyro_var, self.accel_var))

    def _publish_ultrasonic(self, stamp) -> None:
        p = self.params
        rmin, rmax = p.ultrasonic_range
        fov = math.radians(p.ultrasonic_fov_deg)
        for name, pub, r in zip(ULTRASONIC_NAMES, self.range_pubs, self.sim.ultrasonic()):
            pub.publish(make_range(stamp, f"us_{name}_link",
                                   normalise_range(float(r), rmin, rmax, self.no_echo_as_max),
                                   fov, rmin, rmax))

    def _publish_scan(self, stamp) -> None:
        ranges, angle_min, inc = self.sim.scan()
        rmin, rmax = self.params.lidar_range
        if self.scan_meta is None:
            self.scan_meta = laser_scan_fields(len(ranges), angle_min, inc, self.scan_rate)
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.laser_frame
        for k, v in self.scan_meta.items():
            setattr(msg, k, v)
        msg.range_min, msg.range_max = float(rmin), float(rmax)
        msg.ranges = float32_array(clean_scan(ranges, rmin, rmax))
        self.scan_pub.publish(msg)

    def _publish_ground_truth(self, stamp) -> None:
        t0, p0 = self.gt_prev
        p1 = tuple(self.sim.true_pose)
        vx, _vy, wz = body_twist_from_poses(p0, p1, self.sim.t - t0)
        self.gt_prev = (self.sim.t, p1)
        self.gt_pub.publish(make_odometry(stamp, self.world_frame, self.base_frame, p1[0], p1[1],
                                          p1[2], vx, wz, [0.0] * 36, [0.0] * 36))


def main(args=None) -> None:
    run_node(RoverSimNode, args)


if __name__ == "__main__":
    main()
