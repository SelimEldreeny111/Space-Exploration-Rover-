"""explorer: frontier-based autonomous exploration for the MISSION X rover.

Started / stopped by ``/rover/mode`` (std_msgs/UInt8): a transition to
AUTONOMOUS (2) starts exploring, any other mode cancels the current goal and
stops (``autostart:=true`` starts without waiting for the mode, e.g. in the
simulator evaluation). Every ``planning_period_s`` it detects frontiers on
``/map`` (rover_mission.frontiers), sends the best one to Nav2 as a
``NavigateToPose`` goal, blacklists rejected / aborted / timed-out goals and
re-plans when the target frontier has been explored on the way. When no
frontier remains it (optionally) drives back to the start pose (map origin),
publishes ``COMPLETE`` on ``/explorer/status`` and asks the ground-station
bridge for mode IDLE via ``/rover/set_mode``.
"""
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, UInt8
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rover_interfaces.srv import SetMode

from .frontiers import FrontierExplorer, FrontierParams, GridInfo, grid_from_occupancy

MODE_IDLE = 0
MODE_AUTONOMOUS = 2


def _yaw_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Explorer(Node):
    def __init__(self) -> None:
        super().__init__("explorer")
        dp = self.declare_parameter
        self.map_frame = dp("map_frame", "map").value
        self.base_frame = dp("base_frame", "base_footprint").value
        self.period = float(dp("planning_period_s", 2.0).value)
        self.goal_timeout = float(dp("goal_timeout_s", 120.0).value)
        self.return_to_start = bool(dp("return_to_start", True).value)
        self.patience = int(dp("no_frontier_patience", 3).value)
        self.replan_radius = float(dp("replan_radius", 0.5).value)
        autostart = bool(dp("autostart", False).value)
        self.params = FrontierParams(
            free_thresh=int(dp("free_thresh", 25).value),
            occ_thresh=int(dp("occ_thresh", 65).value),
            min_frontier_size=int(dp("min_frontier_size", 8).value),
            clearance=float(dp("clearance", 0.45).value),
            w_distance=float(dp("w_distance", 1.0).value),
            w_size=float(dp("w_size", 0.5).value),
            min_goal_distance=float(dp("min_goal_distance", 0.5).value),
            blacklist_radius=float(dp("blacklist_radius", 0.6).value))
        self.explorer = FrontierExplorer(self.params)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, dp("map_topic", "map").value, self._on_map, map_qos)
        self.create_subscription(UInt8, "rover/mode", self._on_mode, 10)
        self.status_pub = self.create_publisher(String, "explorer/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "explorer/frontiers", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "explorer/goal", 10)
        self.nav = ActionClient(self, NavigateToPose, dp("navigate_action", "navigate_to_pose").value)
        self.set_mode_client = self.create_client(SetMode, "rover/set_mode")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_msg: Optional[OccupancyGrid] = None
        self.active = False
        self.returning = False
        self.status = "IDLE"
        self.last_mode: Optional[int] = None
        self.goal_seq = 0
        self.goal_handle = None
        self.goal_pending = False
        self.goal_xy: Optional[Tuple[float, float]] = None
        self.goal_sent_t = 0.0
        self.cancel_reason = ""
        self.no_frontier_count = 0
        self.goals_reached = 0
        self.reached = []
        self.create_timer(self.period, self._plan)
        self.create_timer(1.0, lambda: self._publish_status(self.status))
        if autostart:
            self._start()

    # ------------------------------------------------------------ mode handling
    def _on_mode(self, msg: UInt8) -> None:
        mode = int(msg.data)
        if mode == self.last_mode:
            return                                  # the mode is re-published periodically
        self.last_mode = mode
        if mode == MODE_AUTONOMOUS:
            self._start()
        elif self.active or self.returning:
            self._stop("mode changed")

    def _start(self) -> None:
        self.get_logger().info("exploration started")
        self.active, self.returning = True, False
        self.no_frontier_count = 0
        self.explorer.reset()
        self.reached = []
        self._set_status("EXPLORING")

    def _stop(self, why: str) -> None:
        self.get_logger().info(f"exploration stopped ({why})")
        self.active = self.returning = False
        self._cancel_goal("stopped")
        self._set_status("IDLE")

    # ------------------------------------------------------------ planning loop
    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def _plan(self) -> None:
        if not self.active:
            return
        if self.map_msg is None:
            self._set_status("WAITING_FOR_MAP")
            return
        xy = self._robot_xy()
        if xy is None:
            self._set_status("WAITING_FOR_TF")
            return
        if not self.nav.server_is_ready():
            self._set_status("WAITING_FOR_NAV2")
            return
        m = self.map_msg
        info = GridInfo(m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                        m.info.width, m.info.height)
        grid = grid_from_occupancy(m.data, m.info.width, m.info.height)

        if self.goal_pending or self.goal_handle is not None:
            if time.monotonic() - self.goal_sent_t > self.goal_timeout:
                self.get_logger().warn(f"goal {self.goal_xy} timed out -> blacklisted")
                self.explorer.blacklist_goal(self.goal_xy)
                self._cancel_goal("timeout")
            elif (not self.goal_pending and
                  not self.explorer.frontier_near(grid, info, self.goal_xy, self.replan_radius)):
                self.get_logger().info("target frontier explored on the way -> re-planning")
                self._cancel_goal("replan")
            return
        self._set_status("EXPLORING")
        candidates = self.explorer.candidates(grid, info, xy)
        self._publish_markers(candidates)
        best = None
        for cand in candidates:
            # a frontier that survives a visit cannot be observed from there
            if any(math.dist(cand.goal, r) <= self.params.blacklist_radius for r in self.reached):
                self.explorer.blacklist_goal(cand.goal)
                continue
            best = cand
            break
        if best is None:
            self.no_frontier_count += 1
            if self.no_frontier_count >= self.patience:
                self._complete()
            return
        self.no_frontier_count = 0
        yaw = math.atan2(best.goal[1] - xy[1], best.goal[0] - xy[0])
        self._send_goal(best.goal, yaw)

    def _complete(self) -> None:
        self.get_logger().info(f"no frontiers left after {self.goals_reached} goals: exploration complete")
        self.active = False
        if self.return_to_start:
            self.returning = True
            self._set_status("RETURNING")
            self._send_goal((0.0, 0.0), 0.0)
        else:
            self._finish()

    def _finish(self) -> None:
        self.returning = False
        self._set_status("COMPLETE")
        if self.set_mode_client.service_is_ready():
            self.set_mode_client.call_async(SetMode.Request(mode=MODE_IDLE))

    # ------------------------------------------------------------ Nav2 goals
    def _send_goal(self, xy: Tuple[float, float], yaw: float) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(xy[0]), float(xy[1])
        q = _yaw_quaternion(yaw)
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_xy = (float(xy[0]), float(xy[1]))
        self.goal_pending = True
        self.goal_sent_t = time.monotonic()
        self.cancel_reason = ""
        self.goal_pub.publish(goal.pose)
        self.get_logger().info(f"goal #{seq}: ({xy[0]:.2f}, {xy[1]:.2f})")
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self._on_goal_response(f, seq))

    def _on_goal_response(self, future, seq: int) -> None:
        handle = future.result()
        if seq != self.goal_seq:
            # superseded or stopped while the goal was pending: cancel it now
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        self.goal_pending = False
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"goal #{seq} rejected -> blacklisted")
            self._goal_failed()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, seq))

    def _on_result(self, future, seq: int) -> None:
        if seq != self.goal_seq:
            return
        self.goal_handle = None
        status = future.result().status
        if self.returning:
            self._finish()
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            self.reached.append(self.goal_xy)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"goal #{seq} aborted -> blacklisted")
            self._goal_failed()
        # CANCELED: re-plan / timeout / stop were handled where the cancel was issued

    def _goal_failed(self) -> None:
        self.goal_handle = None
        if self.returning:
            self._finish()
        elif self.goal_xy is not None:
            self.explorer.blacklist_goal(self.goal_xy)

    def _cancel_goal(self, reason: str) -> None:
        self.cancel_reason = reason
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_pending = False
        self.goal_seq += 1                        # ignore late callbacks of the old goal

    # ------------------------------------------------------------ outputs
    def _set_status(self, status: str) -> None:
        if status != self.status:
            self.status = status
            self._publish_status(status)

    def _publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))

    def _publish_markers(self, frontiers) -> None:
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        pts = Marker()
        pts.header.frame_id = self.map_frame
        pts.header.stamp = self.get_clock().now().to_msg()
        pts.ns, pts.id, pts.type, pts.action = "frontier_goals", 0, Marker.SPHERE_LIST, Marker.ADD
        pts.scale.x = pts.scale.y = pts.scale.z = 0.15
        pts.color.r, pts.color.g, pts.color.b, pts.color.a = 0.1, 0.6, 1.0, 0.9
        pts.pose.orientation.w = 1.0
        pts.points = [Point(x=float(f.goal[0]), y=float(f.goal[1]), z=0.05) for f in frontiers]
        arr.markers.append(pts)
        self.marker_pub.publish(arr)


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = Explorer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
