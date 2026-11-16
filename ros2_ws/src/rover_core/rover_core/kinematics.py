"""Skid-steer kinematics and wheel odometry for the six-wheel rover.

The rover has no steering actuators: the three wheels on each side are driven
together, so it behaves as a skid-steer (differential) vehicle. Lateral wheel
slip during turns is captured with the *effective track* b_eff = chi * track
(Mandow et al., IROS 2007; Martinez et al., IJRR 2005).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from statistics import median
from typing import Sequence, Tuple

from .params import DEFAULT, RoverParams


def twist_to_sides(v: float, w: float, b_eff: float) -> Tuple[float, float]:
    """Body twist (v [m/s], w [rad/s]) -> (left, right) wheel surface speeds [m/s]."""
    return v - w * b_eff / 2.0, v + w * b_eff / 2.0


def sides_to_twist(v_left: float, v_right: float, b_eff: float) -> Tuple[float, float]:
    """(left, right) wheel surface speeds -> body twist (v, w)."""
    return (v_left + v_right) / 2.0, (v_right - v_left) / b_eff


def limit_twist(v: float, w: float, p: RoverParams = DEFAULT, wheel_max: float = None) -> Tuple[float, float]:
    """Clamp a twist to the software limits while preserving the curvature."""
    v = max(-p.max_speed, min(p.max_speed, v))
    w = max(-p.max_yaw_rate, min(p.max_yaw_rate, w))
    if wheel_max is None:
        wheel_max = p.motor.no_load_omega * p.wheel_radius * 0.9
    vl, vr = twist_to_sides(v, w, p.b_eff)
    m = max(abs(vl), abs(vr))
    if m > wheel_max:
        s = wheel_max / m
        v, w = v * s, w * s
    return v, w


def robust_side_delta(deltas: Sequence[float]) -> float:
    """Combine the three wheel displacements of one side.

    The median rejects a single slipping, spinning or airborne wheel (common
    on a rocker-bogie while climbing), unlike the plain mean.
    """
    return median(deltas)


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def compose_local(self, dx: float, dy: float, dth: float) -> "Pose2D":
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(self.x + c * dx - s * dy, self.y + s * dx + c * dy,
                      wrap_angle(self.theta + dth))


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class WheelOdometry:
    """Integrates six encoder tick counters into a planar pose.

    * ticks are cumulative signed counters (+ = forward), ICD order
      (LF, LM, LR, RF, RM, RR)
    * wraps of the 32-bit counters are handled
    * optional gyro yaw-rate fusion: if ``gyro_z`` is passed to :meth:`update`
      the heading increment comes from the gyro (weight ``gyro_weight``),
      which removes most of the skid-steer heading error.
    """

    def __init__(self, params: RoverParams = DEFAULT, gyro_weight: float = 0.0) -> None:
        self.p = params
        self.gyro_weight = gyro_weight
        self.pose = Pose2D()
        self.v = 0.0
        self.w = 0.0
        self._last_ticks = None
        self._last_t = None

    def reset(self, pose: Pose2D = None) -> None:
        self.pose = pose or Pose2D()
        self._last_ticks = None
        self._last_t = None

    @staticmethod
    def _diff32(a: int, b: int) -> int:
        d = (a - b) & 0xFFFFFFFF
        return d - (1 << 32) if d & 0x80000000 else d

    def update(self, ticks: Sequence[int], t: float, gyro_z: float = None) -> Pose2D:
        if self._last_ticks is None:
            self._last_ticks, self._last_t = list(ticks), t
            return self.pose
        dt = t - self._last_t
        mpc = self.p.metres_per_count
        d = [self._diff32(a, b) * mpc for a, b in zip(ticks, self._last_ticks)]
        self._last_ticks, self._last_t = list(ticks), t
        dl = robust_side_delta(d[0:3])
        dr = robust_side_delta(d[3:6])
        ds = (dl + dr) / 2.0
        dth_wheels = (dr - dl) / self.p.b_eff
        if gyro_z is not None and dt > 0 and self.gyro_weight > 0:
            dth = self.gyro_weight * gyro_z * dt + (1 - self.gyro_weight) * dth_wheels
        else:
            dth = dth_wheels
        # exact arc integration (midpoint heading)
        self.pose = self.pose.compose_local(ds * math.cos(dth / 2), ds * math.sin(dth / 2), dth)
        if dt > 0:
            self.v, self.w = ds / dt, dth / dt
        return self.pose
