"""ROS-free helpers of the ``rover_sim`` node (unit-tested without rclpy)."""
from __future__ import annotations

import array
import math
import os
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class RateSchedule:
    """Fires at a fixed rate in *simulated* time; missed periods are skipped
    (no bursts after a slow step)."""
    rate_hz: float
    next_t: float = 0.0

    def due(self, t: float) -> bool:
        if self.rate_hz <= 0.0:
            return False
        if t + 1e-9 < self.next_t:
            return False
        period = 1.0 / self.rate_hz
        self.next_t += period
        if self.next_t <= t:
            self.next_t = t + period
        return True


class CollisionCounter:
    """Counts collision *events* of a ``rover_core.sim.RoverSim``.

    ``RoverSim.collided`` latches at the first contact (it is never reset) and
    ``RoverSim.collisions`` counts colliding *steps*; a rover pushing against
    an obstacle re-collides intermittently (it creeps forward between hits).
    An event is therefore a colliding step after more than ``gap_s`` of
    simulated time without contact."""

    def __init__(self, gap_s: float = 1.0) -> None:
        self.gap_s = gap_s
        self.events = 0
        self._steps = 0
        self._last_hit_t = -math.inf

    def update(self, t: float, colliding_steps: int) -> bool:
        """Feed the simulated time and ``RoverSim.collisions`` after every
        step; returns True when a new collision event starts."""
        hit = colliding_steps > self._steps
        self._steps = colliding_steps
        if not hit:
            return False
        new = t - self._last_hit_t > self.gap_s
        self._last_hit_t = t
        if new:
            self.events += 1
        return new


def float32_array(values) -> array.array:
    """Fast conversion for float32[] message fields (LaserScan.ranges)."""
    a = array.array("f")
    a.frombytes(np.ascontiguousarray(values, dtype=np.float32).tobytes())
    return a


def laser_scan_fields(n: int, angle_min: float, increment: float, rate_hz: float) -> dict:
    """LaserScan metadata for *n* beams starting at *angle_min*."""
    scan_time = 1.0 / rate_hz if rate_hz > 0 else 0.0
    return {"angle_min": float(angle_min),
            "angle_max": float(angle_min + (n - 1) * increment),
            "angle_increment": float(increment),
            "scan_time": float(scan_time),
            "time_increment": float(scan_time / n) if n else 0.0}


def clean_scan(ranges: Sequence[float], range_min: float, range_max: float) -> np.ndarray:
    """NaN or out-of-range readings -> +inf (REP-117 "no return")."""
    r = np.asarray(ranges, dtype=np.float64).copy()
    bad = ~np.isfinite(r) | (r < range_min) | (r > range_max)
    r[bad] = np.inf
    return r


def body_twist_from_poses(p0: Tuple[float, float, float], p1: Tuple[float, float, float],
                          dt: float) -> Tuple[float, float, float]:
    """Finite-difference body-frame twist (vx, vy, wz) between two planar poses."""
    if dt <= 0:
        return 0.0, 0.0, 0.0
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    th = p0[2] + 0.5 * math.remainder(p1[2] - p0[2], 2 * math.pi)   # midpoint heading
    c, s = math.cos(th), math.sin(th)
    return ((c * dx + s * dy) / dt, (-s * dx + c * dy) / dt,
            math.remainder(p1[2] - p0[2], 2 * math.pi) / dt)


def seconds_to_stamp(t: float) -> Tuple[int, int]:
    """Float seconds -> (sec, nanosec) without nanosec == 1e9 overflow."""
    ns = int(round(t * 1e9))
    return ns // 1_000_000_000, ns % 1_000_000_000


def resolve_world_path(value: str, worlds_dir: Optional[str]) -> Optional[str]:
    """'' -> None (use ``World.default_lab()``); a bare name is looked up in
    the package ``worlds`` directory (``lab`` or ``lab.yaml``)."""
    if not value:
        return None
    if os.path.isabs(value) or os.path.exists(value):
        return value
    if worlds_dir:
        for cand in (value, value + ".yaml"):
            p = os.path.join(worlds_dir, cand)
            if os.path.exists(p):
                return p
    raise FileNotFoundError(f"world '{value}' not found (worlds dir: {worlds_dir})")
