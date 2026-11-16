"""Lightweight rover simulator (kinematics + sensor models) — "digital twin".

Models (all parameters from the ICD / datasheets, see rover_core.params):

* Skid-steer drive: the wheel-speed controller uses the *nominal* effective
  track ``b_eff = chi_nominal * track``; the vehicle actually turns with
  ``chi_true`` (unknown to the controller and to odometry) -> systematic
  heading error as on the real rover.
* Side-speed dynamics: first-order lag (tau) + acceleration limit + motor
  torque-speed limit; longitudinal slip (random, slowly varying); per-wheel
  rotation noise and occasional free-spin events of single wheels (to
  exercise the median wheel fusion of the odometry).
* Encoders: 840 counts / wheel revolution (ICD §9.2), quantised.
* IMU (MPU-6050): gyro bias + white noise, accelerometer noise.
* LiDAR (LDS-01 / HLS-LFCD2): 360 beams at 1 deg, 0.12-3.5 m, 5 Hz, noise
  sigma = 10 mm (< 0.5 m) or 1.75 % of range, 2 % dropouts; sees only
  obstacles taller than its mounting height.
* Ultrasonic (HC-SR04): 15 deg cone sampled with 5 rays, 0.02-4 m, sigma 1 cm;
  sees an obstacle if its top is above the lower edge of the cone.
* Collision: footprint rectangle vs obstacles taller than ``climb_height``.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..kinematics import twist_to_sides, wrap_angle
from ..params import DEFAULT, G_EARTH, RoverParams
from .world import World


def raycast(world: World, x: float, y: float, angles: np.ndarray, rmax: float,
            min_height=None, sensor_height: float = 0.0, half_fov: float = 0.0,
            step: Optional[float] = None) -> np.ndarray:
    """Distance to the first visible obstacle along each ray (inf if none).

    Visibility: if ``min_height`` is given, an obstacle is visible when its
    height >= min_height (planar LiDAR). Otherwise the cone rule is used:
    height >= sensor_height - r * tan(half_fov) (lower cone edge), height > 0.
    """
    step = step or world.resolution * 0.5
    n = int(math.ceil(rmax / step))
    r = (np.arange(1, n + 1) * step).astype(np.float32)
    ca, sa = np.cos(angles).astype(np.float32), np.sin(angles).astype(np.float32)
    xs = x + ca[:, None] * r[None, :]
    ys = y + sa[:, None] * r[None, :]
    h = world.height_at(xs, ys)
    if min_height is not None:
        hit = h >= min_height
    else:
        thr = sensor_height - r * math.tan(half_fov)
        hit = (h > 0.0) & (h >= thr[None, :])
    first = hit.argmax(axis=1)
    any_hit = hit.any(axis=1)
    # the surface lies between the last free and the first occupied sample:
    # centre the estimate to remove the +step/2 quantisation bias
    return np.where(any_hit, r[first] - 0.5 * step, np.inf).astype(np.float64)


class RoverSim:
    def __init__(self, world: World, params: RoverParams = DEFAULT, seed: int = 0,
                 chi_true: Optional[float] = None, start: Optional[Tuple[float, float, float]] = None,
                 tau: float = 0.15, accel_limit: float = 0.8, long_slip: float = 0.03,
                 wheel_noise: float = 0.01, spin_event_rate: float = 0.02,
                 gyro_bias: Optional[float] = None, gyro_noise: float = 0.004,
                 accel_noise: float = 0.05, climb_height: float = 0.08,
                 lidar_offset: Tuple[float, float] = (0.05, 0.0), noise: bool = True) -> None:
        self.world = world
        self.p = params
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.chi_true = params.skid_chi * 1.08 if chi_true is None else chi_true
        x, y, th = start if start is not None else world.start
        self.x, self.y, self.th = float(x), float(y), float(th)
        self.t = 0.0
        self.tau, self.accel_limit = tau, accel_limit
        self.long_slip_mean = long_slip
        self.wheel_noise = wheel_noise
        self.spin_event_rate = spin_event_rate
        self.gyro_bias = (self.rng.normal(0.0, 0.003) if gyro_bias is None else gyro_bias) if noise else 0.0
        self.gyro_noise = gyro_noise if noise else 0.0
        self.accel_noise = accel_noise if noise else 0.0
        self.climb_height = climb_height
        self.lidar_offset = lidar_offset
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.side = np.zeros(2)            # actual left/right surface speeds [m/s]
        self.wheel_angle = np.zeros(6)     # rad, ICD order LF LM LR RF RM RR
        self.v = 0.0                       # true body speed
        self.w = 0.0                       # true body yaw rate
        self._prev_v = 0.0
        self._acc = 0.0
        self._slip = self.long_slip_mean
        self._spin = np.zeros(6)           # remaining duration of free-spin events [s]
        self.collided = False
        self.collisions = 0
        self.distance = 0.0

    # ------------------------------------------------------------ commands
    def set_cmd(self, v: float, w: float) -> None:
        self.cmd_v = float(np.clip(v, -self.p.max_speed, self.p.max_speed))
        self.cmd_w = float(np.clip(w, -self.p.max_yaw_rate * 1.5, self.p.max_yaw_rate * 1.5))

    @property
    def true_pose(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.th)

    # ------------------------------------------------------------ physics
    def step(self, dt: float) -> None:
        p = self.p
        tgt = np.array(twist_to_sides(self.cmd_v, self.cmd_w, p.b_eff))
        # motor speed limit (torque-speed line at the rated load -> 63 rpm)
        vmax = p.motor.rated_rpm * 2 * math.pi / 60 * p.wheel_radius
        tgt = np.clip(tgt, -vmax, vmax)
        dv = (tgt - self.side) * min(1.0, dt / self.tau)
        dv = np.clip(dv, -self.accel_limit * dt, self.accel_limit * dt)
        self.side = self.side + dv
        vl, vr = self.side
        # slowly varying longitudinal slip
        if self.noise:
            self._slip += (self.long_slip_mean - self._slip) * dt / 2.0 + self.rng.normal(0, 0.01) * math.sqrt(dt)
            self._slip = float(np.clip(self._slip, 0.0, 0.15))
        slip = self._slip if self.noise else 0.0
        v = 0.5 * (vl + vr) * (1.0 - slip)
        w = (vr - vl) / (self.chi_true * p.track)
        # integrate (exact arc)
        x0, y0, th0 = self.x, self.y, self.th
        if abs(w) > 1e-9:
            R = v / w
            th1 = th0 + w * dt
            x1 = x0 + R * (math.sin(th1) - math.sin(th0))
            y1 = y0 - R * (math.cos(th1) - math.cos(th0))
        else:
            th1 = th0
            x1 = x0 + v * dt * math.cos(th0)
            y1 = y0 + v * dt * math.sin(th0)
        if self._footprint_collides(x1, y1, th1):
            self.collided = True
            self.collisions += 1
            self.side[:] = 0.0
            v = w = 0.0
        else:
            self.x, self.y, self.th = x1, y1, wrap_angle(th1)
            self.distance += abs(v) * dt
        # wheels rotate with their side speed (+ noise, + free-spin events)
        rot = np.array([vl, vl, vl, vr, vr, vr]) / p.wheel_radius
        if self.noise:
            rot = rot * (1.0 + self.rng.normal(0.0, self.wheel_noise, 6))
            new = self.rng.random(6) < self.spin_event_rate * dt
            self._spin[new] = self.rng.uniform(0.2, 0.8, new.sum())
            spinning = self._spin > 0
            rot[spinning] *= 1.35
            self._spin = np.maximum(0.0, self._spin - dt)
        self.wheel_angle += rot * dt
        self._acc = (v - self._prev_v) / dt if dt > 0 else 0.0
        self._prev_v = v
        self.v, self.w = v, w
        self.t += dt

    def _footprint_collides(self, x: float, y: float, th: float) -> bool:
        L, W = self.p.length, self.p.width
        u = np.linspace(-L / 2, L / 2, 21)
        s = np.linspace(-W / 2, W / 2, 15)
        U, S = np.meshgrid(u, s)
        c, sn = math.cos(th), math.sin(th)
        X = x + c * U - sn * S
        Y = y + sn * U + c * S
        return bool((self.world.height_at(X, Y) > self.climb_height).any())

    # ------------------------------------------------------------ sensors
    def wheel_ticks(self) -> List[int]:
        cpr = self.p.counts_per_wheel_rev
        return [int(math.floor(a / (2 * math.pi) * cpr)) for a in self.wheel_angle]

    def imu(self) -> Dict[str, float]:
        n = self.rng.normal if self.noise else (lambda m, s: m)
        return {
            "ax": self._acc + n(0.0, self.accel_noise),
            "ay": self.v * self.w + n(0.0, self.accel_noise),
            "az": G_EARTH + n(0.0, self.accel_noise),
            "gx": n(0.0, self.gyro_noise), "gy": n(0.0, self.gyro_noise),
            "gz": self.w + self.gyro_bias + n(0.0, self.gyro_noise),
        }

    def lidar_pose(self) -> Tuple[float, float, float]:
        ox, oy = self.lidar_offset
        c, s = math.cos(self.th), math.sin(self.th)
        return self.x + c * ox - s * oy, self.y + s * ox + c * oy, self.th

    def scan(self) -> Tuple[np.ndarray, float, float]:
        p = self.p
        lx, ly, lth = self.lidar_pose()
        n = p.lidar_beams
        inc = 2 * math.pi / n
        rel = np.arange(n) * inc
        rmin, rmax = p.lidar_range
        r = raycast(self.world, lx, ly, lth + rel, rmax + 0.05, min_height=p.lidar_height - 0.02)
        if self.noise:
            sig = np.where(r < 0.5, 0.010, 0.0175 * r)
            r = r + self.rng.normal(0.0, 1.0, n) * np.where(np.isfinite(r), sig, 0.0)
            r[self.rng.random(n) < 0.02] = np.inf
        r[(r < rmin) | (r > rmax)] = np.inf
        return r, 0.0, inc

    def ultrasonic(self) -> List[float]:
        p = self.p
        out = []
        half = math.radians(p.ultrasonic_fov_deg / 2)
        rmin, rmax = p.ultrasonic_range
        c, s = math.cos(self.th), math.sin(self.th)
        for (ux, uy, uyaw) in p.ultrasonic_poses:
            sx = self.x + c * ux - s * uy
            sy = self.y + s * ux + c * uy
            a0 = self.th + math.radians(uyaw)
            angles = a0 + np.linspace(-half, half, 5)
            r = raycast(self.world, sx, sy, angles, rmax, sensor_height=p.ultrasonic_height,
                        half_fov=half).min()
            if self.noise and math.isfinite(r):
                r += self.rng.normal(0.0, 0.01)
            out.append(r if rmin <= r <= rmax else math.inf)
        return out
