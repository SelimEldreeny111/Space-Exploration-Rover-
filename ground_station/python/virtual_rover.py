#!/usr/bin/env python3
"""MISSION X virtual rover -- emulates the rover end of the ground-station link.

Answers every legacy and v2 ground-station command (ICD v2.0 §6) the way the
on-board computer ("Brain", address 0x01) does, backed by a small simulation
(skid-steer kinematics, rocks seen by the six ultrasonics, terrain slope ->
roll/pitch, battery drain, gas plume, soil moisture, metal objects, camera).
Frames addressed to the MCUs (0x02/0x03/0x04) are answered as if the Brain had
forwarded them (ICD §5).

Transports: a serial port (e.g. one end of a com0com pair, or a USB-UART +
HC-12 to talk to a real ground station) or a TCP socket.

    python virtual_rover.py --tcp 5760                  # v2 Brain on 127.0.0.1:5760
    python virtual_rover.py --tcp 5760 --legacy         # 2023 rover: 7-byte TLM, legacy commands only
    python virtual_rover.py --tcp 5760 --ber 1e-4       # inject bit errors (both directions)
    python virtual_rover.py --tcp 5760 --pace 9600      # throttle the TCP link to HC-12 throughput
    python virtual_rover.py --port COM8 --baud 9600     # serial
    python virtual_rover.py --tcp 5760 --safe-link      # Mega "safe mode" link (ICD §1)

Pair it with ``gs_cli.py --tcp 127.0.0.1:5760 ...``.
"""
from __future__ import annotations

import argparse
import heapq
import io
import itertools
import logging
import math
import random
import socket
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import gs_common  # noqa: F401  (puts rover_core on sys.path)
from gs_common import LEGACY_CHUNK, LEGACY_CHUNKS, LEGACY_H, LEGACY_W, describe
from rover_core.params import DEFAULT as ROVER
from rover_core.protocol import (IMAGE_CHUNK_PAYLOAD, NO_ECHO, Addr, ArmSetJoints, Drive, Fault,
                                 ImageChunkHeader, Mode, Nack, NackReason, PT, TlmLegacy, TlmV2,
                                 clamp_i16, clamp_i32, clamp_u16, plausible_length)
from rover_core.ssp import Frame, StreamDecoder

log = logging.getLogger("virtual_rover")

G = 9.81
DT = 0.05                      # simulation step [s]
BODY_R = 0.45                  # collision radius of the rover body [m]
JPEG_W, JPEG_H = 160, 120      # v2 JPEG resolution
ARM_LIMITS = [(0, 180), (15, 165), (0, 180), (0, 180), (0, 180), (10, 80)]  # ICD §9.5

#: request DATA lengths the Brain accepts from the GS (others -> NACK BAD_LENGTH)
BRAIN_REQUESTS = {
    PT.PING: {0}, PT.GET_IMAGE: {0, 1}, PT.CAP_IMAGE: {0}, PT.GET_TLM: {0},
    PT.START_SCAN: {0}, PT.STOP_SCAN: {0}, PT.COLLECT_OBJECT: {0}, PT.ESTOP: {0},
    PT.CLEAR_ESTOP: {0}, PT.SET_MODE: {1}, PT.DRIVE: {4}, PT.ARM_SET_POSE: {1},
    PT.ARM_SET_JOINTS: {8},
}
LEGACY_REQUESTS = {PT.PING, PT.GET_IMAGE, PT.CAP_IMAGE, PT.GET_TLM, PT.START_SCAN, PT.STOP_SCAN,
                   PT.COLLECT_OBJECT}
SAFE_LINK_REQUESTS = {PT.PING, PT.GET_TLM, PT.STOP_SCAN, PT.ESTOP, PT.CLEAR_ESTOP}


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_deg(a: float) -> float:
    """-> (-180, 180]"""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


# ===================================================================== world
class World:
    """10 m x 10 m arena (walls at +-HALF) with rocks, metal objects and a gas plume."""
    HALF = 10.0

    def __init__(self, seed: int = 7) -> None:
        rng = random.Random(seed)
        self.rocks: List[Tuple[float, float, float]] = []
        while len(self.rocks) < 16:
            x, y, r = rng.uniform(-9, 9), rng.uniform(-9, 9), rng.uniform(0.2, 0.6)
            if math.hypot(x, y) < 2.2:
                continue
            if any(math.hypot(x - a, y - b) < r + c + 0.9 for a, b, c in self.rocks):
                continue
            self.rocks.append((x, y, r))
        self.metal = [(1.8, 0.3)] + [(rng.uniform(-7, 7), rng.uniform(-7, 7)) for _ in range(5)]
        self.gas = [(rng.uniform(-5, 5), rng.uniform(-5, 5), 1.8), (rng.uniform(-6, 6), rng.uniform(-6, 6), 2.4)]

    @staticmethod
    def height(x: float, y: float) -> float:
        return 0.18 * math.sin(0.45 * x) * math.cos(0.38 * y) + 0.07 * math.sin(1.1 * x + 0.8 * y)

    @staticmethod
    def soil(x: float, y: float) -> float:
        return 600 + 170 * math.sin(0.55 * x + 0.3) * math.cos(0.42 * y)

    def gas_level(self, x: float, y: float, k: int) -> float:
        gx, gy, s = self.gas[k]
        return math.exp(-((x - gx) ** 2 + (y - gy) ** 2) / (2 * s * s))

    def raycast(self, x: float, y: float, ang: float, rmax: float) -> float:
        dx, dy = math.cos(ang), math.sin(ang)
        best = rmax
        for cx, cy, r in self.rocks:
            fx, fy = x - cx, y - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            if disc < 0:
                continue
            sq = math.sqrt(disc)
            t = -b - sq if -b - sq >= 0 else -b + sq
            if 0 <= t < best:
                best = t
        h = self.HALF
        for p, d in ((x, dx), (y, dy)):
            if d > 1e-9:
                best = min(best, (h - p) / d)
            elif d < -1e-9:
                best = min(best, (-h - p) / d)
        return max(0.0, best)


# ================================================================ camera
def _hash01(ix: int, iy: int, seed: int) -> float:
    n = (ix * 374761393 + iy * 668265263 + seed * 1442695041) & 0xFFFFFFFF
    n = ((n ^ (n >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((n ^ (n >> 16)) & 0xFFFF) / 65535.0


def _vnoise(x: float, y: float, seed: int) -> float:
    ix, iy = math.floor(x), math.floor(y)
    fx, fy = x - ix, y - iy
    sx, sy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
    a, b = _hash01(ix, iy, seed), _hash01(ix + 1, iy, seed)
    c, d = _hash01(ix, iy + 1, seed), _hash01(ix + 1, iy + 1, seed)
    return a + (b - a) * sx + (c - a) * sy + (a - b - c + d) * sx * sy


def _fbm(x: float, y: float, seed: int, octaves: int = 4) -> float:
    s, amp, f, norm = 0.0, 1.0, 1.0, 0.0
    for o in range(octaves):
        s += amp * _vnoise(x * f, y * f, seed + o * 101)
        norm += amp
        amp *= 0.5
        f *= 2.0
    return s / norm


@dataclass
class Scene:
    """What the camera saw at CAP_IMAGE time (rendered on demand at any size)."""
    seed: int
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    rock_bearings: Tuple[Tuple[float, float, float], ...] = ()  # (bearing_deg, distance_m, radius_m)

    def render(self, w: int, h: int) -> bytes:
        rng = random.Random(self.seed)
        horizon = h * (0.36 + self.pitch_deg / 70.0)
        tilt = math.tan(math.radians(self.roll_deg))
        rocks = []
        for bearing, dist, rad in self.rock_bearings:  # visible rocks from the world
            cx = 0.5 - bearing / 62.0
            depth = max(0.08, min(1.0, 1.4 / max(dist, 0.3)))
            cy = horizon / h + (1 - horizon / h) * min(0.92, 0.18 + 0.75 * depth)
            rocks.append((cx, cy, max(0.035, 0.09 * rad * depth * 3.0), rng.uniform(0.55, 0.8)))
        for _ in range(2 + rng.randrange(3)):  # pebbles / foreground stones
            rocks.append((rng.uniform(0.05, 0.95), rng.uniform(0.62, 0.95), rng.uniform(0.02, 0.06),
                          rng.uniform(0.5, 0.75)))
        rocks.sort(key=lambda r: r[1])
        out = bytearray(w * h)
        s = self.seed & 0xFFFF
        for py in range(h):
            for px in range(w):
                u, v = px / w, py / h
                hz = horizon + (px - w / 2) * tilt
                if py < hz:
                    ridge = hz - h * (0.05 + 0.06 * _fbm(px / w * 3.0, 0.5, s + 7, 3))
                    if py > ridge:  # distant hills
                        val = 120 + 30 * _fbm(px / 9.0, py / 9.0, s + 3, 3)
                    else:           # dusty sky, brighter towards the horizon
                        val = 150 + 55 * (py / max(1.0, hz)) + 6 * _fbm(u * 3, v * 3, s + 11, 2)
                else:
                    depth = (py - hz) / max(1.0, h - hz)
                    sc = 4.0 + 18.0 * depth
                    tex = _fbm(u * sc, depth * sc * 1.7, s, 4)
                    val = 92 + 70 * tex + 25 * depth
                    for cx, cy, r, shade in rocks:
                        dx, dy = (u - cx) / r, (v - cy) / (r * 0.62)
                        d2 = dx * dx + dy * dy
                        if d2 < 1.0 and dy < 0.55:
                            nz = math.sqrt(1.0 - d2)
                            lam = max(0.0, -0.55 * dx - 0.45 * dy + 0.7 * nz)
                            val = 40 + 170 * shade * lam + 18 * _fbm(u * 40, v * 40, s + 5, 2)
                        elif 0.0 < dy < 1.6 and dx * dx + (dy - 0.55) ** 2 < 1.1:
                            val *= 0.62    # shadow
                val *= 1.0 - 0.25 * ((u - 0.5) ** 2 + (v - 0.5) ** 2)   # vignette
                out[py * w + px] = max(0, min(255, int(val)))
        return bytes(out)

    def jpeg(self, quality: int) -> Optional[bytes]:
        try:
            from PIL import Image
        except ImportError:  # pragma: no cover - Pillow is optional for the rover
            return None
        img = Image.frombytes("L", (JPEG_W, JPEG_H), self.render(JPEG_W, JPEG_H))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=max(1, min(95, quality)))
        return buf.getvalue()


# ================================================================== the rover
class RoverModel:
    """State machine + physics of the rover as seen from the GS link."""

    def __init__(self, *, seed: int = 1, legacy: bool = False, safe_link: bool = False,
                 time_scale: float = 1.0, clock: Callable[[], float] = time.monotonic,
                 world: Optional[World] = None) -> None:
        self.lock = threading.RLock()
        self.rng = random.Random(seed)
        self.seed = seed
        self.legacy = legacy
        self.safe_link = safe_link
        self.time_scale = time_scale
        self.clock = clock
        self.world = world or World()
        self.t0 = self.t = clock()
        self.mode = Mode.SAFE if safe_link else Mode.IDLE
        self.estop = False
        self.cmd_timeout = False
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.dv = 0.0
        self.drive: Optional[Tuple[float, float, float]] = None   # (v, w, until)
        self.soc = 0.86
        self.current = 1.4
        self.collect_until = 0.0
        self.capture_busy_until = 0.0
        self.scene: Optional[Scene] = None
        self.captures = 0
        self.roll = self.pitch = 0.0
        self.roll_rate = self.pitch_rate = 0.0
        self.yaw_drift = 0.0
        self.noise = {k: 0.0 for k in ("temp", "rh", "p", "mq2", "mq4", "soil", "rain")}
        self.ranges_m = [ROVER.ultrasonic_range[1]] * 6
        self._avoid_turn = 0.0
        self.handled: Counter = Counter()
        self._update_ranges()

    # ----------------------------------------------------------- simulation
    def advance(self, now: Optional[float] = None) -> None:
        with self.lock:
            now = self.clock() if now is None else now
            gap = now - self.t
            if gap <= 0:
                return
            if gap > 30.0:           # do not replay long idle periods step by step
                self.t = now - 30.0
                gap = 30.0
            n = max(1, int(math.ceil(gap / DT)))
            dt = gap / n
            for _ in range(n):
                self.t += dt
                self._step(dt)
            self.t = now

    def _update_ranges(self) -> None:
        rmin, rmax = ROVER.ultrasonic_range
        half_fov = math.radians(ROVER.ultrasonic_fov_deg / 2)
        c, s = math.cos(self.th), math.sin(self.th)
        out = []
        for sx, sy, syaw in ROVER.ultrasonic_poses:
            wx, wy = self.x + c * sx - s * sy, self.y + s * sx + c * sy
            a = self.th + math.radians(syaw)
            r = min(self.world.raycast(wx, wy, a + k * half_fov, rmax + 0.01) for k in (-1, 0, 1))
            out.append(max(rmin, r))
        self.ranges_m = out

    def ranges_mm(self) -> List[int]:
        rmax = ROVER.ultrasonic_range[1]
        return [NO_ECHO if r > rmax else clamp_u16(r * 1000) for r in self.ranges_m]

    def _wander(self, t: float) -> Tuple[float, float]:
        fl, fc, fr, left, right = self.ranges_m[0:5]
        front = min(fl, fc, fr)
        if front < 0.9:
            if not self._avoid_turn:                  # commit to one direction (no dithering)
                self._avoid_turn = 1.0 if fl >= fr else -1.0   # +w = turn left
            return (0.0 if front < 0.5 else 0.08), 0.75 * self._avoid_turn
        self._avoid_turn = 0.0
        if min(left, right) < 0.5:                    # keep the flanks clear of rocks
            return 0.15, 0.6 * (1.0 if right < left else -1.0)
        return 0.28, 0.32 * math.sin(t / 5.0) + 0.14 * math.sin(t / 2.3 + 1.0)

    def _collides(self, x: float, y: float) -> bool:
        """Rover body (radius BODY_R) against rocks and arena walls."""
        if abs(x) > World.HALF - BODY_R or abs(y) > World.HALF - BODY_R:
            return True
        return any(math.hypot(x - cx, y - cy) < r + BODY_R for cx, cy, r in self.world.rocks)

    def _step(self, dt: float) -> None:
        t = self.t - self.t0
        if self.drive is not None and self.t >= self.drive[2]:
            if self.drive[0] or self.drive[1]:
                self.cmd_timeout = True      # watchdog stopped the motors
            self.drive = None
        v_cmd = w_cmd = 0.0
        collecting = self.t < self.collect_until
        if self.estop or collecting:
            pass
        elif self.mode == Mode.AUTONOMOUS:
            v_cmd, w_cmd = self._wander(t)
        elif self.drive is not None:
            v_cmd, w_cmd = self.drive[0], self.drive[1]
        a = min(1.0, dt / 0.25)              # first-order motor response
        v_old = self.v
        self.v += (v_cmd - self.v) * a
        self.w += (w_cmd - self.w) * a
        if self.estop:
            self.v = self.w = 0.0
        self._update_ranges()
        if self.v > 0 and min(self.ranges_m[0:3]) < 0.3:
            self.v = 0.0                     # bumper: do not drive into rocks
        if self.v < 0 and self.ranges_m[5] < 0.3:
            self.v = 0.0
        self.dv = (self.v - v_old) / dt
        self.th = wrap_pi(self.th + self.w * dt)
        nx = self.x + self.v * math.cos(self.th) * dt
        ny = self.y + self.v * math.sin(self.th) * dt
        if self._collides(nx, ny) and not self._collides(self.x, self.y):
            self.v = 0.0                               # blocked: stay put
        else:
            self.x, self.y = nx, ny
        # attitude from terrain slope (+pitch nose up, +roll right side down)
        h, ds, c, s = self.world.height, 0.3, math.cos(self.th), math.sin(self.th)
        pitch_t = math.degrees(math.atan2(h(self.x + ds * c, self.y + ds * s) - h(self.x - ds * c, self.y - ds * s),
                                          2 * ds))
        roll_t = math.degrees(math.atan2(h(self.x - ds * s, self.y + ds * c) - h(self.x + ds * s, self.y - ds * c),
                                         2 * ds))
        vib = 0.6 * abs(self.v) / ROVER.max_speed
        k = min(1.0, dt / 0.3)
        new_pitch = self.pitch + (pitch_t - self.pitch) * k + self.rng.gauss(0, vib * 0.3 + 0.01)
        new_roll = self.roll + (roll_t - self.roll) * k + self.rng.gauss(0, vib * 0.3 + 0.01)
        self.pitch_rate, self.roll_rate = (new_pitch - self.pitch) / dt, (new_roll - self.roll) / dt
        self.pitch, self.roll = new_pitch, new_roll
        self.yaw_drift += 0.005 * dt         # gyro drift 0.3 deg/min
        # power
        self.current = (1.4 + 7.0 * abs(self.v) / ROVER.max_speed + 2.0 * abs(self.w) / ROVER.max_yaw_rate
                        + (1.5 if collecting else 0.0))
        self.soc = max(0.0, self.soc - self.current * dt / 3600.0 / 5.0)
        # slowly varying sensor noise (AR(1))
        for key, sigma in (("temp", 0.05), ("rh", 0.15), ("p", 1.5), ("mq2", 2.0), ("mq4", 1.5),
                           ("soil", 1.5), ("rain", 0.8)):
            self.noise[key] = 0.995 * self.noise[key] + self.rng.gauss(0, sigma)

    # ------------------------------------------------------------- sensors
    def battery_V(self) -> float:
        return 11.0 + 1.6 * self.soc ** 1.3 - 0.035 * self.current + self.rng.gauss(0, 0.005)

    def env(self) -> dict:
        t = self.t - self.t0
        x, y, n = self.x, self.y, self.noise
        near_metal = any(math.hypot(x - mx, y - my) < 0.6 for mx, my in self.world.metal)
        return {
            "temp": 26.5 + 2.5 * math.sin(2 * math.pi * t / 900.0) + n["temp"],
            "rh": 41.0 + 5.0 * math.sin(2 * math.pi * t / 1200.0 + 1.0) + n["rh"],
            "p": 100850.0 + 40.0 * math.sin(2 * math.pi * t / 1800.0) - 12.0 * self.world.height(x, y) + n["p"],
            "mq2": 140 + 420 * self.world.gas_level(x, y, 0) + n["mq2"],
            "mq4": 95 + 300 * self.world.gas_level(x, y, 1) + n["mq4"],
            "soil": self.world.soil(x, y) + n["soil"],
            "rain": 1010 - abs(n["rain"]) * 3,
            "metal": 1 if near_metal else 0,
        }

    def faults(self) -> int:
        f = 0
        if self.estop:
            f |= Fault.ESTOP
        if self.cmd_timeout:
            f |= Fault.CMD_TIMEOUT
        if self.battery_V() < 10.8:
            f |= Fault.LOW_BATTERY
        if self.safe_link:
            f |= Fault.SAFE_MODE
        return f

    def tlm_v2(self) -> TlmV2:
        e = self.env()
        rr, pr = math.radians(self.roll), math.radians(self.pitch)
        vib = 300 * abs(self.v) / ROVER.max_speed
        noise = lambda: self.rng.gauss(0, vib + 15)  # noqa: E731
        rng_mm = self.ranges_mm()
        valid = [r for r in rng_mm if r != NO_ECHO]
        t = TlmV2(
            tlm_version=2, mode=int(self.mode), fault_flags=self.faults(),
            battery_mV=clamp_u16(self.battery_V() * 1000), uptime_s=int(self.t - self.t0),
            temperature_cC=clamp_i16(e["temp"] * 100), humidity_cP=clamp_u16(e["rh"] * 100),
            pressure_Pa=int(round(e["p"])), roll_cdeg=clamp_i16(self.roll * 100),
            pitch_cdeg=clamp_i16(self.pitch * 100),
            yaw_cdeg=clamp_i16(wrap_deg(math.degrees(self.th) + self.yaw_drift) * 100),
            ax=clamp_i16((G * math.sin(pr) + self.dv) * 1000 + noise()),
            ay=clamp_i16((G * math.sin(rr) * math.cos(pr) + self.v * self.w) * 1000 + noise()),
            az=clamp_i16(G * math.cos(rr) * math.cos(pr) * 1000 + noise()),
            gx=clamp_i16(math.radians(self.roll_rate) * 1000), gy=clamp_i16(math.radians(self.pitch_rate) * 1000),
            gz=clamp_i16(self.w * 1000 + self.rng.gauss(0, 3)),
            mq2_raw=int(max(0, min(1023, e["mq2"]))), mq4_raw=int(max(0, min(1023, e["mq4"]))),
            soil_raw=int(max(0, min(1023, e["soil"]))), rain_raw=int(max(0, min(1023, e["rain"]))),
            proximity=e["metal"], x_mm=clamp_i32(self.x * 1000), y_mm=clamp_i32(self.y * 1000),
            heading_cdeg=clamp_i16(wrap_deg(math.degrees(self.th)) * 100),
            range_min_mm=min(valid) if valid else NO_ECHO,
        )
        if self.safe_link:  # Mega safe mode: fields it cannot measure are 0 (ICD §7.1)
            keep = {"tlm_version", "mode", "fault_flags", "battery_mV", "uptime_s", "range_min_mm"}
            for name in list(vars(t)):
                if name not in keep:
                    setattr(t, name, 0)
        return t

    def tlm_legacy(self) -> TlmLegacy:
        e = self.env()
        gas = int(max(0, min(1023, e["mq2"])))
        cm = [0 if r == NO_ECHO else min(255, r // 10) for r in self.ranges_mm()]
        return TlmLegacy(ir=e["metal"], gas_hi=gas >> 8, gas_lo=gas & 0xFF, us1_cm=cm[1], us2_cm=cm[5],
                         temp_c=max(0, min(255, int(round(e["temp"])))),
                         rh_pct=max(0, min(255, int(round(e["rh"])))))

    # -------------------------------------------------------------- camera
    def capture(self) -> Scene:
        self.captures += 1
        seen = []
        for cx, cy, r in self.world.rocks:
            d = math.hypot(cx - self.x, cy - self.y)
            b = math.degrees(wrap_pi(math.atan2(cy - self.y, cx - self.x) - self.th))
            if d < 6.0 and abs(b) < 31:
                seen.append((b, d, r))
        self.scene = Scene(seed=(self.seed * 7919 + self.captures * 104729) & 0x7FFFFFFF,
                           pitch_deg=self.pitch, roll_deg=self.roll, rock_bearings=tuple(seen))
        return self.scene

    def image_frames(self, req: Frame, me: int, quality: Optional[int]) -> List[Frame]:
        scene = self.scene or self.capture()
        if quality is None:                          # legacy: 50 x 200 B, type 0x08
            px = scene.render(LEGACY_W, LEGACY_H)
            return [Frame(req.src, me, PT.GET_IMAGE, px[i * LEGACY_CHUNK:(i + 1) * LEGACY_CHUNK])
                    for i in range(LEGACY_CHUNKS)]
        blob, w, h, fmt = None, LEGACY_W, LEGACY_H, 0
        if quality > 0:
            blob = scene.jpeg(quality)
            if blob is not None:
                w, h, fmt = JPEG_W, JPEG_H, 1
        if blob is None:
            blob = scene.render(LEGACY_W, LEGACY_H)
        total = (len(blob) + IMAGE_CHUNK_PAYLOAD - 1) // IMAGE_CHUNK_PAYLOAD
        return [Frame(req.src, me, PT.IMAGE_CHUNK,
                      ImageChunkHeader(i, total, w, h, fmt).pack()
                      + blob[i * IMAGE_CHUNK_PAYLOAD:(i + 1) * IMAGE_CHUNK_PAYLOAD]) for i in range(total)]

    # ------------------------------------------------------------- commands
    def handle(self, fr: Frame, now: Optional[float] = None) -> List[Tuple[float, Frame]]:
        """Process one received frame; return [(delay_s, response_frame), ...]."""
        with self.lock:
            now = self.clock() if now is None else now
            self.advance(now)
            if fr.dest not in (Addr.ROVER, Addr.MOBILITY, Addr.PAYLOAD, Addr.ARM, Addr.BROADCAST):
                return []
            self.handled[fr.type] += 1
            me = Addr.ROVER if fr.dest == Addr.BROADCAST else fr.dest
            if self.legacy:
                return self._handle_legacy(fr, now)
            if self.safe_link:
                return self._handle_safe_link(fr, now)
            if me != Addr.ROVER:
                return self._handle_mcu(fr, me, now)
            return self._handle_brain(fr, me, now)

    def _out(self, fr: Frame, me: int, ptype: int, data: bytes = b"", delay: float = 0.0):
        return delay * self.time_scale, Frame(fr.src, me, ptype, bytes(data))

    def _ack(self, fr: Frame, me: int, delay: float = 0.003):
        return [self._out(fr, me, PT.ACK, bytes([fr.type]), delay)]

    def _nack(self, fr: Frame, me: int, reason: int):
        return [self._out(fr, me, PT.NACK, Nack(fr.type, int(reason)).pack(), 0.003)]

    def _handle_brain(self, fr: Frame, me: int, now: float) -> List[Tuple[float, Frame]]:
        t, d = fr.type, bytes(fr.data)
        if t not in BRAIN_REQUESTS:
            return self._nack(fr, me, NackReason.UNKNOWN_TYPE)
        if len(d) not in BRAIN_REQUESTS[t]:
            return self._nack(fr, me, NackReason.BAD_LENGTH)
        collecting = now < self.collect_until
        if t == PT.PING:
            return [self._out(fr, me, PT.ACK, b"", 0.002)]          # LEN 0: legacy-compatible
        if t == PT.GET_TLM:
            return [self._out(fr, me, PT.GET_TLM, self.tlm_v2().pack(), 0.005)]
        if t == PT.CAP_IMAGE:
            if now < self.capture_busy_until:
                return self._nack(fr, me, NackReason.BUSY)
            self.capture()
            self.capture_busy_until = now + 0.35 * self.time_scale
            return [self._out(fr, me, PT.CAP_IMAGE, b"", 0.35)]      # echo after storing
        if t == PT.GET_IMAGE:
            frames = self.image_frames(fr, me, d[0] if d else None)
            return [(0.05 * self.time_scale, f) for f in frames]
        if t == PT.START_SCAN:
            if self.estop or self.mode == Mode.SAFE:
                return self._nack(fr, me, NackReason.NOT_PERMITTED)
            if collecting:
                return self._nack(fr, me, NackReason.BUSY)
            self.mode, self.drive = Mode.AUTONOMOUS, None
            return [self._out(fr, me, PT.START_SCAN, b"", 0.01)]
        if t == PT.STOP_SCAN:
            if self.mode != Mode.SAFE:
                self.mode = Mode.IDLE
            self.drive, self.v, self.w = None, 0.0, 0.0
            return [self._out(fr, me, PT.STOP_SCAN, b"", 0.01)]
        if t == PT.COLLECT_OBJECT:
            if self.estop:
                return self._nack(fr, me, NackReason.NOT_PERMITTED)
            if collecting:
                return self._nack(fr, me, NackReason.BUSY)
            if self.mode == Mode.AUTONOMOUS:
                self.mode = Mode.IDLE
            self.drive = None
            self.collect_until = now + 6.0 * self.time_scale
            return [self._out(fr, me, PT.COLLECT_OBJECT, b"", 0.01)]
        if t == PT.ESTOP:
            self.estop, self.drive, self.v, self.w = True, None, 0.0, 0.0
            if self.mode != Mode.SAFE:
                self.mode = Mode.IDLE
            return self._ack(fr, me, 0.002)
        if t == PT.CLEAR_ESTOP:
            self.estop = False
            self.cmd_timeout = False
            return self._ack(fr, me)
        if t == PT.SET_MODE:
            m = d[0]
            if m > Mode.SAFE:
                return self._nack(fr, me, NackReason.NOT_PERMITTED)
            if self.estop and m in (Mode.TELEOP, Mode.AUTONOMOUS):
                return self._nack(fr, me, NackReason.NOT_PERMITTED)
            self.mode = Mode(m)
            self.drive = None
            self.cmd_timeout = False
            return self._ack(fr, me)
        if t == PT.DRIVE:
            if self.estop:
                return self._nack(fr, me, NackReason.NOT_PERMITTED)
            if self.mode == Mode.AUTONOMOUS or collecting:
                return self._nack(fr, me, NackReason.BUSY)
            cmd = Drive.unpack(d)
            if self.mode == Mode.IDLE:
                self.mode = Mode.TELEOP
            scale = 0.3 if self.mode == Mode.SAFE else 1.0
            v = max(-100, min(100, cmd.v_pct)) / 100.0 * ROVER.max_speed * scale
            w = max(-100, min(100, cmd.w_pct)) / 100.0 * ROVER.max_yaw_rate * scale
            self.drive = (v, w, now + cmd.duration_ms / 1000.0) if (v or w) and cmd.duration_ms else None
            self.cmd_timeout = False
            return self._ack(fr, me)
        if t in (PT.ARM_SET_POSE, PT.ARM_SET_JOINTS):
            return self._arm(fr, me)
        return self._nack(fr, me, NackReason.UNKNOWN_TYPE)  # pragma: no cover

    def _arm(self, fr: Frame, me: int):
        if self.estop:
            return self._nack(fr, me, NackReason.NOT_PERMITTED)
        if fr.type == PT.ARM_SET_POSE:
            if len(fr.data) != 1:
                return self._nack(fr, me, NackReason.BAD_LENGTH)
            if fr.data[0] > 7:
                return self._nack(fr, me, NackReason.NOT_PERMITTED)
            return self._ack(fr, me)
        if len(fr.data) != ArmSetJoints.size():
            return self._nack(fr, me, NackReason.BAD_LENGTH)
        js = ArmSetJoints.unpack(bytes(fr.data))
        if any(not lo <= v <= hi for v, (lo, hi) in zip(js.deg, ARM_LIMITS)):
            return self._nack(fr, me, NackReason.NOT_PERMITTED)
        return self._ack(fr, me)

    def _handle_mcu(self, fr: Frame, me: int, now: float):
        """Frames the Brain forwards to an MCU (ICD §5); the MCU answers."""
        t = fr.type
        if t == PT.PING:
            return [self._out(fr, me, PT.ACK, b"", 0.008)]
        if me == Addr.MOBILITY and t in (PT.ESTOP, PT.CLEAR_ESTOP) and not fr.data:
            self.estop = t == PT.ESTOP
            if self.estop:
                self.drive, self.v, self.w = None, 0.0, 0.0
            return self._ack(fr, me, 0.008)
        if me == Addr.MOBILITY and t == PT.SET_VELOCITY and len(fr.data) == 6:
            return []                                    # no response (ICD §6.2)
        if me == Addr.ARM and t in (PT.ARM_SET_POSE, PT.ARM_SET_JOINTS):
            return self._arm(fr, me)
        return self._nack(fr, me, NackReason.UNKNOWN_TYPE)

    def _handle_legacy(self, fr: Frame, now: float):
        """2023 single-MCU rover: legacy command set only, silent on anything else."""
        me = Addr.ROVER
        if fr.dest not in (Addr.ROVER, Addr.BROADCAST) or fr.type not in LEGACY_REQUESTS:
            return []
        t = fr.type
        if t == PT.PING:
            return [self._out(fr, me, PT.ACK, b"", 0.004)]
        if t == PT.GET_TLM:
            return [self._out(fr, me, PT.GET_TLM, self.tlm_legacy().pack(), 0.01)]
        if t == PT.GET_IMAGE:
            return [(0.05 * self.time_scale, f) for f in self.image_frames(fr, me, None)]
        if t == PT.CAP_IMAGE:
            self.capture()
            return [self._out(fr, me, t, b"", 0.35)]
        if t == PT.START_SCAN:
            self.mode = Mode.AUTONOMOUS
        elif t == PT.STOP_SCAN:
            self.mode, self.drive = Mode.IDLE, None
        elif t == PT.COLLECT_OBJECT:
            self.collect_until = now + 6.0 * self.time_scale
        return [self._out(fr, me, t, b"", 0.01)]

    def _handle_safe_link(self, fr: Frame, now: float):
        """HC-12 wired to the Mega (ICD §1 L1-safe): PING/GET_TLM/STOP_SCAN/ESTOP(+CLEAR)."""
        me = Addr.MOBILITY
        if fr.dest not in (Addr.ROVER, Addr.MOBILITY, Addr.BROADCAST):
            return []
        if fr.type not in SAFE_LINK_REQUESTS:
            return self._nack(fr, me, NackReason.NOT_PERMITTED)
        if fr.data:
            return self._nack(fr, me, NackReason.BAD_LENGTH)
        t = fr.type
        if t == PT.PING:
            return [self._out(fr, me, PT.ACK, b"", 0.004)]
        if t == PT.GET_TLM:
            return [self._out(fr, me, PT.GET_TLM, self.tlm_v2().pack(), 0.006)]
        if t == PT.STOP_SCAN:
            self.drive, self.v, self.w = None, 0.0, 0.0
            return [self._out(fr, me, PT.STOP_SCAN, b"", 0.004)]
        self.estop = t == PT.ESTOP
        if self.estop:
            self.drive, self.v, self.w = None, 0.0, 0.0
        return self._ack(fr, me, 0.004)


# ============================================================ link emulation
class BitErrorInjector:
    """Flips bits with probability ``ber`` (independent errors, geometric gaps)."""

    def __init__(self, ber: float, rng: random.Random) -> None:
        self.ber = ber
        self.rng = rng
        self.flipped = 0
        self._gap = self._next_gap()

    def _next_gap(self) -> int:
        if self.ber <= 0:
            return 1 << 62
        return int(math.log(1.0 - self.rng.random()) / math.log(1.0 - self.ber))

    def apply(self, data: bytes) -> bytes:
        if self.ber <= 0 or not data:
            return data
        nbits = len(data) * 8
        if self._gap >= nbits:
            self._gap -= nbits
            return data
        out = bytearray(data)
        pos = self._gap
        while pos < nbits:
            out[pos >> 3] ^= 0x80 >> (pos & 7)
            self.flipped += 1
            pos += 1 + self._next_gap()
        self._gap = pos - nbits
        return bytes(out)


class RoverLink:
    """Runs a :class:`RoverModel` behind a byte stream.

    ``on_bytes()`` is fed with received bytes (uplink BER applied first); the
    responses are queued with their processing delay and written by a writer
    thread (downlink BER applied, optional pacing at ``pace_baud`` to emulate
    the HC-12 throughput on a TCP link)."""

    def __init__(self, model: RoverModel, write_fn: Callable[[bytes], object], *, ber_up: float = 0.0,
                 ber_down: float = 0.0, pace_baud: int = 0, transparency: bool = False,
                 timeout_s: Optional[float] = 0.05, length_check: bool = False, seed: int = 99,
                 name: str = "link") -> None:
        self.model = model
        self.write_fn = write_fn
        self.pace_baud = pace_baud
        self.transparency = transparency
        self.name = name
        rng = random.Random(seed)
        self.up = BitErrorInjector(ber_up, rng)
        self.down = BitErrorInjector(ber_down, rng)
        # ICD §3: 50 ms inter-byte timeout; the §3-5a length check is optional on
        # the Brain (without it, wrong lengths of known types are NACKed BAD_LENGTH)
        self.decoder = StreamDecoder(own_addr=None, transparency=transparency, timeout_s=timeout_s,
                                     length_check=plausible_length if length_check else None)
        self.tx_frames = self.tx_bytes = self.rx_frames = 0
        self._heap: list = []
        self._seq = itertools.count()
        self._cv = threading.Condition()
        self._stop = False
        self._writer = threading.Thread(target=self._write_loop, name=f"{name}-tx", daemon=True)
        self._writer.start()

    def on_bytes(self, data: bytes) -> None:
        now = time.monotonic()
        for fr in self.decoder.feed(self.up.apply(data)):
            self.rx_frames += 1
            log.info("%s RX %s", self.name, describe(fr))
            for delay, rsp in self.model.handle(fr, now):
                self.schedule(now + delay, rsp)

    def schedule(self, due: float, fr: Frame) -> None:
        with self._cv:
            heapq.heappush(self._heap, (due, next(self._seq), fr))
            self._cv.notify()

    def _write_loop(self) -> None:
        while True:
            with self._cv:
                while not self._stop and not self._heap:
                    self._cv.wait(0.2)
                if self._stop:
                    return
                due, _, fr = self._heap[0]
                wait = due - time.monotonic()
                if wait > 0:
                    self._cv.wait(wait)
                    continue
                heapq.heappop(self._heap)
            wire = fr.encode(self.transparency)
            data = self.down.apply(wire)
            try:
                self.write_fn(data)
            except OSError as exc:
                log.warning("%s write failed: %s", self.name, exc)
                return
            self.tx_frames += 1
            self.tx_bytes += len(data)
            log.info("%s TX %s", self.name, describe(fr))
            if self.pace_baud:
                time.sleep(len(data) * 10.0 / self.pace_baud)

    def pending(self) -> int:
        with self._cv:
            return len(self._heap)

    def close(self) -> None:
        with self._cv:
            self._stop = True
            self._cv.notify_all()
        self._writer.join(timeout=1.0)


class TcpRoverServer:
    """TCP server: every client connection talks to the same simulated rover."""

    def __init__(self, model: RoverModel, host: str = "127.0.0.1", port: int = 5760, **link_kw) -> None:
        self.model = model
        self.link_kw = link_kw
        self.links: List[RoverLink] = []
        self._sock = socket.create_server((host, port))
        self._sock.settimeout(0.2)
        self.port = self._sock.getsockname()[1]
        self.host = host
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    def start(self) -> "TcpRoverServer":
        th = threading.Thread(target=self.serve_forever, name="rover-accept", daemon=True)
        th.start()
        self._threads.append(th)
        return self

    def serve_forever(self) -> None:
        log.warning("virtual rover listening on tcp://%s:%d", self.host, self.port)
        while not self._stop.is_set():
            try:
                conn, peer = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            th = threading.Thread(target=self._client, args=(conn, peer), daemon=True)
            th.start()
            self._threads.append(th)

    def _client(self, conn: socket.socket, peer) -> None:
        log.warning("GS connected from %s:%d", *peer[:2])
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(0.2)
        link = RoverLink(self.model, conn.sendall, name=f"tcp:{peer[1]}", **self.link_kw)
        self.links.append(link)
        try:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                link.on_bytes(data)
        finally:
            link.close()
            conn.close()
            log.warning("GS %s:%d disconnected", *peer[:2])

    def stop(self) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        for th in self._threads:
            th.join(timeout=1.0)


def serve_serial(model: RoverModel, port: str, baud: int, **link_kw) -> None:  # pragma: no cover - hw
    import serial  # pyserial
    ser = serial.Serial(port, baud, timeout=0.02)
    link = RoverLink(model, ser.write, name=port, **link_kw)
    log.warning("virtual rover on %s @ %d baud", port, baud)
    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                link.on_bytes(data)
    finally:
        link.close()
        ser.close()


# ====================================================================== CLI
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="MISSION X virtual rover (GS link emulator, ICD v2.0)",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__.split(
                                     "\n\n", 3)[3])
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--tcp", type=int, metavar="PORT", help="listen on a TCP port")
    where.add_argument("--port", help="serial port (COM8, /dev/ttyUSB0, ...)")
    ap.add_argument("--host", default="127.0.0.1", help="TCP bind address (default 127.0.0.1)")
    ap.add_argument("--baud", type=int, default=9600, help="serial baud rate (default 9600)")
    ap.add_argument("--legacy", action="store_true", help="behave like the 2023 rover (7-byte TLM)")
    ap.add_argument("--safe-link", action="store_true", help="behave like the Mega answering on the HC-12")
    ap.add_argument("--ber", type=float, default=0.0, help="bit error rate, both directions (e.g. 1e-4)")
    ap.add_argument("--ber-up", type=float, help="bit error rate GS->rover (overrides --ber)")
    ap.add_argument("--ber-down", type=float, help="bit error rate rover->GS (overrides --ber)")
    ap.add_argument("--pace", type=int, default=0, metavar="BAUD",
                    help="throttle TCP output to BAUD (8N1) like the HC-12 radio (default off)")
    ap.add_argument("--time-scale", type=float, default=1.0, help="scale processing delays (0 = instant)")
    ap.add_argument("--transparency", action="store_true", help="SLIP escaping (must match the GS)")
    ap.add_argument("--length-check", action="store_true",
                    help="reject implausible LEN at the header (ICD §3 step 5a) instead of NACKing it")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("-v", "--verbose", action="count", default=0, help="-v: log every frame")
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO if a.verbose else logging.WARNING,
                        format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    model = RoverModel(seed=a.seed, legacy=a.legacy, safe_link=a.safe_link, time_scale=a.time_scale)
    link_kw = dict(ber_up=a.ber if a.ber_up is None else a.ber_up,
                   ber_down=a.ber if a.ber_down is None else a.ber_down,
                   transparency=a.transparency, length_check=a.length_check, seed=a.seed + 1)
    kind = "legacy 2023 rover" if a.legacy else ("Mega safe-mode link" if a.safe_link else "v2 Brain")
    if a.port:
        serve_serial(model, a.port, a.baud, **link_kw)
        return 0
    srv = TcpRoverServer(model, a.host, a.tcp, pace_baud=a.pace, **link_kw)
    print(f"MISSION X virtual rover ({kind}) on tcp://{a.host}:{srv.port} "
          f"ber_up={link_kw['ber_up']:g} ber_down={link_kw['ber_down']:g} pace={a.pace or 'off'}  (Ctrl+C to stop)",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
