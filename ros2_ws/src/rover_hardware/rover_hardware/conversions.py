"""ROS-free conversions shared by the MISSION X bridges and the simulator.

Everything in this module is plain Python (+ numpy for images) so it can be
unit-tested without rclpy. ICD units (mm, mrad/s, centi-degrees, ...) are
converted to SI here and back; the ROS nodes only copy values into messages.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np

from rover_core.params import DEFAULT, RoverParams
from rover_core.kinematics import limit_twist, wrap_angle
from rover_core.protocol import (IMAGE_CHUNK_PAYLOAD, NO_ECHO, PT, ArmPose, ArmSetJoints, ArmState,
                                 Drive, EnvTlm, ImageChunkHeader, Imu, SetVelocity, TlmV2,
                                 clamp_i16, clamp_i32, clamp_u16)

# ----------------------------------------------------------------- constants
ULTRASONIC_NAMES = ("front_left", "front_center", "front_right", "left", "right", "rear")
WHEEL_JOINTS = ("left_front_wheel_joint", "left_mid_wheel_joint", "left_rear_wheel_joint",
                "right_front_wheel_joint", "right_mid_wheel_joint", "right_rear_wheel_joint")
ARM_JOINT_NAMES = ("arm_base_yaw", "arm_shoulder", "arm_elbow", "arm_wrist_pitch",
                   "arm_wrist_roll", "arm_gripper")
#: ICD section 9.5 servo limits [deg] and home pose
ARM_LIMITS_DEG = ((0, 180), (15, 165), (0, 180), (0, 180), (0, 180), (10, 80))
ARM_HOME_DEG = (90, 90, 90, 90, 90, 10)
LEGACY_IMAGE_SIZE = (100, 100)        # width, height (GET_IMAGE LEN 0)
LEGACY_CHUNK = 200                    # 50 chunks x 200 B
UNKNOWN_VARIANCE = 1.0e3              # covariance used for unobserved dimensions


# ----------------------------------------------------------------- geometry
def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) for intrinsic Z-Y-X (yaw, pitch, roll) Euler angles."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rpy_from_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def diag6(values: Sequence[float]) -> List[float]:
    """6x6 row-major covariance with the given diagonal."""
    cov = [0.0] * 36
    for i, v in enumerate(values):
        cov[7 * i] = float(v)
    return cov


def diag3(values: Sequence[float]) -> List[float]:
    cov = [0.0] * 9
    for i, v in enumerate(values):
        cov[4 * i] = float(v)
    return cov


# ----------------------------------------------------------------- parameters
def rover_params(**overrides) -> RoverParams:
    """``rover_core.params.DEFAULT`` with selected fields replaced (None = keep)."""
    return replace(DEFAULT, **{k: v for k, v in overrides.items() if v is not None})


# ----------------------------------------------------------------- mobility
def twist_to_set_velocity(v: float, w: float, params: RoverParams = DEFAULT,
                          timeout_ms: int = 500) -> SetVelocity:
    """Body twist [m/s, rad/s] -> SET_VELOCITY payload, limited with
    :func:`rover_core.kinematics.limit_twist` (curvature preserved)."""
    if not (math.isfinite(v) and math.isfinite(w)):
        v = w = 0.0
    v, w = limit_twist(v, w, params)
    return SetVelocity(clamp_i16(v * 1000.0), clamp_i16(w * 1000.0), clamp_u16(timeout_ms))


def odom_covariances(v: float, w: float) -> Tuple[List[float], List[float]]:
    """Pose / twist covariances (row-major 6x6) for the skid-steer wheel odometry.

    The twist uncertainty grows with speed and, for the yaw rate, strongly
    with turning (lateral skid, effective-track uncertainty). vy = 0 is a
    reasonably tight non-holonomic constraint. Unobserved dimensions get
    :data:`UNKNOWN_VARIANCE`. The pose covariance is only informative (the
    EKF fuses velocities)."""
    av, aw = abs(v), abs(w)
    sv = 0.01 + 0.05 * av
    svy = 0.01 + 0.05 * aw
    sw = 0.02 + 0.20 * aw + 0.05 * av
    pose = diag6([0.05, 0.05, UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, 0.2])
    twist = diag6([sv * sv, svy * svy, UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, sw * sw])
    return pose, twist


def normalise_range(r_m: float, min_range: float, max_range: float,
                    no_echo_as_max: bool = False) -> float:
    """REP-117 range value: +inf for no echo / beyond range (or ``max_range``
    when *no_echo_as_max*, which lets the Nav2 RangeSensorLayer clear its cone)."""
    if r_m is None or math.isnan(r_m) or r_m > max_range:
        return max_range if no_echo_as_max else math.inf
    return float(r_m)


def ranges_to_meters(range_mm: Sequence[int], min_range: float = 0.02, max_range: float = 4.0,
                     no_echo_as_max: bool = False) -> List[float]:
    """RANGES payload values [mm] (0xFFFF = no echo) -> metres."""
    out = []
    for r in range_mm:
        m = math.inf if r == NO_ECHO else r / 1000.0
        out.append(normalise_range(m, min_range, max_range, no_echo_as_max))
    return out


def wheel_angles(ticks: Sequence[int], counts_per_rev: int) -> List[float]:
    """Cumulative encoder counts -> wheel joint angles [rad] (for /joint_states)."""
    return [2.0 * math.pi * t / counts_per_rev for t in ticks]


# ----------------------------------------------------------------- payload
@dataclass
class ImuSI:
    roll: float
    pitch: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


def imu_to_si(m: Imu) -> ImuSI:
    """IMU payload (mm/s^2, mrad/s, centi-degrees) -> SI."""
    return ImuSI(math.radians(m.roll_cdeg / 100.0), math.radians(m.pitch_cdeg / 100.0),
                 m.ax / 1000.0, m.ay / 1000.0, m.az / 1000.0,
                 m.gx / 1000.0, m.gy / 1000.0, m.gz / 1000.0)


@dataclass
class EnvSI:
    temperature_c: float
    humidity_pct: float
    pressure_pa: float
    bmp_temperature_c: float
    mq2_raw: int
    mq4_raw: int
    soil_raw: int
    rain_raw: int
    metal_detected: bool
    status: int

    @property
    def dht_ok(self) -> bool:
        return bool(self.status & 0x01)

    @property
    def bmp_ok(self) -> bool:
        return bool(self.status & 0x02)

    @property
    def mpu_ok(self) -> bool:
        return bool(self.status & 0x04)

    @property
    def gas_warm(self) -> bool:
        return bool(self.status & 0x08)


def env_to_si(e: EnvTlm) -> EnvSI:
    """ENV_TLM payload -> SI. Invalid sensors (status bits) give NaN; the
    temperature falls back from the DHT11 to the BMP180."""
    dht_ok, bmp_ok = bool(e.status & 0x01), bool(e.status & 0x02)
    nan = float("nan")
    t_dht = e.dht_temp_cC / 100.0 if dht_ok else nan
    t_bmp = e.bmp_temp_cC / 100.0 if bmp_ok else nan
    return EnvSI(
        temperature_c=t_dht if dht_ok else t_bmp,
        humidity_pct=e.dht_rh_cP / 100.0 if dht_ok else nan,
        pressure_pa=float(e.bmp_pressure_Pa) if bmp_ok else nan,
        bmp_temperature_c=t_bmp,
        mq2_raw=e.mq2_raw, mq4_raw=e.mq4_raw, soil_raw=e.soil_raw, rain_raw=e.rain_raw,
        metal_detected=bool(e.proximity), status=e.status)


# ----------------------------------------------------------------- arm
def clamp_arm_deg(deg: Sequence[float]) -> List[int]:
    """Round and clamp six servo angles to the ICD limits (NaN -> home)."""
    if len(deg) != 6:
        raise ValueError(f"expected 6 joint angles, got {len(deg)}")
    out = []
    for d, (lo, hi), home in zip(deg, ARM_LIMITS_DEG, ARM_HOME_DEG):
        d = float(d)
        out.append(home if not math.isfinite(d) else int(max(lo, min(hi, round(d)))))
    return out


def arm_command_payload(use_pose: bool, pose_id: int, joint_deg: Sequence[float],
                        duration_ms: int) -> Tuple[int, bytes]:
    """ArmCommand fields -> (packet type, DATA) for the arm MCU."""
    if use_pose:
        pid = int(pose_id)
        if pid not in {int(p) for p in ArmPose}:
            raise ValueError(f"unknown arm pose id {pid}")
        return int(PT.ARM_SET_POSE), bytes((pid,))
    return int(PT.ARM_SET_JOINTS), ArmSetJoints(clamp_arm_deg(joint_deg),
                                                clamp_u16(duration_ms)).pack()


def arm_state_to_radians(state: ArmState) -> List[float]:
    """ARM_STATE servo angles [deg] -> joint positions [rad] (servo angle)."""
    return [math.radians(d) for d in state.deg]


# ----------------------------------------------------------------- ground station
def drive_to_twist(d: Drive, max_speed: float, max_yaw_rate: float) -> Tuple[float, float, float]:
    """DRIVE payload (+-100 %) -> (v [m/s], w [rad/s], duration [s])."""
    pv = max(-100, min(100, d.v_pct))
    pw = max(-100, min(100, d.w_pct))
    return pv / 100.0 * max_speed, pw / 100.0 * max_yaw_rate, d.duration_ms / 1000.0


@dataclass
class TelemetrySnapshot:
    """Latest rover state in SI units, assembled by the ground-station bridge."""
    mode: int = 0
    fault_flags: int = 0
    battery_v: float = float("nan")
    uptime_s: float = 0.0
    temperature_c: float = float("nan")
    humidity_pct: float = float("nan")
    pressure_pa: float = float("nan")
    roll: float = 0.0
    pitch: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    mq2_raw: int = 0
    mq4_raw: int = 0
    soil_raw: int = 0
    rain_raw: int = 0
    metal_detected: bool = False
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    ranges: Sequence[float] = field(default_factory=tuple)


def _fin(x: Optional[float], default: float = 0.0) -> float:
    return float(x) if x is not None and math.isfinite(x) else default


def build_tlm_v2(s: TelemetrySnapshot) -> TlmV2:
    """Assemble the 56-byte GET_TLM v2 response (ICD section 7.1).
    Unavailable values are sent as 0 (range_min as 0xFFFF)."""
    finite = [r for r in s.ranges if r is not None and math.isfinite(r) and r >= 0.0]
    heading_cdeg = clamp_i16(math.degrees(wrap_angle(_fin(s.heading))) * 100.0)
    return TlmV2(
        tlm_version=2,
        mode=max(0, min(255, int(s.mode))),
        fault_flags=max(0, min(255, int(s.fault_flags))),
        battery_mV=clamp_u16(_fin(s.battery_v) * 1000.0),
        uptime_s=max(0, min(2 ** 32 - 1, int(_fin(s.uptime_s)))),
        temperature_cC=clamp_i16(_fin(s.temperature_c) * 100.0),
        humidity_cP=clamp_u16(_fin(s.humidity_pct) * 100.0),
        pressure_Pa=max(0, min(2 ** 32 - 1, int(round(_fin(s.pressure_pa))))),
        roll_cdeg=clamp_i16(math.degrees(_fin(s.roll)) * 100.0),
        pitch_cdeg=clamp_i16(math.degrees(_fin(s.pitch)) * 100.0),
        yaw_cdeg=heading_cdeg,
        ax=clamp_i16(_fin(s.ax) * 1000.0), ay=clamp_i16(_fin(s.ay) * 1000.0),
        az=clamp_i16(_fin(s.az) * 1000.0),
        gx=clamp_i16(_fin(s.gx) * 1000.0), gy=clamp_i16(_fin(s.gy) * 1000.0),
        gz=clamp_i16(_fin(s.gz) * 1000.0),
        mq2_raw=clamp_u16(s.mq2_raw), mq4_raw=clamp_u16(s.mq4_raw),
        soil_raw=clamp_u16(s.soil_raw), rain_raw=clamp_u16(s.rain_raw),
        proximity=1 if s.metal_detected else 0,
        x_mm=clamp_i32(_fin(s.x) * 1000.0), y_mm=clamp_i32(_fin(s.y) * 1000.0),
        heading_cdeg=heading_cdeg,
        range_min_mm=min(NO_ECHO - 1, clamp_u16(min(finite) * 1000.0)) if finite else NO_ECHO,
    )


# ----------------------------------------------------------------- images
def gray_from_raw(data: bytes, width: int, height: int, encoding: str,
                  step: int = 0) -> np.ndarray:
    """sensor_msgs/Image buffer -> (height, width) uint8 gray image, without
    cv_bridge. Supports mono8/16, rgb8/bgr8, rgba8/bgra8, yuyv and uyvy."""
    enc = encoding.lower()
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    bpp = {"mono8": 1, "8uc1": 1, "mono16": 2, "16uc1": 2, "rgb8": 3, "bgr8": 3, "8uc3": 3,
           "rgba8": 4, "bgra8": 4, "8uc4": 4, "yuyv": 2, "yuv422_yuy2": 2, "uyvy": 2,
           "yuv422": 2}.get(enc)
    if bpp is None:
        raise ValueError(f"unsupported image encoding '{encoding}'")
    step = step or width * bpp
    if buf.size < step * height:
        raise ValueError("image buffer too small")
    rows = buf[:step * height].reshape(height, step)[:, :width * bpp]
    if bpp == 1:
        return rows.copy()
    if enc in ("mono16", "16uc1"):
        return (rows.reshape(height, width, 2)[:, :, 1]).copy()   # little-endian high byte
    if enc in ("yuyv", "yuv422_yuy2"):
        return rows.reshape(height, width, 2)[:, :, 0].copy()     # Y Cb Y Cr
    if enc in ("uyvy", "yuv422"):
        return rows.reshape(height, width, 2)[:, :, 1].copy()     # Cb Y Cr Y
    px = rows.reshape(height, width, bpp).astype(np.float32)
    if enc.startswith("rgb"):
        r, g, b = px[:, :, 0], px[:, :, 1], px[:, :, 2]
    else:  # bgr, bgra, 8uc3/8uc4 (OpenCV order)
        b, g, r = px[:, :, 0], px[:, :, 1], px[:, :, 2]
    return np.clip(0.299 * r + 0.587 * g + 0.114 * b + 0.5, 0, 255).astype(np.uint8)


def resize_nearest(img: np.ndarray, width: int, height: int) -> np.ndarray:
    h, w = img.shape[:2]
    ys = (np.arange(height) * h / height).astype(int)
    xs = (np.arange(width) * w / width).astype(int)
    return img[ys][:, xs].copy()


def placeholder_image(width: int = 100, height: int = 100, counter: int = 0) -> np.ndarray:
    """Test pattern sent when no camera image is available: diagonal
    gradient, border and an X (the counter shifts the gradient)."""
    yy, xx = np.mgrid[0:height, 0:width]
    img = ((xx + yy + counter * 8) * 255 // max(1, width + height - 2)).astype(np.uint8)
    img[[0, -1], :] = 255
    img[:, [0, -1]] = 255
    d = np.arange(min(width, height))
    img[d, d] = 0
    img[d, min(width, height) - 1 - d] = 0
    return img


def legacy_image_chunks(img: np.ndarray) -> List[bytes]:
    """100x100 gray image -> 50 chunks of 200 B (legacy GET_IMAGE, type 0x08)."""
    w, h = LEGACY_IMAGE_SIZE
    if img.shape != (h, w):
        img = resize_nearest(img, w, h)
    raw = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
    return [raw[i:i + LEGACY_CHUNK] for i in range(0, len(raw), LEGACY_CHUNK)]


def v2_image_chunks(payload: bytes, width: int, height: int, fmt: int) -> List[bytes]:
    """Image payload -> IMAGE_CHUNK (0x09) DATA blocks: 9-byte header + <= 200 B."""
    total = max(1, -(-len(payload) // IMAGE_CHUNK_PAYLOAD))
    if total > 0xFFFF:
        raise ValueError("image too large for IMAGE_CHUNK")
    out = []
    for seq in range(total):
        part = payload[seq * IMAGE_CHUNK_PAYLOAD:(seq + 1) * IMAGE_CHUNK_PAYLOAD]
        out.append(ImageChunkHeader(seq, total, width, height, fmt).pack() + part)
    return out


def reassemble_v2_chunks(chunks: Sequence[bytes]) -> Tuple[ImageChunkHeader, bytes]:
    """Inverse of :func:`v2_image_chunks` (ground-station side / tests)."""
    hdr_size = ImageChunkHeader.size()
    parts = {}
    hdr = None
    for c in chunks:
        hdr = ImageChunkHeader.unpack(c)
        parts[hdr.seq] = c[hdr_size:]
    if hdr is None or len(parts) != hdr.total:
        raise ValueError("missing image chunks")
    return hdr, b"".join(parts[i] for i in range(hdr.total))
