"""SSP addresses, packet types and payload layouts (ICD v2.0 §4, §6, §7).

Every payload class offers ``pack() -> bytes`` and ``unpack(bytes) -> obj``.
All multi-byte fields are little-endian.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field, fields
from enum import IntEnum
from typing import ClassVar, List, Sequence


class Addr(IntEnum):
    ROVER = 0x01
    MOBILITY = 0x02
    PAYLOAD = 0x03
    ARM = 0x04
    GROUND = 0x0B
    BROADCAST = 0xFF


class PT(IntEnum):
    """Packet types."""
    PING = 0x00
    ACK = 0x01
    NACK = 0x02
    GET_IMAGE = 0x08
    IMAGE_CHUNK = 0x09
    CAP_IMAGE = 0x0C
    GET_TLM = 0x0F
    START_SCAN = 0x16
    STOP_SCAN = 0x1C
    COLLECT_OBJECT = 0x20
    SET_VELOCITY = 0x30
    WHEEL_ODOM = 0x31
    RANGES = 0x32
    ENV_TLM = 0x33
    IMU = 0x34
    ARM_SET_JOINTS = 0x40
    ARM_SET_POSE = 0x41
    ARM_STATE = 0x42
    HEARTBEAT = 0x50
    ESTOP = 0x51
    CLEAR_ESTOP = 0x52
    SET_MODE = 0x53
    DRIVE = 0x60


LEGACY_TYPES = frozenset({PT.PING, PT.ACK, PT.GET_IMAGE, PT.CAP_IMAGE, PT.GET_TLM,
                          PT.START_SCAN, PT.STOP_SCAN, PT.COLLECT_OBJECT})
#: requests whose positive response echoes the request type with LEN 0
ECHO_ACK_TYPES = frozenset({PT.CAP_IMAGE, PT.START_SCAN, PT.STOP_SCAN, PT.COLLECT_OBJECT})


class Mode(IntEnum):
    IDLE = 0
    TELEOP = 1
    AUTONOMOUS = 2
    SAFE = 3


class NackReason(IntEnum):
    BAD_LENGTH = 1
    UNKNOWN_TYPE = 2
    BUSY = 3
    NOT_PERMITTED = 4
    HW_FAULT = 5


class Fault(IntEnum):
    ESTOP = 1 << 0
    CMD_TIMEOUT = 1 << 1
    LOW_BATTERY = 1 << 2
    ENCODER = 1 << 3
    SAFE_MODE = 1 << 4


class ArmPose(IntEnum):
    HOME = 0
    STOW = 1
    READY = 2
    REACH_GROUND = 3
    GRIP_OPEN = 4
    GRIP_CLOSE = 5
    LIFT_SAMPLE = 6
    DEPOSIT = 7


#: wheel order used in WHEEL_ODOM / motor arrays
WHEELS = ("left_front", "left_mid", "left_rear", "right_front", "right_mid", "right_rear")
#: ultrasonic order used in RANGES
ULTRASONICS = ("front_left", "front_center", "front_right", "left", "right", "rear")
NO_ECHO = 0xFFFF


class _Struct:
    """Mixin: fixed-layout payload described by FMT and the dataclass fields."""
    FMT: ClassVar[str] = "<"

    def _values(self) -> list:
        vals: list = []
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, (list, tuple)):
                vals.extend(v)
            else:
                vals.append(v)
        return vals

    def pack(self) -> bytes:
        return struct.pack(self.FMT, *self._values())

    @classmethod
    def size(cls) -> int:
        return struct.calcsize(cls.FMT)


def _check_len(cls, data: bytes) -> None:
    if len(data) != cls.size():
        raise ValueError(f"{cls.__name__}: expected {cls.size()} bytes, got {len(data)}")


@dataclass
class SetVelocity(_Struct):
    v_mm_s: int = 0
    w_mrad_s: int = 0
    timeout_ms: int = 500
    FMT: ClassVar[str] = "<hhH"

    @classmethod
    def from_si(cls, v: float, w: float, timeout_s: float = 0.5) -> "SetVelocity":
        clamp = lambda x: max(-32768, min(32767, int(round(x))))
        return cls(clamp(v * 1000.0), clamp(w * 1000.0), max(0, min(65535, int(timeout_s * 1000))))

    @classmethod
    def unpack(cls, data: bytes) -> "SetVelocity":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


@dataclass
class WheelOdom(_Struct):
    t_ms: int = 0
    ticks: List[int] = field(default_factory=lambda: [0] * 6)
    FMT: ClassVar[str] = "<I6i"

    @classmethod
    def unpack(cls, data: bytes) -> "WheelOdom":
        _check_len(cls, data)
        v = struct.unpack(cls.FMT, data)
        return cls(v[0], list(v[1:]))


@dataclass
class Ranges:
    range_mm: List[int] = field(default_factory=lambda: [NO_ECHO] * 6)

    def pack(self) -> bytes:
        return struct.pack(f"<B{len(self.range_mm)}H", len(self.range_mm), *self.range_mm)

    @classmethod
    def unpack(cls, data: bytes) -> "Ranges":
        if not data:
            raise ValueError("Ranges: empty")
        n = data[0]
        if len(data) != 1 + 2 * n:
            raise ValueError(f"Ranges: expected {1 + 2 * n} bytes, got {len(data)}")
        return cls(list(struct.unpack(f"<{n}H", data[1:])))

    def meters(self) -> List[float]:
        return [float("inf") if r == NO_ECHO else r / 1000.0 for r in self.range_mm]


@dataclass
class EnvTlm(_Struct):
    dht_temp_cC: int = 0
    dht_rh_cP: int = 0
    bmp_pressure_Pa: int = 0
    bmp_temp_cC: int = 0
    mq2_raw: int = 0
    mq4_raw: int = 0
    soil_raw: int = 0
    rain_raw: int = 0
    proximity: int = 0
    status: int = 0
    FMT: ClassVar[str] = "<hHIhHHHHBB"  # 20 bytes

    @classmethod
    def unpack(cls, data: bytes) -> "EnvTlm":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


@dataclass
class Imu(_Struct):
    t_ms: int = 0
    ax: int = 0  # mm/s^2
    ay: int = 0
    az: int = 0
    gx: int = 0  # mrad/s
    gy: int = 0
    gz: int = 0
    roll_cdeg: int = 0
    pitch_cdeg: int = 0
    FMT: ClassVar[str] = "<I8h"  # 20 bytes

    @classmethod
    def unpack(cls, data: bytes) -> "Imu":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


@dataclass
class ArmSetJoints(_Struct):
    deg: List[int] = field(default_factory=lambda: [90, 90, 90, 90, 90, 10])
    duration_ms: int = 1000
    FMT: ClassVar[str] = "<6BH"

    @classmethod
    def unpack(cls, data: bytes) -> "ArmSetJoints":
        _check_len(cls, data)
        v = struct.unpack(cls.FMT, data)
        return cls(list(v[:6]), v[6])


@dataclass
class ArmState(_Struct):
    deg: List[int] = field(default_factory=lambda: [90] * 6)
    moving: int = 0
    FMT: ClassVar[str] = "<7B"

    @classmethod
    def unpack(cls, data: bytes) -> "ArmState":
        _check_len(cls, data)
        v = struct.unpack(cls.FMT, data)
        return cls(list(v[:6]), v[6])


@dataclass
class Heartbeat(_Struct):
    uptime_ms: int = 0
    mode: int = 0
    fault_flags: int = 0
    FMT: ClassVar[str] = "<IBB"

    @classmethod
    def unpack(cls, data: bytes) -> "Heartbeat":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


@dataclass
class Drive(_Struct):
    v_pct: int = 0
    w_pct: int = 0
    duration_ms: int = 500
    FMT: ClassVar[str] = "<bbH"

    @classmethod
    def unpack(cls, data: bytes) -> "Drive":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


@dataclass
class Nack(_Struct):
    rejected_type: int = 0
    reason: int = 0
    FMT: ClassVar[str] = "<BB"

    @classmethod
    def unpack(cls, data: bytes) -> "Nack":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


@dataclass
class ImageChunkHeader(_Struct):
    seq: int = 0
    total: int = 0
    width: int = 0
    height: int = 0
    fmt: int = 0  # 0 gray8, 1 jpeg
    FMT: ClassVar[str] = "<HHHHB"  # 9 bytes

    @classmethod
    def unpack(cls, data: bytes) -> "ImageChunkHeader":
        return cls(*struct.unpack(cls.FMT, data[:cls.size()]))


IMAGE_CHUNK_PAYLOAD = 200


@dataclass
class TlmLegacy(_Struct):
    """7-byte telemetry of the 2023 Arduino Mega firmware."""
    ir: int = 0
    gas_hi: int = 0
    gas_lo: int = 0
    us1_cm: int = 0
    us2_cm: int = 0
    temp_c: int = 0
    rh_pct: int = 0
    FMT: ClassVar[str] = "<7B"

    @property
    def gas_raw(self) -> int:
        return (self.gas_hi << 8) | self.gas_lo

    @classmethod
    def unpack(cls, data: bytes) -> "TlmLegacy":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


@dataclass
class TlmV2(_Struct):
    """56-byte GET_TLM response (ICD §7.1)."""
    tlm_version: int = 2
    mode: int = 0
    fault_flags: int = 0
    battery_mV: int = 0
    uptime_s: int = 0
    temperature_cC: int = 0
    humidity_cP: int = 0
    pressure_Pa: int = 0
    roll_cdeg: int = 0
    pitch_cdeg: int = 0
    yaw_cdeg: int = 0
    ax: int = 0
    ay: int = 0
    az: int = 0
    gx: int = 0
    gy: int = 0
    gz: int = 0
    mq2_raw: int = 0
    mq4_raw: int = 0
    soil_raw: int = 0
    rain_raw: int = 0
    proximity: int = 0
    x_mm: int = 0
    y_mm: int = 0
    heading_cdeg: int = 0
    range_min_mm: int = NO_ECHO
    FMT: ClassVar[str] = "<BBBHI hHI hhh hhh hhh HHHH B ii h H".replace(" ", "")

    @classmethod
    def unpack(cls, data: bytes) -> "TlmV2":
        _check_len(cls, data)
        return cls(*struct.unpack(cls.FMT, data))


#: allowed DATA lengths per packet type (union of request and response
#: directions, legacy included). Types not listed are accepted with any length.
_ALLOWED_LEN = {
    PT.PING: {0}, PT.ACK: {0, 1}, PT.NACK: {2},
    PT.GET_IMAGE: {0, 1, 200}, PT.IMAGE_CHUNK: set(range(9, 210)),
    PT.CAP_IMAGE: {0}, PT.GET_TLM: None,  # legacy brains sent 7/17/68 B
    PT.START_SCAN: {0}, PT.STOP_SCAN: {0}, PT.COLLECT_OBJECT: {0},
    PT.SET_VELOCITY: {6}, PT.WHEEL_ODOM: {28}, PT.RANGES: {1 + 2 * n for n in range(7)},
    PT.ENV_TLM: {20}, PT.IMU: {20}, PT.ARM_SET_JOINTS: {8}, PT.ARM_SET_POSE: {1},
    PT.ARM_STATE: {7}, PT.HEARTBEAT: {6}, PT.ESTOP: {0}, PT.CLEAR_ESTOP: {0},
    PT.SET_MODE: {1}, PT.DRIVE: {4},
}


def plausible_length(ptype: int, n: int, strict: bool = False) -> bool:
    """Length plausibility check for :class:`rover_core.ssp.StreamDecoder`.

    ``strict=True`` additionally rejects unknown packet types (use on MCUs that
    only accept their own command set)."""
    try:
        allowed = _ALLOWED_LEN[PT(ptype)]
    except (ValueError, KeyError):
        return not strict
    return True if allowed is None else n in allowed


def decode_tlm(data: bytes):
    """Decode a GET_TLM response of either generation."""
    if len(data) == TlmLegacy.size():
        return TlmLegacy.unpack(data)
    if len(data) == TlmV2.size() and data[0] == 2:
        return TlmV2.unpack(data)
    raise ValueError(f"unknown telemetry layout ({len(data)} bytes)")


def clamp_i16(x: float) -> int:
    return max(-32768, min(32767, int(round(x))))


def clamp_u16(x: float) -> int:
    return max(0, min(65535, int(round(x))))


def clamp_i32(x: float) -> int:
    return max(-(2 ** 31), min(2 ** 31 - 1, int(round(x))))
