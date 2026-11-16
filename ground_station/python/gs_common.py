"""Shared helpers for the MISSION X Python ground-station tools.

* puts ``ros2_ws/src/rover_core`` on ``sys.path`` so the reference SSP
  implementation (``rover_core.ssp`` / ``rover_core.protocol``) is reused
  instead of duplicated (override with the ``MISSIONX_ROVER_CORE`` env var),
* human-readable frame descriptions (same wording as ``web/js/protocol.js``),
* the telemetry CSV schema shared with the web ground station.
"""
from __future__ import annotations

import math
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

ROVER_CORE = Path(os.environ.get(
    "MISSIONX_ROVER_CORE",
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "rover_core"))
if str(ROVER_CORE) not in sys.path:
    sys.path.insert(0, str(ROVER_CORE))

from rover_core.ssp import Frame  # noqa: E402
from rover_core.protocol import (  # noqa: E402
    NO_ECHO, Addr, ArmPose, Drive, Fault, Heartbeat, ImageChunkHeader, Mode, Nack, NackReason, PT,
    SetVelocity, TlmLegacy, TlmV2, decode_tlm)

#: legacy GET_IMAGE geometry (ICD §6.1): 100x100 gray8 in 50 chunks of 200 B
LEGACY_W = LEGACY_H = 100
LEGACY_CHUNKS = 50
LEGACY_CHUNK = 200
#: wire size of one legacy chunk frame: FEND + 4 header + 200 + CRC 2 + FEND
LEGACY_CHUNK_WIRE = 1 + 4 + LEGACY_CHUNK + 2 + 1

FAULT_LABELS = [
    (Fault.ESTOP, "ESTOP"), (Fault.CMD_TIMEOUT, "CMD_TIMEOUT"), (Fault.LOW_BATTERY, "LOW_BATTERY"),
    (Fault.ENCODER, "ENCODER"), (Fault.SAFE_MODE, "SAFE_LINK"),
]


def name_of(enum_cls, value: int, prefix: str) -> str:
    try:
        return enum_cls(value).name
    except ValueError:
        return f"{prefix}_0x{value:02X}"


def type_name(t: int) -> str:
    return name_of(PT, t, "TYPE")


def addr_name(a: int) -> str:
    try:
        return Addr(a).name
    except ValueError:
        return f"0x{a:02X}"


def mode_name(m: int) -> str:
    return name_of(Mode, m, "MODE")


def fault_names(flags: int) -> list[str]:
    return [n for bit, n in FAULT_LABELS if flags & bit]


def parse_int(s: str) -> int:
    """'0x0F', '15', '0b1111' -> int"""
    return int(str(s), 0)


def _f(v, d=1) -> str:
    return "-" if v is None else f"{v:.{d}f}"


def describe_tlm(data: bytes) -> str:
    try:
        t = decode_tlm(data)
    except ValueError:
        return f"TLM {len(data)} B (unknown layout)"
    if isinstance(t, TlmLegacy):
        return (f"TLM legacy: IR={t.ir} gas={t.gas_raw} US1={t.us1_cm}cm US2={t.us2_cm}cm "
                f"T={t.temp_c}°C RH={t.rh_pct}%")
    faults = fault_names(t.fault_flags)
    rmin = "none" if t.range_min_mm == NO_ECHO else f"{t.range_min_mm / 1000:.2f}m"
    return (f"TLM v2: {mode_name(t.mode)} batt={t.battery_mV / 1000:.2f}V "
            f"T={t.temperature_cC / 100:.1f}°C RH={t.humidity_cP / 100:.1f}% p={t.pressure_Pa / 100:.1f}hPa "
            f"pos=({t.x_mm / 1000:.2f},{t.y_mm / 1000:.2f})m hdg={t.heading_cdeg / 100:.1f}° rmin={rmin}"
            + (f" faults={'|'.join(faults)}" if faults else ""))


def describe(fr: Frame) -> str:
    """One-line meaning of a frame (mirrors describeFrame() in web/js/protocol.js)."""
    d, t = bytes(fr.data), fr.type
    name = type_name(t)
    try:
        if t == PT.PING:
            return "PING"
        if t == PT.ACK:
            return f"ACK of {type_name(d[0])}" if d else "ACK"
        if t == PT.NACK and len(d) == 2:
            n = Nack.unpack(d)
            return f"NACK of {type_name(n.rejected_type)}: {name_of(NackReason, n.reason, 'REASON')}"
        if t == PT.GET_IMAGE:
            if not d:
                return "GET_IMAGE (legacy 100x100 gray)"
            if len(d) == 1:
                return f"GET_IMAGE v2 quality={d[0]}"
            return f"legacy image chunk ({len(d)} B)"
        if t == PT.IMAGE_CHUNK and len(d) >= 9:
            h = ImageChunkHeader.unpack(d)
            return (f"IMAGE_CHUNK {h.seq + 1}/{h.total} {h.width}x{h.height} "
                    f"{'jpeg' if h.fmt == 1 else 'gray8'} ({len(d) - 9} B)")
        if t == PT.GET_TLM:
            return describe_tlm(d) if d else "GET_TLM"
        if t == PT.SET_MODE and len(d) == 1:
            return f"SET_MODE {mode_name(d[0])}"
        if t == PT.DRIVE and len(d) == 4:
            v = Drive.unpack(d)
            return f"DRIVE v={v.v_pct}% w={v.w_pct}% for {v.duration_ms} ms"
        if t == PT.SET_VELOCITY and len(d) == 6:
            v = SetVelocity.unpack(d)
            return f"SET_VELOCITY v={v.v_mm_s}mm/s w={v.w_mrad_s}mrad/s timeout={v.timeout_ms}ms"
        if t == PT.HEARTBEAT and len(d) == 6:
            h = Heartbeat.unpack(d)
            return f"HEARTBEAT up={h.uptime_ms / 1000:.1f}s {mode_name(h.mode)} faults=0x{h.fault_flags:02X}"
        if t == PT.ARM_SET_POSE and len(d) == 1:
            return f"ARM_SET_POSE {name_of(ArmPose, d[0], 'POSE')}"
    except (ValueError, IndexError):
        pass
    return f"{name} ({len(d)} B)" if d else name


# ------------------------------------------------------------- telemetry CSV
#: identical to CSV_COLUMNS in web/js/protocol.js
CSV_COLUMNS = ["time_iso", "tlm_version", "mode", "fault_flags", "battery_V", "uptime_s",
               "temperature_C", "humidity_pct", "pressure_hPa", "roll_deg", "pitch_deg", "yaw_deg",
               "ax_mps2", "ay_mps2", "az_mps2", "gx_radps", "gy_radps", "gz_radps",
               "mq2_raw", "mq4_raw", "soil_raw", "rain_raw", "proximity", "x_m", "y_m", "heading_deg",
               "range_min_m", "ir", "gas_raw", "us1_cm", "us2_cm"]


def normalize_tlm(tlm, t: float | None = None) -> dict:
    """Telemetry of either generation -> dict keyed by CSV_COLUMNS (engineering units)."""
    t = datetime.now(timezone.utc).timestamp() if t is None else t
    s = {c: None for c in CSV_COLUMNS}
    s["time_iso"] = datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z")
    if isinstance(tlm, TlmV2):
        s.update(tlm_version=tlm.tlm_version, mode=tlm.mode, fault_flags=tlm.fault_flags,
                 battery_V=tlm.battery_mV / 1000, uptime_s=tlm.uptime_s,
                 temperature_C=tlm.temperature_cC / 100, humidity_pct=tlm.humidity_cP / 100,
                 pressure_hPa=tlm.pressure_Pa / 100, roll_deg=tlm.roll_cdeg / 100,
                 pitch_deg=tlm.pitch_cdeg / 100, yaw_deg=tlm.yaw_cdeg / 100,
                 ax_mps2=tlm.ax / 1000, ay_mps2=tlm.ay / 1000, az_mps2=tlm.az / 1000,
                 gx_radps=tlm.gx / 1000, gy_radps=tlm.gy / 1000, gz_radps=tlm.gz / 1000,
                 mq2_raw=tlm.mq2_raw, mq4_raw=tlm.mq4_raw, soil_raw=tlm.soil_raw, rain_raw=tlm.rain_raw,
                 proximity=tlm.proximity, x_m=tlm.x_mm / 1000, y_m=tlm.y_mm / 1000,
                 heading_deg=tlm.heading_cdeg / 100,
                 range_min_m=None if tlm.range_min_mm == NO_ECHO else tlm.range_min_mm / 1000)
    elif isinstance(tlm, TlmLegacy):
        us = [v for v in (tlm.us1_cm, tlm.us2_cm) if v > 0]
        s.update(tlm_version=1, temperature_C=tlm.temp_c, humidity_pct=tlm.rh_pct, ir=tlm.ir,
                 gas_raw=tlm.gas_raw, us1_cm=tlm.us1_cm, us2_cm=tlm.us2_cm,
                 range_min_m=min(us) / 100 if us else None)
    return s


def csv_cell(v) -> str:
    """CSV formatting: integers as-is, floats with at most 6 decimals, None empty."""
    if v is None:
        return ""
    if isinstance(v, float):
        if not math.isfinite(v):
            return ""
        if v == 0:
            return "0"
        return f"{v:.6f}".rstrip("0").rstrip(".")
    return str(v)
