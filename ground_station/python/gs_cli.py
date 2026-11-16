#!/usr/bin/env python3
"""MISSION X ground station -- command-line interface (ICD v2.0).

Talks SSP to the rover over a serial port (USB-UART + HC-12 radio) or a TCP
socket (``virtual_rover.py``), using the reference codec ``rover_core.ssp``.

Examples::

    python gs_cli.py --port COM7 ping                       # one PING, prints RTT
    python gs_cli.py --port COM7 --ping-test 100            # RTT statistics
    python gs_cli.py --tcp 127.0.0.1:5760 tlm               # one telemetry frame, decoded
    python gs_cli.py --tcp 127.0.0.1:5760 tlm --poll 1 --count 60 --csv tlm.csv
    python gs_cli.py --tcp 127.0.0.1:5760 image -o rock.png             # legacy 100x100 GET_IMAGE
    python gs_cli.py --tcp 127.0.0.1:5760 image --v2 --quality 60 -o rock.png --capture
    python gs_cli.py --tcp 127.0.0.1:5760 estop | clear-estop | scan-start | scan-stop | collect | cap
    python gs_cli.py --tcp 127.0.0.1:5760 mode teleop
    python gs_cli.py --tcp 127.0.0.1:5760 drive 40 0 1500   # 40 % forward for 1.5 s
    python gs_cli.py --tcp 127.0.0.1:5760 raw 0x0F           # any type/data, prints decoded replies
    python gs_cli.py --tcp 127.0.0.1:5760 shell              # interactive (or: shell < script.txt)
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import queue
import shlex
import socket
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import gs_common  # noqa: F401  (puts rover_core on sys.path)
from gs_common import (CSV_COLUMNS, LEGACY_CHUNK, LEGACY_CHUNK_WIRE, LEGACY_CHUNKS, LEGACY_H, LEGACY_W,
                       addr_name, csv_cell, describe, fault_names, mode_name, normalize_tlm, parse_int,
                       type_name)
from rover_core.protocol import (ECHO_ACK_TYPES, NO_ECHO, Addr, ArmPose, Drive, ImageChunkHeader, Mode,
                                 Nack, NackReason, PT, TlmLegacy, TlmV2, decode_tlm, plausible_length)
from rover_core.ssp import Frame, StreamDecoder

__all__ = ["GroundStation", "TcpLink", "SerialLink", "Reply", "ImageResult", "ImageAssembler", "main"]


# ===================================================================== links
class LinkClosed(ConnectionError):
    pass


class TcpLink:
    """Byte link over TCP (virtual rover, or a serial-to-TCP bridge)."""

    def __init__(self, host: str, port: int, connect_timeout: float = 5.0) -> None:
        self.sock = socket.create_connection((host, port), timeout=connect_timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.05)
        self.name = f"tcp://{host}:{port}"
        self.baud = None

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)

    def read(self) -> bytes:
        try:
            data = self.sock.recv(4096)
        except socket.timeout:
            return b""
        except OSError as exc:
            raise LinkClosed(str(exc)) from exc
        if not data:
            raise LinkClosed("connection closed by peer")
        return data

    def close(self) -> None:
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


class SerialLink:
    """Byte link over a serial port (USB-UART adapter wired to the HC-12)."""

    def __init__(self, port: str, baud: int = 9600) -> None:
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.name = f"{port}@{baud}"
        self.baud = baud

    def write(self, data: bytes) -> None:
        self.ser.write(data)

    def read(self) -> bytes:
        try:
            return self.ser.read(self.ser.in_waiting or 1)
        except Exception as exc:  # serial.SerialException (device unplugged)
            raise LinkClosed(str(exc)) from exc

    def close(self) -> None:
        self.ser.close()


def open_link(port: Optional[str] = None, tcp: Optional[str] = None, baud: int = 9600):
    if tcp:
        host, _, p = tcp.rpartition(":")
        return TcpLink(host or "127.0.0.1", int(p))
    if port:
        return SerialLink(port, baud)
    raise ValueError("either a serial port or --tcp host:port is required")


# ============================================================ request logic
@dataclass
class Rx:
    frame: Frame
    t: float          # monotonic time the bytes were read
    bytes_in: int     # decoder byte counter right after the frame's closing flag


@dataclass
class Reply:
    ok: bool
    request: Frame
    frame: Optional[Frame] = None
    rtt: Optional[float] = None
    error: Optional[str] = None          # 'timeout' | 'nack' | 'closed'
    nack: Optional[Nack] = None
    attempts: int = 1
    value: object = None                 # decoded payload (telemetry)

    def __str__(self) -> str:
        rtt = f"  rtt={self.rtt * 1000:.1f} ms" if self.rtt is not None else ""
        if self.ok:
            return f"OK    {describe(self.frame)}{rtt}"
        if self.error == "nack":
            return (f"NACK  {type_name(self.nack.rejected_type)} rejected: "
                    f"{_enum_name(NackReason, self.nack.reason)}{rtt}")
        return f"{self.error.upper():5s} no reply to {type_name(self.request.type)} ({self.attempts} attempt(s))"


def _enum_name(cls, v: int) -> str:
    try:
        return cls(v).name
    except ValueError:
        return str(v)


def classify(req: Frame, fr: Frame) -> Optional[str]:
    """'done' if *fr* completes request *req*, 'nack' if it rejects it, else None.

    Only one request is outstanding at a time (half-duplex radio), so replies are
    matched on type; the source address is not required to equal req.dest
    (the Mega answers as 0x02 in safe mode)."""
    rt, d = req.type, bytes(fr.data)
    if fr.type == PT.NACK and len(d) == 2 and d[0] == rt:
        return "nack"
    if rt == PT.GET_TLM:
        return "done" if fr.type == PT.GET_TLM and d else None
    if rt in ECHO_ACK_TYPES:
        return "done" if fr.type == rt and not d else None
    if fr.type == PT.ACK and (not d or d[0] == rt):
        return "done"
    return None


# ================================================================== images
@dataclass
class ImageResult:
    width: int
    height: int
    fmt: int                      # 0 gray8, 1 jpeg
    data: bytes                   # gray pixels (missing chunks = 0) or JPEG bytes
    total: int
    received: List[int]
    missing: List[int]
    legacy: bool
    elapsed: float
    nack: Optional[Nack] = None

    @property
    def complete(self) -> bool:
        return self.total > 0 and not self.missing

    def summary(self) -> str:
        if self.nack:
            return f"image request rejected: {_enum_name(NackReason, self.nack.reason)}"
        kind = "legacy GET_IMAGE" if self.legacy else f"IMAGE_CHUNK {'jpeg' if self.fmt else 'gray8'}"
        miss = f", missing {self.missing}" if self.missing else ""
        return (f"{kind} {self.width}x{self.height}: {len(self.received)}/{self.total} chunks, "
                f"{len(self.data)} B in {self.elapsed:.2f} s{miss}")

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        if self.fmt == 1 and path.suffix.lower() in (".jpg", ".jpeg"):
            path.write_bytes(self.data)
            return path
        from PIL import Image, ImageFile
        if self.fmt == 1:
            ImageFile.LOAD_TRUNCATED_IMAGES = True
            img = Image.open(io.BytesIO(self.data))
            img.load()
        else:
            img = Image.frombytes("L", (self.width, self.height), self.data)
        img.save(path)
        return path


class ImageAssembler:
    """Collects legacy (type 0x08, 200 B) or v2 IMAGE_CHUNK frames.

    Legacy chunks carry no sequence number. Every legacy chunk frame is exactly
    208 bytes on the wire, so the number of bytes the decoder consumed between
    two valid chunks reveals how many chunks were lost in between (a CRC-failed
    chunk still arrives as ~208 bytes); those slots are left empty instead of
    shifting the rest of the picture."""

    def __init__(self, bytes_at_request: int) -> None:
        self.legacy: Optional[bool] = None
        self.width = self.height = self.total = 0
        self.fmt = 0
        self.chunks: Dict[int, bytes] = {}
        self._slot = 0
        self._mark = bytes_at_request

    def note_other(self, fr: Frame) -> None:
        """A non-image frame arrived during the download: exclude its bytes."""
        self._mark += len(fr.data) + 8

    def add(self, fr: Frame, bytes_in: int) -> bool:
        d = bytes(fr.data)
        if fr.type == PT.GET_IMAGE and len(d) == LEGACY_CHUNK:
            if self.legacy is not True:
                self.legacy, self.width, self.height, self.total, self.fmt = True, LEGACY_W, LEGACY_H, LEGACY_CHUNKS, 0
            gap = bytes_in - self._mark - LEGACY_CHUNK_WIRE
            lost = int(round(gap / LEGACY_CHUNK_WIRE)) if gap > LEGACY_CHUNK_WIRE // 2 else 0
            self._slot += lost
            if self._slot < self.total:
                self.chunks[self._slot] = d
            self._slot += 1
            self._mark = bytes_in
            return True
        if fr.type == PT.IMAGE_CHUNK and len(d) >= ImageChunkHeader.size():
            h = ImageChunkHeader.unpack(d)
            if self.legacy is not False or (h.total, h.width, h.height, h.fmt) != (
                    self.total, self.width, self.height, self.fmt):
                self.legacy, self.total, self.width, self.height, self.fmt = False, h.total, h.width, h.height, h.fmt
                self.chunks = {}
            if h.seq < h.total:
                self.chunks[h.seq] = d[ImageChunkHeader.size():]
            self._mark = bytes_in
            return True
        return False

    @property
    def complete(self) -> bool:
        return self.total > 0 and len(self.chunks) >= self.total

    def missing(self) -> List[int]:
        return [i for i in range(self.total) if i not in self.chunks]

    def result(self, elapsed: float, nack: Optional[Nack] = None) -> ImageResult:
        size = LEGACY_CHUNK if self.legacy else 200
        if self.fmt == 0:
            buf = bytearray(self.width * self.height)
            for i, c in self.chunks.items():
                buf[i * size:i * size + len(c)] = c[:max(0, len(buf) - i * size)]
            data = bytes(buf)
        else:
            data = b"".join(self.chunks.get(i, bytes(size)) for i in range(self.total))
        return ImageResult(self.width, self.height, self.fmt, data, self.total, sorted(self.chunks),
                           self.missing(), bool(self.legacy), elapsed, nack)


# =========================================================== ground station
class GroundStation:
    """SSP ground-station client: one outstanding request at a time (the HC-12
    is half-duplex), CRC-checked replies, NACK handling, retries, statistics."""

    def __init__(self, link, *, dest: int = Addr.ROVER, own: int = Addr.GROUND, timeout: float = 2.0,
                 retries: int = 0, ibt_s: Optional[float] = None, transparency: bool = False,
                 length_check: bool = True, on_frame: Optional[Callable[[str, Frame], None]] = None) -> None:
        self.link = link
        self.dest = dest
        self.own = own
        self.timeout = timeout
        self.retries = retries
        self.transparency = transparency
        self.on_frame = on_frame
        if ibt_s is None:   # ICD §3 step 6: 50 ms on the radio; a little more slack over TCP
            ibt_s = 0.05 if getattr(link, "baud", None) else 0.1
        self.decoder = StreamDecoder(own_addr=own, transparency=transparency, timeout_s=ibt_s,
                                     length_check=plausible_length if length_check else None)
        self.stats = {"tx_frames": 0, "tx_bytes": 0, "requests": 0, "replies": 0, "timeouts": 0,
                      "nacks": 0, "retries": 0, "unsolicited": 0}
        self.unsolicited: List[Rx] = []
        self.last_tlm = None
        self._rxq: "queue.Queue[Optional[Rx]]" = queue.Queue()
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, name="gs-rx", daemon=True)
        self._reader.start()

    # ------------------------------------------------------------ plumbing
    def _read_loop(self) -> None:
        while not self._closed.is_set():
            try:
                data = self.link.read()
            except LinkClosed:
                break
            if not data:
                continue
            now = time.monotonic()
            for b in data:      # byte-wise: every frame is tagged with its stream position
                for fr in self.decoder.feed(bytes((b,)), now=now):
                    self._rxq.put(Rx(fr, now, self.decoder.stats.bytes_in))
        self._closed.set()
        self._rxq.put(None)

    def close(self) -> None:
        self._closed.set()
        try:
            self.link.close()
        finally:
            self._reader.join(timeout=1.0)

    def __enter__(self) -> "GroundStation":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _handle_unsolicited(self, rx: Rx) -> None:
        self.stats["unsolicited"] += 1
        self.unsolicited.append(rx)
        del self.unsolicited[:-200]
        if rx.frame.type == PT.GET_TLM and rx.frame.data:
            try:
                self.last_tlm = decode_tlm(bytes(rx.frame.data))
            except ValueError:
                pass
        if self.on_frame:
            self.on_frame("RX", rx.frame)

    def _drain(self) -> None:
        while True:
            try:
                rx = self._rxq.get_nowait()
            except queue.Empty:
                return
            if rx is not None:
                self._handle_unsolicited(rx)

    def send(self, ptype: int, data: bytes = b"", dest: Optional[int] = None) -> Frame:
        """Send one frame without waiting for a reply."""
        if self._closed.is_set():
            raise LinkClosed("link is closed")
        fr = Frame(self.dest if dest is None else dest, self.own, int(ptype), bytes(data))
        wire = fr.encode(self.transparency)
        self.link.write(wire)
        self.stats["tx_frames"] += 1
        self.stats["tx_bytes"] += len(wire)
        if self.on_frame:
            self.on_frame("TX", fr)
        return fr

    def request(self, ptype: int, data: bytes = b"", *, dest: Optional[int] = None,
                timeout: Optional[float] = None, retries: Optional[int] = None) -> Reply:
        """Send a command and wait for its reply (ACK/echo/data or NACK)."""
        timeout = self.timeout if timeout is None else timeout
        retries = self.retries if retries is None else retries
        with self._lock:
            self._drain()
            self.stats["requests"] += 1
            req = None
            for attempt in range(1, retries + 2):
                if attempt > 1:
                    self.stats["retries"] += 1
                t0 = time.monotonic()           # before the write: the reply may beat us back
                req = self.send(ptype, data, dest)
                deadline = time.monotonic() + timeout
                while True:
                    rem = deadline - time.monotonic()
                    if rem <= 0:
                        break
                    try:
                        rx = self._rxq.get(timeout=rem)
                    except queue.Empty:
                        break
                    if rx is None:
                        return Reply(False, req, error="closed", attempts=attempt)
                    verdict = classify(req, rx.frame)
                    if verdict is None:
                        self._handle_unsolicited(rx)
                        continue
                    if self.on_frame:
                        self.on_frame("RX", rx.frame)
                    rtt = max(0.0, rx.t - t0)
                    if verdict == "nack":
                        self.stats["nacks"] += 1
                        return Reply(False, req, rx.frame, rtt, "nack", Nack.unpack(bytes(rx.frame.data)),
                                     attempt)
                    self.stats["replies"] += 1
                    value = None
                    if rx.frame.type == PT.GET_TLM:
                        try:
                            value = self.last_tlm = decode_tlm(bytes(rx.frame.data))
                        except ValueError:
                            value = None
                    return Reply(True, req, rx.frame, rtt, attempts=attempt, value=value)
                self.stats["timeouts"] += 1
            return Reply(False, req, error="timeout", attempts=retries + 1)

    # ------------------------------------------------------------ commands
    def ping(self, **kw) -> Reply:
        return self.request(PT.PING, **kw)

    def get_tlm(self, **kw) -> Reply:
        return self.request(PT.GET_TLM, **kw)

    def cap_image(self, **kw) -> Reply:
        kw.setdefault("timeout", max(self.timeout, 3.0))
        return self.request(PT.CAP_IMAGE, **kw)

    def start_scan(self, **kw) -> Reply:
        return self.request(PT.START_SCAN, **kw)

    def stop_scan(self, **kw) -> Reply:
        return self.request(PT.STOP_SCAN, **kw)

    def collect_object(self, **kw) -> Reply:
        return self.request(PT.COLLECT_OBJECT, **kw)

    def estop(self, **kw) -> Reply:
        kw.setdefault("retries", max(self.retries, 2))     # safety command: always retried
        return self.request(PT.ESTOP, **kw)

    def clear_estop(self, **kw) -> Reply:
        return self.request(PT.CLEAR_ESTOP, **kw)

    def set_mode(self, mode: int, **kw) -> Reply:
        return self.request(PT.SET_MODE, bytes([int(mode)]), **kw)

    def drive(self, v_pct: int, w_pct: int, duration_ms: int = 500, **kw) -> Reply:
        return self.request(PT.DRIVE, Drive(int(v_pct), int(w_pct), int(duration_ms)).pack(), **kw)

    def arm_pose(self, pose: int, **kw) -> Reply:
        kw.setdefault("dest", Addr.ARM)
        return self.request(PT.ARM_SET_POSE, bytes([int(pose)]), **kw)

    def get_image(self, *, v2: bool = False, quality: int = 0, dest: Optional[int] = None,
                  first_timeout: Optional[float] = None, idle_timeout: float = 2.5, max_time: float = 180.0,
                  progress: Optional[Callable[[ImageAssembler], None]] = None) -> ImageResult:
        """GET_IMAGE: legacy (LEN 0, 50 x 200 B chunks) or v2 (LEN 1 = quality, IMAGE_CHUNK)."""
        with self._lock:
            self._drain()
            self.stats["requests"] += 1
            asm = ImageAssembler(self.decoder.stats.bytes_in)
            self.send(PT.GET_IMAGE, bytes([quality]) if v2 else b"", dest)
            t0 = time.monotonic()
            deadline = t0 + (first_timeout or max(self.timeout, 3.0))
            nack = None
            while not asm.complete and time.monotonic() - t0 < max_time:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    break
                try:
                    rx = self._rxq.get(timeout=rem)
                except queue.Empty:
                    break
                if rx is None:
                    break
                fr = rx.frame
                if fr.type == PT.NACK and len(fr.data) == 2 and fr.data[0] == PT.GET_IMAGE:
                    nack = Nack.unpack(bytes(fr.data))
                    self.stats["nacks"] += 1
                    break
                if asm.add(fr, rx.bytes_in):
                    if self.on_frame:
                        self.on_frame("RX", fr)
                    deadline = time.monotonic() + idle_timeout
                    if progress:
                        progress(asm)
                else:
                    asm.note_other(fr)
                    self._handle_unsolicited(rx)
            if asm.total:
                self.stats["replies"] += 1
            else:
                self.stats["timeouts"] += 0 if nack else 1
            return asm.result(time.monotonic() - t0, nack)

    def ping_test(self, n: int, interval: float = 0.0, **kw) -> dict:
        rtts: List[float] = []
        for i in range(n):
            r = self.ping(retries=0, **kw)
            if r.ok:
                rtts.append(r.rtt * 1000.0)
            if interval and i < n - 1:
                time.sleep(interval)
        res = {"sent": n, "received": len(rtts), "lost": n - len(rtts),
               "loss_pct": 100.0 * (n - len(rtts)) / n if n else 0.0}
        if rtts:
            q = sorted(rtts)
            res.update(min_ms=q[0], avg_ms=statistics.fmean(q), max_ms=q[-1],
                       stdev_ms=statistics.pstdev(q), p50_ms=q[len(q) // 2],
                       p95_ms=q[min(len(q) - 1, int(round(0.95 * (len(q) - 1))))])
        return res

    def poll_tlm(self, interval: float = 1.0, count: Optional[int] = None, duration: Optional[float] = None,
                 csv_path: Optional[str | Path] = None, on_sample: Optional[Callable[[Optional[dict], Reply], None]]
                 = None) -> List[dict]:
        """Poll GET_TLM every *interval* s; optionally append rows to a CSV file."""
        rows: List[dict] = []
        fh = writer = None
        if csv_path:
            fh = open(csv_path, "w", newline="", encoding="utf-8")
            writer = csv.writer(fh)
            writer.writerow(CSV_COLUMNS)
        try:
            t_start = t_next = time.monotonic()
            n = 0
            while (count is None or n < count) and (duration is None or time.monotonic() - t_start < duration):
                r = self.get_tlm()
                n += 1
                sample = normalize_tlm(r.value, time.time()) if r.ok and r.value is not None else None
                if sample:
                    rows.append(sample)
                    if writer:
                        writer.writerow([csv_cell(sample[c]) for c in CSV_COLUMNS])
                        fh.flush()
                if on_sample:
                    on_sample(sample, r)
                t_next += interval
                time.sleep(max(0.0, t_next - time.monotonic()))
        finally:
            if fh:
                fh.close()
        return rows

    def link_stats(self) -> dict:
        return {**self.decoder.stats.as_dict(), **self.stats}


# ================================================================== output
def format_tlm(t) -> str:
    if isinstance(t, TlmLegacy):
        return "\n".join([
            "Telemetry (legacy 7-byte)",
            f"  IR/proximity : {t.ir}",
            f"  gas (raw)    : {t.gas_raw}  (hi={t.gas_hi} lo={t.gas_lo})",
            f"  ultrasonic   : US1={t.us1_cm} cm  US2={t.us2_cm} cm",
            f"  temperature  : {t.temp_c} °C",
            f"  humidity     : {t.rh_pct} %RH"])
    if isinstance(t, TlmV2):
        f = fault_names(t.fault_flags)
        rmin = "no echo" if t.range_min_mm == NO_ECHO else f"{t.range_min_mm / 1000:.3f} m"
        return "\n".join([
            f"Telemetry v{t.tlm_version} (56 B)",
            f"  mode         : {mode_name(t.mode)}    faults: {', '.join(f) if f else 'none'} (0x{t.fault_flags:02X})",
            f"  battery      : {t.battery_mV / 1000:.3f} V    uptime: {t.uptime_s} s",
            f"  environment  : {t.temperature_cC / 100:.2f} °C  {t.humidity_cP / 100:.2f} %RH  "
            f"{t.pressure_Pa / 100:.2f} hPa",
            f"  attitude     : roll {t.roll_cdeg / 100:+.2f}°  pitch {t.pitch_cdeg / 100:+.2f}°  "
            f"yaw {t.yaw_cdeg / 100:+.2f}°",
            f"  accel [m/s²] : {t.ax / 1000:+.3f} {t.ay / 1000:+.3f} {t.az / 1000:+.3f}    "
            f"gyro [rad/s]: {t.gx / 1000:+.3f} {t.gy / 1000:+.3f} {t.gz / 1000:+.3f}",
            f"  gas / soil   : MQ-2 {t.mq2_raw}  MQ-4 {t.mq4_raw}  soil {t.soil_raw}  rain {t.rain_raw}",
            f"  proximity    : {'METAL DETECTED' if t.proximity else 'clear'}",
            f"  pose         : x {t.x_mm / 1000:+.3f} m  y {t.y_mm / 1000:+.3f} m  "
            f"heading {t.heading_cdeg / 100:+.2f}°",
            f"  closest obst.: {rmin}"])
    return repr(t)


MODES = {"idle": Mode.IDLE, "teleop": Mode.TELEOP, "auto": Mode.AUTONOMOUS, "autonomous": Mode.AUTONOMOUS,
         "safe": Mode.SAFE}
ADDRS = {"rover": Addr.ROVER, "brain": Addr.ROVER, "mobility": Addr.MOBILITY, "payload": Addr.PAYLOAD,
         "arm": Addr.ARM, "broadcast": Addr.BROADCAST, "ground": Addr.GROUND}


def parse_mode(s: str) -> int:
    return int(MODES[s.lower()]) if s.lower() in MODES else parse_int(s)


def parse_addr(s: str) -> int:
    return int(ADDRS[s.lower()]) if s.lower() in ADDRS else parse_int(s)


def parse_type(s: str) -> int:
    try:
        return int(PT[s.upper()])
    except KeyError:
        return parse_int(s)


def parse_pose(s: str) -> int:
    try:
        return int(ArmPose[s.upper()])
    except KeyError:
        return parse_int(s)


# ===================================================================== CLI
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="gs_cli.py", description="MISSION X ground station CLI (SSP, ICD v2.0)",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("Examples::", 1)[1])
    ap.add_argument("--port", help="serial port of the USB-UART/HC-12 (COM7, /dev/ttyUSB0)")
    ap.add_argument("--tcp", metavar="HOST:PORT", help="connect to virtual_rover.py (e.g. 127.0.0.1:5760)")
    ap.add_argument("--baud", type=int, default=9600, help="serial baud rate (default 9600, ICD L1)")
    ap.add_argument("--dest", type=parse_addr, default=Addr.ROVER, help="destination address (default rover 0x01)")
    ap.add_argument("--timeout", type=float, default=2.0, help="reply timeout per attempt [s] (default 2.0)")
    ap.add_argument("--retries", type=int, default=0, help="re-send a command N times on timeout")
    ap.add_argument("--ibt-ms", type=float, help="inter-byte timeout [ms] (default 50 serial / 100 TCP)")
    ap.add_argument("--transparency", action="store_true", help="SLIP escaping (must match the rover)")
    ap.add_argument("--ping-test", type=int, metavar="N", help="send N PINGs and print RTT statistics")
    ap.add_argument("--interval", type=float, default=0.0, help="pause between PINGs of --ping-test [s]")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every TX/RX frame (hex + meaning)")
    sub = ap.add_subparsers(dest="cmd", metavar="COMMAND")
    add_commands(sub)
    return ap


def add_commands(sub) -> None:
    p = sub.add_parser("ping", help="PING -> ACK, prints round-trip time")
    p.add_argument("-n", "--count", type=int, default=1, help="number of pings (>1 prints statistics)")
    p = sub.add_parser("tlm", help="GET_TLM (decoded); --poll for periodic logging")
    p.add_argument("--poll", type=float, metavar="SECONDS", help="poll interval")
    p.add_argument("--count", type=int, help="number of samples (default: until Ctrl+C)")
    p.add_argument("--duration", type=float, help="stop after this many seconds")
    p.add_argument("--csv", help="write samples to a CSV file")
    sub.add_parser("cap", help="CAP_IMAGE (take a picture)")
    p = sub.add_parser("image", help="GET_IMAGE and save it (PNG/JPEG)")
    p.add_argument("-o", "--output", default="rover_image.png", help="output file (default rover_image.png)")
    p.add_argument("--v2", action="store_true", help="v2 request (IMAGE_CHUNK with sequence numbers)")
    p.add_argument("--quality", type=int, default=0, help="v2: 0 = raw gray8, 1..100 = JPEG quality")
    p.add_argument("--capture", action="store_true", help="send CAP_IMAGE first")
    sub.add_parser("scan-start", help="START_SCAN (autonomous exploration)")
    sub.add_parser("scan-stop", help="STOP_SCAN (cancel navigation, stop)")
    sub.add_parser("collect", help="COLLECT_OBJECT (arm sample collection)")
    sub.add_parser("estop", help="ESTOP (latched motor stop)")
    sub.add_parser("clear-estop", help="CLEAR_ESTOP")
    p = sub.add_parser("mode", help="SET_MODE idle|teleop|auto|safe")
    p.add_argument("mode", type=parse_mode)
    p = sub.add_parser("drive", help="DRIVE v_pct w_pct [duration_ms]  (open-loop teleop)")
    p.add_argument("v_pct", type=int)
    p.add_argument("w_pct", type=int)
    p.add_argument("duration_ms", type=int, nargs="?", default=500)
    p = sub.add_parser("arm-pose", help="ARM_SET_POSE home|stow|ready|reach_ground|grip_open|... (to ARM 0x04)")
    p.add_argument("pose", type=parse_pose)
    p = sub.add_parser("raw", help="send any TYPE [HEXDATA] and print the decoded reply")
    p.add_argument("type", type=parse_type, help="type number or name (0x0F, GET_TLM, ...)")
    p.add_argument("data", nargs="?", default="", help="DATA as hex (e.g. '28 00 F4 01')")
    p.add_argument("--to", type=parse_addr, help="destination address for this frame")
    p = sub.add_parser("monitor", help="print every received frame for a while")
    p.add_argument("--duration", type=float, default=10.0)
    sub.add_parser("stats", help="link statistics")
    sub.add_parser("shell", help="interactive prompt (reads commands from stdin; also scriptable)")


class Cli:
    def __init__(self, gs: GroundStation, args, out=None) -> None:
        self.gs, self.args, self.out = gs, args, out or sys.stdout

    def p(self, *a) -> None:
        print(*a, file=self.out, flush=True)

    def reply(self, r: Reply) -> int:
        if self.args.json:
            self.p(json.dumps({"ok": r.ok, "error": r.error, "rtt_ms": None if r.rtt is None else r.rtt * 1000,
                               "reply": describe(r.frame) if r.frame else None,
                               "attempts": r.attempts}))
        else:
            self.p(str(r))
            if r.ok and r.value is not None:
                self.p(format_tlm(r.value))
        return 0 if r.ok else 1

    def run(self, a) -> int:
        gs, cmd = self.gs, a.cmd
        if cmd == "ping":
            if a.count <= 1:
                return self.reply(gs.ping())
            return self.ping_stats(a.count)
        if cmd == "tlm":
            if a.poll is None and a.csv is None and a.count is None:
                r = gs.get_tlm()
                if self.args.json and r.ok and r.value is not None:
                    self.p(json.dumps(normalize_tlm(r.value)))
                    return 0
                return self.reply(r)
            return self.poll(a)
        if cmd == "cap":
            return self.reply(gs.cap_image())
        if cmd == "image":
            if a.capture:
                r = gs.cap_image()
                self.p(str(r))
                if not r.ok:
                    return 1
            last = [0]

            def progress(asm: ImageAssembler) -> None:
                n = len(asm.chunks)
                if not self.args.json and (n - last[0] >= 10 or asm.complete):
                    last[0] = n
                    self.p(f"  ... {n}/{asm.total} chunks")
            res = gs.get_image(v2=a.v2, quality=a.quality, progress=progress)
            if not res.total:
                self.p(res.summary() if res.nack else "TIMEOUT no image chunks received")
                return 1
            path = res.save(a.output)
            if self.args.json:
                self.p(json.dumps({"ok": res.complete, "file": str(path), "width": res.width, "height": res.height,
                                   "fmt": res.fmt, "chunks": res.total, "missing": res.missing,
                                   "elapsed_s": res.elapsed}))
            else:
                self.p(res.summary())
                self.p(f"saved {path}")
            return 0 if res.complete else 1
        simple = {"scan-start": gs.start_scan, "scan-stop": gs.stop_scan, "collect": gs.collect_object,
                  "estop": gs.estop, "clear-estop": gs.clear_estop}
        if cmd in simple:
            return self.reply(simple[cmd]())
        if cmd == "mode":
            return self.reply(gs.set_mode(a.mode))
        if cmd == "drive":
            return self.reply(gs.drive(a.v_pct, a.w_pct, a.duration_ms))
        if cmd == "arm-pose":
            return self.reply(gs.arm_pose(a.pose))
        if cmd == "raw":
            data = bytes.fromhex(a.data.replace(":", " ")) if a.data else b""
            r = gs.request(a.type, data, dest=a.to)
            return self.reply(r)
        if cmd == "monitor":
            t_end = time.monotonic() + a.duration
            n0 = len(gs.unsolicited)
            while time.monotonic() < t_end:
                time.sleep(0.1)
                gs._drain()
            for rx in gs.unsolicited[n0:]:
                self.p(f"RX {describe(rx.frame)}")
            return 0
        if cmd == "stats":
            st = gs.link_stats()
            self.p(json.dumps(st) if self.args.json else "\n".join(f"  {k:16s} {v}" for k, v in st.items()))
            return 0
        if cmd == "shell":
            return self.shell()
        self.p(f"unknown command {cmd}")
        return 2

    def ping_stats(self, n: int) -> int:
        st = self.gs.ping_test(n, self.args.interval)
        if self.args.json:
            self.p(json.dumps(st))
        else:
            self.p(f"PING x{st['sent']}: {st['received']} replies, {st['lost']} lost ({st['loss_pct']:.1f} %)")
            if st["received"]:
                self.p(f"  RTT ms: min {st['min_ms']:.1f}  avg {st['avg_ms']:.1f}  max {st['max_ms']:.1f}  "
                       f"stdev {st['stdev_ms']:.1f}  p50 {st['p50_ms']:.1f}  p95 {st['p95_ms']:.1f}")
        return 0 if st["received"] == st["sent"] else 1

    def poll(self, a) -> int:
        interval = a.poll if a.poll is not None else 1.0

        def show(sample, r):
            if sample is None:
                self.p(str(r))
            elif self.args.json:
                self.p(json.dumps(sample))
            elif sample["tlm_version"] == 2:
                self.p(f"{sample['time_iso']}  {mode_name(sample['mode']):10s} batt {sample['battery_V']:.2f} V  "
                       f"T {sample['temperature_C']:.1f} °C  RH {sample['humidity_pct']:.1f} %  "
                       f"pos ({sample['x_m']:+.2f}, {sample['y_m']:+.2f}) m  hdg {sample['heading_deg']:+.1f}°")
            else:
                self.p(f"{sample['time_iso']}  legacy T {sample['temperature_C']} °C  RH {sample['humidity_pct']} %  "
                       f"gas {sample['gas_raw']}  US {sample['us1_cm']}/{sample['us2_cm']} cm")
        try:
            rows = self.gs.poll_tlm(interval, count=a.count, duration=a.duration, csv_path=a.csv, on_sample=show)
        except KeyboardInterrupt:
            rows = []
        if a.csv and not self.args.json:
            self.p(f"wrote {a.csv} ({len(rows)} samples)")
        return 0

    def shell(self) -> int:
        parser = argparse.ArgumentParser(prog="", add_help=False, exit_on_error=False)
        sub = parser.add_subparsers(dest="cmd")
        add_commands(sub)
        interactive = sys.stdin.isatty()
        if interactive:
            self.p(f"MISSION X GS shell on {self.gs.link.name}. Commands: "
                   + ", ".join(sorted(sub.choices)) + ", help, quit")
        rc = 0
        while True:
            try:
                line = input("gs> ") if interactive else sys.stdin.readline()
            except EOFError:
                break
            if not interactive and line == "":
                break
            for part in line.split(";"):
                part = part.split("#", 1)[0].strip()
                if not part:
                    continue
                if part in ("quit", "exit"):
                    return rc
                if part in ("help", "?"):
                    self.p("commands: " + ", ".join(sorted(sub.choices)) + "  (e.g. 'drive 40 0 1000; tlm')")
                    continue
                try:
                    a = parser.parse_args(shlex.split(part))
                except (argparse.ArgumentError, SystemExit, ValueError) as exc:
                    self.p(f"error: {exc}")
                    rc = 2
                    continue
                if a.cmd in (None, "shell"):
                    continue
                if not interactive:
                    self.p(f"gs> {part}")
                rc = self.run(a) or rc
        return rc


def _utf8_output() -> None:
    """Piped output on Windows defaults to the ANSI code page; make '°' survive."""
    for stream in (sys.stdout, sys.stderr):
        try:
            if not stream.isatty() and (stream.encoding or "").lower().replace("-", "") != "utf8":
                stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError, OSError):
            pass


def main(argv=None) -> int:
    _utf8_output()
    ap = build_parser()
    args = ap.parse_args(argv)
    if not (args.port or args.tcp):
        ap.error("give --port COMx (HC-12 via USB-UART) or --tcp host:port (virtual_rover.py)")
    if args.cmd is None and not args.ping_test:
        ap.error("a COMMAND or --ping-test N is required")
    try:
        link = open_link(args.port, args.tcp, args.baud)
    except (OSError, ValueError) as exc:
        print(f"cannot open link: {exc}", file=sys.stderr)
        return 3

    def trace(direction: str, fr: Frame) -> None:
        print(f"  {direction} {fr.encode(args.transparency).hex(' ').upper():<40s} {describe(fr)}",
              file=sys.stderr, flush=True)

    gs = GroundStation(link, dest=args.dest, timeout=args.timeout, retries=args.retries,
                       ibt_s=None if args.ibt_ms is None else args.ibt_ms / 1000.0,
                       transparency=args.transparency, on_frame=trace if args.verbose else None)
    cli = Cli(gs, args)
    try:
        rc = 0
        if args.ping_test:
            rc = cli.ping_stats(args.ping_test)
        if args.cmd:
            rc = cli.run(args) or rc
        return rc
    except KeyboardInterrupt:
        return 130
    finally:
        gs.close()


if __name__ == "__main__":
    sys.exit(main())
