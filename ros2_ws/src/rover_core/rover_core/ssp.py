"""Simple Serial Protocol (SSP) framing — reference implementation (ICD v2.0 §2-§3).

Wire format (transparency OFF, wire-compatible with the 2023 LabVIEW GS)::

    C0 | DEST | SRC | TYPE | LEN | DATA[LEN] | CRC_L | CRC_H | C0

CRC-16/XMODEM over DEST..DATA, transmitted low byte first.

With ``transparency=True`` the bytes between the flags are SLIP-escaped
(0xC0 -> DB DC, 0xDB -> DB DD) and the receiver delimits frames by FEND only.

The :class:`StreamDecoder` is *length-driven* when transparency is off and
re-synchronises on the next flag after any framing/CRC error without losing a
byte that could start a valid frame (ICD §3, step 5).
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Iterable, List, Optional

from .crc import crc16_xmodem

__all__ = [
    "FEND", "FESC", "TFEND", "TFESC", "MAX_DATA", "Frame", "encode",
    "StreamDecoder", "DecoderStats", "SSPError",
]

FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD
MAX_DATA = 255
HEADER_LEN = 4  # DEST SRC TYPE LEN


class SSPError(ValueError):
    """Raised for invalid arguments when building frames."""


@dataclass(frozen=True)
class Frame:
    dest: int
    src: int
    type: int
    data: bytes = b""

    def __post_init__(self) -> None:
        for name in ("dest", "src", "type"):
            v = getattr(self, name)
            if not 0 <= v <= 0xFF:
                raise SSPError(f"{name}={v} out of range 0..255")
        if len(self.data) > MAX_DATA:
            raise SSPError(f"DATA length {len(self.data)} > {MAX_DATA}")

    # ------------------------------------------------------------------ helpers
    @property
    def body(self) -> bytes:
        """DEST..DATA (the CRC-protected part)."""
        return bytes((self.dest, self.src, self.type, len(self.data))) + bytes(self.data)

    @property
    def crc(self) -> int:
        return crc16_xmodem(self.body)

    def encode(self, transparency: bool = False) -> bytes:
        return encode(self.dest, self.src, self.type, self.data, transparency)

    def reply(self, ptype: int, data: bytes = b"", own_addr: Optional[int] = None) -> "Frame":
        """Build a response with swapped addresses (ICD §4)."""
        src = self.dest if own_addr is None else own_addr
        return Frame(dest=self.src, src=src, type=ptype, data=bytes(data))

    def __repr__(self) -> str:  # compact, hex
        return (f"Frame(dest=0x{self.dest:02X}, src=0x{self.src:02X}, "
                f"type=0x{self.type:02X}, len={len(self.data)}, data={self.data.hex(' ')})")


def _escape(payload: bytes) -> bytes:
    out = bytearray()
    for b in payload:
        if b == FEND:
            out += bytes((FESC, TFEND))
        elif b == FESC:
            out += bytes((FESC, TFESC))
        else:
            out.append(b)
    return bytes(out)


def encode(dest: int, src: int, ptype: int, data: bytes | bytearray | Iterable[int] = b"",
           transparency: bool = False) -> bytes:
    """Encode one SSP frame (with both FEND flags)."""
    data = bytes(data)
    if len(data) > MAX_DATA:
        raise SSPError(f"DATA length {len(data)} > {MAX_DATA}")
    for name, v in (("dest", dest), ("src", src), ("type", ptype)):
        if not 0 <= v <= 0xFF:
            raise SSPError(f"{name}={v} out of range")
    body = bytes((dest, src, ptype, len(data))) + data
    crc = crc16_xmodem(body)
    inner = body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
    if transparency:
        inner = _escape(inner)
    return bytes((FEND,)) + inner + bytes((FEND,))


@dataclass
class DecoderStats:
    frames_ok: int = 0
    crc_errors: int = 0
    framing_errors: int = 0
    escape_errors: int = 0
    timeouts: int = 0
    filtered: int = 0          # valid frames not addressed to us
    bytes_in: int = 0
    bytes_discarded: int = 0   # bytes seen while hunting for a flag

    def as_dict(self) -> dict:
        return dict(self.__dict__)


class StreamDecoder:
    """Incremental SSP receiver.

    Parameters
    ----------
    own_addr:
        If given, frames whose DEST is neither ``own_addr`` nor 0xFF are counted
        in ``stats.filtered`` and not returned (unless ``promiscuous``).
    transparency:
        SLIP escaping on/off (must match the sender).
    timeout_s:
        Inter-byte timeout inside a frame (ICD §3 step 6). ``None`` disables it.
    clock:
        Monotonic clock, injectable for tests.
    """

    def __init__(self, own_addr: Optional[int] = None, transparency: bool = False,
                 timeout_s: Optional[float] = 0.05, promiscuous: bool = False,
                 clock: Callable[[], float] = time.monotonic,
                 length_check: Optional[Callable[[int, int], bool]] = None) -> None:
        self.own_addr = own_addr
        self.transparency = transparency
        self.timeout_s = timeout_s
        self.promiscuous = promiscuous
        #: optional ``f(type, len) -> bool``; an implausible LEN for a known
        #: TYPE is treated as a framing error as soon as the header is complete
        #: (see protocol.plausible_length). Speeds up recovery from false syncs.
        self.length_check = length_check
        self._clock = clock
        self.stats = DecoderStats()
        self._in_frame = False
        self._buf = bytearray()
        self._esc = False
        self._last_t: Optional[float] = None
        self._queue: deque[int] = deque()

    # ------------------------------------------------------------------ public
    def reset(self) -> None:
        self._in_frame = False
        self._buf.clear()
        self._esc = False
        self._queue.clear()

    def feed(self, data: bytes | bytearray | Iterable[int], now: Optional[float] = None) -> List[Frame]:
        """Feed received bytes; return the list of complete, CRC-valid frames."""
        t = self._clock() if now is None else now
        if (self.timeout_s is not None and self._last_t is not None and self._in_frame
                and self._buf and (t - self._last_t) > self.timeout_s):
            self.stats.timeouts += 1
            self._in_frame = False
            self._buf.clear()
            self._esc = False
        data = bytes(data)
        if data:
            self._last_t = t
        self.stats.bytes_in += len(data)
        self._queue.extend(data)
        out: List[Frame] = []
        step = self._step_slip if self.transparency else self._step_len
        while self._queue:
            b = self._queue.popleft()
            fr = step(b)
            if fr is not None:
                out.append(fr)
        return out

    # --------------------------------------------------------------- internals
    def _check(self, body: bytes) -> tuple[Optional[Frame], str]:
        """Validate DEST..CRC. Returns (frame, status) with status in
        {'ok', 'filtered', 'framing', 'crc'} and updates the statistics."""
        if len(body) < HEADER_LEN + 2 or len(body) != HEADER_LEN + body[3] + 2:
            self.stats.framing_errors += 1
            return None, "framing"
        crc_rx = body[-2] | (body[-1] << 8)
        if crc16_xmodem(body[:-2]) != crc_rx:
            self.stats.crc_errors += 1
            return None, "crc"
        fr = Frame(body[0], body[1], body[2], bytes(body[HEADER_LEN:-2]))
        if (self.own_addr is not None and not self.promiscuous
                and fr.dest not in (self.own_addr, 0xFF)):
            self.stats.filtered += 1
            return None, "filtered"
        self.stats.frames_ok += 1
        return fr, "ok"

    def _accept(self, body: bytes) -> Optional[Frame]:
        return self._check(body)[0]

    def _resync(self) -> None:
        """Push back the bytes after the failed opening flag (ICD §3 step 5)."""
        raw = bytes(self._buf)
        self._buf.clear()
        idx = raw.find(FEND)
        if idx < 0:
            self._in_frame = False
            self.stats.bytes_discarded += len(raw)
            return
        self.stats.bytes_discarded += idx
        self._in_frame = True
        # re-process everything after that flag before the remaining input
        self._queue.extendleft(reversed(raw[idx + 1:]))

    def _step_len(self, b: int) -> Optional[Frame]:
        if not self._in_frame:
            if b == FEND:
                self._in_frame = True
                self._buf.clear()
            else:
                self.stats.bytes_discarded += 1
            return None
        if b == FEND and len(self._buf) < 3:
            # A flag where DEST/SRC/TYPE is expected: idle flag, closing flag of
            # the previous frame, or a truncated header -> restart the header.
            # (LEN may legitimately be 0xC0 = 192, so position 3 is exempt.)
            if self._buf:
                self.stats.framing_errors += 1
                self._buf.clear()
            return None
        self._buf.append(b)
        n = len(self._buf)
        if n < HEADER_LEN:
            return None
        if n == HEADER_LEN and self.length_check is not None \
                and not self.length_check(self._buf[2], self._buf[3]):
            self.stats.framing_errors += 1
            self._resync()
            return None
        need = HEADER_LEN + self._buf[3] + 2
        if n <= need:
            return None
        # n == need + 1 -> this byte must be the closing flag
        if b != FEND:
            self.stats.framing_errors += 1
            self._resync()
            return None
        fr, status = self._check(bytes(self._buf[:-1]))
        if status == "crc":
            # The opening flag may have been a 0xC0 inside an earlier frame's
            # payload: rescan the buffered bytes (closing flag included).
            self._resync()
            return None
        # ok / filtered: the closing flag doubles as the next opening flag
        self._buf.clear()
        self._in_frame = True
        return fr

    def _step_slip(self, b: int) -> Optional[Frame]:
        if b == FEND:
            fr = None
            if self._in_frame and self._buf and not self._esc:
                fr = self._accept(bytes(self._buf))
            elif self._esc:
                self.stats.escape_errors += 1
            self._in_frame = True
            self._buf.clear()
            self._esc = False
            return fr
        if not self._in_frame:
            self.stats.bytes_discarded += 1
            return None
        if self._esc:
            self._esc = False
            if b == TFEND:
                self._buf.append(FEND)
            elif b == TFESC:
                self._buf.append(FESC)
            else:
                self.stats.escape_errors += 1
                self._in_frame = False
                self._buf.clear()
            return None
        if b == FESC:
            self._esc = True
            return None
        self._buf.append(b)
        if len(self._buf) > HEADER_LEN + MAX_DATA + 2:
            self.stats.framing_errors += 1
            self._in_frame = False
            self._buf.clear()
        return None
