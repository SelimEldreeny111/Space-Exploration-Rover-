#!/usr/bin/env python
"""Generate cross-implementation test vectors for the C++ MissionX_SSP library.

The Python reference implementation (ros2_ws/src/rover_core: crc.py, ssp.py,
protocol.py) is the oracle. This script writes a plain-text vector file that
firmware/tests/test_ssp.cpp replays through the C++ code, which must produce
exactly the same CRCs, wire bytes, decoded frame lists, decoder statistics,
length-plausibility table and payload byte layouts.

Usage:  python gen_vectors.py <out_file> [seed]

File format (one record per line, hex without spaces, '-' = empty):
  CRC     <data> <crc16>
  ENC     <transparency> <dest> <src> <type> <data> <wire>
  LEN     <type> <bitmap non-strict> <bitmap strict>     (bit n = LEN n allowed)
  STREAM  <name> <transparency> <lencheck 0|1|2> <own_addr|-1> <timeout_ms>
  CHUNK   <t_ms> <bytes>
  FRAME   <dest> <src> <type> <data>                     (expected output, in order)
  STATS   ok crc framing escape timeouts filtered bytes_in bytes_discarded
  END
  PAYLOAD <name> <bytes> <field values...>
"""
from __future__ import annotations

import os
import random
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROVER_CORE = os.path.normpath(os.path.join(HERE, "..", "..", "ros2_ws", "src", "rover_core"))
sys.dont_write_bytecode = True  # never write __pycache__ into ros2_ws (read-only for us)
sys.path.insert(0, ROVER_CORE)

from rover_core.crc import crc16_xmodem  # noqa: E402
from rover_core.ssp import FEND, FESC, Frame, StreamDecoder, encode  # noqa: E402
from rover_core import protocol as P  # noqa: E402
from rover_core.protocol import PT, plausible_length  # noqa: E402


def hx(b: bytes) -> str:
    return b.hex().upper() if b else "-"


KNOWN = [t for t in PT]
ADDRS = [0x01, 0x02, 0x03, 0x04, 0x0B, 0xFF]


def allowed_lengths(t: PT) -> list[int]:
    a = P._ALLOWED_LEN[t]
    return list(range(256)) if a is None else sorted(a)


def rand_frame(rng: random.Random, max_unknown_len: int = 60) -> Frame:
    r = rng.random()
    dest = rng.choice(ADDRS) if rng.random() < 0.85 else rng.randrange(256)
    src = rng.choice(ADDRS) if rng.random() < 0.85 else rng.randrange(256)
    if r < 0.65:
        t = rng.choice(KNOWN)
        lens = allowed_lengths(t)
        n = rng.choice(lens) if len(lens) < 256 else rng.randrange(0, 70)
        ptype = int(t)
    elif r < 0.85:
        ptype = rng.randrange(256)
        n = rng.randrange(0, max_unknown_len)
    else:
        ptype = rng.randrange(256)
        n = rng.randrange(0, 256)
    data = bytes(rng.randrange(256) for _ in range(n))
    if rng.random() < 0.2 and n:
        # sprinkle flags / escapes into the payload
        d = bytearray(data)
        for _ in range(rng.randrange(1, 4)):
            d[rng.randrange(n)] = rng.choice((FEND, FESC))
        data = bytes(d)
    return Frame(dest, src, ptype, data)


def corrupt(rng: random.Random, wire: bytearray, n_mut: int) -> bytearray:
    for _ in range(n_mut):
        if not wire:
            break
        op = rng.randrange(6)
        i = rng.randrange(len(wire))
        if op == 0:                       # dropped byte
            del wire[i]
        elif op == 1:                     # bit flip
            wire[i] ^= 1 << rng.randrange(8)
        elif op == 2:                     # inserted garbage (sometimes a flag)
            ins = bytes(rng.choice((FEND, FESC, rng.randrange(256)))
                        for _ in range(rng.randrange(1, 5)))
            wire[i:i] = ins
        elif op == 3:                     # duplicated byte
            wire[i:i] = wire[i:i + 1]
        elif op == 4:                     # truncated tail of a frame (burst loss)
            del wire[i:i + rng.randrange(1, 12)]
        else:                             # random burst overwrite
            for k in range(i, min(len(wire), i + rng.randrange(1, 6))):
                wire[k] = rng.randrange(256)
    return wire


LENCHECKS = {0: None, 1: plausible_length, 2: lambda t, n: plausible_length(t, n, strict=True)}


def emit_stream(out, name, wire_chunks, transparency, lencheck, own, timeout_ms):
    """wire_chunks: list of (t_ms, bytes). Runs the Python decoder and writes
    the block with the expected frames and statistics."""
    dec = StreamDecoder(own_addr=None if own < 0 else own, transparency=transparency,
                        timeout_s=None if timeout_ms == 0 else timeout_ms / 1000.0,
                        length_check=LENCHECKS[lencheck])
    frames = []
    for t_ms, chunk in wire_chunks:
        frames += dec.feed(chunk, now=t_ms / 1000.0)
    out.write(f"STREAM {name} {int(transparency)} {lencheck} {own} {timeout_ms}\n")
    for t_ms, chunk in wire_chunks:
        out.write(f"CHUNK {t_ms} {hx(chunk)}\n")
    for f in frames:
        out.write(f"FRAME {f.dest} {f.src} {f.type} {hx(f.data)}\n")
    s = dec.stats
    out.write(f"STATS {s.frames_ok} {s.crc_errors} {s.framing_errors} {s.escape_errors} "
              f"{s.timeouts} {s.filtered} {s.bytes_in} {s.bytes_discarded}\nEND\n")
    return len(frames)


def chunked(rng: random.Random, wire: bytes, t0: int = 0, gaps=False):
    chunks, i, t = [], 0, t0
    while i < len(wire):
        n = rng.randrange(1, 40)
        chunks.append((t, wire[i:i + n]))
        i += n
        if gaps:
            # never exactly at the 50 ms boundary (float vs integer compare)
            t += rng.choice((0, 1, 2, 5, 10, 20, 30)) if rng.random() < 0.85 \
                else rng.choice((80, 120, 300, 1000))
        else:
            t += 1
    return chunks


# --------------------------------------------------------------- payloads
def rand_field(rng: random.Random, c: str) -> int:
    lim = {"B": (0, 255), "b": (-128, 127), "H": (0, 65535), "h": (-32768, 32767),
           "I": (0, 2 ** 32 - 1), "i": (-(2 ** 31), 2 ** 31 - 1)}[c]
    r = rng.random()
    if r < 0.1:
        return lim[0]
    if r < 0.2:
        return lim[1]
    return rng.randint(*lim)


PAYLOAD_CLASSES = {
    "SET_VELOCITY": P.SetVelocity, "WHEEL_ODOM": P.WheelOdom, "ENV_TLM": P.EnvTlm,
    "IMU": P.Imu, "ARM_SET_JOINTS": P.ArmSetJoints, "ARM_STATE": P.ArmState,
    "HEARTBEAT": P.Heartbeat, "DRIVE": P.Drive, "NACK": P.Nack, "TLM_LEGACY": P.TlmLegacy,
    "TLM_V2": P.TlmV2,
}


def emit_payloads(out, rng: random.Random, n_each: int) -> int:
    count = 0
    for name, cls in PAYLOAD_CLASSES.items():
        fmt = cls.FMT
        chars = [c for c in fmt if c.isalpha()]
        # expand repeat counts such as "6i" / "6B" / "8h"
        expanded, num = [], ""
        for c in fmt[1:]:
            if c.isdigit():
                num += c
                continue
            expanded += [c] * (int(num) if num else 1)
            num = ""
        assert len(expanded) >= len(chars)
        for _ in range(n_each):
            vals = [rand_field(rng, c) for c in expanded]
            raw = struct.pack(fmt, *vals)
            obj = cls.unpack(raw)
            assert obj.pack() == raw, name
            assert len(raw) == cls.size()
            out.write(f"PAYLOAD {name} {hx(raw)} {' '.join(str(v) for v in obj._values())}\n")
            count += 1
    for _ in range(n_each):
        n = rng.randrange(0, 7)
        r = P.Ranges([rand_field(rng, "H") for _ in range(n)])
        raw = r.pack()
        assert P.Ranges.unpack(raw) == r
        out.write(f"PAYLOAD RANGES {hx(raw)} {n} {' '.join(str(v) for v in r.range_mm)}\n")
        count += 1
    return count


# ------------------------------------------------------------------- main
def main() -> int:
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "vectors.txt")
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 20231115
    rng = random.Random(seed)
    counts = dict(crc=0, enc=0, len=0, streams=0, frames=0, payloads=0)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", newline="\n") as out:
        out.write(f"# generated by gen_vectors.py seed={seed} from {ROVER_CORE}\n")
        # CRC
        for data in (b"", b"123456789", bytes([0x01, 0x0B, 0x0F, 0x00])):
            out.write(f"CRC {hx(data)} {crc16_xmodem(data):04X}\n")
            counts["crc"] += 1
        for _ in range(200):
            data = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 300)))
            out.write(f"CRC {hx(data)} {crc16_xmodem(data):04X}\n")
            counts["crc"] += 1
        # encoder
        for transparency in (False, True):
            for _ in range(300):
                f = rand_frame(rng)
                out.write(f"ENC {int(transparency)} {f.dest} {f.src} {f.type} {hx(f.data)} "
                          f"{hx(encode(f.dest, f.src, f.type, f.data, transparency))}\n")
                counts["enc"] += 1
        # length plausibility table (must equal _ALLOWED_LEN)
        for t in range(256):
            bm = bytearray(32)
            bs = bytearray(32)
            for n in range(256):
                if plausible_length(t, n):
                    bm[n // 8] |= 1 << (n % 8)
                if plausible_length(t, n, strict=True):
                    bs[n // 8] |= 1 << (n % 8)
            out.write(f"LEN {t} {bm.hex().upper()} {bs.hex().upper()}\n")
            counts["len"] += 1

        # streams ---------------------------------------------------------
        sid = 0

        def stream(name, chunks, transparency, lencheck, own, timeout_ms):
            nonlocal sid
            sid += 1
            counts["streams"] += 1
            counts["frames"] += emit_stream(out, f"{name}#{sid}", chunks, transparency,
                                            lencheck, own, timeout_ms)

        for transparency in (False, True):
            for lencheck in (0, 1, 2):
                for own in (-1, 0x02):
                    # clean traffic
                    frames = [rand_frame(rng) for _ in range(60)]
                    wire = b"".join(f.encode(transparency) for f in frames)
                    stream("clean", chunked(rng, wire), transparency, lencheck, own, 0)
                    # shared flags between frames (C0 a C0 b C0)
                    wire = bytearray(frames[0].encode(transparency))
                    for f in frames[1:30]:
                        wire += f.encode(transparency)[1:]
                    stream("shared", chunked(rng, bytes(wire)), transparency, lencheck, own, 0)
                    # corrupted traffic: drops, bit flips, garbage, truncations
                    for k in range(12):
                        frames = [rand_frame(rng) for _ in range(rng.randrange(20, 70))]
                        wire = bytearray(b"".join(f.encode(transparency) for f in frames))
                        wire = corrupt(rng, wire, rng.randrange(1, 25))
                        lead = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 20)))
                        stream("corrupt", chunked(rng, lead + bytes(wire)), transparency,
                               lencheck, own, 0)
                    # mostly small frames (false starts inside payloads)
                    frames = [Frame(rng.choice(ADDRS), 0x0B, int(rng.choice(KNOWN[:10])), b"")
                              for _ in range(40)]
                    carrier = [Frame(0x0B, 0x01, PT.GET_TLM,
                                     b"".join(Frame(1, 11, PT.PING).encode() for _ in range(3)))]
                    wire = bytearray(b"".join(f.encode(transparency) for f in carrier + frames))
                    wire = corrupt(rng, wire, 5)
                    stream("falsesync", chunked(rng, bytes(wire[rng.randrange(1, 12):])),
                           transparency, lencheck, own, 0)
                    # timed traffic with inter-byte timeouts (50 ms)
                    frames = [rand_frame(rng) for _ in range(50)]
                    wire = bytearray(b"".join(f.encode(transparency) for f in frames))
                    wire = corrupt(rng, wire, rng.randrange(0, 6))
                    stream("timed", chunked(rng, bytes(wire), t0=rng.randrange(0, 1000), gaps=True),
                           transparency, lencheck, own, 50)

        counts["payloads"] = emit_payloads(out, rng, 40)
    print("gen_vectors: " + ", ".join(f"{k}={v}" for k, v in counts.items()) + f" -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
