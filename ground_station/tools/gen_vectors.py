#!/usr/bin/env python3
"""Generate cross-check vectors for the JavaScript SSP codec (web/js/ssp.js).

The Python reference implementation (``rover_core.ssp`` / ``rover_core.protocol``
from ``ros2_ws/src/rover_core``) produces:

* CRC-16/XMODEM vectors and the ICD §2.1 reference frames,
* random frames with their wire encoding (transparency off and on),
* corrupted byte streams (dropped bytes, bit flips, inserted noise, late
  start, flag-rich image payloads) together with the frames *and* decoder
  statistics that ``rover_core.ssp.StreamDecoder`` extracts from them, for
  several decoder configurations (length check, address filter, SLIP),
* inter-byte timeout scenarios (explicit timestamps),
* payload encodings for every struct in ``rover_core.protocol``,
* the full ``plausible_length`` table.

``web/test/crosscheck.test.mjs`` replays everything through the JavaScript
implementation and requires identical results.

    python tools/gen_vectors.py              # (re)write web/test/test_vectors.json
    python tools/gen_vectors.py --check      # exit 1 if the file is stale
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
GS_ROOT = HERE.parent
DEFAULT_OUT = GS_ROOT / "web" / "test" / "test_vectors.json"
ROVER_CORE = Path(os.environ.get("MISSIONX_ROVER_CORE",
                                 GS_ROOT.parent / "ros2_ws" / "src" / "rover_core"))
if str(ROVER_CORE) not in sys.path:
    sys.path.insert(0, str(ROVER_CORE))

from rover_core.crc import crc16_xmodem  # noqa: E402
from rover_core.ssp import FEND, Frame, StreamDecoder, encode  # noqa: E402
from rover_core import protocol as P  # noqa: E402

SEED = 20231115


def hx(b: bytes) -> str:
    return bytes(b).hex().upper()


def frame_json(f: Frame) -> dict:
    return {"dest": f.dest, "src": f.src, "type": f.type, "data": hx(f.data)}


# ------------------------------------------------------------------ frames
def realistic_frames(rng: random.Random, n: int) -> list[Frame]:
    """Traffic that looks like a real GS session (incl. 0xC0-rich payloads)."""
    G, R = P.Addr.GROUND, P.Addr.ROVER
    out: list[Frame] = []
    for _ in range(n):
        k = rng.randrange(10)
        if k == 0:
            out.append(Frame(R, G, P.PT.PING))
        elif k == 1:
            out.append(Frame(G, R, P.PT.ACK, bytes([rng.choice([0x51, 0x52, 0x53, 0x60])])
                             if rng.random() < 0.5 else b""))
        elif k == 2:
            out.append(Frame(R, G, P.PT.GET_TLM))
        elif k == 3:
            t = P.TlmV2(mode=rng.randrange(4), fault_flags=rng.randrange(32),
                        battery_mV=rng.randrange(10000, 12700), uptime_s=rng.randrange(10 ** 6),
                        temperature_cC=rng.randrange(-1000, 5000), humidity_cP=rng.randrange(10000),
                        pressure_Pa=rng.randrange(90000, 105000), roll_cdeg=rng.randrange(-3000, 3000),
                        pitch_cdeg=rng.randrange(-3000, 3000), yaw_cdeg=rng.randrange(-18000, 18000),
                        x_mm=rng.randrange(-50000, 50000), y_mm=rng.randrange(-50000, 50000),
                        heading_cdeg=rng.randrange(-18000, 18000), range_min_mm=rng.randrange(20, 4000),
                        mq2_raw=rng.randrange(1024), proximity=rng.randrange(2))
            out.append(Frame(G, R, P.PT.GET_TLM, t.pack()))
        elif k == 4:
            out.append(Frame(G, R, P.PT.GET_TLM, bytes(rng.randrange(256) for _ in range(7))))
        elif k == 5:  # legacy image chunk, pixel values biased towards 0xC0
            out.append(Frame(G, R, P.PT.GET_IMAGE,
                             bytes(rng.choice((0xC0, 0xC0, rng.randrange(256))) for _ in range(200))))
        elif k == 6:
            hdr = P.ImageChunkHeader(rng.randrange(60), 60, 160, 120, 1).pack()
            out.append(Frame(G, R, P.PT.IMAGE_CHUNK, hdr + bytes(rng.randrange(256)
                                                                  for _ in range(rng.randrange(201)))))
        elif k == 7:
            out.append(Frame(R, G, P.PT.DRIVE, P.Drive(rng.randrange(-100, 101), rng.randrange(-100, 101),
                                                       rng.randrange(1000)).pack()))
        elif k == 8:
            out.append(Frame(G, R, P.PT.NACK, bytes([rng.randrange(256), rng.randrange(1, 6)])))
        else:
            out.append(Frame(rng.randrange(256), rng.randrange(256), rng.randrange(256),
                             bytes(rng.randrange(256) for _ in range(rng.randrange(40)))))
    return out


def random_frames(rng: random.Random, n: int, avoid_fend_header: bool) -> list[Frame]:
    out = []
    while len(out) < n:
        f = Frame(rng.randrange(256), rng.randrange(256), rng.randrange(256),
                  bytes(rng.randrange(256) for _ in range(rng.randrange(256))))
        if avoid_fend_header and FEND in (f.dest, f.src, f.type):
            continue
        out.append(f)
    return out


def corrupt(stream: bytes, rng: random.Random, p_drop: float, p_flip: float, p_insert: float) -> bytes:
    out = bytearray()
    for b in stream:
        if rng.random() < p_drop:
            continue
        if rng.random() < p_insert:
            out.append(rng.randrange(256))
        for bit in range(8):
            if rng.random() < p_flip:
                b ^= 1 << bit
        out.append(b)
    return bytes(out)


def chunks_of(data: bytes, rng: random.Random, lo: int = 1, hi: int = 64) -> list[bytes]:
    out, i = [], 0
    while i < len(data):
        n = rng.randrange(lo, hi + 1)
        out.append(data[i:i + n])
        i += n
    return out


LENGTH_CHECKS = {
    None: None,
    "plausible": P.plausible_length,
    "strict": lambda t, n: P.plausible_length(t, n, strict=True),
}


def run_scenario(name: str, chunks: list[tuple[float | None, bytes]], *, own_addr=None,
                 transparency=False, timeout_s=None, promiscuous=False, length_check=None) -> dict:
    dec = StreamDecoder(own_addr=own_addr, transparency=transparency, timeout_s=timeout_s,
                        promiscuous=promiscuous, clock=lambda: 0.0,
                        length_check=LENGTH_CHECKS[length_check])
    frames: list[Frame] = []
    for now, data in chunks:
        frames += dec.feed(data, now=now if now is not None else 0.0)
    return {
        "name": name,
        "config": {"own_addr": own_addr, "transparency": transparency, "timeout_s": timeout_s,
                   "promiscuous": promiscuous, "length_check": length_check},
        "chunks": [[now, hx(d)] for now, d in chunks],
        "expected_frames": [frame_json(f) for f in frames],
        "expected_stats": dec.stats.as_dict(),
    }


# ----------------------------------------------------------------- builder
def build() -> dict:
    rng = random.Random(SEED)
    out: dict = {"generator": "ground_station/tools/gen_vectors.py", "seed": SEED,
                 "rover_core_version": __import__("rover_core").__version__}

    # CRC
    crc = [{"data": hx(b"123456789"), "crc": crc16_xmodem(b"123456789")}]
    for _ in range(40):
        d = bytes(rng.randrange(256) for _ in range(rng.randrange(300)))
        crc.append({"data": hx(d), "crc": crc16_xmodem(d)})
    out["crc"] = crc

    # ICD §2.1 reference vectors
    G, R = P.Addr.GROUND, P.Addr.ROVER
    ref = [("GS->Rover PING", Frame(R, G, P.PT.PING)),
           ("GS->Rover GET_TLM", Frame(R, G, P.PT.GET_TLM)),
           ("Rover->GS ACK", Frame(G, R, P.PT.ACK)),
           ("Rover->GS legacy TLM", Frame(G, R, P.PT.GET_TLM, bytes.fromhex("00870046 2E0816"))),
           ]
    out["reference"] = [{"name": n, "frame": frame_json(f), "wire": hx(f.encode())} for n, f in ref]

    # encodings (both transparency settings)
    enc = []
    for f in random_frames(rng, 60, avoid_fend_header=False) + realistic_frames(rng, 40):
        for tr in (False, True):
            enc.append({"frame": frame_json(f), "transparency": tr, "wire": hx(encode(f.dest, f.src, f.type,
                                                                                    f.data, tr))})
    out["encode"] = enc

    # streams
    sc = []
    clean = random_frames(rng, 80, avoid_fend_header=True)
    wire = b"".join(f.encode() for f in clean)
    sc.append(run_scenario("clean_random_chunked", [(None, c) for c in chunks_of(wire, rng)]))

    traffic = realistic_frames(rng, 160)
    wire = b"".join(f.encode() for f in traffic)
    bad = corrupt(wire, rng, p_drop=0.002, p_flip=3e-4, p_insert=0.001)
    for lc in (None, "plausible", "strict"):
        sc.append(run_scenario(f"corrupted_realistic_lc={lc}", [(None, bad)], length_check=lc))
    sc.append(run_scenario("corrupted_realistic_chunked_plausible",
                           [(None, c) for c in chunks_of(bad, rng)], length_check="plausible"))
    sc.append(run_scenario("corrupted_realistic_own_addr_GS", [(None, bad)], own_addr=int(G),
                           length_check="plausible"))
    sc.append(run_scenario("corrupted_realistic_own_addr_promiscuous", [(None, bad)], own_addr=int(R),
                           promiscuous=True))
    sc.append(run_scenario("late_start_mid_frame", [(None, wire[rng.randrange(3, 400):])],
                           length_check="plausible"))

    heavy = realistic_frames(rng, 120)
    wire = b"".join(f.encode() for f in heavy)
    for lc in (None, "plausible"):
        sc.append(run_scenario(f"heavy_corruption_lc={lc}",
                               [(None, corrupt(wire, rng, p_drop=0.01, p_flip=2e-3, p_insert=0.005))],
                               length_check=lc))

    rnd = random_frames(rng, 60, avoid_fend_header=False)  # 0xC0 may appear in headers
    wire = b"".join(f.encode() for f in rnd)
    sc.append(run_scenario("random_with_fend_headers", [(None, corrupt(wire, rng, 0.003, 5e-4, 0.002))]))

    slip = realistic_frames(rng, 80) + random_frames(rng, 30, avoid_fend_header=False)
    wire = b"".join(f.encode(transparency=True) for f in slip)
    sc.append(run_scenario("slip_clean", [(None, c) for c in chunks_of(wire, rng)], transparency=True))
    sc.append(run_scenario("slip_corrupted", [(None, corrupt(wire, rng, 0.003, 5e-4, 0.002))],
                           transparency=True, own_addr=int(G)))

    # inter-byte timeout (explicit timestamps, seconds)
    ping = Frame(R, G, P.PT.PING).encode()
    tlm = Frame(G, R, P.PT.GET_TLM, P.TlmV2(battery_mV=12000).pack()).encode()
    t_chunks = [(0.000, ping[:4]), (1.000, ping), (1.010, tlm[:20]), (1.040, tlm[20:40]),
                (1.200, tlm[40:]), (1.300, tlm), (1.310, tlm[:5]), (1.330, tlm[5:])]
    for to in (0.05, None):
        sc.append(run_scenario(f"timeouts_{to}", t_chunks, timeout_s=to, length_check="plausible"))
    t2, t = [], 5.0
    for f in realistic_frames(rng, 40):
        w = f.encode()
        cut = rng.randrange(1, len(w))
        t2.append((round(t, 4), w[:cut]))
        t += rng.choice((0.001, 0.02, 0.049, 0.051, 0.2))
        t2.append((round(t, 4), w[cut:]))
        t += 0.01
    for tr in (False,):
        sc.append(run_scenario("timeouts_random_gaps", t2, timeout_s=0.05, transparency=tr))
    out["streams"] = sc

    # payloads
    pl = []

    def add(cls, obj):
        pl.append({"cls": cls.__name__, "fields": obj_fields(obj), "hex": hx(obj.pack())})

    for _ in range(6):
        add(P.TlmV2, P.TlmV2(*[rng_val(rng, c) for c in struct_codes(P.TlmV2.FMT)]))
        add(P.TlmLegacy, P.TlmLegacy(*[rng.randrange(256) for _ in range(7)]))
        add(P.EnvTlm, P.EnvTlm(*[rng_val(rng, c) for c in struct_codes(P.EnvTlm.FMT)]))
        add(P.Imu, P.Imu(*[rng_val(rng, c) for c in struct_codes(P.Imu.FMT)]))
        add(P.WheelOdom, P.WheelOdom(rng_val(rng, "I"), [rng_val(rng, "i") for _ in range(6)]))
        add(P.SetVelocity, P.SetVelocity(rng_val(rng, "h"), rng_val(rng, "h"), rng_val(rng, "H")))
        add(P.ArmSetJoints, P.ArmSetJoints([rng.randrange(256) for _ in range(6)], rng_val(rng, "H")))
        add(P.ArmState, P.ArmState([rng.randrange(256) for _ in range(6)], rng.randrange(2)))
        add(P.Heartbeat, P.Heartbeat(rng_val(rng, "I"), rng.randrange(4), rng.randrange(32)))
        add(P.Drive, P.Drive(rng_val(rng, "b"), rng_val(rng, "b"), rng_val(rng, "H")))
        add(P.Nack, P.Nack(rng.randrange(256), rng.randrange(1, 6)))
        add(P.ImageChunkHeader, P.ImageChunkHeader(*[rng_val(rng, c) for c in "HHHHB"]))
        r = P.Ranges([rng.choice((P.NO_ECHO, rng.randrange(20, 4000))) for _ in range(rng.randrange(7))])
        pl.append({"cls": "Ranges", "fields": {"range_mm": r.range_mm}, "hex": hx(r.pack())})
    add(P.TlmV2, P.TlmV2())
    add(P.SetVelocity, P.SetVelocity())
    out["payloads"] = pl

    sv = []
    for v, w, to in [(0.3, -0.5, 0.5), (0.0005, -0.0005, 0.0), (0.0025, 0.0035, 0.0119),
                     (40.0, -40.0, 100.0), (-0.0015, 0.0045, 1.2345)] + \
            [(rng.uniform(-1, 1), rng.uniform(-2, 2), rng.uniform(0, 2)) for _ in range(10)]:
        s = P.SetVelocity.from_si(v, w, to)
        sv.append({"v": v, "w": w, "timeout_s": to, "fields": obj_fields(s)})
    out["set_velocity_from_si"] = sv

    dt = []
    for d in (P.TlmV2(mode=1).pack(), bytes.fromhex("00870046 2E0816"), bytes(56), bytes(17), b""):
        try:
            res = type(P.decode_tlm(d)).__name__
        except ValueError:
            res = "error"
        dt.append({"hex": hx(d), "result": res})
    out["decode_tlm"] = dt

    table = {}
    for t in range(256):
        for strict in (False, True):
            ok = [n for n in range(256) if P.plausible_length(t, n, strict=strict)]
            table[f"{t}:{int(strict)}"] = "any" if len(ok) == 256 else ok
    out["plausible_length"] = table
    out["sizes"] = {c.__name__: c.size() for c in (P.TlmV2, P.EnvTlm, P.Imu, P.WheelOdom, P.SetVelocity,
                                                   P.ArmSetJoints, P.ArmState, P.Heartbeat, P.TlmLegacy,
                                                   P.Drive, P.Nack, P.ImageChunkHeader)}
    return out


def struct_codes(fmt: str) -> list[str]:
    import re
    codes = []
    for num, c in re.findall(r"(\d*)([a-zA-Z])", fmt.replace("<", "")):
        codes += [c] * (int(num) if num else 1)
    return codes


_RANGES = {"B": (0, 255), "b": (-128, 127), "H": (0, 65535), "h": (-32768, 32767),
           "I": (0, 2 ** 32 - 1), "i": (-2 ** 31, 2 ** 31 - 1)}


def rng_val(rng: random.Random, code: str) -> int:
    lo, hi = _RANGES[code]
    if rng.random() < 0.15:
        return rng.choice((lo, hi))
    return rng.randint(lo, hi)


def obj_fields(obj) -> dict:
    from dataclasses import fields
    return {f.name: (list(getattr(obj, f.name)) if isinstance(getattr(obj, f.name), list)
                     else getattr(obj, f.name)) for f in fields(obj)}


def render() -> str:
    return json.dumps(build(), indent=1, sort_keys=True) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-o", "--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--check", action="store_true", help="only verify that the file is up to date")
    a = ap.parse_args(argv)
    text = render()
    if a.check:
        cur = a.out.read_text(encoding="utf-8") if a.out.exists() else ""
        if cur != text:
            print(f"{a.out} is stale; run tools/gen_vectors.py", file=sys.stderr)
            return 1
        print(f"{a.out} is up to date")
        return 0
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(text, encoding="utf-8", newline="\n")
    d = json.loads(text)
    print(f"wrote {a.out} ({len(text) // 1024} KiB): {len(d['crc'])} CRC, {len(d['encode'])} encodings, "
          f"{len(d['streams'])} streams, {len(d['payloads'])} payloads")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
