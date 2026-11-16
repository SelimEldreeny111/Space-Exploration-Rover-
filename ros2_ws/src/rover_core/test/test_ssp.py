import random

import pytest

from rover_core.crc import crc16_xmodem
from rover_core.ssp import FEND, Frame, StreamDecoder, encode
from rover_core.protocol import (PT, Addr, EnvTlm, Imu, Ranges, SetVelocity, TlmLegacy,
                                 TlmV2, WheelOdom, ArmSetJoints, ArmState, Heartbeat,
                                 decode_tlm, plausible_length)


def h(s: str) -> bytes:
    return bytes.fromhex(s.replace(" ", ""))


# --------------------------------------------------------------------- CRC
def test_crc_check_value():
    assert crc16_xmodem(b"123456789") == 0x31C3


def test_crc_matches_labview_capture():
    # frame captured from the 2023 LabVIEW GS: C0 01 0B 0F 00 7B 96 C0
    assert crc16_xmodem(bytes([0x01, 0x0B, 0x0F, 0x00])) == 0x967B


# --------------------------------------------------------- reference vectors
VECTORS = [
    ("GS->Rover PING", Frame(Addr.ROVER, Addr.GROUND, PT.PING), "C0 01 0B 00 00 45 86 C0"),
    ("GS->Rover GET_TLM", Frame(Addr.ROVER, Addr.GROUND, PT.GET_TLM), "C0 01 0B 0F 00 7B 96 C0"),
    ("Rover->GS ACK", Frame(Addr.GROUND, Addr.ROVER, PT.ACK), "C0 0B 01 01 00 1E 1A C0"),
    ("Rover->GS legacy TLM", Frame(Addr.GROUND, Addr.ROVER, PT.GET_TLM, h("00 87 00 46 2E 08 16")),
     "C0 0B 01 0F 07 00 87 00 46 2E 08 16 80 D9 C0"),
]


@pytest.mark.parametrize("name,frame,wire", VECTORS)
def test_reference_vectors(name, frame, wire):
    assert frame.encode() == h(wire), name


# ----------------------------------------------------------- round trips
@pytest.mark.parametrize("transparency", [False, True])
def test_random_round_trip(transparency):
    rng = random.Random(1)
    frames = [Frame(rng.randrange(256), rng.randrange(256), rng.randrange(256),
                    bytes(rng.randrange(256) for _ in range(rng.randrange(256))))
              for _ in range(300)]
    # avoid DEST/SRC/TYPE == 0xC0 when transparency is off (cannot occur in practice)
    if not transparency:
        frames = [f for f in frames if FEND not in (f.dest, f.src, f.type)]
    stream = b"".join(f.encode(transparency) for f in frames)
    dec = StreamDecoder(transparency=transparency, timeout_s=None)
    # feed in random chunk sizes
    out, i = [], 0
    while i < len(stream):
        n = rng.randrange(1, 40)
        out += dec.feed(stream[i:i + n])
        i += n
    assert out == frames
    assert dec.stats.crc_errors == 0 and dec.stats.framing_errors == 0


def test_payload_containing_flags_without_transparency():
    f = Frame(0x0B, 0x01, PT.GET_TLM, bytes([0xC0, 0xC0, 0xDB, 0x00, 0xC0]))
    dec = StreamDecoder(timeout_s=None)
    assert dec.feed(b"\x00\x13" + f.encode() + f.encode()) == [f, f]


def test_shared_flags_between_frames():
    a = Frame(1, 11, PT.PING)
    b = Frame(1, 11, PT.GET_TLM)
    wire = a.encode() + b.encode()[1:]  # C0 a C0 b C0 (flag shared)
    assert StreamDecoder(timeout_s=None).feed(wire) == [a, b]


def test_crc_error_then_recovery():
    a = Frame(1, 11, PT.START_SCAN)
    bad = bytearray(a.encode())
    bad[3] ^= 0x04  # corrupt TYPE
    dec = StreamDecoder(timeout_s=None)
    out = dec.feed(bytes(bad) + a.encode())
    assert out == [a]
    assert dec.stats.crc_errors == 1


TRAFFIC = b"".join(Frame(1, 11, PT.PING).encode() for _ in range(40))  # > 257 B


@pytest.mark.parametrize("check", [None, plausible_length])
def test_dropped_byte_resynchronises(check):
    frames = [Frame(1, 11, t) for t in (PT.PING, PT.GET_TLM, PT.START_SCAN, PT.STOP_SCAN)]
    wire = bytearray(b"".join(f.encode() for f in frames))
    del wire[9]  # drop a byte inside the 2nd frame
    dec = StreamDecoder(timeout_s=None, length_check=check)
    out = dec.feed(bytes(wire))
    if check is None:
        # a false LEN may hold the decision until LEN+2 bytes have arrived,
        # but no valid frame is lost once more traffic arrives
        out += dec.feed(TRAFFIC)
    assert frames[0] in out and frames[2] in out and frames[3] in out
    assert frames[1] not in out


@pytest.mark.parametrize("check", [None, plausible_length])
def test_false_sync_inside_payload_recovers(check):
    # a frame whose payload contains something that looks like a frame start
    inner = Frame(1, 11, PT.PING).encode()
    carrier = Frame(0x0B, 0x01, PT.GET_TLM, inner + b"\x01\x02")
    nxt = Frame(1, 11, PT.STOP_SCAN)
    wire = carrier.encode()
    # receiver switched on late, in the middle of the carrier frame
    dec = StreamDecoder(timeout_s=None, length_check=check)
    out = dec.feed(wire[6:] + nxt.encode()) + dec.feed(TRAFFIC)
    assert nxt in out


def test_timeout_drops_partial_frame():
    clock = [0.0]
    dec = StreamDecoder(timeout_s=0.05, clock=lambda: clock[0])
    f = Frame(1, 11, PT.PING).encode()
    dec.feed(f[:4])
    clock[0] = 1.0
    out = dec.feed(f)
    assert out == [Frame(1, 11, PT.PING)]
    assert dec.stats.timeouts == 1


def test_address_filter():
    dec = StreamDecoder(own_addr=Addr.MOBILITY, timeout_s=None)
    a = Frame(Addr.MOBILITY, Addr.ROVER, PT.PING)
    b = Frame(Addr.PAYLOAD, Addr.ROVER, PT.PING)
    c = Frame(Addr.BROADCAST, Addr.ROVER, PT.PING)
    assert dec.feed(a.encode() + b.encode() + c.encode()) == [a, c]
    assert dec.stats.filtered == 1


def test_reply_swaps_addresses():
    req = Frame(Addr.ROVER, Addr.GROUND, PT.PING)
    rsp = req.reply(PT.ACK)
    assert (rsp.dest, rsp.src, rsp.type) == (Addr.GROUND, Addr.ROVER, PT.ACK)


def test_transparency_escapes_flags():
    f = Frame(1, 11, PT.GET_TLM, bytes([0xC0, 0xDB]))
    wire = f.encode(transparency=True)
    assert wire.count(bytes([FEND])) == 2
    assert StreamDecoder(transparency=True, timeout_s=None).feed(wire) == [f]


# ------------------------------------------------------------- payloads
def test_payload_sizes():
    assert TlmV2.size() == 56
    assert EnvTlm.size() == 20
    assert Imu.size() == 20
    assert WheelOdom.size() == 28
    assert SetVelocity.size() == 6
    assert ArmSetJoints.size() == 8
    assert ArmState.size() == 7
    assert Heartbeat.size() == 6
    assert TlmLegacy.size() == 7


def test_payload_round_trips():
    t = TlmV2(mode=2, battery_mV=12450, temperature_cC=2510, pressure_Pa=101325,
              x_mm=-1234, y_mm=5678, heading_cdeg=-9000, range_min_mm=812)
    assert TlmV2.unpack(t.pack()) == t
    assert isinstance(decode_tlm(t.pack()), TlmV2)
    assert isinstance(decode_tlm(h("00 87 00 46 2E 08 16")), TlmLegacy)
    w = WheelOdom(123456, [1, -2, 3, -4, 5, 2 ** 31 - 1])
    assert WheelOdom.unpack(w.pack()) == w
    r = Ranges([100, 2000, 0xFFFF, 4, 5, 6])
    assert Ranges.unpack(r.pack()) == r
    assert r.meters()[2] == float("inf")
    sv = SetVelocity.from_si(0.3, -0.5, 0.5)
    assert (sv.v_mm_s, sv.w_mrad_s, sv.timeout_ms) == (300, -500, 500)


def test_legacy_tlm_fields():
    t = TlmLegacy.unpack(h("00 87 00 46 2E 08 16"))
    assert t.gas_raw == 0x8700 and t.us1_cm == 0x46 and t.temp_c == 8 and t.rh_pct == 22
