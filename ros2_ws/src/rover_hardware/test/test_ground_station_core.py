import math

import numpy as np
import pytest

from rover_core.protocol import PT, Addr, Mode, Nack, NackReason, TlmV2
from rover_core.ssp import Frame, StreamDecoder
from rover_hardware.conversions import TelemetrySnapshot, reassemble_v2_chunks
from rover_hardware.ground_station_core import GroundStationCore


class FakePorts:
    def __init__(self):
        self.mode = Mode.IDLE
        self.latched = False
        self.calls = []
        self.collect_busy = False
        self.routed = []
        self.route_ok = True
        self.img = (np.arange(10000) % 251).astype(np.uint8).reshape(100, 100)

    def get_mode(self):
        return self.mode

    def set_mode(self, mode):
        self.calls.append(("set_mode", int(mode)))
        self.mode = int(mode)

    def estop_latched(self):
        return self.latched

    def estop(self, engage):
        self.calls.append(("estop", engage))
        self.latched = engage

    def drive(self, v, w, duration_s):
        self.calls.append(("drive", v, w, duration_s))

    def stop_navigation(self):
        self.calls.append(("stop_navigation",))

    def start_collect(self):
        if self.collect_busy:
            return False
        self.collect_busy = True
        return True

    def capture_image(self):
        self.calls.append(("capture",))
        return False

    def image_gray(self):
        return self.img

    def image_v2(self, quality):
        self.calls.append(("image_v2", quality))
        return bytes(range(250)) * 2, 20, 25, 0

    def telemetry(self):
        return TelemetrySnapshot(mode=int(self.mode), battery_v=12.0, x=1.0, y=2.0,
                                 heading=math.radians(90), ranges=[0.5, math.inf])

    def route(self, dest, raw):
        self.routed.append((dest, raw))
        return self.route_ok


def gs(ptype, data=b"", dest=Addr.ROVER):
    return Frame(int(dest), int(Addr.GROUND), int(ptype), bytes(data))


@pytest.fixture
def core():
    return GroundStationCore(FakePorts(), drive_max_speed=0.4, drive_max_yaw_rate=0.8)


def test_ping_ack_matches_icd_vector(core):
    (r,) = core.handle(gs(PT.PING))
    assert r.type == PT.ACK and r.data == b"" and r.dest == Addr.GROUND and r.src == Addr.ROVER
    assert r.encode() == bytes.fromhex("C00B0101001E1AC0")       # ICD 2.1 Rover->GS ACK


def test_get_tlm_v2(core):
    (r,) = core.handle(gs(PT.GET_TLM))
    assert r.type == PT.GET_TLM and len(r.data) == 56
    t = TlmV2.unpack(r.data)
    assert t.battery_mV == 12000 and (t.x_mm, t.y_mm) == (1000, 2000)
    assert t.heading_cdeg == 9000 and t.range_min_mm == 500


def test_unknown_type_and_bad_length(core):
    (r,) = core.handle(gs(0x77))
    assert r.type == PT.NACK and Nack.unpack(r.data) == Nack(0x77, NackReason.UNKNOWN_TYPE)
    (r,) = core.handle(gs(PT.PING, b"\x01"))
    assert Nack.unpack(r.data) == Nack(PT.PING, NackReason.BAD_LENGTH)
    (r,) = core.handle(gs(PT.SET_VELOCITY, b"\x00" * 6))        # not a Brain command
    assert Nack.unpack(r.data).reason == NackReason.UNKNOWN_TYPE


def test_start_and_stop_scan(core):
    p = core.ports
    (r,) = core.handle(gs(PT.START_SCAN))
    assert (r.type, r.data) == (PT.START_SCAN, b"") and p.mode == Mode.AUTONOMOUS
    (r,) = core.handle(gs(PT.STOP_SCAN))
    assert (r.type, r.data) == (PT.STOP_SCAN, b"") and p.mode == Mode.IDLE
    assert ("stop_navigation",) in p.calls


def test_start_scan_refused_with_estop(core):
    core.ports.latched = True
    (r,) = core.handle(gs(PT.START_SCAN))
    assert Nack.unpack(r.data) == Nack(PT.START_SCAN, NackReason.NOT_PERMITTED)
    assert core.ports.mode == Mode.IDLE


def test_collect_object_busy(core):
    (r,) = core.handle(gs(PT.COLLECT_OBJECT))
    assert (r.type, r.data) == (PT.COLLECT_OBJECT, b"")
    (r,) = core.handle(gs(PT.COLLECT_OBJECT))
    assert Nack.unpack(r.data) == Nack(PT.COLLECT_OBJECT, NackReason.BUSY)


def test_collect_object_refused_with_estop(core):
    core.ports.latched = True
    (r,) = core.handle(gs(PT.COLLECT_OBJECT))
    assert Nack.unpack(r.data) == Nack(PT.COLLECT_OBJECT, NackReason.NOT_PERMITTED)
    assert not core.ports.collect_busy


def test_estop_and_clear(core):
    (r,) = core.handle(gs(PT.ESTOP))
    assert (r.type, r.data) == (PT.ACK, bytes([PT.ESTOP])) and core.ports.latched
    (r,) = core.handle(gs(PT.CLEAR_ESTOP))
    assert (r.type, r.data) == (PT.ACK, bytes([PT.CLEAR_ESTOP])) and not core.ports.latched


def test_set_mode(core):
    p = core.ports
    (r,) = core.handle(gs(PT.SET_MODE, b"\x02"))
    assert (r.type, r.data) == (PT.ACK, bytes([PT.SET_MODE])) and p.mode == Mode.AUTONOMOUS
    (r,) = core.handle(gs(PT.SET_MODE, b"\x00"))
    assert p.mode == Mode.IDLE and ("stop_navigation",) in p.calls
    (r,) = core.handle(gs(PT.SET_MODE, b"\x09"))
    assert Nack.unpack(r.data).reason == NackReason.NOT_PERMITTED and p.mode == Mode.IDLE


def test_drive(core):
    p = core.ports
    p.mode = Mode.AUTONOMOUS
    (r,) = core.handle(gs(PT.DRIVE, bytes([50, 256 - 25, 0xE8, 0x03])))   # 50 %, -25 %, 1000 ms
    assert (r.type, r.data) == (PT.ACK, bytes([PT.DRIVE]))
    assert p.mode == Mode.TELEOP and ("stop_navigation",) in p.calls
    name, v, w, d = [c for c in p.calls if c[0] == "drive"][0]
    assert (v, w, d) == pytest.approx((0.2, -0.2, 1.0))


def test_routing_to_mcus(core):
    p = core.ports
    req = gs(PT.GET_TLM, dest=Addr.MOBILITY)
    assert core.handle(req) == []
    assert p.routed == [(Addr.MOBILITY, req.encode())]
    p.route_ok = False
    (r,) = core.handle(gs(PT.PING, dest=Addr.ARM))
    assert Nack.unpack(r.data).reason == NackReason.HW_FAULT and r.src == Addr.ROVER


def test_foreign_and_ack_frames_ignored(core):
    assert core.handle(gs(PT.PING, dest=Addr.GROUND)) == []
    assert core.handle(gs(PT.ACK)) == [] and core.handle(gs(PT.NACK, b"\x00\x02")) == []
    assert core.stats["ignored"] == 3


def test_broadcast_ping(core):
    (r,) = core.handle(gs(PT.PING, dest=Addr.BROADCAST))
    assert r.src == Addr.ROVER and r.dest == Addr.GROUND


def test_get_image_legacy(core):
    rs = core.handle(gs(PT.GET_IMAGE))
    assert len(rs) == 50 and all(r.type == PT.GET_IMAGE and len(r.data) == 200 for r in rs)
    assert b"".join(r.data for r in rs) == core.ports.img.tobytes()


def test_get_image_v2(core):
    rs = core.handle(gs(PT.GET_IMAGE, b"\x4b"))                 # quality 75
    assert all(r.type == PT.IMAGE_CHUNK for r in rs) and ("image_v2", 75) in core.ports.calls
    hdr, payload = reassemble_v2_chunks([r.data for r in rs])
    assert payload == bytes(range(250)) * 2 and (hdr.width, hdr.height, hdr.total) == (20, 25, 3)
    (r,) = core.handle(gs(PT.GET_IMAGE, b"\x00\x00"))
    assert Nack.unpack(r.data).reason == NackReason.BAD_LENGTH


def test_cap_image_echo(core):
    (r,) = core.handle(gs(PT.CAP_IMAGE))
    assert (r.type, r.data) == (PT.CAP_IMAGE, b"") and ("capture",) in core.ports.calls


def test_end_to_end_byte_stream(core):
    """GS bytes -> radio decoder -> core -> response bytes -> GS decoder."""
    radio = StreamDecoder(timeout_s=None)
    stream = (bytes.fromhex("C0010B000045 86C0".replace(" ", ""))   # ICD PING vector
              + bytes.fromhex("C0010B0F007B96C0")                   # ICD GET_TLM vector
              + b"\x55\xAA" + gs(PT.START_SCAN).encode())          # noise + START_SCAN
    replies = b"".join(r.encode() for fr in radio.feed(stream) for r in core.handle(fr))
    got = StreamDecoder(own_addr=Addr.GROUND, timeout_s=None).feed(replies)
    assert [f.type for f in got] == [PT.ACK, PT.GET_TLM, PT.START_SCAN]
