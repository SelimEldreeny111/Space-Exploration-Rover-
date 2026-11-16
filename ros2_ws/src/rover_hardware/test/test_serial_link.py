import queue
import time

from rover_core.protocol import PT, Addr
from rover_core.ssp import Frame
from rover_hardware.serial_link import SerialLink, parse_frames


class FakeSerial:
    """pyserial stand-in: read() returns queued chunks (or raises queued exceptions)."""

    def __init__(self, chunks=()):
        self.rx = queue.Queue()
        for c in chunks:
            self.rx.put(c)
        self.tx = bytearray()
        self.closed = False
        self.in_waiting = 0

    def read(self, n):
        try:
            item = self.rx.get(timeout=0.01)
        except queue.Empty:
            return b""
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, data):
        self.tx += data
        return len(data)

    def close(self):
        self.closed = True


def wait_for(pred, timeout=2.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.005)
    return False


def test_frames_are_decoded_across_chunks():
    f1 = Frame(Addr.ROVER, Addr.MOBILITY, PT.HEARTBEAT, bytes(6))
    f2 = Frame(Addr.ROVER, Addr.MOBILITY, PT.RANGES, bytes([6]) + b"\xff\xff" * 6)
    raw = f1.encode() + b"\x00\x13" + f2.encode()
    fake = FakeSerial([raw[:5], raw[5:17], raw[17:]])
    link = SerialLink("fake", 115200, timeout_s=None, open_fn=lambda: fake).start()
    try:
        got = []
        assert wait_for(lambda: got.extend(link.poll()) or len(got) == 2)
        assert got == [f1, f2]
        assert link.connected and link.stats()["frames_ok"] == 2
    finally:
        link.stop()
    assert fake.closed


def test_send_writes_encoded_frame():
    fake = FakeSerial()
    link = SerialLink("fake", 115200, open_fn=lambda: fake).start()
    try:
        assert wait_for(lambda: link.connected)
        fr = Frame(Addr.MOBILITY, Addr.ROVER, PT.ESTOP)
        assert link.send(fr) and link.send(b"\xc0\xc0")
        assert bytes(fake.tx) == fr.encode() + b"\xc0\xc0"
    finally:
        link.stop()
    assert not link.send(fr)                 # closed link -> False, no exception


def test_reconnect_after_open_failure_and_read_error():
    fakes = [FakeSerial([OSError("unplugged")]), FakeSerial()]
    attempts = {"n": 0}

    def opener():
        attempts["n"] += 1
        if attempts["n"] == 1:
            raise OSError("no such device")
        return fakes.pop(0)

    link = SerialLink("fake", 115200, open_fn=opener, reconnect_s=0.01).start()
    try:
        assert wait_for(lambda: link.opens == 2 and link.connected)
        assert "read" in link.last_error or "open" in link.last_error
    finally:
        link.stop()


def test_parse_frames():
    a = Frame(Addr.ARM, Addr.GROUND, PT.ARM_SET_POSE, b"\x02")
    b = Frame(Addr.ARM, Addr.GROUND, PT.PING)
    assert parse_frames(a.encode() + b.encode()) == [a, b]
    assert parse_frames(b"garbage") == []
