"""Thread-based SSP serial link shared by all MCU / radio bridges (ROS-free).

A background thread reads the port (pyserial), feeds a length-driven
:class:`rover_core.ssp.StreamDecoder` and queues CRC-valid frames. The ROS
nodes drain the queue with :meth:`SerialLink.poll` from a timer, so every
rclpy call stays in the executor thread. The port is re-opened automatically
after a USB disconnect.
"""
from __future__ import annotations

import queue
import threading
import time
from typing import Callable, List, Optional, Union

from rover_core.protocol import plausible_length
from rover_core.ssp import Frame, StreamDecoder

try:  # pyserial (python3-serial); optional so that the module imports anywhere
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None


class SerialLink:
    """SSP over a serial port.

    Parameters
    ----------
    port, baud:
        Device and baud rate (115200 for the MCUs, 9600 for the HC-12).
    timeout_s:
        Inter-byte timeout inside a frame (ICD section 3: 20 ms MCU links,
        50 ms radio link).
    own_addr:
        Address filter for the decoder (``None`` = accept every DEST; the
        bridges route by DEST themselves).
    open_fn:
        Factory returning a pyserial-like object (``read``, ``write``,
        ``close``, ``in_waiting``); injectable for tests.
    """

    def __init__(self, port: str, baud: int, *, timeout_s: Optional[float] = 0.02,
                 transparency: bool = False, own_addr: Optional[int] = None,
                 length_check: Optional[Callable[[int, int], bool]] = plausible_length,
                 reconnect_s: float = 2.0, open_fn: Optional[Callable[[], object]] = None,
                 max_queue: int = 2000) -> None:
        self.port = port
        self.baud = int(baud)
        self.transparency = transparency
        self.reconnect_s = reconnect_s
        self.decoder = StreamDecoder(own_addr=own_addr, transparency=transparency,
                                     timeout_s=timeout_s, length_check=length_check)
        self._open_fn = open_fn or self._default_open
        self._ser = None
        self._lock = threading.Lock()
        self._frames: "queue.Queue[Frame]" = queue.Queue(maxsize=max_queue)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.connected = False
        self.last_error = ""
        self.last_rx_time: Optional[float] = None
        self.tx_frames = 0
        self.tx_bytes = 0
        self.tx_errors = 0
        self.rx_dropped = 0
        self.opens = 0

    # ------------------------------------------------------------ lifecycle
    def _default_open(self):
        if serial is None:
            raise RuntimeError("pyserial is not installed (sudo apt install python3-serial)")
        return serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.5)

    def start(self) -> "SerialLink":
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name=f"ssp-{self.port}", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._close()

    def _close(self) -> None:
        with self._lock:
            ser, self._ser = self._ser, None
            self.connected = False
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001 - closing a dead port
                pass

    # ------------------------------------------------------------ reader thread
    def _run(self) -> None:
        while not self._stop.is_set():
            if self._ser is None:
                try:
                    ser = self._open_fn()
                except Exception as exc:  # noqa: BLE001 - any open failure -> retry
                    self.last_error = f"open {self.port}: {exc}"
                    self._stop.wait(self.reconnect_s)
                    continue
                with self._lock:
                    self._ser = ser
                    self.connected = True
                    self.opens += 1
                self.decoder.reset()
            ser = self._ser
            if ser is None:
                continue
            try:
                n = getattr(ser, "in_waiting", 0) or 1
                data = ser.read(n)
            except Exception as exc:  # noqa: BLE001 - USB unplugged etc.
                self.last_error = f"read {self.port}: {exc}"
                self._close()
                continue
            if data:
                self.last_rx_time = time.monotonic()
                for fr in self.decoder.feed(data):
                    try:
                        self._frames.put_nowait(fr)
                    except queue.Full:
                        self.rx_dropped += 1

    # ------------------------------------------------------------ API
    def poll(self, max_frames: int = 500) -> List[Frame]:
        """Return the frames received since the last call (non-blocking)."""
        out: List[Frame] = []
        while len(out) < max_frames:
            try:
                out.append(self._frames.get_nowait())
            except queue.Empty:
                break
        return out

    def send(self, frame: Union[Frame, bytes, bytearray]) -> bool:
        """Write a frame (encoded with this link's transparency) or raw bytes."""
        raw = frame.encode(self.transparency) if isinstance(frame, Frame) else bytes(frame)
        failed = False
        with self._lock:
            ser = self._ser
            if ser is None:
                return False
            try:
                ser.write(raw)
                self.tx_frames += 1
                self.tx_bytes += len(raw)
            except Exception as exc:  # noqa: BLE001
                self.last_error = f"write {self.port}: {exc}"
                self.tx_errors += 1
                failed = True
        if failed:
            self._close()
            return False
        return True

    def stats(self) -> dict:
        d = self.decoder.stats.as_dict()
        d.update(connected=self.connected, opens=self.opens, tx_frames=self.tx_frames,
                 tx_bytes=self.tx_bytes, tx_errors=self.tx_errors, rx_dropped=self.rx_dropped,
                 last_error=self.last_error)
        return d


def parse_frames(raw: bytes) -> List[Frame]:
    """Decode the SSP frames contained in *raw* (wire format, transparency OFF).
    Used for the raw frames carried on the internal ``/ssp/*`` topics."""
    return StreamDecoder(timeout_s=None).feed(raw)
