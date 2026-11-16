"""ROS-free command handler of the ground-station bridge (ICD v2.0 sections 5-7).

:class:`GroundStationCore` turns one CRC-valid SSP frame received over the
HC-12 link into the list of response frames, calling the side effects on a
*ports* object (implemented by the ROS node, faked in the unit tests):

* frames addressed to 0x02 / 0x03 / 0x04 are routed to the MCU bridges;
* frames addressed to 0x01 (or broadcast) are executed by the Brain;
* unknown packet types are answered with NACK (reason 2), wrong lengths with
  NACK (reason 1).
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional, Protocol, Tuple

import numpy as np

from rover_core.protocol import PT, Addr, Drive, Mode, Nack, NackReason, plausible_length
from rover_core.ssp import Frame

from .conversions import (TelemetrySnapshot, build_tlm_v2, drive_to_twist, legacy_image_chunks,
                          v2_image_chunks)

#: SSP address -> bridge name used in the internal /ssp/to_<name> topics
ROUTED: Dict[int, str] = {int(Addr.MOBILITY): "mobility", int(Addr.PAYLOAD): "payload",
                          int(Addr.ARM): "arm"}


class GroundStationPorts(Protocol):
    """Side effects required by :class:`GroundStationCore`."""

    def get_mode(self) -> int: ...
    def set_mode(self, mode: int) -> None: ...
    def estop_latched(self) -> bool: ...
    def estop(self, engage: bool) -> None: ...
    def drive(self, v: float, w: float, duration_s: float) -> None: ...
    def stop_navigation(self) -> None: ...
    def start_collect(self) -> bool: ...          # False -> busy
    def capture_image(self) -> bool: ...
    def image_gray(self) -> np.ndarray: ...       # 100 x 100 uint8
    def image_v2(self, quality: int) -> Tuple[bytes, int, int, int]: ...  # payload, w, h, fmt
    def telemetry(self) -> TelemetrySnapshot: ...
    def route(self, dest: int, raw: bytes) -> bool: ...


class GroundStationCore:
    def __init__(self, ports: GroundStationPorts, own_addr: int = Addr.ROVER,
                 drive_max_speed: float = 0.4, drive_max_yaw_rate: float = 0.8) -> None:
        self.ports = ports
        self.own_addr = int(own_addr)
        self.drive_max_speed = drive_max_speed
        self.drive_max_yaw_rate = drive_max_yaw_rate
        self.stats = {"handled": 0, "routed": 0, "ignored": 0, "nack": 0}
        self._handlers: Dict[int, Callable[[Frame], List[Frame]]] = {
            PT.PING: self._ping,
            PT.GET_TLM: self._get_tlm,
            PT.CAP_IMAGE: self._cap_image,
            PT.GET_IMAGE: self._get_image,
            PT.START_SCAN: self._start_scan,
            PT.STOP_SCAN: self._stop_scan,
            PT.COLLECT_OBJECT: self._collect,
            PT.ESTOP: self._estop,
            PT.CLEAR_ESTOP: self._clear_estop,
            PT.SET_MODE: self._set_mode,
            PT.DRIVE: self._drive,
        }

    # ------------------------------------------------------------ helpers
    def _reply(self, req: Frame, ptype: int, data: bytes = b"") -> Frame:
        return req.reply(ptype, data, own_addr=self.own_addr)

    def _ack(self, req: Frame, with_type: bool = True) -> Frame:
        return self._reply(req, PT.ACK, bytes((req.type,)) if with_type else b"")

    def _nack(self, req: Frame, reason: int) -> Frame:
        self.stats["nack"] += 1
        return self._reply(req, PT.NACK, Nack(req.type, int(reason)).pack())

    # ------------------------------------------------------------ entry point
    def handle(self, fr: Frame) -> List[Frame]:
        if fr.dest in ROUTED:
            if self.ports.route(fr.dest, fr.encode()):
                self.stats["routed"] += 1
                return []
            return [self._nack(fr, NackReason.HW_FAULT)]
        if fr.dest not in (self.own_addr, Addr.BROADCAST):
            self.stats["ignored"] += 1
            return []
        if fr.type in (PT.ACK, PT.NACK):          # never answer an (N)ACK
            self.stats["ignored"] += 1
            return []
        handler = self._handlers.get(fr.type)
        if handler is None:
            return [self._nack(fr, NackReason.UNKNOWN_TYPE)]
        if not plausible_length(fr.type, len(fr.data), strict=True):
            return [self._nack(fr, NackReason.BAD_LENGTH)]
        self.stats["handled"] += 1
        return handler(fr)

    # ------------------------------------------------------------ handlers
    def _ping(self, fr: Frame) -> List[Frame]:
        return [self._ack(fr, with_type=False)]          # legacy: ACK, LEN 0

    def _get_tlm(self, fr: Frame) -> List[Frame]:
        if fr.data:
            return [self._nack(fr, NackReason.BAD_LENGTH)]
        return [self._reply(fr, PT.GET_TLM, build_tlm_v2(self.ports.telemetry()).pack())]

    def _cap_image(self, fr: Frame) -> List[Frame]:
        self.ports.capture_image()                        # placeholder if no camera
        return [self._reply(fr, PT.CAP_IMAGE)]

    def _get_image(self, fr: Frame) -> List[Frame]:
        if len(fr.data) == 0:                             # legacy 100x100 gray, 50 x 200 B
            return [self._reply(fr, PT.GET_IMAGE, c)
                    for c in legacy_image_chunks(self.ports.image_gray())]
        if len(fr.data) == 1:                             # v2 IMAGE_CHUNK, DATA = quality
            payload, w, h, fmt = self.ports.image_v2(fr.data[0])
            return [self._reply(fr, PT.IMAGE_CHUNK, c) for c in v2_image_chunks(payload, w, h, fmt)]
        return [self._nack(fr, NackReason.BAD_LENGTH)]

    def _start_scan(self, fr: Frame) -> List[Frame]:
        if self.ports.estop_latched():
            return [self._nack(fr, NackReason.NOT_PERMITTED)]
        self.ports.set_mode(Mode.AUTONOMOUS)
        return [self._reply(fr, PT.START_SCAN)]

    def _stop_scan(self, fr: Frame) -> List[Frame]:
        self.ports.stop_navigation()
        self.ports.set_mode(Mode.IDLE)
        return [self._reply(fr, PT.STOP_SCAN)]

    def _collect(self, fr: Frame) -> List[Frame]:
        if self.ports.estop_latched():                    # never start the arm while e-stopped
            return [self._nack(fr, NackReason.NOT_PERMITTED)]
        if not self.ports.start_collect():
            return [self._nack(fr, NackReason.BUSY)]
        return [self._reply(fr, PT.COLLECT_OBJECT)]

    def _estop(self, fr: Frame) -> List[Frame]:
        self.ports.estop(True)
        return [self._ack(fr)]

    def _clear_estop(self, fr: Frame) -> List[Frame]:
        self.ports.estop(False)
        return [self._ack(fr)]

    def _set_mode(self, fr: Frame) -> List[Frame]:
        mode = fr.data[0]
        if mode not in {int(m) for m in Mode}:
            return [self._nack(fr, NackReason.NOT_PERMITTED)]
        if mode in (Mode.AUTONOMOUS, Mode.TELEOP) and self.ports.estop_latched():
            return [self._nack(fr, NackReason.NOT_PERMITTED)]
        if self.ports.get_mode() == Mode.AUTONOMOUS and mode != Mode.AUTONOMOUS:
            self.ports.stop_navigation()
        self.ports.set_mode(mode)
        return [self._ack(fr)]

    def _drive(self, fr: Frame) -> List[Frame]:
        if self.ports.estop_latched():
            return [self._nack(fr, NackReason.NOT_PERMITTED)]
        v, w, duration = drive_to_twist(Drive.unpack(fr.data), self.drive_max_speed,
                                        self.drive_max_yaw_rate)
        if self.ports.get_mode() == Mode.AUTONOMOUS:
            self.ports.stop_navigation()
        self.ports.set_mode(Mode.TELEOP)
        self.ports.drive(v, w, duration)
        return [self._ack(fr)]
