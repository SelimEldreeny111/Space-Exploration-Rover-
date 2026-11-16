"""CRC-16/XMODEM (poly 0x1021, init 0x0000, no reflection, xorout 0x0000).

This is the CRC used by the MISSION X Simple Serial Protocol (SSP), identical
to the "CRC16 CCITT XMODEM" block of the 2023 LabVIEW ground station and to the
table used in the original Arduino firmware (crc16.cpp).
"""
from __future__ import annotations

__all__ = ["crc16_xmodem", "CRC16_TABLE"]


def _make_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
        table.append(crc)
    return table


CRC16_TABLE: list[int] = _make_table()


def crc16_xmodem(data: bytes | bytearray | memoryview, crc: int = 0x0000) -> int:
    """Return the CRC-16/XMODEM of *data* (optionally continuing from *crc*)."""
    tbl = CRC16_TABLE
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ tbl[((crc >> 8) ^ b) & 0xFF]
    return crc
