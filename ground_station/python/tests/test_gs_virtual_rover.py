"""gs_cli.GroundStation against virtual_rover over a local TCP socket (threads, in-process)."""
from __future__ import annotations

import time

import pytest
from PIL import Image

import gs_cli
import virtual_rover as vr
from gs_common import LEGACY_CHUNK, LEGACY_CHUNK_WIRE
from rover_core.protocol import (NO_ECHO, Addr, ArmSetJoints, Fault, Mode, NackReason, PT, TlmLegacy, TlmV2)
from rover_core.ssp import Frame


def tlm(gs) -> TlmV2:
    r = gs.get_tlm()
    assert r.ok, r
    return r.value


# ---------------------------------------------------------------- legacy set
def test_ping_ack_len0(rover_gs):
    _, _, gs = rover_gs
    r = gs.ping()
    assert r.ok and r.frame.type == PT.ACK and len(r.frame.data) == 0
    assert (r.frame.dest, r.frame.src) == (Addr.GROUND, Addr.ROVER)      # addresses swapped (ICD §4)
    assert 0 < r.rtt < 1.0


def test_get_tlm_v2_plausible(rover_gs):
    _, _, gs = rover_gs
    t = tlm(gs)
    assert isinstance(t, TlmV2) and t.tlm_version == 2 and t.mode == Mode.IDLE and t.fault_flags == 0
    assert 10_800 < t.battery_mV < 12_700
    assert 1_500 < t.temperature_cC < 3_500 and 2_000 < t.humidity_cP < 6_000
    assert 95_000 < t.pressure_Pa < 105_000
    assert abs(t.az - 9810) < 500 and abs(t.roll_cdeg) < 1500 and abs(t.pitch_cdeg) < 1500
    assert 0 <= t.mq2_raw <= 1023 and 0 <= t.soil_raw <= 1023 and 0 <= t.rain_raw <= 1023
    assert t.range_min_mm == NO_ECHO or 20 <= t.range_min_mm <= 4000
    assert gs.last_tlm is t


def test_tlm_values_vary_slowly(rover_gs):
    _, _, gs = rover_gs
    a = tlm(gs)
    time.sleep(0.3)
    b = tlm(gs)
    assert abs(a.temperature_cC - b.temperature_cC) < 100        # < 1 °C in 0.3 s
    assert abs(a.battery_mV - b.battery_mV) < 100


def test_cap_image_echo(rover_gs):
    model, _, gs = rover_gs
    r = gs.cap_image()
    assert r.ok and r.frame.type == PT.CAP_IMAGE and not r.frame.data
    assert model.scene is not None and model.captures == 1


def test_legacy_image_download_is_exact(rover_gs, tmp_path):
    model, _, gs = rover_gs
    assert gs.cap_image().ok
    res = gs.get_image()
    assert res.legacy and res.complete and (res.width, res.height, res.total) == (100, 100, 50)
    assert res.data == model.scene.render(100, 100)
    png = res.save(tmp_path / "legacy.png")
    img = Image.open(png)
    assert img.size == (100, 100) and img.tobytes() == res.data


def test_get_image_without_capture_captures_implicitly(rover_gs):
    model, _, gs = rover_gs
    res = gs.get_image()
    assert res.complete and model.captures == 1


def test_v2_gray_image(rover_gs):
    model, _, gs = rover_gs
    res = gs.get_image(v2=True, quality=0)
    assert not res.legacy and res.fmt == 0 and res.complete and res.total == 50
    assert res.data == model.scene.render(100, 100)


def test_v2_jpeg_image(rover_gs, tmp_path):
    _, _, gs = rover_gs
    res = gs.get_image(v2=True, quality=60)
    assert res.fmt == 1 and res.complete and res.data[:2] == b"\xff\xd8"
    img = Image.open(res.save(tmp_path / "v2.png"))
    assert img.size == (vr.JPEG_W, vr.JPEG_H)
    raw = res.save(tmp_path / "v2.jpg")
    assert raw.read_bytes() == res.data


def test_start_and_stop_scan(rover_gs):
    model, _, gs = rover_gs
    r = gs.start_scan()
    assert r.ok and r.frame.type == PT.START_SCAN
    assert tlm(gs).mode == Mode.AUTONOMOUS
    time.sleep(1.0)
    t = tlm(gs)
    assert (t.x_mm, t.y_mm) != (0, 0)                                  # exploring
    d = gs.drive(50, 0, 500)
    assert d.error == "nack" and d.nack.reason == NackReason.BUSY     # no teleop during autonomy
    assert gs.stop_scan().ok
    assert tlm(gs).mode == Mode.IDLE


def test_collect_object(rover_gs):
    _, _, gs = rover_gs
    r = gs.collect_object()
    assert r.ok and r.frame.type == PT.COLLECT_OBJECT
    again = gs.collect_object()
    assert again.error == "nack" and again.nack.reason == NackReason.BUSY
    assert gs.drive(30, 0, 300).nack.reason == NackReason.BUSY
    time.sleep(0.7)                                                    # 6 s sequence x time_scale 0.1
    assert gs.collect_object().ok


# ------------------------------------------------------------------ v2 set
def test_estop_latch_and_clear(rover_gs):
    _, _, gs = rover_gs
    assert gs.drive(60, 0, 2000).ok
    r = gs.estop()
    assert r.ok and r.frame.type == PT.ACK and r.frame.data == bytes([PT.ESTOP])
    t = tlm(gs)
    assert t.fault_flags & Fault.ESTOP and t.mode == Mode.IDLE
    x0 = t.x_mm
    time.sleep(0.3)
    assert tlm(gs).x_mm == x0                                          # motors stopped
    for rej in (gs.drive(50, 0, 500), gs.start_scan(), gs.set_mode(Mode.TELEOP), gs.collect_object()):
        assert rej.error == "nack" and rej.nack.reason == NackReason.NOT_PERMITTED
    c = gs.clear_estop()
    assert c.ok and c.frame.data == bytes([PT.CLEAR_ESTOP])
    assert not tlm(gs).fault_flags & Fault.ESTOP
    assert gs.drive(50, 0, 500).ok


@pytest.mark.parametrize("mode", list(Mode))
def test_set_mode(rover_gs, mode):
    _, _, gs = rover_gs
    r = gs.set_mode(mode)
    assert r.ok and r.frame.data == bytes([PT.SET_MODE])
    assert tlm(gs).mode == mode


def test_set_mode_invalid(rover_gs):
    _, _, gs = rover_gs
    r = gs.set_mode(9)
    assert r.error == "nack" and r.nack.rejected_type == PT.SET_MODE and r.nack.reason == NackReason.NOT_PERMITTED


def test_drive_moves_and_turns(rover_gs):
    _, _, gs = rover_gs
    r = gs.drive(80, 0, 1000)
    assert r.ok and r.frame.data == bytes([PT.DRIVE])
    time.sleep(1.2)
    t = tlm(gs)
    assert t.mode == Mode.TELEOP and t.x_mm > 150 and abs(t.heading_cdeg) < 500
    assert gs.drive(0, 60, 800).ok
    time.sleep(1.0)
    assert tlm(gs).heading_cdeg > 1500                                # turned left (+w)


def test_drive_watchdog_sets_cmd_timeout(rover_gs):
    _, _, gs = rover_gs
    assert gs.drive(40, 0, 150).ok
    time.sleep(0.5)
    assert tlm(gs).fault_flags & Fault.CMD_TIMEOUT
    assert gs.drive(0, 0, 0).ok
    assert not tlm(gs).fault_flags & Fault.CMD_TIMEOUT


def test_unknown_type_and_bad_length(rover_gs):
    _, _, gs = rover_gs
    r = gs.request(0x77)
    assert r.error == "nack" and r.nack.reason == NackReason.UNKNOWN_TYPE and r.nack.rejected_type == 0x77
    r = gs.request(PT.PING, b"\x01")
    assert r.error == "nack" and r.nack.reason == NackReason.BAD_LENGTH
    r = gs.request(PT.SET_MODE, b"\x01\x02")
    assert r.error == "nack" and r.nack.reason == NackReason.BAD_LENGTH


def test_frames_routed_to_mcus(rover_gs):
    _, _, gs = rover_gs
    for addr in (Addr.MOBILITY, Addr.PAYLOAD, Addr.ARM):
        r = gs.ping(dest=addr)
        assert r.ok and r.frame.src == addr
    r = gs.arm_pose(1)
    assert r.ok and r.frame.src == Addr.ARM and r.frame.data == bytes([PT.ARM_SET_POSE])
    bad = ArmSetJoints([90, 5, 90, 90, 90, 10], 500).pack()               # shoulder below 15 deg
    r = gs.request(PT.ARM_SET_JOINTS, bad, dest=Addr.ARM)
    assert r.error == "nack" and r.nack.reason == NackReason.NOT_PERMITTED
    r = gs.request(PT.ESTOP, dest=Addr.MOBILITY)
    assert r.ok and r.frame.src == Addr.MOBILITY
    assert tlm(gs).fault_flags & Fault.ESTOP


def test_broadcast_ping(rover_gs):
    _, _, gs = rover_gs
    r = gs.ping(dest=Addr.BROADCAST)
    assert r.ok and r.frame.src == Addr.ROVER


def test_frames_for_other_nodes_are_ignored(rover_gs):
    _, _, gs = rover_gs
    r = gs.ping(dest=0x22, timeout=0.3)
    assert r.error == "timeout"


# ------------------------------------------------------------ rover variants
def test_legacy_rover(make_rover, connect):
    model, srv = make_rover(legacy=True)
    gs = connect(srv, timeout=0.4)
    t = tlm(gs)
    assert isinstance(t, TlmLegacy) and len(t.pack()) == 7
    assert 15 <= t.temp_c <= 35 and 20 <= t.rh_pct <= 60
    assert gs.start_scan().ok and gs.stop_scan().ok and gs.collect_object().ok and gs.cap_image().ok
    assert gs.get_image().complete
    for r in (gs.estop(retries=0), gs.set_mode(1), gs.drive(10, 0, 100)):
        assert r.error == "timeout"                     # the 2023 firmware does not know v2 commands
    res = gs.get_image(v2=True, quality=50)              # legacy rover answers with legacy chunks
    assert res.legacy and res.complete


def test_safe_link_rover(make_rover, connect):
    _, srv = make_rover(safe_link=True)
    gs = connect(srv)
    r = gs.ping()
    assert r.ok and r.frame.src == Addr.MOBILITY
    t = tlm(gs)
    assert t.mode == Mode.SAFE and t.fault_flags & Fault.SAFE_MODE
    assert t.temperature_cC == 0 and t.pressure_Pa == 0 and t.battery_mV > 10_000
    r = gs.cap_image()
    assert r.error == "nack" and r.nack.reason == NackReason.NOT_PERMITTED
    assert gs.estop().ok and tlm(gs).fault_flags & Fault.ESTOP
    assert gs.clear_estop().ok and gs.stop_scan().ok


def test_transparency_option(make_rover, connect):
    _, srv = make_rover(transparency=True)
    gs = connect(srv, transparency=True)
    assert gs.ping().ok
    assert gs.get_image(v2=True, quality=0).complete    # pixel data full of 0xC0/0xDB is escaped


# ------------------------------------------------------------ link features
def test_ping_test_statistics(rover_gs):
    _, _, gs = rover_gs
    st = gs.ping_test(25)
    assert st["sent"] == 25 and st["received"] == 25 and st["lost"] == 0 and st["loss_pct"] == 0
    assert 0 <= st["min_ms"] <= st["p50_ms"] <= st["p95_ms"] <= st["max_ms"] < 1000
    assert st["stdev_ms"] >= 0


def test_poll_tlm_to_csv(rover_gs, tmp_path):
    _, _, gs = rover_gs
    path = tmp_path / "tlm.csv"
    rows = gs.poll_tlm(0.05, count=4, csv_path=path)
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    assert len(rows) == 4 and len(lines) == 5
    assert lines[0].split(",") == gs_cli.CSV_COLUMNS
    first = dict(zip(lines[0].split(","), lines[1].split(",")))
    assert first["tlm_version"] == "2" and 10.8 < float(first["battery_V"]) < 12.7
    assert first["ir"] == "" and first["time_iso"].endswith("Z")


def test_uplink_bit_errors_are_survived_with_retries(make_rover, connect):
    _, srv = make_rover(ber_up=4e-3, seed=3)            # ~23 % of 8-byte requests corrupted
    gs = connect(srv, timeout=0.25, retries=6)
    replies = [gs.ping() for _ in range(30)]
    assert all(r.ok for r in replies)
    assert gs.stats["retries"] > 0
    assert sum(link.decoder.stats.crc_errors + link.decoder.stats.framing_errors for link in srv.links) > 0


def test_downlink_bit_errors_counted_and_survived(make_rover, connect):
    _, srv = make_rover(ber_down=5e-4, seed=5)           # ~25 % of TLM frames corrupted
    gs = connect(srv, timeout=0.25, retries=6)
    replies = [gs.get_tlm() for _ in range(30)]
    assert all(r.ok and isinstance(r.value, TlmV2) for r in replies)
    st = gs.link_stats()
    assert st["crc_errors"] + st["framing_errors"] > 0 and st["retries"] > 0


@pytest.mark.parametrize("seed", [11, 12, 13])
def test_legacy_image_with_bit_errors_keeps_chunks_in_place(make_rover, connect, seed):
    model, srv = make_rover(ber_down=1e-4, seed=seed)    # ~8 bit errors per 10.4 kB image
    gs = connect(srv)
    res = gs.get_image()
    truth = model.scene.render(100, 100)
    assert res.missing, "expected some corrupted chunks"
    assert len(res.received) + len(res.missing) == 50
    for i in res.received:                               # every received chunk is in its right slot
        assert res.data[i * 200:(i + 1) * 200] == truth[i * 200:(i + 1) * 200], f"chunk {i} misplaced"
    for i in res.missing:
        assert res.data[i * 200:(i + 1) * 200] == bytes(200)


def test_v2_image_with_bit_errors_reports_exact_missing_chunks(make_rover, connect):
    model, srv = make_rover(ber_down=1e-4, seed=21)
    gs = connect(srv)
    res = gs.get_image(v2=True, quality=0)
    truth = model.scene.render(100, 100)
    assert res.missing and not res.legacy
    for i in res.received:
        assert res.data[i * 200:(i + 1) * 200] == truth[i * 200:(i + 1) * 200]


def test_image_assembler_infers_lost_legacy_chunks_from_byte_count():
    chunks = [bytes([i]) * LEGACY_CHUNK for i in range(50)]
    asm = gs_cli.ImageAssembler(bytes_at_request=1000)
    pos = 1000
    for i, c in enumerate(chunks):
        pos += LEGACY_CHUNK_WIRE                         # every chunk occupies 208 bytes on the wire
        if i in (0, 7, 8, 49):                           # ...but these failed their CRC
            continue
        assert asm.add(Frame(Addr.GROUND, Addr.ROVER, PT.GET_IMAGE, c), pos)
    res = asm.result(1.0)
    assert res.missing == [0, 7, 8, 49]
    assert all(res.data[i * 200] == i for i in res.received)


def test_rover_length_check_option_drops_bad_lengths_silently(make_rover, connect):
    _, srv = make_rover(length_check=True)
    gs = connect(srv, timeout=0.3)
    assert gs.request(PT.PING, b"").error == "timeout"         # rejected at the header (ICD §3-5a)
    assert gs.ping().ok
    assert sum(link.decoder.stats.framing_errors for link in srv.links) == 1


def test_bit_error_injector_rate():
    import random
    inj = vr.BitErrorInjector(1e-3, random.Random(1))
    data = bytes(100_000)
    out = inj.apply(data)
    flips = sum(bin(b).count("1") for b in out)
    assert flips == inj.flipped and 600 < flips < 1000          # 800 expected
    assert vr.BitErrorInjector(0.0, random.Random(1)).apply(data) == data
