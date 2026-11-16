"""gs_cli.main() end to end against the in-process virtual rover."""
from __future__ import annotations

import csv
import io
import json

import gs_cli
from gs_common import CSV_COLUMNS


def run(srv, *args, capsys=None):
    rc = gs_cli.main(["--tcp", f"127.0.0.1:{srv.port}", "--timeout", "1.0", *args])
    out = capsys.readouterr().out if capsys else ""
    return rc, out


def test_ping_and_ping_test(make_rover, capsys):
    _, srv = make_rover()
    rc, out = run(srv, "ping", capsys=capsys)
    assert rc == 0 and "OK    ACK" in out and "rtt=" in out
    rc, out = run(srv, "--ping-test", "10", capsys=capsys)
    assert rc == 0 and "PING x10: 10 replies, 0 lost" in out and "RTT ms: min" in out
    rc, out = run(srv, "--json", "ping", "-n", "5", capsys=capsys)
    st = json.loads(out)
    assert rc == 0 and st["received"] == 5


def test_tlm_text_and_json(make_rover, capsys):
    _, srv = make_rover()
    rc, out = run(srv, "tlm", capsys=capsys)
    assert rc == 0 and "Telemetry v2 (56 B)" in out and "battery" in out
    rc, out = run(srv, "--json", "tlm", capsys=capsys)
    assert rc == 0 and json.loads(out)["tlm_version"] == 2


def test_tlm_poll_csv(make_rover, capsys, tmp_path):
    _, srv = make_rover()
    path = tmp_path / "poll.csv"
    rc, out = run(srv, "tlm", "--poll", "0.05", "--count", "3", "--csv", str(path), capsys=capsys)
    rows = list(csv.reader(path.open(encoding="utf-8")))
    assert rc == 0 and rows[0] == CSV_COLUMNS and len(rows) == 4 and "wrote" in out


def test_every_command(make_rover, capsys, tmp_path):
    model, srv = make_rover()
    expect = [
        (["cap"], 0, "OK    CAP_IMAGE"),
        (["scan-start"], 0, "OK    START_SCAN"),
        (["drive", "30", "0", "500"], 1, "NACK  DRIVE rejected: BUSY"),
        (["scan-stop"], 0, "OK    STOP_SCAN"),
        (["mode", "teleop"], 0, "ACK of SET_MODE"),
        (["drive", "30", "-10", "500"], 0, "ACK of DRIVE"),
        (["estop"], 0, "ACK of ESTOP"),
        (["collect"], 1, "NACK  COLLECT_OBJECT rejected: NOT_PERMITTED"),
        (["clear-estop"], 0, "ACK of CLEAR_ESTOP"),
        (["collect"], 0, "OK    COLLECT_OBJECT"),
        (["arm-pose", "stow"], 0, "ACK of ARM_SET_POSE"),
        (["raw", "GET_TLM"], 0, "TLM v2"),
        (["raw", "0x53", "02"], 0, "ACK of SET_MODE"),
        (["raw", "0x77"], 1, "UNKNOWN_TYPE"),
        (["stats"], 0, "frames_ok"),
    ]
    for args, rc_exp, text in expect:
        rc, out = run(srv, *args, capsys=capsys)
        assert rc == rc_exp and text in out, (args, rc, out)


def test_image_commands(make_rover, capsys, tmp_path):
    _, srv = make_rover()
    p1, p2 = tmp_path / "a.png", tmp_path / "b.jpg"
    rc, out = run(srv, "image", "-o", str(p1), "--capture", capsys=capsys)
    assert rc == 0 and "legacy GET_IMAGE 100x100: 50/50 chunks" in out and p1.exists()
    rc, out = run(srv, "image", "--v2", "--quality", "70", "-o", str(p2), capsys=capsys)
    assert rc == 0 and "IMAGE_CHUNK jpeg" in out and p2.read_bytes()[:2] == b"\xff\xd8"


def test_verbose_trace_shows_hex(make_rover, capsys):
    _, srv = make_rover()
    rc = gs_cli.main(["--tcp", f"127.0.0.1:{srv.port}", "-v", "ping"])
    err = capsys.readouterr().err
    assert rc == 0 and "TX C0 01 0B 00 00 45 86 C0" in err and "RX C0 0B 01 01 00 1E 1A C0" in err


def test_scripted_shell(make_rover, capsys, monkeypatch):
    _, srv = make_rover()
    script = "ping; tlm\n# comment\nestop\nmode autonomous\nclear-estop; mode auto\nbogus\nquit\nping\n"
    monkeypatch.setattr("sys.stdin", io.StringIO(script))
    rc, out = run(srv, "shell", capsys=capsys)
    assert "gs> ping" in out and "Telemetry v2" in out and "ACK of ESTOP" in out
    assert "NACK  SET_MODE rejected: NOT_PERMITTED" in out and "ACK of CLEAR_ESTOP" in out
    assert "error:" in out and out.count("gs> ping") == 1          # stopped at 'quit'
    assert rc == 2


def test_no_link_is_a_usage_error(capsys):
    try:
        gs_cli.main(["ping"])
    except SystemExit as exc:
        assert exc.code == 2
    else:  # pragma: no cover
        raise AssertionError("expected usage error")


def test_connection_refused(capsys):
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    assert gs_cli.main(["--tcp", f"127.0.0.1:{port}", "ping"]) == 3
