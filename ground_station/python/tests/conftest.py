"""Fixtures: an in-process virtual rover on an ephemeral TCP port + GS clients."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

PY_DIR = Path(__file__).resolve().parents[1]
if str(PY_DIR) not in sys.path:
    sys.path.insert(0, str(PY_DIR))

import gs_cli  # noqa: E402
import virtual_rover as vr  # noqa: E402


@pytest.fixture
def make_rover():
    """make_rover(**model_kw, ber_up=, ber_down=, transparency=) -> (model, server)."""
    servers = []

    def factory(*, legacy=False, safe_link=False, time_scale=0.1, seed=1, **link_kw):
        model = vr.RoverModel(seed=seed, legacy=legacy, safe_link=safe_link, time_scale=time_scale)
        srv = vr.TcpRoverServer(model, "127.0.0.1", 0, **link_kw).start()
        servers.append(srv)
        return model, srv

    yield factory
    for s in servers:
        s.stop()


@pytest.fixture
def connect():
    """connect(server, **gs_kw) -> GroundStation over TCP."""
    clients = []

    def factory(srv, **kw):
        kw.setdefault("timeout", 1.0)
        gs = gs_cli.GroundStation(gs_cli.TcpLink("127.0.0.1", srv.port), **kw)
        clients.append(gs)
        return gs

    yield factory
    for c in clients:
        c.close()


@pytest.fixture
def rover_gs(make_rover, connect):
    model, srv = make_rover()
    return model, srv, connect(srv)
