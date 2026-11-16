import math
import os

import numpy as np
import pytest

from rover_sim.sim_core import (CollisionCounter, RateSchedule, body_twist_from_poses, clean_scan,
                                float32_array, laser_scan_fields, resolve_world_path,
                                seconds_to_stamp)

PKG = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
WORLDS = os.path.join(PKG, "worlds")
NAV_MAPS = os.path.normpath(os.path.join(PKG, "..", "rover_navigation", "maps"))


# ------------------------------------------------------------------ pure helpers
def test_rate_schedule_counts_and_skips():
    s = RateSchedule(5.0)
    n = sum(s.due(i * 0.02) for i in range(1, 501))          # 10 s of 50 Hz ticks
    assert 49 <= n <= 51
    s2 = RateSchedule(10.0)
    assert s2.due(0.0) and not s2.due(0.05) and s2.due(0.1)
    assert s2.due(5.0) and not s2.due(5.01)                  # long stall: one event, no burst
    assert not RateSchedule(0.0).due(1.0)


def test_float32_array_keeps_inf():
    a = float32_array(np.array([1.5, np.inf, 0.25]))
    assert a.typecode == "f" and list(a) == [1.5, math.inf, 0.25]


def test_laser_scan_fields():
    inc = 2 * math.pi / 360
    f = laser_scan_fields(360, 0.0, inc, 5.0)
    assert f["angle_max"] == pytest.approx(2 * math.pi - inc)
    assert f["scan_time"] == pytest.approx(0.2) and f["time_increment"] == pytest.approx(0.2 / 360)


def test_clean_scan():
    r = clean_scan([0.05, 1.0, float("nan"), 3.6, 3.5, np.inf], 0.12, 3.5)
    assert np.isinf(r[0]) and r[1] == 1.0 and np.isinf(r[2]) and np.isinf(r[3]) and r[4] == 3.5


def test_body_twist_from_poses():
    vx, vy, w = body_twist_from_poses((1.0, 1.0, math.pi / 2), (1.0, 1.5, math.pi / 2), 1.0)
    assert (vx, vy, w) == pytest.approx((0.5, 0.0, 0.0), abs=1e-9)
    vx, vy, w = body_twist_from_poses((0, 0, 3.1), (0, 0, -3.1), 0.5)       # across +-pi
    assert vx == pytest.approx(0.0) and w == pytest.approx((2 * math.pi - 6.2) / 0.5)
    assert body_twist_from_poses((0, 0, 0), (1, 0, 0), 0.0) == (0.0, 0.0, 0.0)


def test_collision_counter_counts_events_not_steps():
    c = CollisionCounter(gap_s=1.0)
    assert not c.update(0.02, 0) and c.events == 0
    assert c.update(0.04, 1) and c.events == 1               # first contact
    steps = 1
    for k in range(3, 100):                                   # pushing: intermittent hits
        steps += k % 2
        c.update(0.02 * k, steps)
    assert c.events == 1
    assert not c.update(5.0, steps)                           # no new colliding step
    assert c.update(10.0, steps + 1) and c.events == 2        # new contact after a free gap


def test_collision_events_with_roversim():
    """RoverSim.collided latches after the first contact, so rover_sim_node
    counts events from the per-step counter RoverSim.collisions."""
    from rover_core.sim import Box, RoverSim, World
    w = World(size=(8.0, 6.0), resolution=0.02, start=(2.0, 3.0, 0.0))
    w.add_outer_walls()
    w.add_box(Box(3.4, 3.0, 0.3, 2.0, 0, 1.0, "wall"))
    rover = RoverSim(w, noise=False)
    counter = CollisionCounter()

    def drive(v, seconds):
        rover.set_cmd(v, 0.0)
        for _ in range(int(round(seconds / 0.02))):
            rover.step(0.02)
            counter.update(rover.t, rover.collisions)

    drive(0.3, 8.0)                       # into the wall, then keep pushing
    assert rover.collided and rover.collisions > 10 and counter.events == 1
    drive(-0.3, 2.0)                      # back off: RoverSim.collided stays latched
    assert rover.collided and counter.events == 1
    drive(0.3, 6.0)                       # second contact
    assert counter.events == 2


def test_seconds_to_stamp():
    assert seconds_to_stamp(1.5) == (1, 500_000_000)
    assert seconds_to_stamp(1.9999999999) == (2, 0)
    assert seconds_to_stamp(0.02) == (0, 20_000_000)


def test_resolve_world_path():
    assert resolve_world_path("", WORLDS) is None
    assert resolve_world_path("lab", WORLDS) == os.path.join(WORLDS, "lab.yaml")
    assert resolve_world_path("lab.yaml", WORLDS) == os.path.join(WORLDS, "lab.yaml")
    with pytest.raises(FileNotFoundError):
        resolve_world_path("mars_yard", WORLDS)


# ------------------------------------------------------------------ rover_core.sim contract
sim = pytest.importorskip("rover_core.sim", reason="rover_core.sim not available")


def _read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, idx = [], 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            idx = data.index(b"\n", idx) + 1
            continue
        j = idx
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[idx:j])
        idx = j
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[idx + 1:idx + 1 + w * h], np.uint8).reshape(h, w)[::-1, :]


def test_shipped_lab_world_has_low_and_tall_obstacles():
    from rover_core.params import DEFAULT
    world = sim.World.from_yaml(os.path.join(WORLDS, "lab.yaml"))
    h = world.height
    low = (h > 0) & (h < DEFAULT.lidar_height - 0.02)
    assert low.sum() > 1000 and (h >= 0.9).sum() > 10000
    rover = sim.RoverSim(world, params=DEFAULT, seed=0)
    rover.step(0.02)
    assert not rover.collided                                   # start pose is free


def test_nav_map_is_lidar_view_of_world():
    """The Nav2 map contains the benches but not the low crates (map = start frame)."""
    img = _read_pgm(os.path.join(NAV_MAPS, "lab.pgm"))          # row 0 = bottom
    res, ox, oy = 0.05, -1.3, -1.9

    def occ(xw, yw):        # world coordinates -> map pixel (map origin = start pose)
        col, row = int((xw - 1.3 - ox) / res), int((yw - 1.9 - oy) / res)
        return img[row, col] < 100

    assert occ(5.2, 3.6)          # island bench (0.9 m)
    assert occ(0.05, 3.0)         # outer wall
    assert not occ(3.0, 2.6)      # crate0 (0.20 m): invisible to the LiDAR
    assert not occ(2.0, 1.9)      # free floor ahead of the start pose


def test_roversim_api_contract(tmp_path):
    from rover_core.kinematics import WheelOdometry
    from rover_core.params import DEFAULT
    world = sim.World.default_lab()
    rover = sim.RoverSim(world, params=DEFAULT, seed=1)
    assert len(rover.true_pose) == 3 and rover.t == 0.0
    odo = WheelOdometry(DEFAULT)
    odo.update(rover.wheel_ticks(), rover.t)
    rover.set_cmd(0.3, 0.0)
    for _ in range(100):                                        # 2 s
        rover.step(0.02)
        odo.update(rover.wheel_ticks(), rover.t)
    assert rover.t == pytest.approx(2.0)
    ticks = rover.wheel_ticks()
    assert len(ticks) == 6 and all(isinstance(t, int) for t in ticks)
    imu = rover.imu()
    assert {"ax", "ay", "az", "gx", "gy", "gz"} <= set(imu) and 9.0 < imu["az"] < 10.6
    ranges, amin, inc = rover.scan()
    assert len(ranges) == 360 and inc == pytest.approx(2 * math.pi / 360)
    us = rover.ultrasonic()
    assert len(us) == 6 and all(u > 0 for u in us)
    assert isinstance(rover.collided, bool)
    x0, y0, _ = world.start
    travelled = math.hypot(rover.true_pose[0] - x0, rover.true_pose[1] - y0)
    assert 0.3 < travelled < 0.7
    assert odo.pose.x == pytest.approx(travelled, rel=0.25)    # wheel odometry ~ ground truth


def test_low_crate_seen_by_ultrasonic_not_by_lidar(tmp_path):
    from rover_core.params import DEFAULT
    (tmp_path / "w.yaml").write_text(
        "name: crate_test\nsize: [6.0, 4.0]\nresolution: 0.02\nstart: [1.0, 2.0, 0.0]\n"
        "boxes:\n  - {name: crate, x: 2.5, y: 2.0, sx: 0.4, sy: 0.6, yaw: 0.0, height: 0.2}\n",
        encoding="utf-8")
    world = sim.World.from_yaml(str(tmp_path / "w.yaml"))
    rover = sim.RoverSim(world, params=DEFAULT, seed=2)
    rover.step(0.02)
    ranges, amin, inc = rover.scan()
    ahead = np.concatenate([ranges[-5:], ranges[:6]])           # +-5 deg around the heading
    assert np.all(~np.isfinite(ahead) | (ahead > 2.0))          # LiDAR looks over the crate
    front_center = rover.ultrasonic()[1]
    assert front_center < 1.0                                   # ~0.78 m to the crate face
