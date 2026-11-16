import math
import os

import numpy as np

from rover_core.params import DEFAULT
from rover_core.kinematics import WheelOdometry
from rover_core.sim import Box, RoverSim, World


def empty_world():
    w = World(size=(8.0, 6.0), resolution=0.02, start=(2.0, 3.0, 0.0))
    w.add_outer_walls()
    return w


def test_straight_drive_matches_odometry():
    sim = RoverSim(empty_world(), noise=False, chi_true=DEFAULT.skid_chi)
    odo = WheelOdometry()
    odo.update(sim.wheel_ticks(), sim.t)
    sim.set_cmd(0.3, 0.0)
    for _ in range(500):
        sim.step(0.01)
        odo.update(sim.wheel_ticks(), sim.t)
    assert sim.x > 2.0 + 1.2
    assert abs(odo.pose.x - (sim.x - 2.0)) < 0.02
    assert not sim.collided


def test_turn_rate_uses_true_chi():
    sim = RoverSim(empty_world(), noise=False, chi_true=DEFAULT.skid_chi * 1.2)
    sim.set_cmd(0.0, 0.5)
    for _ in range(200):
        sim.step(0.01)
    # rover turns slower than commanded because true chi > nominal chi
    assert 0 < sim.th < 0.5 * 2.0


def test_lidar_ignores_low_obstacles_ultrasonic_sees_them():
    w = empty_world()
    w.add_box(Box(4.0, 3.0, 0.3, 0.6, 0, 0.20, "crate"))   # low crate 1.5 m ahead of LiDAR
    sim = RoverSim(w, noise=False)
    r, amin, inc = sim.scan()
    assert r[0] > 4.0 or math.isinf(r[0]) or r[0] > 3.4    # LiDAR sees the far wall / nothing
    us = sim.ultrasonic()
    assert 1.0 < us[1] < 1.6                                 # front-centre ultrasonic sees the crate


def test_lidar_sees_tall_obstacles():
    w = empty_world()
    w.add_box(Box(4.0, 3.0, 0.3, 0.6, 0, 1.0, "bench"))
    sim = RoverSim(w, noise=False)
    r, _, _ = sim.scan()
    assert abs(r[0] - (4.0 - 0.15 - 2.05)) < 0.05


def test_collision_with_low_obstacle():
    w = empty_world()
    w.add_box(Box(3.2, 3.0, 0.3, 0.6, 0, 0.20, "crate"))
    sim = RoverSim(w, noise=False)
    sim.set_cmd(0.3, 0.0)
    for _ in range(300):
        sim.step(0.01)
    assert sim.collided


def test_default_lab_and_pgm_roundtrip(tmp_path):
    lab = World.default_lab()
    assert lab.height.max() >= 1.8
    y = lab.save_pgm(str(tmp_path / "lab_gt"), min_height=0.6)
    assert os.path.exists(y) and os.path.exists(str(tmp_path / "lab_gt.pgm"))


def test_from_yaml(tmp_path):
    p = tmp_path / "w.yaml"
    p.write_text("name: t\nresolution: 0.05\nsize: [4, 3]\nstart: [1, 1, 90]\n"
                 "boxes:\n  - {x: 2, y: 1.5, sx: 0.5, sy: 0.5, yaw: 0, height: 0.3}\n"
                 "cylinders:\n  - {x: 3, y: 2, r: 0.1, height: 2.0}\n")
    w = World.from_yaml(str(p))
    assert w.nx == 80 and w.ny == 60
    assert math.isclose(w.start[2], math.pi / 2)
    assert w.height_at(np.array([2.0]), np.array([1.5]))[0] == np.float32(0.3)
