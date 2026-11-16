import math

import numpy as np
import pytest

from rover_mission.frontiers import (UNKNOWN, FrontierExplorer, FrontierParams, GridInfo,
                                     cluster_cells, detect_frontiers, dilate, frontier_mask,
                                     grid_from_occupancy)

RES = 0.05


def make_grid(h=100, w=100, fill=UNKNOWN):
    return np.full((h, w), fill, dtype=np.int16)


def info_for(g, ox=0.0, oy=0.0):
    return GridInfo(RES, ox, oy, g.shape[1], g.shape[0])


def walls(g, r0, r1, c0, c1):
    """Closed room interior rows r0..r1, cols c0..c1 (free), walls around it."""
    g[r0 - 1:r1 + 2, c0 - 1:c1 + 2] = 100
    g[r0:r1 + 1, c0:c1 + 1] = 0
    return g


# ------------------------------------------------------------------ basics
def test_grid_info_round_trip():
    info = GridInfo(0.05, -2.0, 1.0, 40, 30)
    r, c = info.world_to_cell(-1.0, 1.5)
    assert (r, c) == (10, 20)
    x, y = info.cell_to_world(r, c)
    assert x == pytest.approx(-0.975) and y == pytest.approx(1.525)
    assert info.world_to_cell(-2.1, 1.5) is None and info.world_to_cell(0.0, 2.5) is None


def test_grid_from_occupancy_row_major():
    g = grid_from_occupancy([0, 1, 2, 3, 4, 5], 3, 2)
    assert g.shape == (2, 3) and g[1, 0] == 3


def test_dilate_disk():
    m = np.zeros((11, 11), bool)
    m[5, 5] = True
    d = dilate(m, 2)
    assert d.sum() == 13 and d[5, 7] and not d[7, 7]            # disk of radius 2


def test_cluster_8_connectivity():
    m = np.zeros((6, 6), bool)
    m[0, 0] = m[1, 1] = m[2, 2] = True                           # diagonal chain -> 1 cluster
    m[5, 5] = True
    sizes = sorted(len(c) for c in cluster_cells(m))
    assert sizes == [1, 3]


# ------------------------------------------------------------------ detection
def test_closed_room_has_no_frontier():
    g = walls(make_grid(), 20, 60, 20, 60)
    assert not frontier_mask(g).any()
    assert detect_frontiers(g, info_for(g), (2.0, 2.0)) == []


def test_open_doorway_gives_one_frontier_on_that_side():
    g = walls(make_grid(), 20, 60, 20, 60)
    g[35:46, 61] = 0                                             # doorway in the east wall
    info = info_for(g)
    fr = detect_frontiers(g, info, (2.0, 2.0), FrontierParams(min_frontier_size=3, clearance=0.1))
    assert len(fr) == 1
    gx, gy = fr[0].goal
    assert gx == pytest.approx(info.cell_to_world(40, 61)[0], abs=0.06)
    assert 35 * RES <= gy <= 46 * RES


def test_small_frontiers_are_ignored():
    g = walls(make_grid(), 20, 60, 20, 60)
    g[40:42, 61] = 0                                             # 2-cell gap
    assert detect_frontiers(g, info_for(g), (2.0, 2.0), FrontierParams(min_frontier_size=8)) == []


def test_clearance_moves_goal_away_from_walls():
    g = make_grid()
    g[:, :50] = 0                                                # known free half, unknown right half
    g[10:90, 30] = 100                                           # wall touching ...
    g[49:52, 40:50] = 100                                        # ... and an obstacle at the frontier
    info = info_for(g)
    f = detect_frontiers(g, info, (0.5, 2.5), FrontierParams(clearance=0.3))[0]
    r, c = info.world_to_cell(*f.goal)
    occ = np.argwhere(g >= 65)
    assert np.min(np.hypot(occ[:, 0] - r, occ[:, 1] - c)) * RES >= 0.3 - 1e-9
    assert c == 49                                               # still on the frontier column


def test_cost_prefers_near_frontier_unless_far_one_is_much_larger():
    g = make_grid(100, 200)
    g[40:61, 0:200] = 0                                          # corridor, unknown above/below
    g[39, :] = 100
    g[61, :] = 100
    g[39, 20:30] = 0                                             # small opening near the robot
    g[39, 120:190] = 0                                           # large opening far away
    info = info_for(g)
    robot = info.cell_to_world(50, 25)
    near_only = FrontierParams(w_distance=1.0, w_size=0.0, clearance=0.0, min_frontier_size=3)
    best = detect_frontiers(g, info, robot, near_only)
    assert len(best) == 2 and best[0].goal[0] < 2.0              # nearest wins
    gain = FrontierParams(w_distance=1.0, w_size=3.0, clearance=0.0, min_frontier_size=3)
    best = detect_frontiers(g, info, robot, gain)
    assert best[0].length > 3.0                                  # big frontier wins


def test_blacklist_and_min_distance():
    g = walls(make_grid(), 20, 60, 20, 60)
    g[35:46, 61] = 0
    info = info_for(g)
    ex = FrontierExplorer(FrontierParams(min_frontier_size=3, clearance=0.1, blacklist_radius=0.5))
    f = ex.select(g, info, (2.0, 2.0))
    assert f is not None
    ex.blacklist_goal(f.goal)
    assert ex.is_blacklisted((f.goal[0] + 0.3, f.goal[1])) and ex.select(g, info, (2.0, 2.0)) is None
    ex.reset()
    near = (f.goal[0] - 0.2, f.goal[1])                          # robot right next to it
    assert ex.select(g, info, near) is None                      # closer than min_goal_distance
    assert ex.select(g, info, (2.0, 2.0)) is not None


def test_frontier_near():
    g = walls(make_grid(), 20, 60, 20, 60)
    g[35:46, 61] = 0
    info = info_for(g)
    ex = FrontierExplorer()
    assert ex.frontier_near(g, info, info.cell_to_world(40, 61), 0.3)
    assert not ex.frontier_near(g, info, info.cell_to_world(25, 25), 0.3)
    assert not ex.frontier_near(g, info, (-5.0, -5.0), 0.3)


def test_exploration_converges_on_simulated_sensor():
    """Integration: reveal a hidden map with a disk 'sensor' around each goal;
    the explorer must uncover the whole room and then report no frontier."""
    truth = walls(make_grid(80, 120), 5, 74, 5, 114)
    truth[5:60, 60] = 100                                        # inner wall with a gap
    known = make_grid(80, 120)
    info = info_for(truth)

    def sense(xy, radius=1.2):
        r0, c0 = info.world_to_cell(*xy)
        k = int(radius / RES)
        rr, cc = np.ogrid[:truth.shape[0], :truth.shape[1]]
        disk = (rr - r0) ** 2 + (cc - c0) ** 2 <= k * k
        known[disk] = truth[disk]

    ex = FrontierExplorer(FrontierParams(min_frontier_size=4, clearance=0.15, min_goal_distance=0.2))
    robot = info.cell_to_world(20, 20)
    sense(robot)
    for _ in range(200):
        f = ex.select(known, info, robot)
        if f is None:
            break
        robot = f.goal                                           # "navigate" there
        sense(robot)
    else:
        pytest.fail("exploration did not terminate")
    free_truth = truth == 0
    assert (known[free_truth] == 0).mean() > 0.97
