"""Frontier-based exploration logic (ROS-free, numpy only).

A *frontier cell* is a known-free cell with at least one unknown 4-neighbour.
Frontier cells are grouped into 8-connected clusters; each cluster large
enough becomes a :class:`Frontier` whose goal is the cluster cell closest to
its centroid among the cells with enough clearance from obstacles (so Nav2
does not reject it as lethal). Clusters are ranked with the classic
explore_lite style cost

    cost = w_distance * distance(robot, goal) - w_size * frontier_length [m]

and failed goals are blacklisted (radius ``blacklist_radius``).

Grid convention = nav_msgs/OccupancyGrid: ``grid[row, col]`` with row = y
index, col = x index, values -1 (unknown) or 0..100.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

UNKNOWN = -1
_N8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


@dataclass(frozen=True)
class GridInfo:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def cell_to_world(self, row: float, col: float) -> Tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)


@dataclass
class FrontierParams:
    free_thresh: int = 25            # occupancy <= free_thresh -> free
    occ_thresh: int = 65             # occupancy >= occ_thresh -> obstacle
    min_frontier_size: int = 8       # cells
    clearance: float = 0.45          # [m] goal distance from obstacles (~ half rover width + margin)
    w_distance: float = 1.0
    w_size: float = 0.5
    min_goal_distance: float = 0.5   # [m] closer goals cannot reveal anything new
    blacklist_radius: float = 0.6    # [m]


@dataclass
class Frontier:
    cells: np.ndarray                # (N, 2) int rows/cols
    centroid: Tuple[float, float]
    goal: Tuple[float, float]
    distance: float                  # robot -> goal [m]
    length: float                    # frontier size [m]
    cost: float

    @property
    def size(self) -> int:
        return int(len(self.cells))


# ----------------------------------------------------------------- grid ops
def grid_from_occupancy(data: Sequence[int], width: int, height: int) -> np.ndarray:
    """OccupancyGrid.data (row-major, row 0 = bottom) -> int16 array [row, col]."""
    return np.asarray(data, dtype=np.int16).reshape(height, width)


def _shift(mask: np.ndarray, dr: int, dc: int) -> np.ndarray:
    """out[r, c] = mask[r + dr, c + dc] (False outside the grid)."""
    out = np.zeros_like(mask)
    h, w = mask.shape
    rs, re = max(0, -dr), min(h, h - dr)
    cs, ce = max(0, -dc), min(w, w - dc)
    if rs < re and cs < ce:
        out[rs:re, cs:ce] = mask[rs + dr:re + dr, cs + dc:ce + dc]
    return out


def dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Binary dilation with a disk of *radius_cells*."""
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    r2 = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if (dr or dc) and dr * dr + dc * dc <= r2:
                out |= _shift(mask, dr, dc)
    return out


def frontier_mask(grid: np.ndarray, free_thresh: int = 25) -> np.ndarray:
    free = (grid >= 0) & (grid <= free_thresh)
    unknown = grid < 0
    near_unknown = (_shift(unknown, 1, 0) | _shift(unknown, -1, 0) |
                    _shift(unknown, 0, 1) | _shift(unknown, 0, -1))
    return free & near_unknown


def cluster_cells(mask: np.ndarray) -> List[np.ndarray]:
    """8-connected components of *mask* -> list of (N, 2) arrays (row, col)."""
    seen = np.zeros_like(mask, dtype=bool)
    h, w = mask.shape
    clusters = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        q = deque([(int(r0), int(c0))])
        cells = []
        while q:
            r, c = q.popleft()
            cells.append((r, c))
            for dr, dc in _N8:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    q.append((rr, cc))
        clusters.append(np.array(cells, dtype=int))
    return clusters


# ----------------------------------------------------------------- detection
def detect_frontiers(grid: np.ndarray, info: GridInfo, robot_xy: Tuple[float, float],
                     p: FrontierParams = FrontierParams()) -> List[Frontier]:
    """All frontiers with a valid goal, sorted by increasing cost."""
    fmask = frontier_mask(grid, p.free_thresh)
    if not fmask.any():
        return []
    occupied = grid >= p.occ_thresh
    blocked = dilate(occupied, int(math.ceil(p.clearance / info.resolution)))
    rx, ry = robot_xy
    out: List[Frontier] = []
    for cells in cluster_cells(fmask):
        if len(cells) < p.min_frontier_size:
            continue
        ok = cells[~blocked[cells[:, 0], cells[:, 1]]]
        if len(ok) == 0:
            continue
        cr, cc = cells[:, 0].mean(), cells[:, 1].mean()
        k = int(np.argmin((ok[:, 0] - cr) ** 2 + (ok[:, 1] - cc) ** 2))
        goal = info.cell_to_world(ok[k, 0], ok[k, 1])
        dist = math.hypot(goal[0] - rx, goal[1] - ry)
        length = len(cells) * info.resolution
        out.append(Frontier(cells=cells, centroid=info.cell_to_world(cr, cc), goal=goal,
                            distance=dist, length=length,
                            cost=p.w_distance * dist - p.w_size * length))
    out.sort(key=lambda f: f.cost)
    return out


class FrontierExplorer:
    """Frontier selection with a blacklist of failed / unobservable goals."""

    def __init__(self, params: FrontierParams = FrontierParams()) -> None:
        self.p = params
        self.blacklist: List[Tuple[float, float]] = []

    def reset(self) -> None:
        self.blacklist.clear()

    def blacklist_goal(self, xy: Tuple[float, float]) -> None:
        self.blacklist.append((float(xy[0]), float(xy[1])))

    def is_blacklisted(self, xy: Tuple[float, float]) -> bool:
        r2 = self.p.blacklist_radius ** 2
        return any((xy[0] - bx) ** 2 + (xy[1] - by) ** 2 <= r2 for bx, by in self.blacklist)

    def candidates(self, grid: np.ndarray, info: GridInfo,
                   robot_xy: Tuple[float, float]) -> List[Frontier]:
        return [f for f in detect_frontiers(grid, info, robot_xy, self.p)
                if not self.is_blacklisted(f.goal) and f.distance >= self.p.min_goal_distance]

    def select(self, grid: np.ndarray, info: GridInfo,
               robot_xy: Tuple[float, float]) -> Optional[Frontier]:
        c = self.candidates(grid, info, robot_xy)
        return c[0] if c else None

    def frontier_near(self, grid: np.ndarray, info: GridInfo, xy: Tuple[float, float],
                      radius: float) -> bool:
        """True while frontier cells remain within *radius* of *xy* (the
        current target is still worth reaching)."""
        cell = info.world_to_cell(*xy)
        if cell is None:
            return False
        k = int(math.ceil(radius / info.resolution))
        r0, c0 = cell
        sub = grid[max(0, r0 - k):r0 + k + 1, max(0, c0 - k):c0 + k + 1]
        return bool(frontier_mask(sub, self.p.free_thresh).any())
