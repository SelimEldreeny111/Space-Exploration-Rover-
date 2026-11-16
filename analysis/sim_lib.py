"""Algorithms for the simulation study (pure numpy).

* GridMap        - log-odds occupancy grid with ray-traced updates
* HectorMatcher  - multi-resolution Gauss-Newton scan-to-map matching
                   (Kohlbrecher et al., SSRR 2011), the front-end of hector_slam
* astar          - 8-connected A* on a cost grid
* Costmap        - static layer + sensor marking/clearing (LiDAR, ultrasonic)
                   + inflation, mirroring the Nav2 layered costmap
* dwa_control    - Dynamic Window Approach (Fox et al., 1997) with the real
                   rectangular footprint
"""
from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from rover_core.params import DEFAULT


# ====================================================================== maps
class GridMap:
    def __init__(self, size, res, origin=(0.0, 0.0), l_occ=0.85, l_free=-0.40, l_clamp=5.0):
        self.res = float(res)
        self.origin = (float(origin[0]), float(origin[1]))
        self.nx = int(math.ceil(size[0] / res))
        self.ny = int(math.ceil(size[1] / res))
        self.L = np.zeros((self.ny, self.nx), np.float32)
        self.l_occ, self.l_free, self.l_clamp = l_occ, l_free, l_clamp

    def prob(self) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def _idx(self, x, y):
        ix = np.floor((x - self.origin[0]) / self.res).astype(np.int64)
        iy = np.floor((y - self.origin[1]) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix[ok], iy[ok]

    def update(self, pose, ranges, angles, rmax, sensor_offset=(0.0, 0.0)):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        sx = x + c * sensor_offset[0] - s * sensor_offset[1]
        sy = y + s * sensor_offset[0] + c * sensor_offset[1]
        ok = np.isfinite(ranges) & (ranges <= rmax)
        r = ranges[ok]
        a = angles[ok] + th
        if r.size == 0:
            return
        step = self.res * 0.5
        n = int(math.ceil(r.max() / step))
        t = np.arange(n) * step
        mask = t[None, :] < (r[:, None] - self.res * 0.75)
        fx = sx + np.cos(a)[:, None] * t[None, :]
        fy = sy + np.sin(a)[:, None] * t[None, :]
        ix, iy = self._idx(fx[mask], fy[mask])
        free = np.unique(iy * self.nx + ix)
        ex, ey = self._idx(sx + np.cos(a) * r, sy + np.sin(a) * r)
        occ = np.unique(ey * self.nx + ex)
        free = np.setdiff1d(free, occ, assume_unique=True)
        Lf = self.L.reshape(-1)
        Lf[free] += self.l_free
        Lf[occ] += self.l_occ
        np.clip(self.L, -self.l_clamp, self.l_clamp, out=self.L)

    def interp(self, px, py):
        """Bilinear occupancy probability and world-frame gradient."""
        P = self._P
        u = (px - self.origin[0]) / self.res - 0.5
        v = (py - self.origin[1]) / self.res - 0.5
        u0 = np.floor(u).astype(np.int64)
        v0 = np.floor(v).astype(np.int64)
        du, dv = u - u0, v - v0
        ok = (u0 >= 0) & (u0 < self.nx - 1) & (v0 >= 0) & (v0 < self.ny - 1)
        u0c, v0c = np.clip(u0, 0, self.nx - 2), np.clip(v0, 0, self.ny - 2)
        m00 = P[v0c, u0c]
        m10 = P[v0c, u0c + 1]
        m01 = P[v0c + 1, u0c]
        m11 = P[v0c + 1, u0c + 1]
        M = (1 - dv) * ((1 - du) * m00 + du * m10) + dv * ((1 - du) * m01 + du * m11)
        dMu = (1 - dv) * (m10 - m00) + dv * (m11 - m01)
        dMv = (1 - du) * (m01 - m00) + du * (m11 - m10)
        M = np.where(ok, M, 0.0)
        return M, np.where(ok, dMu, 0.0) / self.res, np.where(ok, dMv, 0.0) / self.res

    def freeze(self):
        self._P = self.prob()


class HectorMatcher:
    """Multi-resolution scan-to-map Gauss-Newton matcher + mapper."""

    def __init__(self, size, origin=(0.0, 0.0), res=0.05, levels=3, rmax=3.5, sensor_offset=(0.05, 0.0)):
        self.maps = [GridMap(size, res * (2 ** k), origin) for k in range(levels)]
        self.rmax = rmax
        self.off = sensor_offset
        self.pose = None

    def _endpoints(self, ranges, angles):
        ok = np.isfinite(ranges) & (ranges <= self.rmax)
        r, a = ranges[ok], angles[ok]
        return r * np.cos(a) + self.off[0], r * np.sin(a) + self.off[1]

    def match(self, guess, ranges, angles, iters=(4, 4, 6)):
        sx, sy = self._endpoints(ranges, angles)
        if sx.size < 20:
            return np.array(guess, float)
        xi = np.array(guess, float)
        for lvl in range(len(self.maps) - 1, -1, -1):
            m = self.maps[lvl]
            m.freeze()
            for _ in range(iters[lvl] if lvl < len(iters) else 4):
                c, s = math.cos(xi[2]), math.sin(xi[2])
                wx = xi[0] + c * sx - s * sy
                wy = xi[1] + s * sx + c * sy
                M, gx, gy = m.interp(wx, wy)
                dth_x = -s * sx - c * sy
                dth_y = c * sx - s * sy
                J = np.stack([gx, gy, gx * dth_x + gy * dth_y], axis=1)
                e = 1.0 - M
                H = J.T @ J + np.diag([1e-3, 1e-3, 1e-3])
                g = J.T @ e
                d = np.linalg.solve(H, g)
                # trust region: hector limits steps implicitly; clamp for robustness
                d[:2] = np.clip(d[:2], -0.15, 0.15)
                d[2] = float(np.clip(d[2], -0.15, 0.15))
                xi = xi + d
                if np.abs(d[:2]).max() < 1e-4 and abs(d[2]) < 1e-4:
                    break
        return xi

    def integrate(self, pose, ranges, angles):
        for m in self.maps:
            m.update(pose, ranges, angles, self.rmax, self.off)

    def process(self, guess, ranges, angles):
        if self.pose is None:
            self.pose = np.array(guess, float)
            self.integrate(self.pose, ranges, angles)
            return self.pose.copy()
        self.pose = self.match(guess, ranges, angles)
        self.integrate(self.pose, ranges, angles)
        return self.pose.copy()


# ================================================================= planning
def astar(cost: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """A* on a grid of traversal costs (np.inf = blocked). Cells are (ix, iy)."""
    ny, nx = cost.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < nx and 0 <= sy < ny and 0 <= gx < nx and 0 <= gy < ny):
        return None
    if not np.isfinite(cost[gy, gx]):
        return None
    g = np.full(cost.shape, np.inf)
    parent = -np.ones(cost.shape, np.int64)
    g[sy, sx] = 0.0
    h0 = math.hypot(gx - sx, gy - sy)
    pq = [(h0, 0.0, sx, sy)]
    nbr = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
           (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    closed = np.zeros(cost.shape, bool)
    while pq:
        f, gc, x, y = heapq.heappop(pq)
        if closed[y, x]:
            continue
        closed[y, x] = True
        if (x, y) == (gx, gy):
            path = []
            idx = y * nx + x
            while idx >= 0:
                py, px = divmod(idx, nx)
                path.append((px, py))
                idx = parent[py, px]
            return path[::-1]
        for dx, dy, dl in nbr:
            xx, yy = x + dx, y + dy
            if 0 <= xx < nx and 0 <= yy < ny and not closed[yy, xx]:
                c = cost[yy, xx]
                if not np.isfinite(c):
                    continue
                ng = gc + dl * c
                if ng < g[yy, xx]:
                    g[yy, xx] = ng
                    parent[yy, xx] = y * nx + x
                    heapq.heappush(pq, (ng + math.hypot(gx - xx, gy - yy), ng, xx, yy))
    return None


class Costmap:
    """Static + sensor layers + inflation (Nav2-like)."""

    def __init__(self, static_occ: np.ndarray, res: float, origin=(0.0, 0.0),
                 inscribed=0.38, inflation=0.75, decay_s=8.0):
        self.res = res
        self.origin = origin
        self.static = static_occ.astype(bool)
        self.dyn = np.zeros_like(self.static, dtype=np.float32)  # time-stamp of marking
        self.dyn_mark = np.zeros_like(self.static, dtype=bool)   # LiDAR obstacle layer
        self.us_mark = np.zeros_like(self.static, dtype=bool)    # range-sensor layer (separate,
        # as nav2_costmap_2d::RangeSensorLayer: LiDAR ray-clearing cannot erase it)
        self.inscribed, self.inflation = inscribed, inflation
        self.ny, self.nx = self.static.shape

    def idx(self, x, y):
        ix = np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(np.int64)
        iy = np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(np.int64)
        return ix, iy

    def _inside(self, ix, iy):
        return (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)

    def mark_lidar(self, pose, ranges, angles, rmax, offset=(0.05, 0.0), mark_range=3.0):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        sx, sy = x + c * offset[0] - s * offset[1], y + s * offset[0] + c * offset[1]
        a = angles + th
        fin = np.isfinite(ranges)
        rr = np.where(fin, ranges, rmax)
        # clear along rays
        step = self.res * 0.7
        n = int(math.ceil(rmax / step))
        t = np.arange(1, n) * step
        mask = t[None, :] < (np.minimum(rr, mark_range)[:, None] - self.res)
        ix, iy = self.idx(sx + np.cos(a)[:, None] * t[None, :], sy + np.sin(a)[:, None] * t[None, :])
        ok = mask & self._inside(ix, iy)
        self.dyn_mark[iy[ok], ix[ok]] = False
        # mark endpoints
        m = fin & (ranges <= mark_range)
        ex, ey = self.idx(sx + np.cos(a[m]) * ranges[m], sy + np.sin(a[m]) * ranges[m])
        ok = self._inside(ex, ey)
        self.dyn_mark[ey[ok], ex[ok]] = True

    def mark_range(self, sensor_pose, r, half_fov, rmax=2.0):
        """Nav2 RangeSensorLayer-like: clear inside the cone, mark the arc."""
        sx, sy, sa = sensor_pose
        ang = sa + np.linspace(-half_fov, half_fov, 7)
        dist = min(r, rmax) if math.isfinite(r) else rmax
        t = np.arange(self.res, dist - self.res * 1.5, self.res * 0.7)
        if t.size:
            ix, iy = self.idx(sx + np.cos(ang)[:, None] * t[None, :], sy + np.sin(ang)[:, None] * t[None, :])
            ok = self._inside(ix, iy)
            self.us_mark[iy[ok], ix[ok]] = False
        if math.isfinite(r) and r <= rmax:
            ex, ey = self.idx(sx + np.cos(ang) * r, sy + np.sin(ang) * r)
            ok = self._inside(ex, ey)
            self.us_mark[ey[ok], ex[ok]] = True

    def obstacles(self) -> np.ndarray:
        return self.static | self.dyn_mark | self.us_mark

    def planning_cost(self) -> Tuple[np.ndarray, np.ndarray]:
        obs = self.obstacles()
        dist = ndimage.distance_transform_edt(~obs) * self.res
        cost = np.ones_like(dist)
        lethal = dist <= self.inscribed
        band = (dist > self.inscribed) & (dist < self.inflation)
        cost[band] += 8.0 * np.exp(-6.0 * (dist[band] - self.inscribed))
        cost[lethal] = np.inf
        return cost, dist


def footprint_points(length=DEFAULT.length, width=DEFAULT.width, nu=9, nv=7):
    u = np.linspace(-length / 2, length / 2, nu)
    v = np.linspace(-width / 2, width / 2, nv)
    U, V = np.meshgrid(u, v)
    return U.ravel(), V.ravel()


FP_U, FP_V = footprint_points()


def dwa_control(pose, vel, path_xy, obs_dist, res, origin, goal_xy,
                vmax=0.40, wmax=0.8, acc_v=0.6, acc_w=1.5, dt_ctrl=0.1, horizon=2.0, margin=0.03):
    """Return (v, w). obs_dist: distance-to-obstacle grid [m] of the local costmap."""
    x, y, th = pose
    v0, w0 = vel
    vs = np.linspace(max(0.0, v0 - acc_v * 0.5), min(vmax, v0 + acc_v * 0.5), 6)
    ws = np.linspace(max(-wmax, w0 - acc_w * 0.5), min(wmax, w0 + acc_w * 0.5), 13)
    V, W = np.meshgrid(vs, ws)
    V, W = V.ravel(), W.ravel()
    steps = int(horizon / dt_ctrl)
    X = np.full(V.shape, x)
    Y = np.full(V.shape, y)
    T = np.full(V.shape, th)
    valid = np.ones(V.shape, bool)
    clear = np.full(V.shape, 10.0)
    ny, nx = obs_dist.shape
    for k in range(steps):
        T = T + W * dt_ctrl
        X = X + V * np.cos(T) * dt_ctrl
        Y = Y + V * np.sin(T) * dt_ctrl
        if k % 2 == 0 or k == steps - 1:
            c, s = np.cos(T)[:, None], np.sin(T)[:, None]
            px = X[:, None] + c * FP_U[None, :] - s * FP_V[None, :]
            py = Y[:, None] + s * FP_U[None, :] + c * FP_V[None, :]
            ix = np.clip(((px - origin[0]) / res).astype(np.int64), 0, nx - 1)
            iy = np.clip(((py - origin[1]) / res).astype(np.int64), 0, ny - 1)
            d = obs_dist[iy, ix].min(axis=1)
            clear = np.minimum(clear, d)
            valid &= d > margin
    # target: lookahead point on the path
    P = np.asarray(path_xy)
    dd = np.hypot(P[:, 0] - x, P[:, 1] - y)
    i0 = int(dd.argmin())
    la = 1.2
    j = i0
    while j < len(P) - 1 and math.hypot(P[j, 0] - x, P[j, 1] - y) < la:
        j += 1
    tx, ty = P[j]
    d_goal = np.hypot(tx - X, ty - Y)
    head = np.abs(np.arctan2(np.sin(np.arctan2(ty - Y, tx - X) - T), np.cos(np.arctan2(ty - Y, tx - X) - T)))
    score = -1.0 * d_goal - 0.35 * head + 0.6 * np.minimum(clear, 0.6) + 0.3 * V
    if not valid.any():
        return None
    score[~valid] = -np.inf
    k = int(score.argmax())
    return float(V[k]), float(W[k])
