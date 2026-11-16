"""2.5-D world model for the MISSION X rover simulator.

The world is a regular grid storing, for every cell, the height [m] of the
solid obstacle standing on it (0 = free floor). Sensors decide visibility from
that height: the mast-mounted LiDAR scans a horizontal plane at ~0.62 m, so
obstacles lower than that are invisible to it, while the body-mounted
ultrasonic sensors (~0.25 m) can see low obstacles. This is exactly the
failure mode reported by the 2023 team ("the LiDAR on top cannot see small
obstacles") and the reason for the ultrasonic costmap layer.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Box:
    x: float
    y: float
    sx: float
    sy: float
    yaw: float = 0.0       # degrees
    height: float = 1.0
    name: str = ""


@dataclass
class Cylinder:
    x: float
    y: float
    r: float
    height: float = 1.0
    name: str = ""


class World:
    def __init__(self, size: Tuple[float, float] = (10.0, 7.0), resolution: float = 0.02,
                 origin: Tuple[float, float] = (0.0, 0.0), name: str = "world",
                 start: Tuple[float, float, float] = (1.0, 1.0, 0.0)) -> None:
        self.name = name
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.size = (float(size[0]), float(size[1]))
        self.nx = int(round(self.size[0] / self.resolution))
        self.ny = int(round(self.size[1] / self.resolution))
        self.height = np.zeros((self.ny, self.nx), dtype=np.float32)  # [row=y, col=x]
        self.start = (float(start[0]), float(start[1]), math.radians(float(start[2])))
        self.boxes: List[Box] = []
        self.cylinders: List[Cylinder] = []

    # ------------------------------------------------------------ geometry
    def cell_centres(self) -> Tuple[np.ndarray, np.ndarray]:
        xs = self.origin[0] + (np.arange(self.nx) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(self.ny) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    def world_to_cell(self, x, y):
        ix = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(np.int64)
        iy = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(np.int64)
        return ix, iy

    def height_at(self, x, y) -> np.ndarray:
        """Obstacle height at world points (outside the map = 2 m wall)."""
        ix, iy = self.world_to_cell(x, y)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        h = np.full(np.shape(ix), 2.0, dtype=np.float32)
        h[inside] = self.height[iy[inside], ix[inside]]
        return h

    def add_box(self, b: Box) -> None:
        self.boxes.append(b)
        X, Y = self.cell_centres()
        c, s = math.cos(math.radians(b.yaw)), math.sin(math.radians(b.yaw))
        dx, dy = X - b.x, Y - b.y
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        m = (np.abs(lx) <= b.sx / 2) & (np.abs(ly) <= b.sy / 2)
        self.height[m] = np.maximum(self.height[m], b.height)

    def add_cylinder(self, cyl: Cylinder) -> None:
        self.cylinders.append(cyl)
        X, Y = self.cell_centres()
        r = max(cyl.r, self.resolution * 0.75)
        m = (X - cyl.x) ** 2 + (Y - cyl.y) ** 2 <= r * r
        self.height[m] = np.maximum(self.height[m], cyl.height)

    def add_outer_walls(self, thickness: float = 0.1, height: float = 2.0) -> None:
        n = max(1, int(round(thickness / self.resolution)))
        self.height[:n, :] = height
        self.height[-n:, :] = height
        self.height[:, :n] = height
        self.height[:, -n:] = height

    # ------------------------------------------------------------ export
    def to_occupancy(self, min_height: float = 0.0, resolution: Optional[float] = None):
        """Return (grid uint8 0/100 [row=y], resolution, origin) of obstacles
        taller than *min_height* (optionally down-sampled with max-pooling)."""
        occ = self.height > min_height
        res = self.resolution
        if resolution is not None and abs(resolution - res) > 1e-9:
            # exact max-pooling onto a grid of the requested resolution (also
            # for non-integer ratios, e.g. 0.02 m -> 0.05 m)
            res = float(resolution)
            nx_t = int(math.ceil(self.size[0] / res - 1e-9))
            ny_t = int(math.ceil(self.size[1] / res - 1e-9))
            X, Y = self.cell_centres()
            ix = np.minimum(((X - self.origin[0]) / res).astype(np.int64), nx_t - 1)
            iy = np.minimum(((Y - self.origin[1]) / res).astype(np.int64), ny_t - 1)
            out = np.zeros(ny_t * nx_t, dtype=bool)
            np.logical_or.at(out, (iy * nx_t + ix)[occ], True)
            occ = out.reshape(ny_t, nx_t)
        return (occ.astype(np.uint8) * 100), res, self.origin

    def save_pgm(self, path: str, min_height: float = 0.0, resolution: float = 0.05) -> str:
        """Write a ROS map_server map (``path``.pgm + ``path``.yaml)."""
        grid, res, origin = self.to_occupancy(min_height, resolution)
        img = np.where(grid > 0, 0, 254).astype(np.uint8)[::-1, :]  # PGM row 0 = top
        base = os.path.splitext(path)[0]
        with open(base + ".pgm", "wb") as f:
            f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
            f.write(img.tobytes())
        with open(base + ".yaml", "w", encoding="utf-8") as f:
            f.write(f"image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {res}\n"
                    f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n"
                    f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
        return base + ".yaml"

    # ------------------------------------------------------------ loaders
    @classmethod
    def from_yaml(cls, path: str) -> "World":
        import yaml  # PyYAML (python3-yaml on Ubuntu)
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        w = cls(size=tuple(d.get("size", (10.0, 7.0))), resolution=d.get("resolution", 0.02),
                origin=tuple(d.get("origin", (0.0, 0.0))), name=d.get("name", "world"),
                start=tuple(d.get("start", (1.0, 1.0, 0.0))))
        if d.get("image"):
            img_path = d["image"]
            if not os.path.isabs(img_path):
                img_path = os.path.join(os.path.dirname(os.path.abspath(path)), img_path)
            w._stamp_image(img_path, d.get("image_resolution", 0.05),
                           tuple(d.get("image_origin", (0.0, 0.0))), d.get("image_height", 2.0))
        for b in d.get("boxes", []) or []:
            w.add_box(Box(**{k: b[k] for k in ("x", "y", "sx", "sy", "yaw", "height", "name") if k in b}))
        for c in d.get("cylinders", []) or []:
            w.add_cylinder(Cylinder(**{k: c[k] for k in ("x", "y", "r", "height", "name") if k in c}))
        if d.get("outer_walls", True):
            w.add_outer_walls(d.get("wall_thickness", 0.1), d.get("wall_height", 2.0))
        return w

    def _stamp_image(self, img_path: str, res: float, origin: Tuple[float, float], height: float) -> None:
        img = _read_pgm(img_path)[::-1, :]  # row 0 = bottom
        occ = img < 0.35 * 255
        X, Y = self.cell_centres()
        ix = np.floor((X - origin[0]) / res).astype(int)
        iy = np.floor((Y - origin[1]) / res).astype(int)
        inside = (ix >= 0) & (ix < img.shape[1]) & (iy >= 0) & (iy < img.shape[0])
        m = np.zeros_like(inside)
        m[inside] = occ[iy[inside], ix[inside]]
        self.height[m] = np.maximum(self.height[m], height)

    @classmethod
    def default_lab(cls) -> "World":
        """~10 m x 7 m university laboratory resembling the 2023 test room:
        benches along three walls, a central island, wooden chairs and a few
        low crates/rocks that the mast-mounted LiDAR cannot see."""
        w = cls(size=(10.0, 7.0), resolution=0.02, name="lab", start=(1.5, 1.5, 0.0))
        B, C = w.add_box, w.add_cylinder
        # benches / cabinets (0.90 m) along the walls
        B(Box(5.0, 6.55, 8.6, 0.7, 0, 0.90, "bench_north"))
        B(Box(9.55, 3.6, 0.7, 4.2, 0, 0.90, "bench_east"))
        B(Box(3.2, 0.45, 3.4, 0.7, 0, 0.90, "bench_south"))
        # central island bench
        B(Box(5.2, 3.6, 2.6, 0.9, 0, 0.90, "island"))
        # cupboard and pillar
        B(Box(0.45, 5.05, 0.7, 1.4, 0, 1.80, "cupboard"))
        C(Cylinder(7.6, 1.6, 0.18, 2.0, "pillar"))
        # wooden chairs: 0.45 m seat (invisible to the LiDAR plane) + thin 0.85 m backrest
        for i, (cx, cy, yaw) in enumerate([(2.6, 5.5, 180), (3.6, 5.6, 170), (6.6, 5.62, 190),
                                           (5.2, 2.55, 0), (4.0, 4.65, 180), (8.6, 4.4, 270)]):
            B(Box(cx, cy, 0.42, 0.42, yaw, 0.45, f"chair{i}_seat"))
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            bx, by = cx - 0.20 * c, cy - 0.20 * s
            B(Box(bx, by, 0.04, 0.42, yaw, 0.85, f"chair{i}_back"))
        # low obstacles: crates / rocks (0.15-0.30 m)
        for i, (cx, cy, sx, sy, yaw, h) in enumerate([(3.0, 2.6, 0.40, 0.30, 15, 0.20),
                                                      (6.9, 3.0, 0.35, 0.35, 40, 0.28),
                                                      (8.0, 5.4, 0.45, 0.30, -10, 0.16),
                                                      (2.2, 3.7, 0.30, 0.30, 0, 0.25)]):
            B(Box(cx, cy, sx, sy, yaw, h, f"crate{i}"))
        w.add_outer_walls(0.1, 2.0)
        return w


def _read_pgm(path: str) -> np.ndarray:
    with open(path, "rb") as f:
        data = f.read()
    # minimal P5/P2 reader
    tokens: list = []
    idx = 0
    def next_token():
        nonlocal idx
        while True:
            while data[idx:idx + 1].isspace():
                idx += 1
            if data[idx:idx + 1] == b"#":
                while data[idx:idx + 1] not in (b"\n", b""):
                    idx += 1
                continue
            start = idx
            while not data[idx:idx + 1].isspace():
                idx += 1
            return data[start:idx]
    magic = next_token()
    wdt, hgt, maxv = int(next_token()), int(next_token()), int(next_token())
    idx += 1
    if magic == b"P5":
        dtype = np.uint8 if maxv < 256 else ">u2"
        img = np.frombuffer(data, dtype=dtype, count=wdt * hgt, offset=idx).reshape(hgt, wdt)
    else:
        vals = data[idx:].split()
        img = np.array([int(v) for v in vals[:wdt * hgt]]).reshape(hgt, wdt)
    return (img.astype(np.float32) * (255.0 / maxv)).astype(np.uint8)
