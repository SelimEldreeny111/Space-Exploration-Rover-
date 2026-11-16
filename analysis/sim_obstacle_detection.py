"""Controlled obstacle-approach study (perception + reactive safety layer).

The rover drives straight at 0.35 m/s towards a 0.30 x 0.30 m obstacle of
height h placed 2.6 m ahead with lateral offset y_o. A reactive safety layer
(equivalent to nav2_collision_monitor's "stop" polygon) runs at 10 Hz and
stops the rover when any sensed point lies inside the swept corridor
(rover width + 2 x 5 cm) closer than 0.35 m ahead of the bumper.
  A) LiDAR only (mast, 0.62 m)      B) LiDAR + six HC-SR04 (0.25 m), stateless
  C) as B, but sonar detections are remembered in the odometry frame for 10 s
     (persistent marking, as in a costmap range layer)
Outcome per run: collision / safe stop / no conflict (obstacle outside path).
Outputs: paper/figures/fig_obstacle_detection.pdf, analysis/out/obstacle_detection.json
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
from matplotlib.colors import ListedColormap

import style
from style import INK, INK2, MUTED, GOOD, CRITICAL, save, plt, PAGE_W
from rover_core.params import DEFAULT
from rover_core.sim import Box, RoverSim, World

DT = 0.02
V = 0.35
STOP_GAP = 0.35
HALF_W = DEFAULT.width / 2 + 0.05
FRONT = DEFAULT.length / 2
HEIGHTS = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.70]
OFFSETS = np.round(np.arange(-0.60, 0.601, 0.10), 2)


def corridor_gap(points_xy_robot):
    if points_xy_robot.size == 0:
        return math.inf
    x, y = points_xy_robot[:, 0], points_xy_robot[:, 1]
    m = (x > FRONT - 0.05) & (np.abs(y) <= HALF_W)
    return float((x[m] - FRONT).min()) if m.any() else math.inf


def lidar_points(sim):
    r, a0, inc = sim.scan()
    ok = np.isfinite(r)
    a = a0 + np.arange(r.size)[ok] * inc
    ox, oy = sim.lidar_offset
    return np.stack([ox + r[ok] * np.cos(a), oy + r[ok] * np.sin(a)], axis=1)


def ultrasonic_points(sim):
    pts = []
    half = math.radians(DEFAULT.ultrasonic_fov_deg / 2)
    for (ux, uy, uyaw), r in zip(DEFAULT.ultrasonic_poses, sim.ultrasonic()):
        if not math.isfinite(r):
            continue
        # an echo only gives range: the reflector may be anywhere on the arc
        for da in np.linspace(-half, half, 5):
            a = math.radians(uyaw) + da
            pts.append((ux + r * math.cos(a), uy + r * math.sin(a)))
    return np.array(pts) if pts else np.zeros((0, 2))


def to_world(sim, pts):
    c, s = math.cos(sim.th), math.sin(sim.th)
    return np.stack([sim.x + c * pts[:, 0] - s * pts[:, 1], sim.y + s * pts[:, 0] + c * pts[:, 1]], 1)


def to_robot(sim, pts):
    c, s = math.cos(sim.th), math.sin(sim.th)
    dx, dy = pts[:, 0] - sim.x, pts[:, 1] - sim.y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], 1)


def run(h, y_o, use_us, seed, memory=False):
    w = World(size=(8.0, 6.0), resolution=0.02, start=(1.0, 3.0, 0.0))
    w.add_outer_walls()
    ox = 1.0 + FRONT + 2.6
    w.add_box(Box(ox, 3.0 + y_o, 0.30, 0.30, 0.0, h, "obstacle"))
    sim = RoverSim(w, seed=seed, start=(1.0, 3.0, 0.0), chi_true=DEFAULT.skid_chi)
    stopped_for = 0.0
    k = 0
    cmd = V
    mem, mem_t = np.zeros((0, 2)), np.zeros(0)
    while sim.t < 20.0:
        if k % 5 == 0:
            gap = corridor_gap(lidar_points(sim)) if k % 10 == 0 else gap_l
            gap_l = gap
            if use_us:
                up = ultrasonic_points(sim)
                gap = min(gap, corridor_gap(up))
                if memory:
                    if up.size:
                        mem = np.vstack([mem, to_world(sim, up)])
                        mem_t = np.concatenate([mem_t, np.full(len(up), sim.t)])
                    keep = mem_t > sim.t - 10.0
                    mem, mem_t = mem[keep], mem_t[keep]
                    if mem.size:
                        gap = min(gap, corridor_gap(to_robot(sim, mem)))
            cmd = 0.0 if gap < STOP_GAP else V
            sim.set_cmd(cmd, 0.0)
        sim.step(DT)
        k += 1
        if sim.collided:
            return "collision", None
        if cmd == 0.0 and abs(sim.v) < 1e-3:
            stopped_for += DT
            if stopped_for > 0.5:
                # clearance between bumper and obstacle face
                face = ox - 0.15 - (sim.x + FRONT)
                return "stop", round(face, 3)
        if sim.x > ox + 0.8:
            return "clear", None
    return "clear", None


def main(seeds=(1, 2, 3)):
    res = {"heights": HEIGHTS, "offsets": OFFSETS.tolist(), "seeds": list(seeds)}
    grids = {}
    for cond, use_us, memory in (("A_lidar_only", False, False), ("B_lidar_plus_ultrasonic", True, False),
                                 ("C_ultrasonic_with_memory", True, True)):
        g = np.zeros((len(HEIGHTS), len(OFFSETS)))
        gaps = []
        counts = {"collision": 0, "stop": 0, "clear": 0}
        for i, h in enumerate(HEIGHTS):
            for j, yo in enumerate(OFFSETS):
                outs = [run(h, float(yo), use_us, s, memory) for s in seeds]
                for o, gp in outs:
                    counts[o] += 1
                    if gp is not None:
                        gaps.append(gp)
                # worst outcome across seeds: collision (2) > stop (1) > clear (0)
                g[i, j] = max({"clear": 0, "stop": 1, "collision": 2}[o] for o, _ in outs)
            print(cond, h, "".join("._X"[int(v)] for v in g[i]), flush=True)
        grids[cond] = g
        conflict = (g > 0)
        res[cond] = {"counts": counts, "conflict_cells": int(conflict.sum()),
                     "collision_cells": int((g == 2).sum()),
                     "mean_stop_clearance_m": round(float(np.mean(gaps)), 3) if gaps else None,
                     "min_stop_clearance_m": round(float(np.min(gaps)), 3) if gaps else None}
    # obstacles below the LiDAR plane that intersect the path
    print(json.dumps({k: v for k, v in res.items() if k.startswith(("A_", "B_"))}, indent=1))
    with open(os.path.join(style.OUT_DIR, "obstacle_detection.json"), "w") as f:
        json.dump(res, f, indent=1)

    cmap = ListedColormap(["#f0efec", GOOD, CRITICAL])
    fig, axs = plt.subplots(1, 3, figsize=(PAGE_W, 2.3), sharey=True)
    for ax, (cond, title) in zip(axs, (("A_lidar_only", "(a) LiDAR only (2023)"),
                                       ("B_lidar_plus_ultrasonic", "(b) + ultrasonic ring, stateless"),
                                       ("C_ultrasonic_with_memory", "(c) + ultrasonic ring, persistent"))):
        g = grids[cond]
        ax.imshow(g, origin="lower", cmap=cmap, vmin=0, vmax=2, aspect="auto",
                  extent=[OFFSETS[0] - 0.05, OFFSETS[-1] + 0.05, -0.5, len(HEIGHTS) - 0.5],
                  interpolation="nearest")
        for i in range(len(HEIGHTS)):
            for j, yo in enumerate(OFFSETS):
                if g[i, j] == 2:
                    ax.text(yo, i, "×", ha="center", va="center", fontsize=6.5, color="white")
        ax.axhline(HEIGHTS.index(0.60) - 0.5, color=INK, lw=0.8, ls="--")
        ax.text(OFFSETS[-1] + 0.03, HEIGHTS.index(0.60) - 0.45, "LiDAR plane", ha="right", va="bottom",
                fontsize=6.3, color=INK)
        ax.set_yticks(range(len(HEIGHTS)))
        ax.set_yticklabels([f"{h:.2f}" for h in HEIGHTS])
        ax.set_xlabel("lateral offset [m]")
        ax.set_xticks([-0.6, -0.3, 0, 0.3, 0.6])
        ax.set_title(title, loc="left", fontsize=7.2)
        ax.grid(False)
        c = res[cond]["counts"]
        ax.text(0.0, -1.55, "", fontsize=6)
    axs[0].set_ylabel("obstacle height [m]")
    handles = [plt.Line2D([], [], marker="s", lw=0, ms=7, color=CRITICAL, label="collision (×)"),
               plt.Line2D([], [], marker="s", lw=0, ms=7, color=GOOD, label="safe stop"),
               plt.Line2D([], [], marker="s", lw=0, ms=7, color="#f0efec", mec=MUTED, mew=0.4,
                          label="no conflict (obstacle clears the body)")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.98), ncol=3, fontsize=6.6)
    save(fig, "fig_obstacle_detection")


if __name__ == "__main__":
    main()
