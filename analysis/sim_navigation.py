"""EXPERIMENTAL - not used in the paper (superseded by sim_obstacle_detection.py).

The simplified A*/DWA planner and sonar costmap here produce artefacts (wide
sonar arcs over-block corridors; cone clearing erases true marks), so mission
outcomes reflect the toy planner as much as the sensor configuration. Kept as
a starting point for a full Nav2-in-the-loop study (docker/ eval service).

Navigation study: does fusing the ultrasonic sensors into the costmap stop
collisions with obstacles that the mast-mounted LiDAR cannot see?

Eight point-to-point missions in the simulated laboratory, each run with
  A) LiDAR-only costmap (2023 configuration: move_base + LiDAR), and
  B) LiDAR + range-sensor layer from the six HC-SR04 (v2 configuration).
Static map = true LiDAR-visible obstacles (a perfect SLAM map), global A*
replanned at 1 Hz, DWA local planner at 10 Hz with the real 1.02 x 0.73 m
footprint, localisation = ground truth (AMCL accuracy is studied separately).
Outputs: paper/figures/fig_navigation.pdf, analysis/out/navigation.json
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

import style
from style import BLUE, ORANGE, INK, INK2, MUTED, CRITICAL, save, plt, PAGE_W
from rover_core.params import DEFAULT
from rover_core.kinematics import wrap_angle
from rover_core.sim import World, RoverSim
from sim_lib import Costmap, astar, dwa_control

DT = 0.02
RES = 0.05
T_MAX = 150.0
GOAL_TOL = 0.35

MISSIONS = [
    ((1.6, 1.6, 0), (6.0, 2.0)),
    ((6.0, 1.8, 180), (1.6, 2.4)),
    ((1.6, 2.4, 90), (1.55, 4.6)),
    ((8.2, 3.2, 90), (5.3, 5.0)),
    ((5.3, 5.0, 0), (8.2, 3.0)),
    ((6.2, 1.9, 0), (8.2, 3.6)),
    ((8.2, 3.6, 270), (4.0, 1.8)),
    ((1.55, 4.6, 270), (6.2, 1.9)),
]


def plan(cm: Costmap, pose, goal):
    cost, dist = cm.planning_cost()
    s = cm.idx(pose[0], pose[1])
    g = cm.idx(goal[0], goal[1])
    s = (int(s[0]), int(s[1]))
    g = (int(g[0]), int(g[1]))
    c2 = cost.copy()
    # never let the start cell be blocked (rover may sit inside inflation)
    c2[max(0, s[1] - 6):s[1] + 7, max(0, s[0] - 6):s[0] + 7] = np.where(
        np.isfinite(c2[max(0, s[1] - 6):s[1] + 7, max(0, s[0] - 6):s[0] + 7]),
        c2[max(0, s[1] - 6):s[1] + 7, max(0, s[0] - 6):s[0] + 7], 50.0)
    p = astar(c2, s, g)
    if p is None:
        return None, dist
    P = np.array([((i + 0.5) * RES + cm.origin[0], (j + 0.5) * RES + cm.origin[1]) for i, j in p])
    for _ in range(3):
        P[1:-1] = 0.25 * P[:-2] + 0.5 * P[1:-1] + 0.25 * P[2:]
    return P, dist


def inject_low_obstacle(start, goal, seed, size=0.30, height=0.20):
    """Return a fresh lab with one low crate placed ON the nominal LiDAR-only
    route of this mission (so only the ultrasonics can see it), such that a
    detour still exists for the 1.02 x 0.73 m rover."""
    from rover_core.sim import Box
    rng = np.random.default_rng(seed)
    base = World.default_lab()
    gt, _, _ = base.to_occupancy(DEFAULT.lidar_height - 0.02, RES)
    nominal, _ = plan(Costmap(gt > 0, RES, inscribed=0.40, inflation=0.9), start, goal)
    if nominal is None:
        raise RuntimeError("mission infeasible")
    L = np.concatenate([[0], np.cumsum(np.hypot(np.diff(nominal[:, 0]), np.diff(nominal[:, 1])))])
    for f in rng.permutation(np.linspace(0.35, 0.7, 8)):
        i = int(np.searchsorted(L, f * L[-1]))
        cx, cy = nominal[min(i, len(nominal) - 1)]
        if math.hypot(cx - start[0], cy - start[1]) < 1.3 or math.hypot(cx - goal[0], cy - goal[1]) < 1.0:
            continue
        w = World.default_lab()
        w.add_box(Box(float(cx), float(cy), size, size, float(rng.uniform(0, 90)), height, "injected_crate"))
        full, _, _ = w.to_occupancy(0.08, RES)
        p, _ = plan(Costmap(full > 0, RES, inscribed=0.40, inflation=0.9), start, goal)
        if p is not None:
            return w, (float(cx), float(cy))
    raise RuntimeError("no feasible injection point")


def run_mission(world, static_occ, start, goal, use_us, seed):
    sim = RoverSim(world, seed=seed, start=(start[0], start[1], math.radians(start[2])))
    cm = Costmap(static_occ, RES, inscribed=0.40, inflation=0.9)
    half = math.radians(DEFAULT.ultrasonic_fov_deg / 2)
    path, dist = None, None
    vel = (0.0, 0.0)
    traj = []
    recovery = 0.0
    status = "timeout"
    k = 0
    fails = 0
    while sim.t < T_MAX:
        x, y, th = sim.true_pose
        if math.hypot(goal[0] - x, goal[1] - y) < GOAL_TOL:
            status = "success"
            break
        if k % 10 == 0:  # LiDAR 5 Hz
            r, a0, inc = sim.scan()
            cm.mark_lidar(sim.true_pose, r, a0 + np.arange(r.size) * inc, DEFAULT.lidar_range[1])
        if use_us and k % 5 == 0:  # ultrasonic 10 Hz
            us = sim.ultrasonic()
            c, s_ = math.cos(th), math.sin(th)
            for (ux, uy, uyaw), rr in zip(DEFAULT.ultrasonic_poses, us):
                cm.mark_range((x + c * ux - s_ * uy, y + s_ * ux + c * uy, th + math.radians(uyaw)),
                              rr, half, rmax=2.0)
        if k % 50 == 0:  # global replan at 1 Hz (also when no path exists)
            p, dist = plan(cm, sim.true_pose, goal)
            path = p
            fails = 0 if p is not None else fails + 1
        if k % 5 == 0:  # controller 10 Hz
            if recovery > 0:
                cmd = (-0.12, 0.0)
                recovery -= 0.1
            elif path is None:
                cmd = (0.0, 0.25)       # no global path: rotate on the spot to re-sense
            else:
                _, dist = cm.planning_cost()
                cmd = dwa_control(sim.true_pose, vel, path, dist, RES, cm.origin, goal)
                if cmd is None:
                    recovery = 0.6
                    cmd = (-0.12, 0.0)
            vel = cmd
            sim.set_cmd(*cmd)
        sim.step(DT)
        k += 1
        if k % 5 == 0:
            traj.append(sim.true_pose)
        if sim.collided:
            status = "collision"
            break
    traj.append(sim.true_pose)
    T = np.array(traj)
    length = float(np.sum(np.hypot(np.diff(T[:, 0]), np.diff(T[:, 1])))) if len(T) > 1 else 0.0
    return {"status": status, "time_s": round(sim.t, 1), "length_m": round(length, 2),
            "traj": T, "us_cells": int(cm.us_mark.sum())}


def main(seeds=(1, 2, 3)):
    world = World.default_lab()
    gt, _, _ = world.to_occupancy(DEFAULT.lidar_height - 0.02, RES)
    static = gt > 0          # the map only contains what the LiDAR can see
    results = {"A_lidar_only": [], "B_lidar_plus_ultrasonic": []}
    keep = {}
    injected = {}
    for mi, (st, gl) in enumerate(MISSIONS):
        for seed in seeds:
            world_m, crate = inject_low_obstacle(st, gl, seed=1000 * mi + seed)
            if seed == seeds[0]:
                injected[mi] = crate
            for cond, use_us in (("A_lidar_only", False), ("B_lidar_plus_ultrasonic", True)):
                r = run_mission(world_m, static, st, gl, use_us, seed=100 * mi + seed)
                print(f"mission {mi} seed {seed} {cond:26s} {r['status']:9s} t={r['time_s']:6.1f}s "
                      f"L={r['length_m']:5.2f}m", flush=True)
                if seed == seeds[0]:
                    keep[(mi, cond)] = r
                results[cond].append({"mission": mi, "seed": seed, **{k: v for k, v in r.items() if k != "traj"}})
    summary = {}
    for cond, rows in results.items():
        n = len(rows)
        succ = [r for r in rows if r["status"] == "success"]
        summary[cond] = {
            "runs": n,
            "success_rate": round(len(succ) / n, 3),
            "collision_rate": round(sum(r["status"] == "collision" for r in rows) / n, 3),
            "timeout_rate": round(sum(r["status"] == "timeout" for r in rows) / n, 3),
            "mean_time_success_s": round(float(np.mean([r["time_s"] for r in succ])), 1) if succ else None,
            "mean_length_success_m": round(float(np.mean([r["length_m"] for r in succ])), 2) if succ else None,
        }
    out = {"missions": MISSIONS, "seeds": list(seeds), "summary": summary, "runs": results,
           "injected_seed0": {str(k): v for k, v in injected.items()}}
    with open(os.path.join(style.OUT_DIR, "navigation.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(summary, indent=1))

    # ------------------------------------------------------------ figure
    fig, axs = plt.subplots(1, 2, figsize=(PAGE_W, 2.75), sharey=True)
    h = world.height
    shade = np.zeros(h.shape + (4,))
    tall = h >= DEFAULT.lidar_height - 0.02
    low = (h > 0.08) & ~tall
    shade[tall] = (0.05, 0.05, 0.05, 0.85)
    shade[low] = (0.89, 0.29, 0.28, 0.75)   # low obstacles (invisible to LiDAR)
    ext = [0, world.size[0], 0, world.size[1]]
    for ax, cond, col, title in ((axs[0], "A_lidar_only", ORANGE, "(a) LiDAR-only costmap"),
                                 (axs[1], "B_lidar_plus_ultrasonic", BLUE, "(b) LiDAR + ultrasonic range layer")):
        ax.imshow(shade, origin="lower", extent=ext, interpolation="nearest")
        s = summary[cond]
        for mi in range(len(MISSIONS)):
            r = keep[(mi, cond)]
            T = r["traj"]
            ax.plot(T[:, 0], T[:, 1], color=col, lw=1.1)
            ax.plot(*MISSIONS[mi][1], marker="*", color=INK, ms=5)
            ax.plot(*injected[mi], marker="s", color="#e34948", ms=4.2, mec=INK, mew=0.4)
            if r["status"] == "collision":
                ax.plot(T[-1, 0], T[-1, 1], marker="X", color=INK, mfc=CRITICAL, ms=7, mew=0.6)
        ax.set_title(f"{title}: {100 * s['success_rate']:.0f} % success, "
                     f"{100 * s['collision_rate']:.0f} % collisions ({s['runs']} runs)", loc="left", fontsize=7.2)
        ax.set_aspect("equal")
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 7)
        ax.grid(False)
        ax.set_xlabel("x [m]")
    axs[0].set_ylabel("y [m]")
    handles = [plt.Line2D([], [], color="#0d0d0d", lw=6, alpha=0.85, label="obstacle ≥ LiDAR plane (0.60 m)"),
               plt.Line2D([], [], color="#e34948", lw=6, alpha=0.75, label="low obstacle (crate, chair seat)"),
               plt.Line2D([], [], marker="s", color="#e34948", mec=INK, mew=0.4, lw=0, ms=4.5,
                          label="crate injected on route (seed 1)"),
               plt.Line2D([], [], marker="*", color=INK, lw=0, ms=6, label="goal"),
               plt.Line2D([], [], marker="X", color=INK, mfc=CRITICAL, lw=0, ms=7, label="collision")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.99), ncol=5, fontsize=6.3)
    save(fig, "fig_navigation")


if __name__ == "__main__":
    seeds = tuple(int(a) for a in sys.argv[1:]) or (1, 2, 3)
    main(seeds)
