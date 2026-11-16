"""SLAM study in the simulated laboratory.

A mapping loop (like the team's 2023 tele-operated Hector-SLAM run) is driven
with pure pursuit along an A* path. The same sensor stream feeds
  (a) mapping at the wheel-odometry pose (nominal chi, mean wheel fusion) and
  (b) Hector-style scan-to-map matching (odometry increment as prior).
Metrics: absolute trajectory error (ATE), map accuracy = distance of mapped
occupied cells to the true LiDAR-visible obstacles, and map completeness.
Outputs: paper/figures/fig_slam.pdf, analysis/out/slam.json, analysis/out/slam_run.npz
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
from scipy import ndimage

import style
from style import BLUE, ORANGE, AQUA, INK, INK2, MUTED, save, plt, PAGE_W
from rover_core.params import DEFAULT
from rover_core.kinematics import Pose2D, wrap_angle
from rover_core.sim import World, RoverSim
from sim_lib import HectorMatcher, GridMap, astar

DT = 0.02
SCAN_EVERY = 10          # 5 Hz
RES = 0.05


def plan_route(world, waypoints, clearance=0.48, k=2):
    """A* route on the true obstacle map (the tele-operator sees everything)."""
    occ = world.height > 0.08
    res = world.resolution * k
    ny, nx = (occ.shape[0] // k) * k, (occ.shape[1] // k) * k
    occ = occ[:ny, :nx].reshape(ny // k, k, nx // k, k).any(axis=(1, 3))
    dist = ndimage.distance_transform_edt(~occ) * res
    cost = np.where(dist <= clearance, np.inf, 1.0 + 3.0 * np.exp(-4.0 * (dist - clearance)))
    segments = []
    for (ax, ay), (bx, by) in zip(waypoints[:-1], waypoints[1:]):
        s = (int(ax / res), int(ay / res))
        g = (int(bx / res), int(by / res))
        p = astar(cost, s, g)
        if p is None:
            raise RuntimeError(f"no path ({ax},{ay})->({bx},{by}); clearance there "
                               f"{dist[s[1], s[0]]:.2f}/{dist[g[1], g[0]]:.2f} m")
        P = np.array([((i + 0.5) * res, (j + 0.5) * res) for i, j in p])
        for _ in range(3):  # light smoothing, end points fixed
            P[1:-1] = 0.25 * P[:-2] + 0.5 * P[1:-1] + 0.25 * P[2:]
        segments.append(P)
    return segments


def pure_pursuit(pose, path, idx, la=0.8, v=0.3):
    x, y, th = pose
    while idx < len(path) - 1 and math.hypot(path[idx, 0] - x, path[idx, 1] - y) < la:
        idx += 1
    tx, ty = path[idx]
    a = wrap_angle(math.atan2(ty - y, tx - x) - th)
    # a 1 m-long skid-steer body must turn on the spot before driving off,
    # otherwise its corners sweep into obstacles beside the path
    if abs(a) > math.radians(20):
        return 0.0, 0.5 * np.sign(a), idx
    v = v * max(0.4, math.cos(2 * a))
    w = 2 * v * math.sin(a) / la
    return v, float(np.clip(w, -0.8, 0.8)), idx


def odo_step(last, ticks, b_eff, fuse=np.mean):
    mpc = DEFAULT.metres_per_count
    d = [(a - b) * mpc for a, b in zip(ticks, last)]
    dl, dr = fuse(d[:3]), fuse(d[3:])
    return 0.5 * (dl + dr), (dr - dl) / b_eff


def compose(p, ds, dth):
    x, y, th = p
    return np.array([x + ds * math.cos(th + dth / 2), y + ds * math.sin(th + dth / 2), wrap_angle(th + dth)])


def map_metrics(gm: GridMap, gt_occ: np.ndarray, observed: np.ndarray):
    P = gm.prob()
    est = P > 0.65
    gt_dist = ndimage.distance_transform_edt(~gt_occ) * gm.res
    est_dist = ndimage.distance_transform_edt(~est) * gm.res
    d = gt_dist[est]
    acc_mean = float(d.mean()) if d.size else float("nan")
    acc_p95 = float(np.percentile(d, 95)) if d.size else float("nan")
    tgt = gt_occ & observed
    complete = float((est_dist[tgt] <= 0.10).mean()) if tgt.any() else float("nan")
    spurious = float((d > 0.15).mean()) if d.size else float("nan")
    return {"accuracy_mean_m": round(acc_mean, 3), "accuracy_p95_m": round(acc_p95, 3),
            "completeness_10cm": round(complete, 3), "spurious_gt15cm": round(spurious, 3),
            "occupied_cells": int(est.sum())}


def main(seed=7):
    world = World.default_lab()
    # waypoints inside the region where the rover can also turn in place
    # (clearance >= circumscribed radius 0.63 m)
    wps = [(1.5, 1.5), (4.0, 1.7), (6.2, 1.9), (7.6, 2.45), (8.2, 3.4), (7.4, 4.5), (5.3, 5.0),
           (7.4, 4.5), (8.2, 3.2), (7.6, 2.45), (6.2, 1.9), (3.5, 1.8), (1.6, 2.4), (1.55, 4.6),
           (1.6, 2.6), (1.8, 1.5)]
    segments = plan_route(world, wps, clearance=0.62)
    route = np.vstack(segments)
    seg = 0
    sim = RoverSim(world, seed=seed, chi_true=DEFAULT.skid_chi * 1.10,
                   start=(1.5, 1.5, 0.0))
    hector = HectorMatcher(world.size, res=RES, levels=3, rmax=DEFAULT.lidar_range[1])
    odo_map = GridMap(world.size, RES)
    odo_pose = np.array(sim.true_pose)
    slam_pose = np.array(sim.true_pose)
    last_ticks = sim.wheel_ticks()
    last_scan_ticks = list(last_ticks)
    odo_at_last_scan = odo_pose.copy()
    truth, est_odo, est_slam, times = [], [], [], []
    idx, k = 0, 0
    t_max = 900.0
    while sim.t < t_max:
        cur = segments[seg]
        if math.hypot(cur[-1, 0] - sim.x, cur[-1, 1] - sim.y) < 0.25:
            seg += 1
            idx = 0
            if seg >= len(segments):
                break
            cur = segments[seg]
        v, w, idx = pure_pursuit(sim.true_pose, cur, idx)
        sim.set_cmd(v, w)
        sim.step(DT)
        ticks = sim.wheel_ticks()
        ds, dth = odo_step(last_ticks, ticks, DEFAULT.b_eff)
        last_ticks = ticks
        odo_pose = compose(odo_pose, ds, dth)
        k += 1
        if k % SCAN_EVERY == 0:
            r, a0, inc = sim.scan()
            ang = a0 + np.arange(r.size) * inc
            # odometry increment since last scan, applied to the SLAM pose (prior)
            c, s = math.cos(-odo_at_last_scan[2]), math.sin(-odo_at_last_scan[2])
            dx, dy = odo_pose[0] - odo_at_last_scan[0], odo_pose[1] - odo_at_last_scan[1]
            lx, ly = c * dx - s * dy, s * dx + c * dy
            dtheta = wrap_angle(odo_pose[2] - odo_at_last_scan[2])
            cs, sn = math.cos(slam_pose[2]), math.sin(slam_pose[2])
            guess = np.array([slam_pose[0] + cs * lx - sn * ly, slam_pose[1] + sn * lx + cs * ly,
                              wrap_angle(slam_pose[2] + dtheta)])
            slam_pose = hector.process(guess, r, ang)
            odo_at_last_scan = odo_pose.copy()
            odo_map.update(tuple(odo_pose), r, ang, DEFAULT.lidar_range[1], (0.05, 0.0))
            truth.append(sim.true_pose)
            est_odo.append(odo_pose.copy())
            est_slam.append(slam_pose.copy())
            times.append(sim.t)
    T, O, S = np.array(truth), np.array(est_odo), np.array(est_slam)
    e_odo = np.hypot(O[:, 0] - T[:, 0], O[:, 1] - T[:, 1])
    e_slam = np.hypot(S[:, 0] - T[:, 0], S[:, 1] - T[:, 1])
    h_odo = np.degrees(np.abs([wrap_angle(a) for a in O[:, 2] - T[:, 2]]))
    h_slam = np.degrees(np.abs([wrap_angle(a) for a in S[:, 2] - T[:, 2]]))
    # ground truth LiDAR-visible occupancy at map resolution
    gt, _, _ = world.to_occupancy(DEFAULT.lidar_height - 0.02, RES)
    gt = gt > 0
    gt = gt[:hector.maps[0].ny, :hector.maps[0].nx]
    observed = np.abs(hector.maps[0].L) > 0.5
    res = {
        "duration_s": round(times[-1], 1), "distance_m": round(sim.distance, 2),
        "scans": len(times), "collisions": sim.collisions, "chi_true_over_nominal": 1.10,
        "ate_rmse_odometry_m": round(float(np.sqrt(np.mean(e_odo ** 2))), 3),
        "ate_rmse_slam_m": round(float(np.sqrt(np.mean(e_slam ** 2))), 3),
        "final_err_odometry_m": round(float(e_odo[-1]), 3),
        "final_err_slam_m": round(float(e_slam[-1]), 3),
        "max_err_slam_m": round(float(e_slam.max()), 3),
        "heading_rmse_odometry_deg": round(float(np.sqrt(np.mean(h_odo ** 2))), 2),
        "heading_rmse_slam_deg": round(float(np.sqrt(np.mean(h_slam ** 2))), 2),
        "map_odometry": map_metrics(odo_map, gt, observed),
        "map_slam": map_metrics(hector.maps[0], gt, observed),
    }
    print(json.dumps(res, indent=1))
    np.savez_compressed(os.path.join(style.OUT_DIR, "slam_run.npz"), truth=T, odo=O, slam=S, t=np.array(times),
                        P_slam=hector.maps[0].prob(), P_odo=odo_map.prob(), gt=gt, route=route)
    with open(os.path.join(style.OUT_DIR, "slam.json"), "w") as f:
        json.dump(res, f, indent=1)

    # ------------------------------------------------------------ figure
    ext = [0, world.size[0], 0, world.size[1]]
    fig = plt.figure(figsize=(PAGE_W, 3.6))
    gs = fig.add_gridspec(2, 3, height_ratios=[2.2, 1.0], hspace=0.45, wspace=0.08)
    gray = plt.get_cmap("Greys")
    panels = [("(a) ground truth (LiDAR-visible obstacles)", np.where(gt, 1.0, 0.0), T, INK),
              ("(b) map from wheel odometry only", hector_to_img(odo_map), O, ORANGE),
              ("(c) Hector-style scan-matching SLAM", hector_to_img(hector.maps[0]), S, BLUE)]
    for i, (title, img, traj, col) in enumerate(panels):
        ax = fig.add_subplot(gs[0, i])
        ax.imshow(img, origin="lower", extent=ext, cmap=gray, vmin=0, vmax=1, interpolation="nearest")
        if i > 0:
            ax.plot(T[:, 0], T[:, 1], color=INK, lw=0.8, ls="--")
        ax.plot(traj[:, 0], traj[:, 1], color=col, lw=1.3)
        ax.set_title(title, loc="left", fontsize=7.2)
        ax.set_xticks([0, 2, 4, 6, 8, 10])
        ax.set_yticks([0, 2, 4, 6] if i == 0 else [])
        ax.grid(False)
        ax.set_aspect("equal")
        if i == 0:
            ax.set_ylabel("y [m]")
        ax.set_xlabel("x [m]")
    ax = fig.add_subplot(gs[1, :])
    ax.plot(times, e_odo, color=ORANGE, lw=1.4, label=f"wheel odometry (ATE RMSE {res['ate_rmse_odometry_m']:.2f} m)")
    ax.plot(times, e_slam, color=BLUE, lw=1.4, label=f"scan-matching SLAM (ATE RMSE {res['ate_rmse_slam_m']:.3f} m)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error [m]")
    ax.set_xlim(0, times[-1])
    ax.legend(loc="upper left")
    save(fig, "fig_slam")


def hector_to_img(gm: GridMap):
    P = gm.prob()
    img = np.full(P.shape, 0.18)          # unknown = light grey
    img[P < 0.35] = 0.0                    # free = white (Greys cmap: 0 = white)
    img[P > 0.65] = 1.0                    # occupied = black
    return img


if __name__ == "__main__":
    main()
