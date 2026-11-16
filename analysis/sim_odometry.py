"""Dead-reckoning study: 4 m square driven twice (open-loop commands).

Estimators (all fed by the same simulated encoder ticks / gyro):
  E1 mean of the 3 wheels per side, nominal chi      (2023-style odometry)
  E2 mean, chi calibrated by a 360 deg spin test     (calibration procedure)
  E3 median of the 3 wheels per side, calibrated chi (v2 bridge)
  E4 median + gyro heading (bias from 5 s standstill)  (EKF-style fusion)
Outputs: paper/figures/fig_odometry.pdf, analysis/out/odometry.json
"""
from __future__ import annotations

import json
import math
import os
from statistics import median

import numpy as np

import style
from style import BLUE, ORANGE, AQUA, YELLOW, INK, INK2, MUTED, save, plt, COL_W, PAGE_W
from rover_core.params import DEFAULT, RoverParams
from rover_core.kinematics import Pose2D, wrap_angle
from rover_core.sim import World, RoverSim

DT = 0.02


class Est:
    def __init__(self, b_eff, fuse="median", gyro=False, gyro_bias=0.0):
        self.b, self.fuse, self.gyro, self.bias = b_eff, fuse, gyro, gyro_bias
        self.p = Pose2D()
        self.last = None

    def update(self, ticks, gz, dt):
        if self.last is None:
            self.last = list(ticks)
            return
        mpc = DEFAULT.metres_per_count
        d = [(a - b) * mpc for a, b in zip(ticks, self.last)]
        self.last = list(ticks)
        f = median if self.fuse == "median" else (lambda v: sum(v) / len(v))
        dl, dr = f(d[:3]), f(d[3:])
        ds = 0.5 * (dl + dr)
        dth = (gz - self.bias) * dt if self.gyro else (dr - dl) / self.b
        self.p = self.p.compose_local(ds * math.cos(dth / 2), ds * math.sin(dth / 2), dth)


def run(seed, laps=2, side=4.0, v=0.3, w=0.4):
    world = World(size=(30.0, 30.0), resolution=0.1, start=(5.0, 5.0, 0.0))
    chi_true = DEFAULT.skid_chi * (1.0 + np.random.default_rng(seed).uniform(0.05, 0.15))
    sim = RoverSim(world, seed=seed, chi_true=chi_true)
    # standstill gyro bias estimate (5 s)
    gz = [sim.imu()["gz"] for _ in range(int(5 / DT))]
    bias_hat = float(np.mean(gz))
    # spin calibration of chi: command 2 full turns, measure heading with a
    # reference (here the true heading change +- 1 % reading error)
    chi_cal = chi_true * (1.0 + np.random.default_rng(seed + 99).normal(0, 0.01))
    nominal = DEFAULT.b_eff
    ests = {"E1": Est(nominal, "mean"), "E2": Est(chi_cal * DEFAULT.track, "mean"),
            "E3": Est(chi_cal * DEFAULT.track, "median"),
            "E4": Est(nominal, "median", gyro=True, gyro_bias=bias_hat)}
    x0, y0, t0 = sim.true_pose
    truth = []
    traj = {k: [] for k in ests}

    def tick(t_end, v_cmd, w_cmd):
        sim.set_cmd(v_cmd, w_cmd)
        for _ in range(int(round(t_end / DT))):
            sim.step(DT)
            tk, g = sim.wheel_ticks(), sim.imu()["gz"]
            for e in ests.values():
                e.update(tk, g, DT)
            truth.append(sim.true_pose)
            for k, e in ests.items():
                traj[k].append((e.p.x, e.p.y))

    for k in ests:
        ests[k].update(sim.wheel_ticks(), 0.0, DT)
    for _ in range(laps * 4):
        tick(side / v, v, 0.0)
        tick(0.6, 0.0, 0.0)
        tick((math.pi / 2) / w, 0.0, w)   # commanded 90 deg (nominal chi)
        tick(0.6, 0.0, 0.0)
    T = np.array(truth)
    # express truth in the start frame
    c, s = math.cos(-t0), math.sin(-t0)
    tx = c * (T[:, 0] - x0) - s * (T[:, 1] - y0)
    ty = s * (T[:, 0] - x0) + c * (T[:, 1] - y0)
    out = {"dist": sim.distance, "truth": np.stack([tx, ty], 1)}
    for k in ests:
        E = np.array(traj[k])
        err = np.hypot(E[:, 0] - tx, E[:, 1] - ty)
        out[k] = {"final": float(err[-1]), "rms": float(np.sqrt(np.mean(err ** 2))), "traj": E}
    return out


def main():
    seeds = range(20)
    runs = [run(s) for s in seeds]
    names = {"E1": "mean of 3 wheels, nominal χ (2023)", "E2": "mean, calibrated χ",
             "E3": "median, calibrated χ", "E4": "median + gyro heading"}
    colors = {"E1": ORANGE, "E2": YELLOW, "E3": AQUA, "E4": BLUE}
    res = {"seeds": len(runs), "mean_distance_m": float(np.mean([r["dist"] for r in runs]))}
    for k in names:
        fin = np.array([r[k]["final"] for r in runs])
        rms = np.array([r[k]["rms"] for r in runs])
        res[k] = {"label": names[k], "final_mean_m": round(float(fin.mean()), 3),
                  "final_std_m": round(float(fin.std()), 3),
                  "final_pct_of_distance": round(float(100 * fin.mean() / res["mean_distance_m"]), 2),
                  "rms_mean_m": round(float(rms.mean()), 3)}
    print(json.dumps(res, indent=1))

    fig, axs = plt.subplots(1, 2, figsize=(PAGE_W, 2.45), gridspec_kw={"width_ratios": [1.05, 1.0]})
    r0 = runs[3]
    ax = axs[0]
    ax.plot(r0["truth"][:, 0], r0["truth"][:, 1], color=INK, lw=1.8, label="ground truth")
    for k in names:
        E = r0[k]["traj"]
        ax.plot(E[:, 0], E[:, 1], color=colors[k], lw=1.2, label=names[k])
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("two laps of a commanded 4 m square (one seed)", loc="left")
    h = [plt.Line2D([], [], color=INK, lw=1.8, label="ground truth")] +         [plt.Line2D([], [], color=colors[k], lw=1.2, label=f"E{i + 1}") for i, k in enumerate(names)]
    ax.legend(handles=h, fontsize=6.2, loc="center left", bbox_to_anchor=(1.0, 0.5), handlelength=1.4)
    ax = axs[1]
    ks = list(names)
    y = np.arange(len(ks))[::-1]
    m = [res[k]["final_pct_of_distance"] for k in ks]
    sd = [100 * res[k]["final_std_m"] / res["mean_distance_m"] for k in ks]
    ax.barh(y, m, xerr=sd, height=0.55, color=[colors[k] for k in ks], edgecolor=style.SURFACE,
            error_kw=dict(ecolor=INK2, lw=0.8, capsize=2))
    for yi, k, mi in zip(y, ks, m):
        ax.text(mi + max(sd) * 0.15 + 0.15, yi + 0.3, f"{mi:.2f} %", fontsize=6.8, color=INK, va="center")
    ax.set_yticks(y)
    ax.set_yticklabels([f"E{i + 1}  {names[k]}" for i, k in enumerate(ks)], fontsize=6.8)
    ax.set_xlabel(f"final position error [% of {res['mean_distance_m']:.0f} m travelled]")
    ax.grid(axis="y", visible=False)
    ax.set_title("dead-reckoning error (20 seeds, mean ± sd)", loc="left")
    fig.tight_layout(w_pad=1.5)
    save(fig, "fig_odometry")
    with open(os.path.join(style.OUT_DIR, "odometry.json"), "w") as f:
        json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
