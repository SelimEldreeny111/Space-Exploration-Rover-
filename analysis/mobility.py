"""Mobility analysis: gradeability and step-climbing capability.

Quasi-static models (all six wheels driven, load shared equally by the
rocker-bogie, Earth tests unless noted). Outputs:
  paper/figures/fig_gradeability.pdf, fig_step_climb.pdf, analysis/out/mobility.json
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

import style
from style import BLUE, ORANGE, AQUA, INK2, MUTED, save, plt, COL_W
from rover_core.params import DEFAULT, G_EARTH, G_MARS, G_MOON

P = DEFAULT
M = P.motor
r = P.wheel_radius
CRR = 0.05          # rolling resistance, solid rubber on hard ground / firm soil (Wong 2008)
T_RATED = M.rated_torque
T_STALL = M.stall_torque
BODIES = [("Earth", G_EARTH, BLUE), ("Mars", G_MARS, ORANGE), ("Moon", G_MOON, AQUA)]


def torque_required_slope(theta, g, mass=P.mass, crr=CRR):
    """Per-wheel torque to climb slope theta [rad] at constant speed."""
    W = mass * g
    return (W / 6.0) * (np.sin(theta) + crr * np.cos(theta)) * r


def max_slope_torque(g, T, mass=P.mass, crr=CRR):
    k = 6.0 * T / (mass * g * r)
    if k >= math.sqrt(1 + crr ** 2):
        return math.pi / 2
    # sin(t) + crr cos(t) = k -> sqrt(1+crr^2) sin(t + atan(crr)) = k
    return math.asin(k / math.sqrt(1 + crr ** 2)) - math.atan(crr)


def max_slope_traction(mu, crr=CRR):
    # all wheels at the friction limit: mu cos(t) = sin(t) + crr cos(t)
    return math.atan(max(mu - crr, 0.0))


def step_climb_ok(h, mu, T, g, mass=P.mass):
    """Can both front wheels climb a step of height h (spanning the full width)?

    Front wheel at the step edge, contact at angle alpha from the downward
    vertical (cos alpha = (r-h)/r; alpha = 90 deg for h >= r, wheel pressed
    against the riser). The four other wheels push with P per front wheel,
    limited by traction and motor torque. Edge reactions (normal N through the
    wheel centre, tangential F from the wheel torque):
        N = P sin(a) + Wf cos(a),  F = Wf sin(a) - P cos(a)
    Requirements: F <= mu N and F r <= T (F <= 0 means the push alone lifts it).
    """
    W = mass * g
    Wf = W / 6.0
    a = math.pi / 2 if h >= r else math.acos((r - h) / r)
    push_per_wheel = min(mu * W / 6.0 - CRR * W / 6.0, T / r - CRR * W / 6.0)
    Pf = max(0.0, 4.0 * push_per_wheel / 2.0)
    N = Pf * math.sin(a) + Wf * math.cos(a)
    F = Wf * math.sin(a) - Pf * math.cos(a)
    if F <= 0:
        return True
    return F <= mu * N + 1e-12 and F * r <= T + 1e-12


def max_step(mu, T, g, hmax=r):
    hs = np.linspace(0.0, hmax, 601)
    ok = [step_climb_ok(h, mu, T, g) for h in hs]
    # largest h such that all smaller heights are also climbable
    best = 0.0
    for h, k in zip(hs, ok):
        if not k:
            break
        best = h
    return best


def main():
    res = {}
    # ------------------------------------------------ gradeability figure
    th = np.radians(np.linspace(0, 45, 451))
    fig, ax = plt.subplots(figsize=(COL_W, 2.35))
    for name, g, c in BODIES:
        ax.plot(np.degrees(th), torque_required_slope(th, g), color=c, lw=1.6, label=name)
    ax.axhline(T_RATED, color=INK2, lw=0.9, ls="--")
    ax.text(0.8, T_RATED + 0.12, f"rated motor torque {T_RATED:.2f} N·m", color=INK2,
            fontsize=6.5, ha="left")
    ax.text(36.0, 6.15, f"stall torque ≈ {T_STALL:.1f} N·m (off scale)", color=MUTED,
            fontsize=6.5, ha="right")
    mu = 0.8
    th_trac = math.degrees(max_slope_traction(mu))
    ax.axvline(th_trac, color=MUTED, lw=0.8, ls="-.")
    ax.text(th_trac - 0.6, 5.0, f"traction limit\n(μ = {mu})", color=MUTED, fontsize=6.5, ha="right")
    th_r = math.degrees(max_slope_torque(G_EARTH, T_RATED))
    ax.plot([th_r], [T_RATED], "o", ms=4.5, mfc="white", mec=BLUE, mew=1.2)
    ax.annotate(f"{th_r:.1f}°", (th_r, T_RATED), xytext=(th_r - 1.2, T_RATED + 0.85),
                fontsize=6.5, color=INK2, ha="right")
    ax.set_xlabel("slope angle [deg]")
    ax.set_ylabel("required torque per wheel [N·m]")
    ax.set_xlim(0, 45)
    ax.set_ylim(0, 6.6)
    ax.legend(loc="upper left", ncol=1, handlelength=1.6)
    save(fig, "fig_gradeability")

    for name, g, _ in BODIES:
        res[f"max_slope_rated_{name}_deg"] = round(math.degrees(max_slope_torque(g, T_RATED)), 1)
        res[f"max_slope_stall_{name}_deg"] = round(math.degrees(max_slope_torque(g, T_STALL)), 1)
    for mu_ in (0.4, 0.6, 0.8):
        res[f"max_slope_traction_mu{mu_}_deg"] = round(math.degrees(max_slope_traction(mu_)), 1)
    res["team_worst_case_torque_Nm"] = round(P.mass * G_EARTH / 6 * r, 3)
    res["flat_rolling_torque_Nm"] = round(P.mass * G_EARTH / 6 * CRR * r, 3)

    # ------------------------------------------------ step-climb figure
    mus = np.linspace(0.2, 1.2, 101)
    fig, ax = plt.subplots(figsize=(COL_W, 2.2))
    cases = [("Earth, rated torque", G_EARTH, T_RATED),
             ("Earth, stall torque", G_EARTH, T_STALL),
             ("Mars, rated torque", G_MARS, T_RATED)]
    curves = {}
    for lab, g, T in cases:
        curves[lab] = np.array([max_step(m, T, g) * 1000 for m in mus])
        res[f"max_step_mm_{lab.replace(', ', '_').replace(' ', '_')}_mu0.8"] = round(max_step(0.8, T, g) * 1000, 1)
        res[f"max_step_mm_{lab.replace(', ', '_').replace(' ', '_')}_mu0.6"] = round(max_step(0.6, T, g) * 1000, 1)
    # When torque is not binding, every force scales with the weight and the
    # limit depends on mu only: Earth/stall and Mars/rated coincide exactly.
    assert np.allclose(curves["Earth, stall torque"], curves["Mars, rated torque"])
    ax.plot(mus, curves["Earth, stall torque"], color=ORANGE, lw=1.6,
            label="friction-limited (Earth stall ≡ Mars rated)")
    ax.plot(mus, curves["Earth, rated torque"], color=BLUE, lw=1.6,
            label="Earth, rated torque 2.97 N·m")
    ax.axhline(r * 1000, color=MUTED, lw=0.8, ls=":")
    ax.text(0.21, r * 1000 + 3, "model limit: wheel radius (100 mm)", color=MUTED, fontsize=6.5)
    ax.set_xlabel("wheel-ground friction coefficient μ")
    ax.set_ylabel("max. climbable step [mm]")
    ax.set_xlim(0.2, 1.2)
    ax.set_ylim(0, 112)
    ax.legend(loc="lower right", handlelength=2.0)
    save(fig, "fig_step_climb")

    # speeds
    res["speed_no_load_mps"] = round(M.no_load_rpm * 2 * math.pi / 60 * r, 3)
    res["speed_rated_mps"] = round(M.rated_rpm * 2 * math.pi / 60 * r, 3)
    res["motor_rated_output_W"] = round(M.rated_output_power, 1)
    res["motor_rated_efficiency"] = round(M.rated_efficiency, 3)
    res["stall_torque_Nm"] = round(T_STALL, 2)
    res["encoder_resolution_mm"] = round(P.metres_per_count * 1000, 3)
    with open(os.path.join(style.OUT_DIR, "mobility.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
